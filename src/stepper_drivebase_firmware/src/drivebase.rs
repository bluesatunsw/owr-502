use stm32g4xx_hal as hal;

use hal::{
    gpio, pac,
    prelude::*,
    rcc::{self, Rcc},
};

use crate::encoder_bus::{EncoderBus, EncoderNcsPins, EncoderSpiPins};
use crate::tmc_registers::{
    AMax, ChopConf, DMax, GConf, GStat, GlobalScalar, IHoldIRun, PwmConf, RampMode, TPowerDown,
    TPwmThrs, TZeroWait, TmcPosition, UnitlessExt, UnitsExt, VMax, VStart, VStop, XActual, XTarget,
    A1, D1, V1,
};
use crate::{
    as_registers::{Settings1, Settings2},
    common::{Channel, ALL_CHANNELS},
};

use crate::stepper_bus::{StepperBus, StepperNcsPins, StepperSpiPins};

/// Gear ratio between the stepper motor shaft and the shaft actually being driven.
const GEAR_RATIO: f32 = 60.;
/// Inverts the position convention for the motors.
const INVERT_STEPPER_DIR: [bool; 4] = [true, true, true, true];
/// Inverts the position convention for the encoders.
const INVERT_ENCODER_DIR: [bool; 4] = [true, false, false, false];
/// What encoder reading we interpret as an angular position of 0 for ROS purposes.
// const ENCODER_ZERO: Radians = Radians(0.0);

type EnnPin = gpio::PD2<gpio::Output>;
type ClkPin = gpio::PA8<gpio::Analog>; // MCO
type DiagPin = gpio::PA10<gpio::Input>;

/// TMC5160 stepper driver (over SPI).
pub struct Drivebase {
    enn: EnnPin,
    _clk: rcc::Mco<gpio::PA8<gpio::AF0>>,
    #[allow(dead_code)]
    diag: DiagPin,
    steppers: StepperBus,
    //encoders: EncoderBus,
}

impl Drivebase {
    pub fn new(
        enn: EnnPin,
        clk_pin: ClkPin,
        diag: DiagPin,
        step_spi_pins: StepperSpiPins,
        step_spi_ncs: StepperNcsPins,
        //enc_spi_pins: EncoderSpiPins,
        //enc_spi_ncs: EncoderNcsPins,
        //spi2: pac::SPI2,
        spi3: pac::SPI3,
        rcc: &mut Rcc,
    ) -> Self {
        // initialise the MCO (CLK pin) at 12 MHz
        let clk = clk_pin.mco(rcc::MCOSrc::HSE, rcc::Prescaler::Div2, rcc);
        clk.enable();

        Drivebase {
            enn,
            _clk: clk,
            diag,
            steppers: Self::config_steppers(StepperBus::new(
                spi3,
                step_spi_pins,
                step_spi_ncs,
                rcc,
            )),
            //encoders: Self::config_encoders(EncoderBus::new(spi2, enc_spi_pins, enc_spi_ncs, rcc)),
        }
    }

    /// Initialises the TMC5160 steppers with a hard-coded but sensible configuration to run them
    /// in ramp mode with their on-chip motion profiling.
    fn config_steppers(mut steppers: StepperBus) -> StepperBus {
        for (&chan, invert) in ALL_CHANNELS.iter().zip(INVERT_STEPPER_DIR) {
            steppers
                .write_reg(
                    chan,
                    GStat::new()
                        .with_reset(true)
                        .with_drv_err(true)
                        .with_uv_cp(true),
                )
                .unwrap();

            steppers
                .write_reg(
                    chan,
                    GConf::new()
                        .with_faststandstill(true)
                        .with_en_pwm_mode(false)
                        .with_shaft(invert),
                )
                .unwrap();

            steppers
                .write_reg(
                    chan,
                    ChopConf::new()
                        .with_toff(5)
                        .with_hstrt(3)
                        .with_hend(1)
                        .with_tbl(1)
                        .with_tpdf(3),
                )
                .unwrap();

            steppers
                .write_reg(
                    chan,
                    PwmConf::new()
                        .with_pwm_ofs(30)
                        .with_pwm_grad(1)
                        .with_pwm_freq(1)
                        .with_pwm_autoscale(true)
                        .with_pwm_autograd(true)
                        .with_freewheel(0)
                        .with_pwm_reg(3)
                        .with_pwm_lim(12),
                )
                .unwrap();

            steppers.write_reg(chan, GlobalScalar(64.ul())).unwrap();

            steppers
                .write_reg(
                    chan,
                    IHoldIRun::new()
                        .with_ihold(10)
                        .with_irun(31)
                        .with_ihold_delay(6),
                )
                .unwrap();

            steppers.write_reg(chan, TPowerDown(0.ul())).unwrap();
            // 50ms
            steppers.write_reg(chan, TZeroWait(1172.ul())).unwrap();

            steppers.write_reg(chan, TPwmThrs(20.0.rps())).unwrap();
            steppers.write_reg(chan, RampMode(3.ul())).unwrap();
            steppers.write_reg(chan, XActual(0.0.rads())).unwrap();
            steppers.write_reg(chan, XTarget(0.0.rads())).unwrap();
            steppers.write_reg(chan, RampMode(0.ul())).unwrap();

            steppers.write_reg(chan, A1(50.0.rps2())).unwrap();
            steppers.write_reg(chan, V1(50.0.rps())).unwrap();
            steppers.write_reg(chan, AMax(30.0.rps2())).unwrap();
            steppers.write_reg(chan, VMax(50.0.rps())).unwrap();
            steppers.write_reg(chan, DMax(40.0.rps2())).unwrap();
            steppers.write_reg(chan, D1(60.0.rps2())).unwrap();
            steppers.write_reg(chan, VStart(3.0.rps())).unwrap();
            steppers.write_reg(chan, VStop(5.0.rps())).unwrap();
        }

        steppers
    }

    /* TODO: This currently hangs on write_reg().
    pub fn config_encoders(mut encoders: EncoderBus) -> EncoderBus {
        return encoders;

        for (&chan, invert) in ALL_CHANNELS.iter().zip(INVERT_ENCODER_DIR) {
            encoders
                .write_reg(chan, Settings1::new().with_dir(invert))
                .unwrap();
            encoders
                .write_reg(chan, Settings2::new().with_hys(0))
                .unwrap();
        }

        encoders
    }
    */

    /// Enables all drivebase channels. Must be called before any actuator commands (e.g.
    /// `set_position()`) in order to actually cause the motors to move. See also `disable_all()`.
    pub fn enable_all(&mut self) {
        self.enn.set_low();
    }

    /// Disables all drivebase channels. Actuator commands (e.g. `set_position()`) will have no
    /// effect until a call to `enable_all()`.
    #[allow(unused)]
    pub fn disable_all(&mut self) {
        self.enn.set_high();
    }

    pub fn set_position(&mut self, channel: Channel, target: TmcPosition) -> Result<(), !> {
        self.steppers
            .write_reg(channel, XTarget(target * GEAR_RATIO))
            .unwrap();
        Ok(())
    }

    /// Returns true if the motor is currently actuating, which is determined by comparing the
    /// TMC5160's ramp profiler target position with its current position.
    #[allow(unused)]
    pub fn is_busy(&mut self, channel: Channel) -> Result<bool, !> {
        Ok(self.steppers.read_reg::<XTarget>(channel).unwrap().1 .0
            != self.steppers.read_reg::<XActual>(channel).unwrap().1 .0)
    }
}
