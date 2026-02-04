//! Implementations to do with the stepper motor drivers (and related parts) themselves.
//! This is actually three peripherals in a trenchcoat:
//! - ADC2 (for reading the voltages off the thermistors for temperature estimation),
//! - SPI3 (for communicating with the TMC5160/A stepper drivers), and
//! - SPI1 (for communicating with the off-board AS5047P encoders).
use core::intrinsics::breakpoint;

use bitfield_struct::bitfield;
use cortex_m::asm::delay;
use stm32g4xx_hal as hal;

use cortex_m_semihosting::hprintln;
use libm::{self, Libm};

use hal::{
    adc::{
        config::{AdcConfig, ClockMode, ClockSource, Prescaler, SampleTime},
        Adc, AdcClaim, AdcCommonExt, Configured,
    },
    gpio,
    pac,
    prelude::*,
    rcc::{self, Rcc},
    spi,
    time::RateExtU32, // for auto .MHz()
};

use crate::stm32g4xx::{
    as_registers::{Settings1, Settings2},
    encoder_bus::{EncoderBus, EncoderNcsPins, EncoderSpiPins},
    tmc_registers::{
        AMax, ChopConf, DMax, GConf, GStat, GlobalScalar, IHoldIRun, PwmConf, RampMode, TPowerDown,
        TPwmThrs, TZeroWait, TmcPosition, UnitlessExt, UnitsExt, VMax, VStart, VStop, XActual,
        XTarget, A1, D1, V1,
    },
    Channel, STM32G4xxGeneralClock, ALL_CHANNELS,
};
use crate::{
    boards::Celsius,
    stm32g4xx::stepper_bus::{StepperBus, StepperNcsPins, StepperSpiPins},
};

/// Gear ratio between the stepper motor shaft and the shaft actually being driven.
const GEAR_RATIO: f32 = 60.;
/// The number of stepper motors that the board is actually controlling, to avoid dealing with
/// motors that don't exist.
pub const NUM_STEPPERS: usize = 4;
/// Inverts the position convention for the motors.
const INVERT_STEPPER_DIR: [bool; 4] = [false, false, false, false];
/// Inverts the position convention for the encoders.
const INVERT_ENCODER_DIR: [bool; 4] = [true, false, false, false];
/// What encoder reading we interpret as an angular position of 0 for ROS purposes.
// const ENCODER_ZERO: Radians = Radians(0.0);

type EnnPin = gpio::PD2<gpio::Output>;
type ClkPin = gpio::PA8<gpio::Analog>; // MCO

type DiagPin = gpio::PA10<gpio::Input>;

pub struct StepperTempPins(pub gpio::PA0, pub gpio::PA1, pub gpio::PB2, pub gpio::PC5);

// the below two enums are just useful to have during development for reference/debugging
#[allow(non_camel_case_types, dead_code)]
enum EncoderRegister {
    ERRFL = 0x0001,
    PROG = 0x0003,
    ZPOSM = 0x0016,
    ZPOSL = 0x0017,
    SETTINGS1 = 0x0018,
    SETTINGS2 = 0x0019,
    DIAAGC = 0x3FFC,
    MAG = 0x3FFD,
    ANGLEUNC = 0x3FFE,
    ANGLECOM = 0x3FFF,
}

/// TMC5160 stepper driver (over SPI).
pub struct STM32G4xxStepperDriver {
    adc: Adc<pac::ADC2, Configured>,
    enn: EnnPin,
    _clk: rcc::Mco<gpio::PA8<gpio::AF0>>,
    #[allow(dead_code)]
    diag: DiagPin,
    temp: StepperTempPins,
    steppers: StepperBus,
    encoders: EncoderBus,
}

impl STM32G4xxStepperDriver {
    pub fn new(
        adc_common: pac::ADC12_COMMON,
        adc2: pac::ADC2,
        spi2: pac::SPI2,
        spi3: pac::SPI3,
        uclock: &mut STM32G4xxGeneralClock,
        rcc: &mut Rcc,
        temp: StepperTempPins,
        enn: EnnPin,
        clk_pin: ClkPin,
        diag: DiagPin, // can be made mut
        step_spi_pins: StepperSpiPins,
        step_spi_ncs: StepperNcsPins,
        enc_spi_pins: EncoderSpiPins,
        enc_spi_ncs: EncoderNcsPins,
    ) -> Self {
        // initialise the MCO (CLK pin)
        // at 12 MHz
        let clk = clk_pin.mco(rcc::MCOSrc::HSE, rcc::Prescaler::Div2, rcc);
        // Pinout is wrong on Rev. A2 (DIAG and CLK swapped),
        // so just use the TMC5160's internal clock.
        clk.enable();

        STM32G4xxStepperDriver {
            adc: adc_common
                .claim(
                    ClockMode::AdcKerCk {
                        // system clock is at 64 MHz, but abs max ADC clock frequency is 60 MHz, so /2
                        prescaler: Prescaler::Div_2,
                        src: ClockSource::SystemClock,
                    },
                    rcc,
                )
                .claim_and_configure(adc2, AdcConfig::default(), uclock),
            enn,
            _clk: clk,
            diag,
            temp,
            steppers: Self::config_stepeprs(StepperBus::new(spi3, step_spi_pins, step_spi_ncs, rcc)),
            encoders: Self::config_encoders(EncoderBus::new(spi2, enc_spi_pins, enc_spi_ncs, rcc)),
        }
    }

    pub fn config_stepeprs(mut steppers: StepperBus) -> StepperBus {
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
            steppers.write_reg(chan, VMax(100.0.rps())).unwrap();
            steppers.write_reg(chan, DMax(40.0.rps2())).unwrap();
            steppers.write_reg(chan, D1(60.0.rps2())).unwrap();
            steppers.write_reg(chan, VStart(3.0.rps())).unwrap();
            steppers.write_reg(chan, VStop(5.0.rps())).unwrap();
        }

        steppers
    }

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

    pub fn enable_all(&mut self) {
        self.enn.set_low();
    }

    fn disable_all(&mut self) {
        self.enn.set_high();
    }

    pub fn set_position(&mut self, channel: Channel, target: TmcPosition) -> Result<(), !> {
        // hprintln!("Setting {:?} to {:?}", channel, target);
        self.steppers.write_reg(channel, XTarget(target * GEAR_RATIO)).unwrap();
        Ok(())
    }

    // TODO: this yields very incorrect values, debugg
    pub fn get_temperature(&mut self, channel: Channel) -> Celsius {
        /*
        // Voltage divider: 3.3V VDD (= VDDA), 10k thermistor (XH103) = R(T), 2.2k resistor
        // ADC reading / 2^16 = 2.2k / (R(T) + 2.2k)
        // resistance of thermistor at temperature T, R(T) = R(T_0) * exp(B * (1/T − 1/T_0) )
        // T = absolute temperature, B = B-constant, T_0 = 273.15 + 25 K, R(T_0) = 10 kOhm
        const CELSIUS_OFFSET: f32 = 273.15;
        const T_0_RECIPROCAL: f32 = 1.0 / (CELSIUS_OFFSET + 25.0);
        // using the (25/85 deg C) B-constant
        const B_RECIPROCAL: f32 = 1.0 / 3434.0;
        const R_T_0: f32 = 10_000.0;
        const R_FIXED: f32 = 2200.0;
        let adc_reading = match channel {
            StepperChannel::Channel0 => self.adc.convert(&self.temp.3, SampleTime::Cycles_640_5),
            StepperChannel::Channel1 => self.adc.convert(&self.temp.0, SampleTime::Cycles_640_5),
            StepperChannel::Channel2 => self.adc.convert(&self.temp.1, SampleTime::Cycles_640_5),
            StepperChannel::Channel3 => self.adc.convert(&self.temp.2, SampleTime::Cycles_640_5),
        };
        let reading_proportion_reciprocal: f32 = 65536.0 / (adc_reading as f32);
        let t_reciprocal = B_RECIPROCAL
            * Libm::<f32>::log(R_FIXED * (reading_proportion_reciprocal - 1.0) / R_T_0)
            + T_0_RECIPROCAL;
        Celsius(1.0 / t_reciprocal - CELSIUS_OFFSET)
         */
        todo!()
    }

    fn adjust(&mut self, channel: Channel) -> Result<(), !> {
        /*
        // how many steps XACTUAL must be out of place for a correction to take place
        const MIN_MICROSTEP_DIFFERENCE: i32 = 4;
        let xtarget = self.read_reg(channel, StepperRegister::XTARGET)?;
        let xactual = self.read_reg(channel, StepperRegister::XACTUAL)?;
        if xtarget.0 != xactual.0 {
            // the motor is still rotating to carry out a command
            return Ok(());
        }
        // Read position from encoder
        let raw_pos: f32 = self.read_enc_reg(channel, EncoderRegister::ANGLECOM)? as f32;
        let mut actual_pos = Radians(raw_pos * 2.0 * PI / (0x4000 as f32)) - ENCODER_ZERO;
        if actual_pos.0 > 2.0 * PI {
            actual_pos = Radians(actual_pos.0 - 2.0 * PI);
        }
        let expected_xactual: i32 =
            (actual_pos.0 * (MICROSTEPS_PER_REV as f32) * (GEAR_RATIO as f32) / (2.0 * PI)) as i32;
        let difference_usteps: i32 = expected_xactual - (xactual.0 as i32);
        if difference_usteps > MIN_MICROSTEP_DIFFERENCE
            || difference_usteps < -MIN_MICROSTEP_DIFFERENCE
        {
            self.write_reg(channel, StepperRegister::XACTUAL, expected_xactual as u32)
                .unwrap();
            hprintln!(
                "Corrected XACTUAL from {} to {}",
                xactual.0,
                expected_xactual
            );
        }
        Ok(())
        */
        todo!()
    }

    fn is_busy(&mut self, channel: Channel) -> Result<bool, !> {
        Ok(self.steppers.read_reg::<XTarget>(channel).unwrap().1 .0
            != self.steppers.read_reg::<XActual>(channel).unwrap().1 .0)
    }
}
