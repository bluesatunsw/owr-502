use embedded_common::{
    stepper_bus::{Channel, ClkPin, DiagPin, EnnPin, StepperBus, StepperNcsPins, StepperSpiPins},
    tmc_registers::{
        AMax, ChopConf, DMax, GConf, GStat, GlobalScalar, IHoldIRun, PwmConf, RampMode, TPowerDown,
        TPwmThrs, TZeroWait, UnitsExt, VMax, VStart, VStop, XActual, XTarget, A1, D1, V1,
    },
};
use stm32g4xx_hal::{pac, rcc::Rcc};

pub struct Motion {
    pub steppers: StepperBus,
}

impl Motion {
    const CHANNEL_U: Channel = Channel::CH0;
    const CHANNEL_V: Channel = Channel::CH1;
    const CHANNEL_C: Channel = Channel::CH2;

    pub fn new(
        step_spi_pins: StepperSpiPins,
        step_spi_ncs: StepperNcsPins,
        clk_pin: ClkPin,
        enn: EnnPin,
        diag: DiagPin,
        spi3: pac::SPI3,
        rcc: &mut Rcc,
    ) -> Self {
        Motion {
            steppers: Self::config_steppers(StepperBus::new(
                spi3,
                step_spi_pins,
                step_spi_ncs,
                clk_pin,
                enn,
                diag,
                rcc,
            )),
        }
    }

    /// Initialises the TMC5160 steppers with a hard-coded but sensible configuration to run them
    /// in ramp mode with their on-chip motion profiling.
    fn config_steppers(mut steppers: StepperBus) -> StepperBus {
        for chan in [Self::CHANNEL_U, Self::CHANNEL_V] {
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
}
