use embedded_common::{
    stepper_bus::{Channel, ClkPin, DiagPin, EnnPin, StepperBus, StepperNcsPins, StepperSpiPins},
    tmc_registers::{
        ChopConf, GConf, GStat, GlobalScalar, IHoldIRun, PwmConf, TPowerDown, TZeroWait,
        UnitlessExt, XTargetDirect,
    },
};
use stm32g4xx_hal::{pac, rcc::Rcc};

const MAX_DUTY: i16 = 255;
// default is bucket on A, pivot on B; setting this to true inverts this
const TMC_CH_SWAP_BUCKET_PIVOT: bool = true;
// default is paver on B; setting this to true puts it on A
const TMC_CH_SWAP_PAVER: bool = true;

pub struct Motion {
    pub steppers: StepperBus,

    pivot_duty: i16,
    bucket_duty: i16,
}

impl Motion {
    pub const CHANNEL_SHOVEL: Channel = Channel::CH2;
    pub const CHANNEL_PAVER: Channel = Channel::CH3;

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

            pivot_duty: 0,
            bucket_duty: 0,
        }
    }

    /// Initialises the TMC5160 steppers to run in a direct PWM control mode
    fn config_steppers(mut steppers: StepperBus) -> StepperBus {
        for chan in [Self::CHANNEL_SHOVEL, Self::CHANNEL_PAVER] {
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
                        .with_en_pwm_mode(true)
                        .with_shaft(false)
                        .with_direct_mode(true),
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
                        .with_pwm_autoscale(false)
                        .with_pwm_ofs(255)
                        .with_pwm_grad(4),
                )
                .unwrap();

            steppers.write_reg(chan, TPowerDown(0.ul())).unwrap();
            // 50ms
            steppers.write_reg(chan, TZeroWait(1172.ul())).unwrap();

            steppers.write_reg(chan, GlobalScalar(64.ul())).unwrap();
        }

        steppers
            .write_reg(
                Self::CHANNEL_SHOVEL,
                IHoldIRun::new()
                    .with_ihold(15)
                    .with_irun(15)
                    .with_ihold_delay(6),
            )
            .unwrap();

        steppers
            .write_reg(
                Self::CHANNEL_PAVER,
                IHoldIRun::new()
                    .with_ihold(7)
                    .with_irun(7)
                    .with_ihold_delay(6),
            )
            .unwrap();

        steppers
    }

    // Other functions should call this function since it handles the INVERT constant correctly
    fn refresh_pivot_and_bucket(&mut self) -> Result<(), ()> {
        self.steppers
            .write_reg(
                Self::CHANNEL_SHOVEL,
                if TMC_CH_SWAP_BUCKET_PIVOT {
                    XTargetDirect::new()
                        .with_a(self.pivot_duty)
                        .with_b(self.bucket_duty)
                } else {
                    XTargetDirect::new()
                        .with_a(self.bucket_duty)
                        .with_b(self.pivot_duty)
                },
            )
            .unwrap();
        Ok(())
    }

    pub fn set_pivot(&mut self, duty: f32) -> Result<(), ()> {
        self.pivot_duty = ((duty * MAX_DUTY as f32) as i16).clamp(-MAX_DUTY, MAX_DUTY);
        return self.refresh_pivot_and_bucket();
    }

    pub fn set_bucket(&mut self, duty: f32) -> Result<(), ()> {
        self.bucket_duty = ((duty * MAX_DUTY as f32) as i16).clamp(-MAX_DUTY, MAX_DUTY);
        return self.refresh_pivot_and_bucket();
    }

    pub fn set_paver(&mut self, duty: f32) -> Result<(), ()> {
        self.steppers
            .write_reg(
                Self::CHANNEL_PAVER,
                if TMC_CH_SWAP_PAVER {
                    XTargetDirect::new()
                        .with_a(((duty * MAX_DUTY as f32) as i16).clamp(-MAX_DUTY, MAX_DUTY))
                        .with_b(0)
                } else {
                    XTargetDirect::new()
                        .with_a(0)
                        .with_b(((duty * MAX_DUTY as f32) as i16).clamp(-MAX_DUTY, MAX_DUTY))
                },
            )
            .unwrap();
        Ok(())
    }
}
