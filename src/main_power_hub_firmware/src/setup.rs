use embedded_common::argb;

use stm32g4xx_hal::{
    gpio::{self, GpioExt},
    pac::USART1,
    prelude::SetDutyCycle,
    pwm::PwmExt,
    pwr::{PwrExt, VoltageScale},
    rcc::{
        Config, FdCanClockSource, PllConfig, PllMDiv, PllNMul, PllPDiv, PllQDiv, PllRDiv, PllSrc,
        RccExt,
    },
    time::RateExtU32,
    stm32::Peripherals
};

pub const NUM_CHANNELS: usize = 4;
pub const NUM_STATUS_LIGHTS: usize = 2;

pub struct Resources {
    pub argb_controller: argb::Controller<USART1, gpio::PA9<gpio::Alternate<7>>>,
    pub power_channels: [PowerChannel; NUM_CHANNELS],
}

pub struct PowerChannel {
    enable: gpio::AnyPin<gpio::Output>,
}

impl PowerChannel {
    // Active-Low
    pub fn enable(&mut self) {
        self.enable.set_low();
    }

    // Inactive-High
    pub fn disable(&mut self) {
        self.enable.set_high();
    }
}

pub fn init(dp: Peripherals) -> Resources {
    // Embedded boilerplate

    // Set up power control peripheral
    // Range1 with boost: maximum possible clock speeds
    let pwr = dp
        .PWR
        .constrain()
        .vos(VoltageScale::Range1 { enable_boost: true })
        .freeze();

    // Set up clocks
    // See clock tree in stm32g4 docs (p. 275)
    let mut rcc = dp.RCC.freeze(
        Config::pll()
            .pll_cfg(PllConfig {
                mux: PllSrc::HSE(24_u32.MHz()),
                m: PllMDiv::DIV_2,
                n: PllNMul::MUL_28,
                r: Some(PllRDiv::DIV_2), // Why limit ourselves @Jonah? :p
                q: Some(PllQDiv::DIV_2),
                p: Some(PllPDiv::DIV_8), // Has to be under 60MHz for it to work i think
            })
            .fdcan_src(FdCanClockSource::PLLQ),
        pwr,
    );

    // Pins
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);

    // ARGB Controller
    let argb_controller =
        argb::Controller::new(dp.USART1, gpioa.pa9.into_alternate().into(), 32, &mut rcc);

    // Configure charge pump PWM source
    // Set and forget - we never actually want to turn this off
    let cp_pwm_pin: gpio::PB7<gpio::AF10> = gpiob.pb7.into_alternate();
    let mut pwm = dp.TIM3.pwm(cp_pwm_pin, 100_u32.kHz(), &mut rcc);
    let _ = pwm.set_duty_cycle_percent(5);
    pwm.enable();

    // Configure channel enable outputs
    let chan_0_enable = PowerChannel {
        enable: gpioc.pc9.into_push_pull_output().into(),
    };

    let chan_1_enable = PowerChannel {
        enable: gpioc.pc8.into_push_pull_output().into(),
    };
    let chan_2_enable = PowerChannel {
        enable: gpioc.pc7.into_push_pull_output().into(),
    };
    let chan_3_enable = PowerChannel {
        enable: gpioc.pc6.into_push_pull_output().into(),
    };

    Resources {
        argb_controller: argb_controller,
        power_channels: [chan_0_enable, chan_1_enable, chan_2_enable, chan_3_enable],
    }
}
