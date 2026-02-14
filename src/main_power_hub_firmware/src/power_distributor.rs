use cortex_m::Peripherals;
use stm32g4xx_hal::{gpio::AnyPin, rcc::Rcc};

// Pins for power distributor
pub struct PowerDistributor {
    cp_pwm: AnyPin<output>, // Clock output - generate PWM for charge pump
    channel_0_enable: AnyPin<output>,
    channel_1_enable: AnyPin<output>,
    channel_2_enable: AnyPin<output>,
    channel_3_enable: AnyPin<output>,
}

