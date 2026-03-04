use core::mem::MaybeUninit;

use stm32f4xx_hal::gpio::{Alternate, AnyPin};

use crate::bsp::six_pwm::STM32F4xxSixPwmDriver;

pub struct AppState {
    pub motioncontrol: MotionControlState,
    pub commutation: CommutationState,
}

pub enum MotionControlState {
    Position,
    Velocity,
    None,
}

pub enum CommutationMode {
    /* Current {
        current: I16F16,
    }, TODO: After Jonah implements current sensing (assuming we do the transform) */
    Voltage { voltage: f32 }, // Testing -> Hall effects + FIXME: Vbus sense
    Disabled,
}

pub struct CommutationState {
    pub mode: CommutationMode,
    pub halls: MaybeUninit<(
        AnyPin<Alternate<2>>,
        AnyPin<Alternate<2>>,
        AnyPin<Alternate<2>>,
    )>,
    pub pwm_driver: MaybeUninit<STM32F4xxSixPwmDriver>,
}
