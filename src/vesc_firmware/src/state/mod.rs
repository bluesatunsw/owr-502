use core::mem::MaybeUninit;

use stm32f4xx_hal::gpio::{AnyPin, Input};

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
    pub halls: MaybeUninit<(AnyPin<Input>, AnyPin<Input>, AnyPin<Input>)>,
    pub pwm_driver: MaybeUninit<STM32F4xxSixPwmDriver>,
}
