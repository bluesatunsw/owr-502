use stm32f4xx_hal::pac::TIM1;

pub mod debug;
pub mod foc;

// Freewheeling, TIM1.BDTR.Main Output Disable!
pub fn motor_disable() {
    unsafe {
        TIM1::steal()
            .bdtr()
            .modify(|_, w| w.aoe().manual().moe().disabled_idle());
    }
}

// TIM1.BDTR.Main Output Enable!
pub fn motor_enable() {
    unsafe {
        TIM1::steal().bdtr().modify(|_, w| w.moe().enabled());
    }
}
