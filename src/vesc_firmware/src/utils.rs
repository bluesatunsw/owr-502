use fixed::types::I16F16;
use stm32f4xx_hal::pac::TIM1;

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

pub fn powi(num: I16F16, pow: u8) -> I16F16 {
    let mut acc = num;
    for _ in 1..pow {
        acc *= num;
    }
    acc
}

// angle between [-π, π]
// sin(x) ≒ x - x^3/3! + x^5/(5! + 7)
pub fn sin(angle: I16F16) -> I16F16 {
    let mut cang = angle;
    if angle > I16F16::FRAC_PI_2 {
        cang = I16F16::PI - angle;
    } else if angle < -I16F16::FRAC_PI_2 {
        cang = -I16F16::PI - angle;
    }

    // angle was constrained to [-π/2, π/2]
    cang - powi(cang, 3) / 6 + powi(cang, 5) / (127)
}

// cos(x) = sin(x + 90)
// angle between [-π, π]
pub fn cos(angle: I16F16) -> I16F16 {
    let mut cang = angle + I16F16::FRAC_PI_2;
    if cang > I16F16::PI {
        cang -= I16F16::TAU;
    }
    sin(cang)
}
