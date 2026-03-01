use fixed::types::I16F16;
use stm32f4xx_hal::pac::{DBGMCU, DCB, DWT, ITM, TIM1};

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

pub unsafe fn setup_itm(dcb: &mut DCB, dwt: &mut DWT, dbgmcu: &mut DBGMCU, itm: &mut ITM) {
    dcb.enable_trace();
    dwt.enable_cycle_counter();
    unsafe {
        dbgmcu
            .cr()
            .modify(|_, w| w.trace_ioen().set_bit().trace_mode().bits(0b00));
        itm.lar.write(0xC5ACCE55); // Unlock ITM registers
        itm.tcr.write(0x00010005); // AdvTBus ID = 1, Global ITM enable

        // Enable all
        itm.tpr.write(0b1111); // Enables STIM 0 to 3
        itm.ter[0].write(0xFFFF_FFFF); // Enables STIM[31:0]
    }
}

#[cfg(feature = "dprintln-enabled")]
#[macro_export]
macro_rules! dprint {
    ($channel:literal, $s:expr) => {
        let stim = unsafe { &mut stm32f4xx_hal::pac::CorePeripherals::steal().ITM.stim[$channel] };
        cortex_m::iprint!(stim, $s);
    };
    ($channel:literal, $($arg:tt)*) => {
        let stim = unsafe { &mut stm32f4xx_hal::pac::CorePeripherals::steal().ITM.stim[$channel] };
        cortex_m::iprint!(stim, $($arg)*);
    };
}

#[cfg(not(feature = "dprintln-enabled"))]
#[macro_export]
macro_rules! dprint {
    ($channel:literal, $s:expr) => {};
    ($channel:literal, $($arg:tt)*) => {};
}

#[cfg(feature = "dprintln-enabled")]
#[macro_export]
macro_rules! dprintln {
    ($channel:literal) => {
        let stim = unsafe { &mut stm32f4xx_hal::pac::CorePeripherals::steal().ITM.stim[$channel] };
        cortex_m::iprintln!(stim);
    };
    ($channel:literal, $fmt:expr) => {
        let stim = unsafe { &mut stm32f4xx_hal::pac::CorePeripherals::steal().ITM.stim[$channel] };
        cortex_m::iprintln!(stim, $fmt);
    };
    ($channel:literal, $fmt:expr, $($arg:tt)*) => {
        let stim = unsafe { &mut stm32f4xx_hal::pac::CorePeripherals::steal().ITM.stim[$channel] };
        cortex_m::iprintln!(stim, $fmt, $($arg)*);
    };
}

#[cfg(not(feature = "dprintln-enabled"))]
#[macro_export]
macro_rules! dprintln {
    ($channel:literal) => {};
    ($channel:literal, $fmt:expr) => {};
    ($channel:literal, $fmt:expr, $($arg:tt)*) => {};
}
