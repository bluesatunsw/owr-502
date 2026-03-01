use stm32f4xx_hal::pac::{DCB, DWT, DBGMCU, ITM};

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
