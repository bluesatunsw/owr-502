use core::ptr::copy_nonoverlapping;

use stm32g4::stm32g474::{DBGMCU, DCB, DWT, ITM};

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

#[cfg(debug_assertions)]
#[macro_export]
macro_rules! dprint {
    ($channel:literal, $s:expr) => {
        let stim = unsafe { &mut stm32g4::stm32g474::CorePeripherals::steal().ITM.stim[$channel] };
        cortex_m::iprint!(stim, $s);
    };
    ($channel:literal, $($arg:tt)*) => {
        let stim = unsafe { &mut stm32g4::stm32g474::CorePeripherals::steal().ITM.stim[$channel] };
        cortex_m::iprint!(stim, format_args!($($arg)*));
    };
}

#[cfg(not(debug_assertions))]
#[macro_export]
macro_rules! dprint {
    ($channel:literal, $s:expr) => {};
    ($channel:literal, $($arg:tt)*) => {};
}

#[cfg(debug_assertions)]
#[macro_export]
macro_rules! dprintln {
    ($channel:literal) => {
        let stim = unsafe { &mut stm32g4::stm32g474::CorePeripherals::steal().ITM.stim[$channel] };
        cortex_m::iprintln!(stim, "\n");
    };
    ($channel:literal, $fmt:expr) => {
        let stim = unsafe { &mut stm32g4::stm32g474::CorePeripherals::steal().ITM.stim[$channel] };
        cortex_m::iprintln!(stim, concat!($fmt, "\n"));
    };
    ($channel:literal, $fmt:expr, $($arg:tt)*) => {
        let stim = unsafe { &mut stm32g4::stm32g474::CorePeripherals::steal().ITM.stim[$channel] };
        cortex_m::iprintln!(stim, format_args!(concat!($fmt, "\n"), $($arg)*));
    };
}

#[cfg(not(debug_assertions))]
#[macro_export]
macro_rules! dprintln {
    ($channel:literal) => {};
    ($channel:literal, $fmt:expr) => {};
    ($channel:literal, $fmt:expr, $($arg:tt)*) => {};
}

pub fn uuid() -> [u8; 16] {
    const UID_ADDRESS: u32 = 0x1FFF_7590;

    let mut uuid: [u8; 16] = [0x4C; 16];
    // SAFETY: 😊
    unsafe {
        copy_nonoverlapping(UID_ADDRESS as *const u8, uuid.as_mut_ptr(), 12);
    }

    uuid
}
