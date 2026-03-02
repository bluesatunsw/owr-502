use core::{mem::transmute_copy, ptr::copy_nonoverlapping, slice};

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

pub fn itm_send_bytes(chan: usize, bytes: &[u8]) {
    if cfg!(not(feature = "dprintln-enabled")) {
        return;
    }

    let stim = unsafe { &mut stm32g4::stm32g474::CorePeripherals::steal().ITM.stim[chan] };

    let chunked = bytes.iter().array_chunks::<4>();
    chunked.clone().for_each(|chunk| {
        while !stim.is_fifo_ready() {}
        stim.write_u32(unsafe { transmute_copy(chunk.map(|&x| x).as_array::<4>().unwrap()) });
    });

    let final_chunk = chunked.into_remainder().array_chunks::<2>();
    final_chunk.clone().for_each(|chunk| {
        while !stim.is_fifo_ready() {}
        stim.write_u16(unsafe { transmute_copy(chunk.map(|&x| x).as_array::<2>().unwrap()) })
    });

    final_chunk.into_remainder().for_each(|&b| {
        while !stim.is_fifo_ready() {}
        stim.write_u8(b);
    });
}

pub fn itm_send_raw<T: Sized>(chan: usize, value: &T) {
    itm_send_bytes(chan, unsafe {
        slice::from_raw_parts((value as *const T) as *const u8, size_of::<T>())
    });
}

pub fn itm_hexdump(chan: usize, bytes: &[u8]) {
    itm_send_raw::<u32>(chan, &(bytes.len() as u32));
    itm_send_bytes(chan, bytes);
}

#[cfg(feature = "dprintln-enabled")]
#[macro_export]
macro_rules! dprint {
    ($channel:literal, $s:expr) => {
        let stim = unsafe { &mut stm32g4::stm32g474::CorePeripherals::steal().ITM.stim[$channel] };
        cortex_m::iprint!(stim, $s);
    };
    ($channel:literal, $($arg:tt)*) => {
        let stim = unsafe { &mut stm32g4::stm32g474::CorePeripherals::steal().ITM.stim[$channel] };
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
        let stim = unsafe { &mut stm32g4::stm32g474::CorePeripherals::steal().ITM.stim[$channel] };
        cortex_m::iprintln!(stim);
    };
    ($channel:literal, $fmt:expr) => {
        let stim = unsafe { &mut stm32g4::stm32g474::CorePeripherals::steal().ITM.stim[$channel] };
        cortex_m::iprintln!(stim, $fmt);
    };
    ($channel:literal, $fmt:expr, $($arg:tt)*) => {
        let stim = unsafe { &mut stm32g4::stm32g474::CorePeripherals::steal().ITM.stim[$channel] };
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

pub fn uuid() -> [u8; 16] {
    const UID_ADDRESS: u32 = 0x1FFF_7590;

    let mut uuid: [u8; 16] = [0x4C; 16];
    // SAFETY: 😊
    unsafe {
        copy_nonoverlapping(UID_ADDRESS as *const u8, uuid.as_mut_ptr(), 12);
    }

    uuid
}
