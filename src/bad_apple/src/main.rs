#![no_std]
#![no_main]
#![feature(breakpoint)]
use core::arch::breakpoint;

use cortex_m_rt::entry;

use embedded_alloc::LlffHeap;
use embedded_common::{clock::conjure_rcc, debug::setup_itm, dprintln};
use panic_semihosting as _;
use stm32g4::stm32g474::{CorePeripherals, Peripherals};

#[unsafe(link_section = ".int")]
fn foo() {
    dprintln!(0, "你好");
}

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

fn initialise_allocator() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 0x4000;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
}

// /home/evan/bluesat/owr-502/src/bad_apple/target/thumbv7em-none-eabihf/debug/bad_apple
#[entry]
fn main() -> ! {
    let rcc = unsafe { conjure_rcc() };
    let mut dp = unsafe { Peripherals::steal() };
    let mut cp = unsafe { CorePeripherals::steal() };
    unsafe {
        setup_itm(&mut cp.DCB, &mut cp.DWT, &mut dp.DBGMCU, &mut cp.ITM);
    }

    initialise_allocator();
    dprintln!(0, "Hello World!!!!");

    loop {
        foo();
    }
}
