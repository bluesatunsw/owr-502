#![no_std]
#![no_main]
use cortex_m_rt::entry;

use panic_semihosting as _;

use cortex_m_semihosting::hprintln;
use stm32g4::stm32g474::interrupt;

#[unsafe(link_section = ".int")]
fn foo() {
    hprintln!("Hello, world!");
}

#[entry]
fn main() -> ! {
    hprintln!("Nihao!");
    loop {
        foo();
    }
}
