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

// cargo run --manifest-path ../wzrd_pack/Cargo.toml --target host-tuple -- -i target/thumbv7em-none-eabihf/release/bad_apple -o bin.wzrd
#[entry]
fn main() -> ! {
    hprintln!("Nihao!");
    loop {
        foo();
    }
}
