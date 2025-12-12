//! firmware for the power module on the rover
//!
//! TODO: Description of firmware 

#![no_std]
#![no_main]

use cortex_m_semihosting::{hprintln};
use panic_semihosting as _;

use cortex_m_rt::entry;

use stm32g4xx_hal::{
    prelude::*,
    gpio, 
    pwr::PwrExt, 
    rcc::*, 
    time::RateExtU32,
    pwm::PwmExt
};

use crate::boards::*;



// This below is used for aavin's strange print function
// use cortex_m_log::{destination::Itm, print, println};

/* NOTE: This code does not currently work when uncommented
// All for println omagawd
dp.DBGMCU.cr().write(|w| unsafe {
    w.trace_ioen().set_bit();   // Enable TPIU
    w.trace_mode().bits(00)     // Async (SWO) Mode
});

let core_dp = cortex_m::Peripherals::take().unwrap();
let mut log = Itm::new(core_dp.ITM);

println!(log, "Amogus");
*/


mod boards;

///////////
// Main. //
///////////

#[entry]
fn main() -> ! {
    // Embedded boilerplate...

    let (mut hled, mut pwr_channel_enable) = boards::init();
    

    hprintln!("Tesitng semihosting!");

    pwr_channel_enable.0.set_low();
    pwr_channel_enable.1.set_low();
    pwr_channel_enable.2.set_low();
    pwr_channel_enable.3.set_low();

    let led_colour_white = RGBLEDColor {
        red: 0x0F,
        green: 0x00,
        blue: 0x0F,
    };

    let led_colour_off = RGBLEDColor {
        red: 0x00,
        green: 0x00,
        blue: 0x00,
    };
    

    // let mut cycles = 0;
    loop {

        hled.set_nth_led(0, led_colour_white);
        hled.set_nth_led(1, led_colour_white);
        hled.set_nth_led(2, led_colour_white);
        hled.set_nth_led(3, led_colour_white);
        hled.set_nth_led(4, led_colour_white);
        hled.set_nth_led(5, led_colour_white);
        hled.render();

        hprintln!("W");
        hprintln!("A");
        hprintln!("I");
        hprintln!("T");
        hprintln!("\n");

        hled.set_nth_led(0, led_colour_off);
        hled.set_nth_led(1, led_colour_off);
        hled.set_nth_led(2, led_colour_off);
        hled.set_nth_led(3, led_colour_off);
        hled.set_nth_led(4, led_colour_off);
        hled.set_nth_led(5, led_colour_off);
        hled.render();

        hprintln!("W");
        hprintln!("A");
        hprintln!("I");
        hprintln!("T");
        hprintln!("\n");

        // cortex_m::asm::nop();

    }
}

