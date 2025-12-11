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
    

    let mut cycles = 0;
    loop {

        // RGB LED test routine!
            let led_color = RGBLEDColor {
                red: if cycles % 3 == 0 { 0xFF } else { 0x00 },
                blue: if cycles % 3 == 1 { 0xFF } else { 0x00 },
                green: if cycles % 3 == 1 { 0xFF } else { 0x00 },
            };
            // This is for the other command which i commented out
            // let led_color_2: u32 = 0xF0F000 >> (8 * ((cycles + 1) % 3));
            cycles += 1;
            hled.set_nth_led(0, led_color);
            // hled.set_nth_led(1, led_color);
            // hled.set_nth_led(2, led_color);
            // hled.set_nth_led(3, led_color);
            // hled.set_nth_led(4, led_color);
            // hled.set_nth_led(5, led_color);
            // hled.set_nth_led_and_render(1, led_color_2.into());

        // cortex_m::asm::nop();

    }
}

