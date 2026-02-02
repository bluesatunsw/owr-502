#![no_std]
#![no_main]

use core::iter::zip;

use cortex_m::asm::delay;
use embedded_common::argb::Colour;
use panic_semihosting as _;

use cortex_m_rt::entry;

use embedded_alloc::LlffHeap as Heap;

use crate::boards::NUM_CHANNELS;
extern crate alloc;
// Global allocator -- required by canadensis.
// -> transitively req'd by embedded_common
#[global_allocator]
static G_HEAP: Heap = Heap::empty();

mod boards;

///////////
// Main. //
///////////

#[entry]
fn main() -> ! {
    let (mut led_controller, pwr_chans) = boards::init();
    let mut led_buf: [Colour; NUM_CHANNELS + 2] = [Colour::AMBER; 6];
    led_controller.display(&led_buf);

    delay(50_000_000);

    for (mut chan, led_idx) in zip(pwr_chans, [2, 3, 4, 5]) {
        chan.enable();
        led_buf[led_idx] = Colour { r: 0, b: 0, g: 255 };
    }

    led_controller.display(&led_buf);
    loop {
        //let sample = adc_controller.adc4.convert(&vsense_pin, SampleTime::Cycles_640_5);
        //let millivolts = adc_controller.adc4.sample_to_millivolts(sample);
    }
}
