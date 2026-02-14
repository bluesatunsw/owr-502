#![no_std]
#![no_main]

mod setup;

use crate::setup::{NUM_CHANNELS, NUM_STATUS_LIGHTS, Resources, init};
use core::panic::PanicInfo;
use cortex_m_rt::entry;
use embedded_alloc::LlffHeap as Heap;
use embedded_common::argb::Colour;

#[global_allocator]
static G_HEAP: Heap = Heap::empty();

#[entry]
fn main() -> ! {
    let dp = stm32g4xx_hal::stm32::Peripherals::take().unwrap();
    let Resources {
        mut argb_controller,
        mut power_channels,
    } = init(dp);

    // Default LED colours
    let mut led_buf: [Colour; NUM_CHANNELS + NUM_STATUS_LIGHTS] =
        [Colour::AMBER; NUM_CHANNELS + NUM_STATUS_LIGHTS];
    argb_controller.display(&led_buf);

    loop {}
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
