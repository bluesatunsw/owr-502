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

mod boards;

///////////
// Main. //
///////////

#[entry]
fn main() -> ! {
    // Embedded boilerplate...
    let dp = stm32g4xx_hal::stm32::Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain().freeze();

    let mut rcc = dp.RCC.freeze(
        // enable HSE @ 24 MHz (stepper board)
        Config::pll()
            .pll_cfg(PllConfig {
                mux: PllSrc::HSE(24.MHz()),
                m: PllMDiv::DIV_3,
                n: PllNMul::MUL_32,
                r: Some(PllRDiv::DIV_2), // Why limit ourselves @Jonah? :p
                q: Some(PllQDiv::DIV_2),
                p: None,
            })
            .fdcan_src(FdCanClockSource::PLLQ),
        pwr
    );

    
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

    hprintln!("Tesitng semihosting!");

    // Set up pins.
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);

                                                // The turbofish can be removed later just here to
    // ARGB LED SETUP                           // remove the error
    let led_tx_pin = gpiob.pb9.into_alternate::<7>();
    let usart3 = dp.USART3;
    let mut hled = STM32G4xxLEDDriver::new(usart3, led_tx_pin, &mut rcc);
    

    let clock_pin: gpio::PB7<gpio::AF10> = gpiob.pb7.into_alternate();
    let mut pwm = dp.TIM3.pwm(clock_pin, 100.kHz(), &mut rcc);
    let _ = pwm.set_duty_cycle_percent(5);
    pwm.enable();

    // J8 CH0
    gpioc.pc9.into_push_pull_output().set_low();
    // J9 CH1
    gpioc.pc8.into_push_pull_output().set_low();
    // J2 CH2
    gpioc.pc7.into_push_pull_output().set_low();
    // J4 CH3
    gpioc.pc6.into_push_pull_output().set_low();

    let mut cycles = 0;
    loop {

        // RGB LED test routine!
            let led_color = RGBLEDColor {
                red: if cycles % 3 == 0 { 0xFF } else { 0x00 },
                blue: if cycles % 3 == 1 { 0xFF } else { 0x00 },
                green: if cycles % 3 == 1 { 0xFF } else { 0x00 },
            };
            let led_color_2: u32 = 0xF0F000 >> (8 * ((cycles + 1) % 3));
            cycles += 1;
            hled.set_nth_led(0, led_color);
            hled.set_nth_led(1, led_color);
            hled.set_nth_led(2, led_color);
            hled.set_nth_led(3, led_color);
            hled.set_nth_led(4, led_color);
            hled.set_nth_led(5, led_color);
            hled.set_nth_led_and_render(1, led_color_2.into());

        cortex_m::asm::nop();

    }
}

