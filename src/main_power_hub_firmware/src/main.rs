//! Firmware for the stepper driver board on the rover drivebase.
//!
//! This mainly publishes sensor data over Cyphal/CAN (on the FDCAN) on a variety of subjects
//! and drives the stepper motors as instructed by Cyphal/CAN messages from the OBC.

#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m_rt::entry;
use stm32g4xx_hal::prelude::*;
use stm32g4xx_hal::{
    gpio::{GpioExt, PB7, AF10}, 
    pwr::PwrExt, 
    rcc::*, 
    time::RateExtU32,
    pwm::PwmExt
};
use cortex_m_log::{destination::Itm, print, println};

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

    
    /*
    // All for println omagawd
    dp.DBGMCU.cr().write(|w| unsafe {
        w.trace_ioen().set_bit();   // Enable TPIU
        w.trace_mode().bits(00)     // Async (SWO) Mode
    });

    let core_dp = cortex_m::Peripherals::take().unwrap();
    let mut log = Itm::new(core_dp.ITM);

    println!(log, "Amogus");
    */



    // Set up pins.
    let gpiob = dp.GPIOB.split(&mut rcc);

    let pin: PB7<AF10> = gpiob.pb7.into_alternate();
    let mut pwm = dp.TIM3.pwm(pin, 100.kHz(), &mut rcc);
    let _ = pwm.set_duty_cycle_percent(5);
    pwm.enable();


    loop {
        cortex_m::asm::nop();
    }
}
