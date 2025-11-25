//! Implementation of the 32-bit microsecond clock(s) necessary for Cyphal/canadensis.
//!
//! This is just a port of the clock code for g4xx. If I get around to improving that
//! implementation I will copy it over here.

use hal::pac;
use hal::rcc::{self, Reset, Enable};
use stm32g0xx_hal as hal;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use canadensis::core::time;

use crate::boards::{CyphalClock, GeneralClock};

// make clock globally available, so both the GeneralClock and CyphalClock can read it
static G_CYPHAL_CLOCK: Mutex<RefCell<Option<STM32G0xxMicroClock>>> =
    Mutex::new(RefCell::new(None));

// panics if the clock not initialised and started as in STM32G0xxCyphalClock
fn get_instant() -> time::Microseconds32 {
    cortex_m::interrupt::free(|cs| {
        if let Some(cyphal_clock) = G_CYPHAL_CLOCK.borrow(cs).borrow_mut().as_mut() {
            time::Microseconds32::from_ticks(
                cyphal_clock.hw_timer.cnt().read().bits()
            )
        } else {
            panic!("Could not borrow global cyphal clock")
        }
    })
}

struct STM32G0xxMicroClock {
    // TIM2 is a 32-bit timer in the G0 series.
    hw_timer: pac::TIM2,
}

// Internal 1 MHz timer to provide time instants for Cyphal.
impl STM32G0xxMicroClock {
    fn new(tim2: pac::TIM2, rcc: &mut rcc::Rcc) -> Self {
        // Reset and enable the timer peripheral.
        pac::TIM2::reset(rcc);
        pac::TIM2::enable(rcc);
        Self {
            hw_timer: tim2,
        }
    }

    // Configure timers: count up at 1 MHz, overflow automatically and silently.
    // this architecture just gives us a 32-bit timer so we don't need to do silly clock chaining yay
    fn start(&mut self) {
        // updates on overflow happen automatically
        // PCLK1 is 64MHz, so prescale by /64
        unsafe {
            // scale 64MHz to 1MHz
            self.hw_timer.psc().write(|w| w.psc().bits(64 - 1));
            // NOTE: do not disable updates, as the prescaler is loaded ON AN UPDATE EVENT
            // so the prescaler won't actually apply until e.g. counter overflows
            // (which we effectively set to happen immediately below)
            self.hw_timer.cnt().write(|w| w.cnt().bits(0xFFFFFFFF));
            // enable timer
            self.hw_timer.cr1().write(|w| w.cen().bit(true));
        }
    }
}

pub struct STM32G0xxCyphalClock {}

impl STM32G0xxCyphalClock {
    pub fn new_singleton(tim2: pac::TIM2, rcc: &mut rcc::Rcc) -> Self {
        // takes in a few hardware peripherals
        cortex_m::interrupt::free(|cs| {
            *G_CYPHAL_CLOCK.borrow(cs).borrow_mut() = Some(STM32G0xxMicroClock::new(tim2, rcc))
        });
        Self {}
    }
}

impl CyphalClock for STM32G0xxCyphalClock {
    fn start(&mut self) {
        cortex_m::interrupt::free(|cs| {
            G_CYPHAL_CLOCK
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .start()
        });
    }
}

impl time::Clock for STM32G0xxCyphalClock {
    fn now(&mut self) -> time::Microseconds32 {
        get_instant()
    }
}

pub struct STM32G0xxGeneralClock {}

impl STM32G0xxGeneralClock {
    pub fn new() -> Self { Self {} }
}

impl GeneralClock for STM32G0xxGeneralClock {
    fn now(&self) -> time::Microseconds32 {
        get_instant()
    }
}
