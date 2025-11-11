//! Implementation of the 32-bit microsecond clock(s) necessary for Cyphal/canadensis.

use hal::pac;
use stm32g4xx_hal as hal;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use canadensis::core::time;

use crate::boards::{CyphalClock, GeneralClock};

// make clock globally available, so both the GeneralClock and CyphalClock can read it
static G_CYPHAL_CLOCK: Mutex<RefCell<Option<STM32G4xxMicroClock>>> =
    Mutex::new(RefCell::new(None));

// panics if the clock not initialised and started as in STM32G4xxCyphalClock
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

struct STM32G4xxMicroClock {
    // TIM2 is a 32-bit timer in the G4 series.
    // We could instead chain 16-bit timers TIM3 and TIM4 to get perfectly hardware-accurate FDCAN
    // frame timestamps, since we can configure FDCAN to use TIM3 as the timestamp counter. But...
    // we're not going to do that!
    hw_timer: pac::TIM2,
}

// Internal 1 MHz timer to provide time instants for Cyphal.
impl STM32G4xxMicroClock {
    fn new(tim2: pac::TIM2, rcc: &mut pac::RCC) -> Self {
        // Reset and enable the timer peripheral.
        rcc.apb1rstr1().modify(|_, w| w.tim2rst().bit(true));
        rcc.apb1rstr1().modify(|_, w| w.tim2rst().bit(false));
        rcc.apb1enr1().modify(|_, w| w.tim2en().bit(true));
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

pub struct STM32G4xxCyphalClock {}

impl STM32G4xxCyphalClock {
    pub fn new_singleton(tim2: pac::TIM2, rcc: &mut pac::RCC) -> Self {
        // takes in a few hardware peripherals
        cortex_m::interrupt::free(|cs| {
            *G_CYPHAL_CLOCK.borrow(cs).borrow_mut() = Some(STM32G4xxMicroClock::new(tim2, rcc))
        });
        Self {}
    }
}

impl CyphalClock for STM32G4xxCyphalClock {
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

impl time::Clock for STM32G4xxCyphalClock {
    fn now(&mut self) -> time::Microseconds32 {
        get_instant()
    }
}

pub struct STM32G4xxGeneralClock {}

impl GeneralClock for STM32G4xxGeneralClock {
    fn now(&self) -> time::Microseconds32 {
        get_instant()
    }
}
