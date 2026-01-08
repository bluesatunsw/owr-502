//! Implementation of the 32-bit microsecond clock(s) necessary for Cyphal/canadensis.
//!
//! NOTE/TODO/WARNING/FIXME: The way that the GeneralClock/CyphalClock is done is janky and not
//! very Rust-y, since safety can be easily violated by trying to make a GeneralClock without first
//! making the CyphalClock and starting it. I need to rewrite this to use the Typestate pattern to
//! make the public interface perfectly safe. Also, reading the clock from multiple threads
//! concurrently is conceptually safe and we shouldn't *need* a Mutex... idk.

use stm32g4xx_hal as hal;

use hal::pac;
use hal::prelude::*;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use canadensis::core::time;

// make clock globally available, so both the GeneralClock and CyphalClock can read it
static G_CYPHAL_CLOCK: Mutex<RefCell<Option<STM32G4xxMicroClock>>> = Mutex::new(RefCell::new(None));

// panics if the clock not initialised and started as in STM32G4xxCyphalClock
fn get_instant() -> time::Microseconds32 {
    cortex_m::interrupt::free(|cs| {
        if let Some(cyphal_clock) = G_CYPHAL_CLOCK.borrow(cs).borrow_mut().as_mut() {
            time::Microseconds32::from_ticks(cyphal_clock.hw_timer.cnt().read().bits())
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
        Self { hw_timer: tim2 }
    }

    // Configure timers: count up at 1 MHz, overflow automatically and silently.
    // this architecture just gives us a 32-bit timer so we don't need to do silly clock chaining yay
    pub fn start(&mut self) {
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

    pub fn start(&mut self) {
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

impl STM32G4xxGeneralClock {
    pub fn new() -> Self {
        Self {}
    }

    pub fn now(&self) -> time::Microseconds32 {
        get_instant()
    }

    fn delay_us(&mut self, mut us: u32) {
        let start_time: u32 = self.now().ticks();
        // we *know* now() is later than start_time, so we can just do a wrapping_sub
        // ...with the caveat that if us is close to u32::MAX, we could wrap around again.
        let extra_wait: bool; // if this is true, we need to wait an additional u32::MAX / 2
        if us > u32::MAX / 2 {
            us = us - (u32::MAX / 2);
            extra_wait = true;
        } else {
            extra_wait = false;
        }
        while self.now().ticks().wrapping_sub(start_time) < us {
            // busy-wait
        }
        if extra_wait {
            let extra_time = self.now().ticks();
            while self.now().ticks().wrapping_sub(extra_time) < u32::MAX / 2 {
                // busy-wait
            }
        }
    }
}

impl DelayNs for STM32G4xxGeneralClock {
    fn delay_ns(&mut self, ns: u32) {
        self.delay_us(ns.div_ceil(1000));
    }
}
