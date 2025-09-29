//! See boards README

use canadensis::core::time as cyphal_time;
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;

use stm32f1xx_hal::{
    pac,
    rcc::{Enable, Reset},
};

struct STM32F103CyphalClock {
    // TIM2 allocated to the microsecond Cyphal timer, lower 16 bits
    hw_timer_lower: pac::TIM2,
    // TIM3 allocated to the microsecond Cyphal timer, upper 16 bits
    hw_timer_upper: pac::TIM3,
    // TODO: Make sure 32-bit hourly overflow handled sensibly.
}

// Internal 1 MHz timer to provide time instants for Cyphal.
impl STM32F103CyphalClock {
    fn new(tim2: pac::TIM2, tim3: pac::TIM3, rcc: &mut pac::RCC) -> Self {
        // Reset and enable the timer peripheral.
        pac::TIM2::reset(rcc);
        pac::TIM3::reset(rcc);
        pac::TIM2::enable(rcc);
        pac::TIM3::enable(rcc);
        Self {
            hw_timer_lower: tim2,
            hw_timer_upper: tim3,
        }
    }

    // Configure timers: count up lower at 1 MHz, wraparound and update upper on overflow.
    fn start(&mut self) {
        // updates on overflow happen automatically
        // set frequency to external clock (8MHz)
        unsafe {
            self.hw_timer_lower.smcr().write(|w| w.sms().bits(0b000)); // select CK_INT
            self.hw_timer_lower.psc().write(|w| w.psc().bits(0x0007)); // scale 8MHz to 1MHz

            // See STM RM0008 Section 15.3.15 Timer synchronization,
            // "Using one timer as prescaler for another timer" for hints on this section.
            // Configure lower timer to generate external trigger output on update (i.e. overflow).
            self.hw_timer_lower.cr2().write(|w| w.mms().bits(0b010));
            // Configure upper timer to be in external clock mode, clocked by ITR1
            // => TIM3 clocked by TIM2 updates (Table 86)
            self.hw_timer_upper
                .smcr()
                .write(|w| w.ts().bits(0b001).sms().bits(0b111));
            // set counters to 0 initially
            self.hw_timer_lower.cnt().write(|w| w.cnt().bits(0));
            self.hw_timer_upper.cnt().write(|w| w.cnt().bits(0));
        }
        // start timers
        self.hw_timer_lower
            .cr1()
            .write(|w| w.urs().set_bit().cen().set_bit());
        self.hw_timer_upper.cr1().write(|w| w.cen().set_bit());
        // desired defaults:
        // DIR = 0 -> count up
        // CMS = 00 -> edge-aligned mode
        // OPM = 0
    }
}

// make clock globally available, so we can write the interrupt handler for it
static G_CYPHAL_CLOCK: Mutex<RefCell<Option<STM32F103CyphalClock>>> =
    Mutex::new(RefCell::new(None));

pub struct CyphalClock {}

// exists to be instantiable locally but passes everything through to global state
// (state has to be global to be modifiable by interrupt handler)
impl CyphalClock {
    pub fn new_singleton(tim2: pac::TIM2, tim3: pac::TIM3, rcc: &mut pac::RCC) -> Self {
        // takes in a few hardware peripherals
        cortex_m::interrupt::free(|cs| {
            *G_CYPHAL_CLOCK.borrow(cs).borrow_mut() = Some(STM32F103CyphalClock::new(tim2, tim3, rcc))
        });
        CyphalClock {}
    }

    pub fn start(&self) {
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

// panics if CyphalClock not initialised and started
fn get_instant() -> cyphal_time::Microseconds32 {
    // TODO: Handle edge case where timer overflows in between reading upper and lower bits
    cortex_m::interrupt::free(|cs| {
        if let Some(cyphal_clock) = G_CYPHAL_CLOCK.borrow(cs).borrow_mut().as_mut() {
            let lower_time = cyphal_clock.hw_timer_lower.cnt().read().bits();
            cyphal_time::Microseconds32::from_ticks(
                (cyphal_clock.hw_timer_upper.cnt().read().bits() << 16) + lower_time
            )
        } else {
            panic!("Could not borrow global cyphal clock")
        }
    })
}

impl cyphal_time::Clock for CyphalClock {
    fn now(&mut self) -> cyphal_time::Microseconds32 {
        get_instant()
    }
}

pub struct GeneralClock {}

impl GeneralClock {
    pub fn now() -> cyphal_time::Microseconds32 {
        get_instant()
    }
}
