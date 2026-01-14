use stm32g4xx_hal::pac;
use canadensis::core::time::{Clock, Microseconds32};

pub struct ClockSystem {
    hw_timer: pac::TIM2,
}

impl ClockSystem {
    pub fn new(tim2: pac::TIM2, rcc: &mut pac::RCC) -> Self {
        // Reset and enable the timer peripheral.
        rcc.apb1rstr1().modify(|_, w| w.tim2rst().bit(true));
        rcc.apb1rstr1().modify(|_, w| w.tim2rst().bit(false));
        rcc.apb1enr1().modify(|_, w| w.tim2en().bit(true));

        // updates on overflow happen automatically
        // PCLK1 is 64MHz, so prescale by /64
        unsafe {
            // scale 64MHz to 1MHz
            tim2.psc().write(|w| w.psc().bits(64 - 1));
            // NOTE: do not disable updates, as the prescaler is loaded ON AN UPDATE EVENT
            // so the prescaler won't actually apply until e.g. counter overflows
            // (which we effectively set to happen immediately below)
            tim2.cnt().write(|w| w.cnt().bits(0xFFFFFFFF));
            // enable timer
            tim2.cr1().write(|w| w.cen().bit(true));
        }

        Self {
            hw_timer: tim2,
        }
    }
}

impl Clock for ClockSystem {
    fn now(&mut self) -> Microseconds32 {
        Microseconds32::from_ticks(self.hw_timer.cnt().read().bits())
    }
}
