use canadensis::core::time::{Clock, Microseconds32};
use fugit::{MicrosDurationU32, RateExtU32};
use stm32g4::stm32g474::{RCC, TIM2};
use stm32g4xx_hal::rcc::{Clocks, PLLClocks, Rcc, RccExt};

pub unsafe fn conjure_rcc() -> Rcc {
    // SAFETY: this is the reason why this function is unsafe
    let mut rcc = unsafe { RCC::steal() }.constrain();
    rcc.clocks = Clocks {
        sys_clk: 168.MHz(),
        core_clk: 168.MHz(),
        ahb_clk: 168.MHz(),
        apb1_clk: 168.MHz(),
        apb1_tim_clk: 168.MHz(),
        apb2_clk: 168.MHz(),
        apb2_tim_clk: 168.MHz(),
        pll_clk: PLLClocks {
            r: Some(168.MHz()),
            q: Some(168.MHz()),
            p: None,
        },
    };

    rcc
}

pub struct MicrosecondClock {
    hw_timer: TIM2,
}

impl MicrosecondClock {
    pub fn new(tim2: TIM2, rcc: &mut Rcc) -> Self {
        // Reset and enable the timer peripheral.
        rcc.apb1rstr1().modify(|_, w| w.tim2rst().bit(true));
        rcc.apb1rstr1().modify(|_, w| w.tim2rst().bit(false));
        rcc.apb1enr1().modify(|_, w| w.tim2en().bit(true));

        // updates on overflow happen automatically
        unsafe {
            // scale 168MHz to 1MHz
            tim2.psc().write(|w| w.psc().bits(168 - 1));
            // NOTE: do not disable updates, as the prescaler is loaded ON AN UPDATE EVENT
            // so the prescaler won't actually apply until e.g. counter overflows
            // (which we effectively set to happen immediately below)
            tim2.cnt().write(|w| w.cnt().bits(0xFFFFFFFF));
            // enable timer
            tim2.cr1().write(|w| w.cen().bit(true));
        }

        Self { hw_timer: tim2 }
    }
}

impl MicrosecondClock {
    pub fn now_const(&self) -> Microseconds32 {
        Microseconds32::from_ticks(self.hw_timer.cnt().read().bits())
    }

    pub fn advance_if_elapsed(
        &self,
        start: &mut Microseconds32,
        period: MicrosDurationU32,
    ) -> bool {
        let end = *start + period;
        let now = self.now_const();

        // end overflowed, so wait until now also overflows
        if *start > end && now > *start {
            return false;
        }

        if now >= end {
            *start = end;
            // prevent double triggering when this hasn't been called in a long time
            if *start + period >= end {
                *start = now;
            }
            true
        } else {
            false
        }
    }
}

impl Clock for MicrosecondClock {
    fn now(&mut self) -> Microseconds32 {
        self.now_const()
    }
}
