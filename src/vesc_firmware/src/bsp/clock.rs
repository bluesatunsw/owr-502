use canadensis::core::time::{self, MicrosecondDuration32, Microseconds32};
use fugit::TimerDurationU32;
use stm32f4xx_hal::{pac::TIM5, prelude::*, rcc::Rcc, timer::CounterUs};

// TIM5 is used as a 32 bit microsecond resolution timer, takes ~1h to overflow
pub struct STM32F4xxCyphalClock {
    timer: CounterUs<TIM5>,
}

impl STM32F4xxCyphalClock {
    pub fn new(tim: TIM5, rcc: &mut Rcc) -> Self {
        let mut us_timer = tim.counter_us(rcc);
        us_timer
            .start(TimerDurationU32::from_ticks(u32::MAX))
            .unwrap();

        STM32F4xxCyphalClock { timer: us_timer }
    }

    pub fn advance_if_elapsed(
        &self,
        start: &mut Microseconds32,
        period: MicrosecondDuration32,
    ) -> bool {
        // Did the period elapse (start+period < now)? Or did the timer overflow (start > now)?
        if *start + period > self.timer.now() || *start > self.timer.now() {
            return false;
        }

        *start = self.timer.now();
        true
    }
}

impl time::Clock for STM32F4xxCyphalClock {
    fn now(&mut self) -> Microseconds32 {
        self.timer.now()
    }
}
