use canadensis::core::time::Microseconds32;
use cortex_m::asm::delay;
use embedded_common::argb::{Colour, Controller};
use stm32g4xx_hal::rcc::Rcc;

use crate::peripherals::{ArgbInstance, ArgbPin};

const LED_COUNT: usize = 6;
const SYS_LED_POS_A: usize = 0;
const SYS_LED_POS_B: usize = 5;

const BLINK_PERIOD_US: u32 = 500_000;
const IDENTIFYING_BLINK_PERIOD_US: u32 = 200_000;

const PURPLE: Colour = Colour {
    r: 255,
    g: 0,
    b: 255,
};

const CYAN: Colour = Colour {
    r: 0,
    g: 255,
    b: 255,
};
const GREEN: Colour = Colour { r: 0, g: 255, b: 0 };
const RED: Colour = Colour { r: 255, g: 0, b: 0 };

pub enum State {
    Idle,
    Flashing,
    Booting,
    BadCrc,
    Error,
}

pub struct ArgbSys {
    controller: Controller<ArgbInstance, ArgbPin>,
    state_colour: Colour,
    identifying: bool,
    phase: bool,
}

impl ArgbSys {
    pub fn new(instance: ArgbInstance, pin: ArgbPin, rcc: &mut Rcc) -> Self {
        Self {
            controller: Controller::new(instance, pin, 63, rcc),
            state_colour: Colour::BLACK,
            identifying: false,
            phase: false,
        }
    }

    pub fn tick(&mut self, identifying: bool, time: Microseconds32) {
        self.identifying = identifying;
        if identifying {
            self.phase =
                (time.ticks() % IDENTIFYING_BLINK_PERIOD_US) > (IDENTIFYING_BLINK_PERIOD_US / 2);
        } else {
            self.phase = (time.ticks() % BLINK_PERIOD_US) > (BLINK_PERIOD_US / 2);
        }

        self.update();
    }

    pub fn set_state(&mut self, state: State) {
        self.state_colour = match state {
            State::Idle => Colour::BLACK,
            State::Flashing => CYAN,
            State::Booting => GREEN,
            State::BadCrc => Colour::AMBER,
            State::Error => RED,
        };

        self.update();
    }

    fn update(&mut self) {
        let mut colour_sequence = [Colour::BLACK; LED_COUNT];
        if self.identifying {
            if self.phase {
                colour_sequence[SYS_LED_POS_A] = Colour::BLACK;
                colour_sequence[SYS_LED_POS_B] = Colour::AMBER;
            } else {
                colour_sequence[SYS_LED_POS_A] = Colour::AMBER;
                colour_sequence[SYS_LED_POS_B] = Colour::BLACK;
            }
        } else {
            if self.phase {
                colour_sequence[SYS_LED_POS_A] = PURPLE;
                colour_sequence[SYS_LED_POS_B] = self.state_colour;
            } else {
                colour_sequence[SYS_LED_POS_A] = self.state_colour;
                colour_sequence[SYS_LED_POS_B] = PURPLE;
            }
        }

        self.controller.display(&colour_sequence);
        delay(12288);
    }
}
