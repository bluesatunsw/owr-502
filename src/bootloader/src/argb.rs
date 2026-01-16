use canadensis::core::time::Microseconds32;
use cortex_m::asm::delay;
use stm32g4xx_hal::{rcc::Rcc, serial::{self, FullConfig, StopBits, TxExt}, time};
use embedded_io::Write;

use crate::peripherals::{ArgbInstance, ArgbPin};

const LED_COUNT: usize = 6;
const SYS_LED_POS_A: usize = 0;
const SYS_LED_POS_B: usize = 5;

const BLINK_PERIOD_US: u32 = 500_000;
const IDENTIFYING_BLINK_PERIOD_US: u32 = 200_000;

#[derive(Copy, Clone)]
struct Colour {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

const BLACK: Colour = Colour { r: 0, g: 0, b: 0 };
const PURPLE: Colour = Colour { r: 255, g: 0, b: 255 };

const CYAN: Colour = Colour { r: 0, g: 255, b: 255 };
const ORANGE: Colour = Colour { r: 255, g: 63, b: 0 };
const GREEN: Colour = Colour { r: 0, g: 255, b: 0 };
const RED: Colour = Colour { r: 255, g: 0, b: 0 };

impl Colour {
    pub fn as_uart_bytes(&self) -> [u8; 8] {
        // we squeeze three WS2812 bits per USART byte
        const LUT: [u8; 8] = [
            // 000 -> H_START L L H L L H L L_STOP
            // TX pin polarity inverted, so data actually 1101101
            // ...and then bits are sent in reverse, so 1011011.
            0b1011011,
            // 001 -> [H]LLHLLHH[L] -> 1101100 -> 0011011
            0b0011011,
            // 010 -> [H]LLHHLHL[L] -> 1100101 -> 1010011
            0b1010011,
            // 011 -> [H]LLHHLHH[L] -> 1100100 -> 0010011
            0b0010011,
            // etc.
            0b1011010,
            0b0011010,
            0b1010010,
            0b0010010
        ];

        // required order for WS2812: G R B
        let mut packed_colour = ((self.g as u32) << 16) | ((self.r as u32) << 8) | (self.b as u32);
        let mut res = [0; 8];
        for i in (0..8).rev() {
            res[i] = LUT[(packed_colour & 0b111) as usize];
            packed_colour >>= 3;
        }

        res
    }

    pub fn dim(self) -> Self {
        Self {
            r: self.r / 4,
            g: self.g / 4,
            b: self.b / 4,
        }
    }
}

pub enum State {
    Idle,
    Flashing,
    Booting,
    BadCrc,
    Error,
}

pub struct ArgbSys {
    hw_uart: serial::Tx<ArgbInstance, ArgbPin, serial::NoDMA>,
    state_colour: Colour,
    identifying: bool,
    phase: bool,
}

impl ArgbSys {
    pub fn new(instance: ArgbInstance, pin: ArgbPin, rcc: &mut Rcc) -> Self {
        Self {
            hw_uart: instance.usart_txonly(
                pin, 
                FullConfig::default()
                    .baudrate(time::Bps(2_500_000))
                    .tx_invert()
                    .wordlength_7()
                    .stopbits(StopBits::STOP1),
                rcc
            ).unwrap(),
            state_colour: BLACK,
            identifying: false,
            phase: false,
        }
    }

    pub fn tick(&mut self, identifying: bool, time: Microseconds32) {
        self.identifying = identifying;
        if identifying {
            self.phase = (time.ticks() % IDENTIFYING_BLINK_PERIOD_US) > (IDENTIFYING_BLINK_PERIOD_US / 2);
        } else {
            self.phase = (time.ticks() % BLINK_PERIOD_US) > (BLINK_PERIOD_US / 2);
        }

        self.update();
    }

    pub fn set_state(&mut self, state: State) {
        self.state_colour = match state {
            State::Idle => BLACK,
            State::Flashing => CYAN,
            State::Booting => GREEN,
            State::BadCrc => ORANGE,
            State::Error => RED,
        };

        self.update();
    }

    fn update(&mut self) {
        let mut colour_sequence = [BLACK; LED_COUNT];
        if self.identifying {
            if self.phase {
                colour_sequence[SYS_LED_POS_A] = BLACK;
                colour_sequence[SYS_LED_POS_B] = ORANGE;
            } else {
                colour_sequence[SYS_LED_POS_A] = ORANGE;
                colour_sequence[SYS_LED_POS_B] = BLACK;
            }
        } else {
            if self.phase {
                colour_sequence[SYS_LED_POS_A] = PURPLE.dim();
                colour_sequence[SYS_LED_POS_B] = self.state_colour.dim();
            } else {
                colour_sequence[SYS_LED_POS_A] = self.state_colour.dim();
                colour_sequence[SYS_LED_POS_B] = PURPLE.dim();
            }
        }

        self.hw_uart.write_all(
            colour_sequence
                .map(|x| x.as_uart_bytes())
                .as_flattened()
        ).unwrap();
        delay(4096);
    }
}
