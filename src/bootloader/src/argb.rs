use stm32g4xx_hal::{rcc::Rcc, serial::{self, FullConfig, StopBits, TxExt}, time};
use embedded_io::Write;

use crate::peripherals::{ArgbInstance, ArgbPin};

const LED_COUNT: usize = 6;
const SYS_LED_POS_A: usize = 1;
const SYS_LED_POS_B: usize = 4;

#[derive(Copy, Clone)]
struct Colour {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

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
    Verifying,
    Booting,
    Error,
}

pub struct ArgbSys {
    hw_uart: serial::Tx<ArgbInstance, ArgbPin, serial::NoDMA>,
    state: State,
    identifying: bool,
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
            state: State::Idle,
            identifying: false,
        }
    }

    pub fn tick(&mut self) {
        let phase = true;

        let mut colour_sequence = [None; LED_COUNT];
        if phase {
            colour_sequence[SYS_LED_POS_A] = Some(Colour {r: 0, g: 0, b: 0});
            colour_sequence[SYS_LED_POS_B] = Some(Colour {r: 0, g: 0, b: 0});
        } else {
            colour_sequence[SYS_LED_POS_A] = Some(Colour {r: 0, g: 0, b: 0});
            colour_sequence[SYS_LED_POS_B] = Some(Colour {r: 0, g: 0, b: 0});
        };

        self.hw_uart.write_all(
            colour_sequence
                .map(|x| 
                    if self.identifying {
                        x.unwrap_or(Colour { r: 0, g: 0, b: 0 })
                    } else {
                        x.unwrap_or(Colour { r: 0, g: 0, b: 0 }).dim()
                    }
                ).map(|x| x.as_uart_bytes()).as_flattened()
        ).unwrap();
    }

    pub fn set_state(&mut self, state: State) {
        self.state = state;
    }

    /*pub fn identify(&mut self) {
        self.identifying = true;
    }*/
}
