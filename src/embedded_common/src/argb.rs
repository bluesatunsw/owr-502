use embedded_io::Write;
use stm32g4xx_hal::{
    rcc::Rcc,
    serial::{self, FullConfig, TxExt, TxPin},
    time,
};

#[derive(Copy, Clone)]
pub struct Colour {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Colour {
    pub(super) fn as_uart_bytes(&self) -> [u8; 8] {
        // we squeeze three WS2812 bits per USART byte
        const LUT: [u8; 8] = [
            // 000 -> H_START L L H L L H L L_STOP
            // TX pin polarity inverted, so data actually 0b1101101
            // ...and then bits are sent in reverse, so 0b1011011.
            // 001 -> [H]LLHLLHH[L] -> 0b1101100 -> 0b0011011,
            // 010 -> [H]LLHHLHL[L] -> 0b1100101 -> 0b1010011,
            // 011 -> [H]LLHHLHH[L] -> 0b1100100 -> 0b0010011, etc.
            0b1011011, 0b0011011, 0b1010011, 0b0010011, 0b1011010, 0b0011010, 0b1010010, 0b0010010,
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

    /// Create a new colour with an intensity scaled by the
    /// given factor x. 255 indicates full and 0 being none.
    #[must_use]
    pub fn scale(&self, x: u8) -> Self {
        Self {
            r: (self.r as u16 * x as u16 / 255) as u8,
            b: (self.b as u16 * x as u16 / 255) as u8,
            g: (self.g as u16 * x as u16 / 255) as u8,
        }
    }

    pub const BLACK: Self = Self { r: 0, g: 0, b: 0 };
    pub const AMBER: Self = Self {
        r: 239,
        g: 112,
        b: 14,
    };
}

pub struct Controller<UART, PIN: TxPin<UART>>
where
    serial::Tx<UART, PIN, serial::NoDMA>: Write,
{
    hw_uart: serial::Tx<UART, PIN, serial::NoDMA>,
    brightness: u8,
}

impl<UART, PIN: TxPin<UART>> Controller<UART, PIN>
where
    serial::Tx<UART, PIN, serial::NoDMA>: Write,
{
    pub fn new<INST: TxExt<UART, FullConfig>>(
        inst: INST,
        pin: PIN,
        brightness: u8,
        rcc: &mut Rcc,
    ) -> Self {
        Self {
            hw_uart: inst
                .usart_txonly(
                    pin,
                    FullConfig::default()
                        .baudrate(time::Bps(2_500_000))
                        .tx_invert()
                        .wordlength_7()
                        .stopbits(serial::StopBits::STOP1),
                    rcc,
                )
                .unwrap(),
            brightness,
        }
    }

    pub fn display<const N: usize>(&mut self, pattern: &[Colour; N]) {
        self.hw_uart
            .write_all(
                pattern
                    .map(|x| x.scale(self.brightness).as_uart_bytes())
                    .as_flattened(),
            )
            .unwrap();
    }
}
