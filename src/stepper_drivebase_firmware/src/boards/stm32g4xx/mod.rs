//! Implementations of some specific low-level drivers for stepper boards Rev. A2, B2 (STM32G474) and
//! the WeAct dev board (STM32G431). The 32-bit microsecond clock and FDCAN driver have whole
//! modules to themselves.

use cfg_if::cfg_if;
use cortex_m_semihosting::hprintln;

use crate::boards::{CyphalClock, RGBLEDDriver, RGBLEDColor};
pub mod clock;
pub use clock::{STM32G4xxCyphalClock, STM32G4xxGeneralClock};
pub mod fdcan;
pub use fdcan::STM32G4xxCanDriver;

use hal::{
    prelude::*,
    time::{self, RateExtU32},
    pwr::PwrExt,
    rcc::{Config, PllConfig, PllSrc, PllMDiv, PllNMul, PllQDiv, PllRDiv, FdCanClockSource, Rcc},
    gpio,
    serial,
    pac,
};
use stm32g4xx_hal as hal;   // don't need to put this in a cfg_if because which board it targets is
                            // specified as a feature in Cargo.toml
use embedded_io::Write;     // why aren't the embedded-io traits re-exported??

// RGB LED (WS2812) driver.
//
// The WS2812 protocol uses short (nominal 400 ns on/850 off) and long (800 on/ 450 ns off) pulses
// to encode 0s and 1s. With a bit time of 400 ns (2.5 Mbaud), we can use three bits per WS2812 bit
// to send these pulses over USART, using the Tx pin in inverted polarity mode and minding the
// start and stop bits. The resulting timings are just within the WS2812 specs!

type LedTxPin = gpio::PB6<gpio::AF7>;

pub const NUM_LEDS: usize = 6;  // make sure you set this correctly!

pub struct STM32G4xxLEDDriver {
    usart: hal::serial::Tx<pac::USART1, LedTxPin, serial::NoDMA>,
    // bytes are stored G R B
    colors: [RGBLEDColor; NUM_LEDS],
}

impl STM32G4xxLEDDriver {
    // Initialise the USART1 peripheral.
    fn new(usart1: pac::USART1, tx_pin: LedTxPin, rcc: &mut Rcc) -> Self 
    {
        const BAUDRATE: time::Bps = time::Bps(2_500_000);
        // Polarity of TX pin inverted so we're idle-low
        let config = hal::serial::config::FullConfig::default()
            .baudrate(BAUDRATE)
            .tx_invert()
            .wordlength_7()
            .stopbits(serial::config::StopBits::STOP1);
        let usart1 = usart1.usart_txonly(tx_pin, config, rcc).unwrap();

        Self {
            usart: usart1,
            colors: [RGBLEDColor::default(); NUM_LEDS]
        }
    }
}
impl RGBLEDDriver for STM32G4xxLEDDriver {
    fn set_nth_led(&mut self, n: usize, color: RGBLEDColor) {
        self.colors[n] = color;
    }

    fn render(&mut self) {
        // we squeeze three WS2812 bits per USART byte
        const TRIBIT_LUT: [u8; 8] = [
            // 000 -> H_START L L H L L H L [L STOP]
            // TX pin polarity inverted, so data actually 1101101
            // ...and then bits are sent in reverse, so 1011011 (hah it's a palindrome)
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
        for i in 0..NUM_LEDS {
            let mut packet = [0u8; 8];
            let color = self.colors[i];
            // required order for WS2812: G R B
            let shifted_color = ((color.green as u32) << 16) | ((color.red as u32) << 8) | (color.blue as u32);
            for j in 0..8 {
                let tribit = ((shifted_color & (0xE00000 >> (j * 3))) >> (21 - j * 3)) as usize;
                packet[j] = TRIBIT_LUT[tribit];
            }
            self.usart.write_all(&packet).unwrap();
        }
    }

    fn set_nth_led_and_render(&mut self, n: usize, color: RGBLEDColor) {
        self.set_nth_led(n, color);
        self.render();
    }
}

// TODO: I2C driver

pub struct I2CDriver {
    
}

pub fn init() -> (STM32G4xxCyphalClock, STM32G4xxGeneralClock, STM32G4xxCanDriver, STM32G4xxLEDDriver, I2CDriver) {
    // Embedded boilerplate...
    let dp = hal::stm32::Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain().freeze();

    cfg_if! {
        // We just want to use the external (8 MHz || 24 MHz) HSE crystal, route this into the PLL,
        // get 64 MHz out for SYSCLK via PLLR and route 128 MHz to FDCAN via PLLQ.
        if #[cfg(any(feature = "rev_a2", feature = "rev_b2"))] {
            let mut rcc = dp.RCC.freeze(
                // enable HSE @ 24 MHz (stepper board)
                Config::pll()
                    .pll_cfg(PllConfig {
                        mux: PllSrc::HSE(24.MHz()),
                        m: PllMDiv::DIV_3,
                        n: PllNMul::MUL_32,
                        r: Some(PllRDiv::DIV_4),
                        q: Some(PllQDiv::DIV_2),
                        p: None,
                    })
                    .fdcan_src(FdCanClockSource::PLLQ),
                pwr
            );
        } else if #[cfg(feature = "we_act_dev")] {
            let mut rcc = dp.RCC.freeze(
                // enable HSE @ 8 MHz (WeAct)
                Config::pll()
                    .pll_cfg(PllConfig {
                        mux: PllSrc::HSE(8.MHz()),
                        m: PllMDiv::DIV_1,
                        n: PllNMul::MUL_32,
                        r: Some(PllRDiv::DIV_4),
                        q: Some(PllQDiv::DIV_2),
                        p: None,
                    })
                    .fdcan_src(FdCanClockSource::PLLQ),
                pwr
            );
        }
    }

    // set PB8 to output mode to limit power consumption on reset
    unsafe {
        dp.GPIOB.moder().write(|w| w.moder8().bits(0b01));
    }

    // initialise all the other peripheral drivers
    let mut cyphal_clock = STM32G4xxCyphalClock::new_singleton(dp.TIM2, &mut rcc);
    let can_driver = STM32G4xxCanDriver::new_singleton(dp.FDCAN1, dp.GPIOA, &mut rcc);

    let gpiob = dp.GPIOB.split(&mut rcc);
    let led_tx_pin = gpiob.pb6.into_alternate();
    let usart1 = dp.USART1;
    let led_driver = STM32G4xxLEDDriver::new(usart1, led_tx_pin, &mut rcc);

    cyphal_clock.start();

    (cyphal_clock, STM32G4xxGeneralClock {}, can_driver, led_driver, I2CDriver {})

    // TODO: remaining communication interfaces.
    // I2C1: pin PA15, PB7
    // SPI: PC10, PC11, PC12, 
}
