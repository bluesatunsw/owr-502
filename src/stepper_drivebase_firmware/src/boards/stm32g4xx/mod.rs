//! Implementations of some specific low-level drivers for stepper boards Rev. A2, B2 (STM32G474) and
//! the WeAct dev board (STM32G431). The 32-bit microsecond clock and FDCAN driver have whole
//! modules to themselves.

use cfg_if::cfg_if;
use cortex_m_semihosting::hprintln;

use crate::boards::{
    CyphalClock, RGBLEDDriver, RGBLEDColor,
};
pub mod clock;
pub use clock::{STM32G4xxCyphalClock, STM32G4xxGeneralClock};
pub mod fdcan;
pub use fdcan::STM32G4xxCanDriver;
pub mod stepper;
pub use stepper::{STM32G4xxStepperDriver, StepperTempPins, StepperCSPins, EncoderCSPins};

use hal::{
    prelude::*,
    time::{self, RateExtU32},
    pwr::PwrExt,
    rcc::{self, MCOExt, Config, PllConfig, PllSrc, PllMDiv, PllNMul, PllQDiv, PllRDiv, FdCanClockSource, Rcc},
    gpio,
    serial,
    pac,
};
use stm32g4xx_hal as hal;   // don't need to put this in a cfg_if because which board it targets is
                            // specified as a feature in Cargo.toml
use embedded_io::Write;

// TODO: set up TRACESWO or UART for faster debug logging

/// The number of RGB LEDs (WS2812s) on the board.
pub const NUM_LEDS: usize = 6;

type LedTxPin = gpio::PB6<gpio::AF7>;

/// RGB LED (WS2812) driver.
///
/// The WS2812 protocol uses short (nominal 400 ns on/850 off) and long (800 on/450 ns off) pulses
/// to encode 0s and 1s. With a bit time of 400 ns (2.5 Mbaud), we can use three bits per WS2812 bit
/// to send these pulses over USART, using the Tx pin in inverted polarity mode and taking the
/// start and stop bits as free high and low bit periods respectively.
/// The resulting timings are solidly within the WS2812 spec.

pub struct STM32G4xxLEDDriver {
    usart: hal::serial::Tx<pac::USART1, LedTxPin, serial::NoDMA>,
    colors: [RGBLEDColor; NUM_LEDS],
}

impl STM32G4xxLEDDriver {
    // Initialise the USART1 peripheral.
    fn new(usart1: pac::USART1, tx_pin: LedTxPin, rcc: &mut Rcc) -> Self 
    {
        const BAUDRATE: time::Bps = time::Bps(2_500_000);
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
// I2C1: pin PA15, PB7

pub struct I2CDriver {
    
}

pub fn init() -> (
    STM32G4xxCyphalClock,
    STM32G4xxGeneralClock,
    STM32G4xxCanDriver,
    STM32G4xxLEDDriver,
    I2CDriver,
    STM32G4xxStepperDriver
) {
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

    // Set up pins.
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);

    // Set PB8 to output mode to limit power consumption on reset
    gpiob.pb8.into_push_pull_output();

    let mut cyphal_clock = STM32G4xxCyphalClock::new_singleton(dp.TIM2, &mut rcc);

    // FDCAN
    let can_rx_pin = gpioa.pa11.into_alternate();
    let can_tx_pin = gpioa.pa12.into_alternate();
    let can_driver = STM32G4xxCanDriver::new_singleton(dp.FDCAN1, can_rx_pin, can_tx_pin, &mut rcc);

    // ARGB LED
    let led_tx_pin = gpiob.pb6.into_alternate();
    let usart1 = dp.USART1;
    let led_driver = STM32G4xxLEDDriver::new(usart1, led_tx_pin, &mut rcc);

    // I2C
    // TODO

    // Steppers/SPI
    let temp0 = gpioa.pa0.into_analog();
    let temp1 = gpioa.pa1.into_analog();
    let temp2 = gpiob.pb2.into_analog();
    let temp3 = gpioc.pc5.into_analog();
    let temp_pins = StepperTempPins(temp0, temp1, temp2, temp3);

    let enn_pin = gpiod.pd2.into_push_pull_output_in_state(gpio::PinState::High);
    let clk_pin = gpioa.pa8.into_analog();
    #[cfg(feature = "rev_a2")]
    let diag_pin = gpioa.pa10.into_push_pull_output();
    #[cfg(not(feature = "rev_a2"))]
    let diag_pin = gpioa.pa10.into_input();

    let sck = gpioc.pc10.into_alternate();
    let miso = gpioc.pc11.into_alternate();
    let mosi = gpioc.pc12.into_alternate();
    let cs0 = gpiob.pb9.into_push_pull_output_in_state(gpio::PinState::High);
    let cs1 = gpioc.pc13.into_push_pull_output_in_state(gpio::PinState::High);
    let cs2 = gpioc.pc14.into_push_pull_output_in_state(gpio::PinState::High);
    let cs3 = gpioc.pc15.into_push_pull_output_in_state(gpio::PinState::High);
    let step_cs_pins = StepperCSPins(cs0.into(), cs1.into(), cs2.into(), cs3.into());
    let step_spi_pins = (sck, miso, mosi);

    let sck = gpiob.pb13.into_alternate();
    let miso = gpiob.pb14.into_alternate();
    let mosi = gpiob.pb15.into_alternate();
    let cs0 = gpioc.pc6.into_push_pull_output_in_state(gpio::PinState::High);
    let cs1 = gpioc.pc7.into_push_pull_output_in_state(gpio::PinState::High);
    let cs2 = gpioc.pc8.into_push_pull_output_in_state(gpio::PinState::High);
    let cs3 = gpioc.pc9.into_push_pull_output_in_state(gpio::PinState::High);
    let enc_cs_pins = EncoderCSPins(cs0.into(), cs1.into(), cs2.into(), cs3.into());
    let enc_spi_pins = (sck, miso, mosi);

    cyphal_clock.start();

    let mut general_clock = STM32G4xxGeneralClock::new();

    let mut stepper_driver = STM32G4xxStepperDriver::new(
        dp.ADC12_COMMON,
        dp.ADC2,
        dp.SPI2,
        dp.SPI3,
        &mut general_clock,
        &mut rcc,
        temp_pins,
        enn_pin,
        clk_pin,
        diag_pin,
        step_spi_pins,
        step_cs_pins,
        enc_spi_pins,
        enc_cs_pins,
    );

    stepper_driver.init_steppers_config();
    stepper_driver.init_encoders_config();
    
    (cyphal_clock, general_clock, can_driver, led_driver, I2CDriver {}, stepper_driver)
}
