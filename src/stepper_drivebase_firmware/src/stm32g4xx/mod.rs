//! Implementations of some specific low-level drivers for stepper boards Rev. A2, B2 (STM32G474) and
//! the WeAct dev board (STM32G431). The 32-bit microsecond clock, FDCAN driver, TMC5160/encoder
//! driver and I2C driver have whole modules to themselves.
use stm32g4xx_hal::{self as hal, serial::TxExt};

pub mod clock;
pub use clock::{STM32G4xxCyphalClock, STM32G4xxGeneralClock};
pub mod fdcan;
pub use fdcan::STM32G4xxCanDriver;
pub mod stepper;
pub use stepper::{STM32G4xxStepperDriver, StepperTempPins, StepperCSPins, EncoderCSPins};
pub mod i2c;
pub use i2c::STM32G4xxI2CDriver;

use hal::{
    time::{self, RateExtU32},
    pwr::PwrExt,
    rcc::{Config, PllConfig, PllSrc, PllMDiv, PllNMul, PllQDiv, PllRDiv, FdCanClockSource, Rcc, RccExt},
    gpio::{self, GpioExt},
    serial,
    pac
};


use embedded_io::Write;

// TODO: set up TRACESWO or UART for faster debug logging

/// The number of RGB LEDs (WS2812s) on the board.
pub const NUM_LEDS: usize = 6;

#[derive(Copy, Clone)]
pub struct RGBLEDColor {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

impl RGBLEDColor {
    pub fn default() -> Self {
        RGBLEDColor {
            red: 5,
            green: 5,
            blue: 5
        }
    }
}

impl From<u32> for RGBLEDColor {
    /// Given a classic RGB hex code
    fn from(value: u32) -> Self {
        RGBLEDColor {
            red: ((value & 0xFF0000) >> 16) as u8,
            green: ((value & 0xFF00) >> 8) as u8,
            blue: (value & 0xFF) as u8,
        }
    }
}

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
    
    pub fn set_nth_led(&mut self, n: usize, color: RGBLEDColor) {
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

    pub fn set_nth_led_and_render(&mut self, n: usize, color: RGBLEDColor) {
        self.set_nth_led(n, color);
        self.render();
    }
}

pub fn init() -> (
    STM32G4xxCyphalClock,
    STM32G4xxGeneralClock,
    STM32G4xxCanDriver,
    STM32G4xxLEDDriver,
    STM32G4xxI2CDriver,
    STM32G4xxStepperDriver
) {
    // Embedded boilerplate...
    let dp = hal::stm32::Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain().freeze();

        // We just want to use the external (8 MHz || 24 MHz) HSE crystal, route this into the PLL,
        // get 64 MHz out for SYSCLK via PLLR and route 128 MHz to FDCAN via PLLQ.
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
    let sda = gpiob.pb7.into_alternate_open_drain();
    let scl = gpioa.pa15.into_alternate_open_drain();
    let i2c_driver = STM32G4xxI2CDriver::new(dp.I2C1, sda, scl, &mut rcc);

    // Steppers/SPI
    let temp0 = gpioa.pa0.into_analog();
    let temp1 = gpioa.pa1.into_analog();
    let temp2 = gpiob.pb2.into_analog();
    let temp3 = gpioc.pc5.into_analog();
    let temp_pins = StepperTempPins(temp0, temp1, temp2, temp3);

    let enn_pin = gpiod.pd2.into_push_pull_output_in_state(gpio::PinState::High);
    let clk_pin = gpioa.pa8.into_analog();

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

    // I2C testing -- remove once confirmed working on hardware
    // page-aligned
    // let short_data: [u8; 8] = [0xFF, 0x55, 0xAA, 0x00, 0x13, 0x37, 0xDE, 0xAD];
    // let mut short_readback: [u8; 8] = [0x00; 8];
    // i2c_driver.eeprom_write(0x3F80, &short_data).unwrap();
    // i2c_driver.eeprom_read(0x3F80, &mut short_readback).unwrap();
    // let mut allgood = true;
    // for i in 0..short_readback.len() {
    //     if short_data[i] != short_readback[i] {
    //         allgood = false;
    //     }
    // }
    // if allgood {
    //     hprintln!("8-byte EEPROM test 1: PASS");
    // } else {
    //     hprintln!("8-byte EEPROM test 1: FAIL");
    // }
    // // try across a page boundary
    // let mut short_readback = [0x00; 8];
    // i2c_driver.eeprom_write(0x1A3D, &short_data).unwrap();
    // i2c_driver.eeprom_read(0x1A3D, &mut short_readback).unwrap();
    // let mut allgood = true;
    // for i in 0..short_readback.len() {
    //     if short_data[i] != short_readback[i] {
    //         allgood = false;
    //     }
    // }
    // if allgood {
    //     hprintln!("8-byte EEPROM test 2: PASS");
    // } else {
    //     hprintln!("8-byte EEPROM test 2: FAIL");
    // }
    // // try a large multi-page write/read
    // let mut long_data: [u8; 256] = [0x00; 256];
    // for i in 0..256 {
    //     long_data[i] = 0;
    // }
    // let mut long_readback: [u8; 256] = [0x00; 256];
    // i2c_driver.eeprom_write(0x0440, &long_data).unwrap();
    // i2c_driver.eeprom_read(0x0440, &mut long_readback).unwrap();
    // let mut allgood = true;
    // for i in 0..long_readback.len() {
    //     if long_data[i] != long_readback[i] {
    //         allgood = false;
    //     }
    // }
    // if allgood {
    //     hprintln!("256-byte EEPROM test 1: PASS");
    // } else {
    //     hprintln!("256-byte EEPROM test 1: FAIL");
    // }
    // // IMU
    // let whoami = i2c_driver.imu_read_reg(ISM330Register::WHO_AM_I).unwrap();
    // let temp = i2c_driver.imu_read_temperature().unwrap();
    // let gyro = i2c_driver.imu_read_gyro().unwrap();
    // let accel = i2c_driver.imu_read_accel().unwrap();
    // hprintln!("WHO_AM_I = {}, TEMP = {} degC, GYRO = X {} Y {} Z {}, ACCEL = X {} Y {} Z {}",
    //     whoami, temp.0, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);

    (cyphal_clock, general_clock, can_driver, led_driver, i2c_driver, stepper_driver)
}
