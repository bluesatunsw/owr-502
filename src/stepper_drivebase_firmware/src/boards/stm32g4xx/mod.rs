//! Implementations of some specific low-level drivers for stepper boards Rev. A2, B2 (STM32G474) and
//! the WeAct dev board (STM32G431). The 32-bit microsecond clock and FDCAN driver have whole
//! modules to themselves.

use cfg_if::cfg_if;
use libm::{self, Libm};
use cortex_m_semihosting::hprintln;

use crate::boards::{CyphalClock, RGBLEDDriver, RGBLEDColor, StepperDriver, StepperChannel, StepperRegister, SPIError, Celsius};
pub mod clock;
pub use clock::{STM32G4xxCyphalClock, STM32G4xxGeneralClock};
pub mod fdcan;
pub use fdcan::STM32G4xxCanDriver;

use hal::{
    prelude::*,
    time::{self, RateExtU32},
    pwr::PwrExt,
    rcc::{self, MCOExt, Config, PllConfig, PllSrc, PllMDiv, PllNMul, PllQDiv, PllRDiv, FdCanClockSource, Rcc},
    gpio,
    serial,
    pac,
    adc::{Adc, AdcClaim, AdcCommonExt, config::{self, AdcConfig, SampleTime, ClockMode, ClockSource, Prescaler}, Configured},
    spi,
};
use stm32g4xx_hal as hal;   // don't need to put this in a cfg_if because which board it targets is
                            // specified as a feature in Cargo.toml
use embedded_io::Write;     // why aren't the embedded-io traits re-exported??

// RGB LED (WS2812) driver.
//
// The WS2812 protocol uses short (nominal 400 ns on/850 off) and long (800 on/450 ns off) pulses
// to encode 0s and 1s. With a bit time of 400 ns (2.5 Mbaud), we can use three bits per WS2812 bit
// to send these pulses over USART, using the Tx pin in inverted polarity mode and taking the
// start and stop bits as free high and low bit periods respectively.
// The resulting timings are solidly within the WS2812 spec.

type LedTxPin = gpio::PB6<gpio::AF7>;

pub const NUM_LEDS: usize = 6;  // make sure this is set appropriately for the board

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

pub struct I2CDriver {
    
}

// TMC5160 stepper driver (over SPI).

type EnnPin = gpio::PD2<gpio::Output>;
//type ClkPin = gpio::PA8<gpio::AF0>; // this is MCO. AF6 is TIM1CH1
type ClkPin = gpio::PA8<gpio::Analog>; // this is MCO. AF6 is TIM1CH1
type DiagPin = gpio::PA10<gpio::Input>;

struct StepperTempPins(gpio::PA0, gpio::PA1, gpio::PB2, gpio::PC5);
struct StepperCSPins(gpio::AnyPin<gpio::Output>, gpio::AnyPin<gpio::Output>, gpio::AnyPin<gpio::Output>, gpio::AnyPin<gpio::Output>);

type StepperSPIPins = (gpio::PC10<gpio::AF6>, gpio::PC11<gpio::AF6>, gpio::PC12<gpio::AF6>);

pub struct STM32G4xxStepperDriver {
    adc: Adc<pac::ADC2, Configured>,
    enn: EnnPin,
    _clk: rcc::Mco<gpio::PA8<gpio::AF0>>,
    diag: DiagPin,
    temp: StepperTempPins,
    spi: spi::Spi<pac::SPI3, StepperSPIPins>,
    spi_cs: StepperCSPins,
}

impl STM32G4xxStepperDriver {
    fn new(
        adc_common: pac::ADC12_COMMON, adc2: pac::ADC2, spi3: pac::SPI3, uclock: &mut STM32G4xxGeneralClock, rcc: &mut Rcc,
        temp: StepperTempPins, enn: EnnPin, clk_pin: ClkPin, diag: DiagPin, spi_pins: StepperSPIPins, spi_cs_pins: StepperCSPins
    ) -> Self {
        // initialise the ADCs for the temperature pins
        // we intend to use them in one-shot mode
        let adc12 = adc_common.claim(
            ClockMode::AdcKerCk {
                // system clock is at 64 MHz, but abs max ADC clock frequency is 60 MHz, so /2
                prescaler: Prescaler::Div_2,
                src: ClockSource::SystemClock,
            },
            rcc
        );
        let adc = adc12.claim_and_configure(adc2, AdcConfig::default(), uclock);

        // initialise the MCO (CLK pin)
        // at 12 MHz
        let clk = clk_pin.mco(rcc::MCOSrc::HSE, rcc::Prescaler::Div2, rcc);

        // initialise the SPI bus, SPI3
        // (using max allowable frequency)
        let spi = spi3.spi(spi_pins, spi::MODE_3, 4.MHz(), rcc);

        // do initial config of the steppers over SPI
        // TODO

        STM32G4xxStepperDriver {
            adc,
            enn,
            _clk: clk,
            diag,
            temp,
            spi,
            spi_cs: spi_cs_pins,
        }
    }

    fn get_cs_pin(&mut self, channel: StepperChannel) -> &mut gpio::AnyPin<gpio::Output> {
        match channel {
            StepperChannel::Channel1 => &mut self.spi_cs.0,
            StepperChannel::Channel2 => &mut self.spi_cs.1,
            StepperChannel::Channel3 => &mut self.spi_cs.2,
            StepperChannel::Channel4 => &mut self.spi_cs.3,
        }
    }

    fn map_spi_error(err: spi::Error) -> SPIError {
        match err {
            spi::Error::Overrun => SPIError::Overrun,
            _ => SPIError::Other,
        }
    }
}

impl StepperDriver for STM32G4xxStepperDriver {
    fn enable_all(&mut self) {
        self.enn.set_low();
    }

    fn disable_all(&mut self) {
        self.enn.set_high();
    }

    fn set_position(&mut self, channel: StepperChannel, target: u32) -> Result<(), SPIError> {
        // TODO
        Ok(())
    }

    fn set_velocity(&mut self, channel: StepperChannel, velocity: i32) -> Result<(), SPIError> {
        // TODO
        Ok(())
    }

    // TODO: TMC5160 SPI interface sends data back on the *subsequent* read, so we could possibly
    // set up a pipelined interface?
    // self.start_pipeline_read(reg).continue_pipeline_read/write(reg, data).end_pipeline(data)
    // need to fix annoying data allocation and ownership churn in above model
    fn read_reg(&mut self, channel: StepperChannel, reg: StepperRegister) -> Result<u32, SPIError> {
        let cs = match channel {
            StepperChannel::Channel1 => &mut self.spi_cs.0,
            StepperChannel::Channel2 => &mut self.spi_cs.1,
            StepperChannel::Channel3 => &mut self.spi_cs.2,
            StepperChannel::Channel4 => &mut self.spi_cs.3,
        };
        cs.set_low();
        // 40-bit (5 byte), first byte is address, MSB of first byte is 0 for read
        let address: u8 = reg.into();
        let send_data = [address, 0u8, 0u8, 0u8, 0u8];
        let mut read_data = [0u8; 5];
        self.spi.transfer(&mut read_data, &send_data).map_err(STM32G4xxStepperDriver::map_spi_error)?;
        // hopefully this still works without CS going high in between?
        self.spi.transfer(&mut read_data, &send_data).map_err(STM32G4xxStepperDriver::map_spi_error)?;
        cs.set_high();
        // reconstruct register
        Ok((read_data[0] as u32) << 24 | (read_data[1] as u32) << 16 | (read_data[2] as u32) << 8 | (read_data[3] as u32))
    }

    fn write_reg(&mut self, channel: StepperChannel, reg: StepperRegister, data: u32) -> Result<(), SPIError> {
        let cs = match channel {
            StepperChannel::Channel1 => &mut self.spi_cs.0,
            StepperChannel::Channel2 => &mut self.spi_cs.1,
            StepperChannel::Channel3 => &mut self.spi_cs.2,
            StepperChannel::Channel4 => &mut self.spi_cs.3,
        };
        cs.set_low();
        let address: u8 = reg.into();
        let send_data = [
            address,
            // terrible hack, make helper function
            ((data & 0xFF000000) >> 24) as u8,
            ((data & 0xFF0000) >> 16) as u8,
            ((data & 0xFF00) >> 8) as u8,
            (data & 0xFF) as u8
        ];
        self.spi.write(&send_data).map_err(STM32G4xxStepperDriver::map_spi_error)?;
        cs.set_high();
        Ok(())
    }

    fn get_temperature(&mut self, channel: StepperChannel) -> Celsius {
        // Voltage divider: 3.3V VDD (= VDDA), 10k thermistor (XH103) = R(T), 2.2k resistor
        // ADC reading / 2^16 = 2.2k / (R(T) + 2.2k)
        // resistance of thermistor at temperature T, R(T) = R(T_0) * exp(B * (1/T − 1/T_0) )
        // T = absolute temperature, B = B-constant, T_0 = 273.15 + 25 K, R(T_0) = 10 kOhm
        const CELSIUS_OFFSET: f32 = 273.15;
        const T_0_RECIPROCAL: f32 = 1.0 / (CELSIUS_OFFSET + 25.0);
        // using the (25/85 deg C) B-constant
        const B_RECIPROCAL: f32 = 1.0 / 3434.0;
        const R_T_0: f32 = 10_000.0;
        const R_FIXED: f32 = 2200.0;
        let adc_reading = match channel {
            StepperChannel::Channel1 => self.adc.convert(&self.temp.0, SampleTime::Cycles_640_5),
            StepperChannel::Channel2 => self.adc.convert(&self.temp.1, SampleTime::Cycles_640_5),
            StepperChannel::Channel3 => self.adc.convert(&self.temp.2, SampleTime::Cycles_640_5),
            StepperChannel::Channel4 => self.adc.convert(&self.temp.3, SampleTime::Cycles_640_5),
        };
        let reading_proportion_reciprocal: f32 = 65536.0 / (adc_reading as f32);
        let t_reciprocal = B_RECIPROCAL * Libm::<f32>::log(R_FIXED * (reading_proportion_reciprocal - 1.0) / R_T_0) + T_0_RECIPROCAL;
        Celsius(1.0 / t_reciprocal - CELSIUS_OFFSET)
    }
}

pub fn init() -> (STM32G4xxCyphalClock, STM32G4xxGeneralClock, STM32G4xxCanDriver, STM32G4xxLEDDriver, I2CDriver, STM32G4xxStepperDriver) {
    // Embedded boilerplate...
    let mut dp = hal::stm32::Peripherals::take().unwrap();
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
    let diag_pin = gpioa.pa10.into_input();

    let sck = gpioc.pc10.into_alternate();
    let miso = gpioc.pc11.into_alternate();
    let mosi = gpioc.pc12.into_alternate();
    let cs0 = gpiob.pb9.into_push_pull_output();
    let cs1 = gpioc.pc13.into_push_pull_output();
    let cs2 = gpioc.pc14.into_push_pull_output();
    let cs3 = gpioc.pc15.into_push_pull_output();
    let cs_pins = StepperCSPins(cs0.into(), cs1.into(), cs2.into(), cs3.into());
    let spi_pins = (sck, miso, mosi);

    cyphal_clock.start();

    let mut general_clock = STM32G4xxGeneralClock::new();

    let stepper_driver = STM32G4xxStepperDriver::new(dp.ADC12_COMMON, dp.ADC2, dp.SPI3, &mut general_clock, &mut rcc, temp_pins, enn_pin, clk_pin, diag_pin, spi_pins, cs_pins);

    (cyphal_clock, general_clock, can_driver, led_driver, I2CDriver {}, stepper_driver)

    // TODO: remaining communication interfaces.
    // I2C1: pin PA15, PB7
    // SPI: PC10, PC11, PC12, 
}
