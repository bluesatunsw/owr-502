//! Implementations of some specific low-level drivers for stepper boards Rev. A2, B2 (STM32G474) and
//! the WeAct dev board (STM32G431). The 32-bit microsecond clock and FDCAN driver have whole
//! modules to themselves.

use cfg_if::cfg_if;
use libm::{self, Libm};
use cortex_m_semihosting::hprintln;

use crate::boards::{
    CyphalClock, RGBLEDDriver, RGBLEDColor,
    StepperDriver, StepperChannel, StepperRegister, SPIError, TMCFlags,
    Celsius, Radians
};
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
    adc::{Adc, AdcClaim, AdcCommonExt, config::{AdcConfig, SampleTime, ClockMode, ClockSource, Prescaler}, Configured},
    spi,
};
use stm32g4xx_hal as hal;   // don't need to put this in a cfg_if because which board it targets is
                            // specified as a feature in Cargo.toml
use embedded_io::Write;

// TODO: set up TRACESWO

// Some constants for configuration.

/// Gear ratio between the stepper motor shaft and the shaft actually being driven.
const GEAR_RATIO: u32 = 60;
/// Number of microsteps used. Must be a power of two no greater than 256.
const MICROSTEPS_PER_STEP: u32 = 2;
/// If this is enabled, we run the stepper with reduced TMC5160 features to make it less likely to trip the
/// overcurrent protection on the power supply. This shouldn't be necessary in production.
const SAFE_MODE: bool = true;
/// The number of RGB LEDs (WS2812s) on the board.
pub const NUM_LEDS: usize = 6;
/// The number of stepper motors that the board is actually controlling, to avoid dealing with
/// motors that don't exist.
pub const NUM_STEPPERS: usize = 4;
/// Inverts the position convention for the motors.
const INVERT_STEPPER_DIR: [bool; 4] = [false, false, false, false];
/// Inverts the position convention for the encoders.
const INVERT_ENCODER_DIR: [bool; 4] = [false, false, false, false];
/// What encoder reading we interpret as an angular position of 0 for ROS purposes.
const ENCODER_ZERO: Radians = Radians(0.0);

const CHANNELS: [StepperChannel; 4] = [
    StepperChannel::Channel1,
    StepperChannel::Channel2,
    StepperChannel::Channel3,
    StepperChannel::Channel4
];

/// Helper function to add some short delays around changes in CS pin state for SPI.
fn set_cs(pin: &mut gpio::AnyPin<gpio::Output>, high: bool) {
    for _ in 0..500 {
        if high {
            pin.set_high();
        } else {
            pin.set_low();
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

type EnnPin = gpio::PD2<gpio::Output>;
type ClkPin = gpio::PA8<gpio::Analog>; // MCO

#[cfg(feature = "rev_a2")]
type DiagPin = gpio::PA10<gpio::Output>;
#[cfg(not(feature = "rev_a2"))]
type DiagPin = gpio::PA10<gpio::Input>;

struct StepperTempPins(gpio::PA0, gpio::PA1, gpio::PB2, gpio::PC5);
struct StepperCSPins(gpio::AnyPin<gpio::Output>, gpio::AnyPin<gpio::Output>, gpio::AnyPin<gpio::Output>, gpio::AnyPin<gpio::Output>);
struct EncoderCSPins(gpio::AnyPin<gpio::Output>, gpio::AnyPin<gpio::Output>, gpio::AnyPin<gpio::Output>, gpio::AnyPin<gpio::Output>);

// SCK, MISO, MOSI
type StepperSPIPins = (gpio::PC10<gpio::AF6>, gpio::PC11<gpio::AF6>, gpio::PC12<gpio::AF6>);
type EncoderSPIPins = (gpio::PB13<gpio::AF5>, gpio::PB14<gpio::AF5>, gpio::PB15<gpio::AF5>);

#[allow(non_camel_case_types)]
enum EncoderRegister {
    ERRFL = 0x0001,
    PROG = 0x0003,
    ZPOSM = 0x0016,
    ZPOSL = 0x0017,
    SETTINGS1 = 0x0018,
    SETTINGS2 = 0x0019,
    DIAAGC = 0x3FFC,
    MAG = 0x3FFD,
    ANGLEUNC = 0x3FFE,
    ANGLECOM = 0x3FFF,
}

/// TMC5160 stepper driver (over SPI).

pub struct STM32G4xxStepperDriver {
    adc: Adc<pac::ADC2, Configured>,
    enn: EnnPin,
    _clk: rcc::Mco<gpio::PA8<gpio::AF0>>,
    #[allow(dead_code)]
    diag: DiagPin,
    temp: StepperTempPins,
    step_spi: spi::Spi<pac::SPI3, StepperSPIPins>,
    enc_spi: spi::Spi<pac::SPI2, EncoderSPIPins>,
    step_spi_cs: StepperCSPins,
    enc_spi_cs: EncoderCSPins,
    position_mode: bool,
}

impl STM32G4xxStepperDriver {
    fn new(
        adc_common: pac::ADC12_COMMON,
        adc2: pac::ADC2,
        spi2: pac::SPI2,
        spi3: pac::SPI3,
        uclock: &mut STM32G4xxGeneralClock,
        rcc: &mut Rcc,
        temp: StepperTempPins,
        enn: EnnPin,
        clk_pin: ClkPin,
        mut diag: DiagPin,
        stepper_spi_pins: StepperSPIPins,
        step_spi_cs: StepperCSPins,
        encoder_spi_pins: EncoderSPIPins,
        enc_spi_cs: EncoderCSPins,
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
        cfg_if! {
            // Pinout is wrong on Rev. A2 (DIAG and CLK swapped),
            // so just use the TMC5160's internal clock.
            if #[cfg(feature = "rev_a2")] {
                diag.set_low();
            } else {
                clk.enable();
            }
        }

        // initialise the TMC5160 SPI bus, SPI3
        let step_spi = spi3.spi(stepper_spi_pins, spi::MODE_3, 2.MHz(), rcc);

        // initialise the AS5047P encoder SPI bus, SPI2
        let enc_spi = spi2.spi(encoder_spi_pins, spi::MODE_1, 2.MHz(), rcc);

        // Would like to do stepper and encoder configuration here but would have to rearrange the
        // code a fair bit to make this possible. Still, right now it's a bit too easy to pass
        // steppers to the main code unconfigured...

        STM32G4xxStepperDriver {
            adc,
            enn,
            _clk: clk,
            diag,
            temp,
            step_spi,
            enc_spi,
            step_spi_cs,
            enc_spi_cs,
            position_mode: true,
        }
    }

    fn init_steppers_config(&mut self) {
        // based on Evan's bringup code
        for i in 0..NUM_STEPPERS {
            let channel = CHANNELS[i];
            let do_invert = INVERT_STEPPER_DIR[i];

            // reset flags
            self.write_reg(channel, StepperRegister::GSTAT, 0x00000007).unwrap();

            // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
            if do_invert {
                // shaft = 1
                self.write_reg(channel, StepperRegister::GCONF, 0x00000014).unwrap();
            } else {
                self.write_reg(channel, StepperRegister::GCONF, 0x00000004).unwrap();
            }
            // reduce current to 0x20/0x100 = 1/8th of maximum current (which is 15 A peak) = ~2A
            // this is the minimum we can reduce it to... still draws peaks above this!!!
            self.write_reg(channel, StepperRegister::GLOBALSCALER, 0x00000020).unwrap();
            // SHORT_CONF: FET short detection lowest sensitivity, SHORTFILTER 3 us, normal shortdelay
            self.write_reg(channel, StepperRegister::SHORT_CONF, 0x00030F0F).unwrap();
            // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)
            // also disable short protection -- it seems to be very sensitive
            self.write_reg(channel, StepperRegister::CHOPCONF, 0xC70100C3).unwrap();
            // COOLCONF: StallGuard minimum sensitivity
            // self.write_reg(channel, StepperRegister::COOLCONF, 0x003F0000).unwrap();
            // IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
            self.write_reg(channel, StepperRegister::IHOLD_IRUN, 0x00000801).unwrap();
            // TPOWERDOWN=10: Delay before power down in stand still
            self.write_reg(channel, StepperRegister::TPOWERDOWN, 0x00000000).unwrap();
            // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
            self.write_reg(channel, StepperRegister::TPWM_THRS, 0x000001F4).unwrap();

            // re-zero position (XACTUAL, XTARGET)
            self.write_reg(channel, StepperRegister::RAMPMODE, 0x00000003).unwrap(); // hold mode
            self.write_reg(channel, StepperRegister::XACTUAL, 0x00000000).unwrap();
            self.write_reg(channel, StepperRegister::XTARGET, 0x00000000).unwrap();
            self.write_reg(channel, StepperRegister::RAMPMODE, 0x00000000).unwrap();

            // some basic motion profiling defaults
            self.write_reg(channel, StepperRegister::A1, 400 * MICROSTEPS_PER_STEP).unwrap();
            self.write_reg(channel, StepperRegister::V1, 300 * MICROSTEPS_PER_STEP).unwrap();
            self.write_reg(channel, StepperRegister::AMAX, 20 * MICROSTEPS_PER_STEP).unwrap();
            self.write_reg(channel, StepperRegister::VMAX, 1000 * MICROSTEPS_PER_STEP).unwrap();
            self.write_reg(channel, StepperRegister::DMAX, 20 * MICROSTEPS_PER_STEP).unwrap();
            self.write_reg(channel, StepperRegister::D1, 100 * MICROSTEPS_PER_STEP).unwrap();
            self.write_reg(channel, StepperRegister::VSTOP, 100).unwrap();
            self.write_reg(channel, StepperRegister::RAMPMODE, 0x00000000).unwrap();
        }
    }

    fn map_spi_error(err: spi::Error) -> SPIError {
        match err {
            spi::Error::Overrun => SPIError::Overrun,
            _ => SPIError::Other,
        }
    }

    // Much like the TMC5160, we could potentially set up a pipelined read API. However, I don't
    // think we actually do chained reads in any significant quantity, so bah humbug.
    fn read_enc_reg(&mut self, channel: StepperChannel, reg: EncoderRegister) -> Result<u16, SPIError> {
        let cs = match channel {
            StepperChannel::Channel1 => &mut self.enc_spi_cs.0,
            StepperChannel::Channel2 => &mut self.enc_spi_cs.1,
            StepperChannel::Channel3 => &mut self.enc_spi_cs.2,
            StepperChannel::Channel4 => &mut self.enc_spi_cs.3,
        };
        // 16-bit: MSB is parity bit (even), bit 14 is 1 for read
        let addr = reg as u32;
        let parity = ((addr.count_ones() + 1) % 2) << 7;
        let top_byte: u8 = (((addr & 0x3F00) >> 8) | 0x40 | parity) as u8;
        let send_data: [u8; 2] = [top_byte, (addr & 0xFF) as u8];
        let mut read_data = [0u8; 2];
        set_cs(cs, false);
        self.enc_spi.write(&send_data).map_err(STM32G4xxStepperDriver::map_spi_error)?;
        <spi::Spi<pac::SPI2, EncoderSPIPins> as SpiBus<u8>>::flush(&mut self.enc_spi)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        set_cs(cs, false);
        self.enc_spi.read(&mut read_data).map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        // reconstruct register
        Ok((read_data[0] as u16) << 8 | (read_data[1] as u16))
    }

    /// Returns the old data
    fn write_enc_reg(&mut self, channel: StepperChannel, reg: EncoderRegister, data: u16) -> Result<u16, SPIError> {
        let cs = match channel {
            StepperChannel::Channel1 => &mut self.enc_spi_cs.0,
            StepperChannel::Channel2 => &mut self.enc_spi_cs.1,
            StepperChannel::Channel3 => &mut self.enc_spi_cs.2,
            StepperChannel::Channel4 => &mut self.enc_spi_cs.3,
        };
        let addr = reg as u32;
        let addr_parity = (addr.count_ones() % 2) << 7;
        let addr_top_byte: u8 = (((addr & 0x3F00) >> 8) | addr_parity) as u8;
        let addr_data: [u8; 2] = [addr_top_byte, (addr & 0xFF) as u8];
        let data_parity: u16 = (((data & 0x3FFF).count_ones() % 2) << 7) as u16;
        let send_data: [u8; 2] = [((data & 0x3F00) >> 8 | data_parity) as u8, (data & 0xFF) as u8];
        let mut read_data = [0u8; 2];
        set_cs(cs, false);
        self.enc_spi.write(&addr_data).map_err(STM32G4xxStepperDriver::map_spi_error)?;
        <spi::Spi<pac::SPI2, EncoderSPIPins> as SpiBus<u8>>::flush(&mut self.enc_spi).
            map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        set_cs(cs, false);
        self.enc_spi.transfer(&mut read_data, &send_data).map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        Ok((read_data[0] as u16) << 8 | (read_data[1] as u16))
    }

    fn dbg_pretty_diaagc(&mut self, channel: StepperChannel) {
        let diaagc = self.read_enc_reg(channel, EncoderRegister::DIAAGC).unwrap();
        let magl = diaagc & (1 << 11) > 0;
        let magh = diaagc & (1 << 10) > 0;
        let cof = diaagc & (1 << 9) > 0;
        let lf = diaagc & (1 << 8) > 0;
        let agc = diaagc & 0xFF;
        hprintln!("MAGL {} MAGH {} COF {} LF {} AGC {}", magl, magh, cof, lf, agc);
    }

    fn dbg_pretty_settings(&mut self, channel: StepperChannel) {
        let settings1 = self.read_enc_reg(channel, EncoderRegister::SETTINGS1).unwrap();
        let dir = settings1 & 0x04 > 0;
        let daecdis = settings1 & 0x10 > 0;
        let datasel = settings1 & 0x40 > 0;
        hprintln!("SETTINGS1 0x{:04x} = DIR {} DAECDIS {} DATASEL {}", settings1, dir, daecdis, datasel);
        let settings2 = self.read_enc_reg(channel, EncoderRegister::SETTINGS2).unwrap();
        let hys = (settings2 & 0x18) >> 3;
        let abires = (settings2 & 0xE0) >> 5;
        hprintln!("SETTINGS2 0x{:04x} = HYS {} ABIRES {}", settings2, hys, abires);
    }

    fn dgb_pretty_data(&mut self, channel: StepperChannel) {
        let mag = self.read_enc_reg(channel, EncoderRegister::MAG).unwrap();
        let angleunc = self.read_enc_reg(channel, EncoderRegister::ANGLEUNC).unwrap();
        let anglecom = self.read_enc_reg(channel, EncoderRegister::ANGLECOM).unwrap();
        hprintln!("MAG 0x{:04x} ANGLEUNC 0x{:04x} ANGLECOM 0x{:04x}", mag, angleunc, anglecom);
    }

    fn init_encoders_config(&mut self) {
        for i in 0..NUM_STEPPERS {
            let channel = CHANNELS[i];
            let do_invert = INVERT_ENCODER_DIR[i];

            let mut settings1 = 0x01;
            // normally CW from topside. if do_invert, CCW from topside (CW from underside)
            if do_invert {
                settings1 |= 0x04;
            }
            // set hysteresis to minimum
            settings1 |= 0x18;
            // TODO: other fields??
            self.write_enc_reg(channel, EncoderRegister::SETTINGS1, settings1).unwrap();
            // TODO: other registers??
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

    fn set_position(&mut self, channel: StepperChannel, target: Radians) -> Result<(), SPIError> {
        const PI: f32 = 3.14159;
        const MICROSTEPS_PER_REV: u32 = 200 * MICROSTEPS_PER_STEP;
        if !self.position_mode {
            self.write_reg(channel, StepperRegister::RAMPMODE, 0x00000000).unwrap();
            self.position_mode = true;
        }
        let target_usteps: i32 = Libm::<f32>::round((target.0) * ((GEAR_RATIO * MICROSTEPS_PER_REV) as f32) / (PI * 2.0)) as i32;
        self.write_reg(channel, StepperRegister::XTARGET, target_usteps as u32).unwrap();
        Ok(())
    }

    // TMC5160 SPI interface sends data back on the *subsequent* read. A fun exercise would be to
    // write a pipelined API to optimise read chains using the Typestate pattern. However, we don't
    // actually do any reads in any significant quantity, so this probably wouldn't be worth it.
    fn read_reg(&mut self, channel: StepperChannel, reg: StepperRegister) -> Result<(u32, TMCFlags), SPIError> {
        let cs = match channel {
            StepperChannel::Channel1 => &mut self.step_spi_cs.0,
            StepperChannel::Channel2 => &mut self.step_spi_cs.1,
            StepperChannel::Channel3 => &mut self.step_spi_cs.2,
            StepperChannel::Channel4 => &mut self.step_spi_cs.3,
        };
        set_cs(cs, false);
        // 40-bit (5 byte), first byte is address, MSB of first byte is 0 for read
        let address: u8 = reg.into();
        let send_data = [address, 0u8, 0u8, 0u8, 0u8];
        let mut read_data = [0u8; 5];
        self.step_spi.write(&send_data).map_err(STM32G4xxStepperDriver::map_spi_error)?;
        // embedded_hal gotcha: writes may return BEFORE they've actually written out all data,
        // so we need to flush before setting CS high again.
        // It is annoying that this line is as verbose as it is...
        <spi::Spi<pac::SPI3, StepperSPIPins> as SpiBus<u8>>::flush(&mut self.step_spi)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        set_cs(cs, false);
        self.step_spi.read(&mut read_data).map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        // reconstruct register
        Ok((
            (read_data[1] as u32) << 24 | (read_data[2] as u32) << 16 | (read_data[3] as u32) << 8 | (read_data[4] as u32),
            TMCFlags(read_data[0])
        ))
    }

    fn write_reg(&mut self, channel: StepperChannel, reg: StepperRegister, data: u32) -> Result<TMCFlags, SPIError> {
        let cs = match channel {
            StepperChannel::Channel1 => &mut self.step_spi_cs.0,
            StepperChannel::Channel2 => &mut self.step_spi_cs.1,
            StepperChannel::Channel3 => &mut self.step_spi_cs.2,
            StepperChannel::Channel4 => &mut self.step_spi_cs.3,
        };
        let address: u8 = (reg as u8) + 0x80;
        let send_data = [
            address,
            // terrible, but we do this exactly once so no point making helper
            ((data & 0xFF000000) >> 24) as u8,
            ((data & 0xFF0000) >> 16) as u8,
            ((data & 0xFF00) >> 8) as u8,
            (data & 0xFF) as u8
        ];
        let mut read_data = [0u8; 5];
        set_cs(cs, false);
        self.step_spi.transfer(&mut read_data, &send_data).map_err(STM32G4xxStepperDriver::map_spi_error)?;
        <spi::Spi<pac::SPI3, StepperSPIPins> as SpiBus<u8>>::flush(&mut self.step_spi)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        Ok(TMCFlags(read_data[0]))
    }

    // TODO: this yields very incorrect values, debug
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

    fn adjust(&mut self, channel: StepperChannel) -> Result<(), SPIError> {
        let xtarget = self.read_reg(channel, StepperRegister::XTARGET)?;
        let xactual = self.read_reg(channel, StepperRegister::XACTUAL)?;
        if xtarget.0 != xactual.0 {
            // the motor is still rotating to carry out a command
            return Ok(());
        }
        // Read position from encoder
        let raw_pos = self.read_enc_reg(channel, EncoderRegister::ANGLECOM)?; // need to figure out
                                                                              // if this is best
                                                                              // register
        let actual_pos: Radians = Radians(0.0); // TODO
        // let expected_xactual = ...
        // if expected_xactual != xactual.0 {
        // self.write_reg(channel, StepperRegister::XACTUAL, expected_xactual);
        // log error to debug
        // }
        Ok(())
    }
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
