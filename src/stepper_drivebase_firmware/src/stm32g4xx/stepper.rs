//! Implementations to do with the stepper motor drivers (and related parts) themselves.
//! This is actually three peripherals in a trenchcoat:
//! - ADC2 (for reading the voltages off the thermistors for temperature estimation),
//! - SPI3 (for communicating with the TMC5160/A stepper drivers), and
//! - SPI1 (for communicating with the off-board AS5047P encoders).
use stm32g4xx_hal as hal;

use cortex_m_semihosting::hprintln;
use libm::{self, Libm};

use hal::{
    adc::{
        config::{AdcConfig, ClockMode, ClockSource, Prescaler, SampleTime},
        Adc, AdcClaim, AdcCommonExt, Configured,
    },
    gpio,
    pac,
    prelude::*,
    rcc::{self, Rcc},
    spi,
    time::RateExtU32, // for auto .MHz()
};

use crate::boards::{Celsius, Radians};
use crate::stm32g4xx::STM32G4xxGeneralClock;

// Types

#[derive(Debug)]
pub enum SPIError {
    Overrun,
    Other,
}

#[derive(Copy, Clone)]
pub enum StepperChannel {
    Channel1,
    Channel2,
    Channel3,
    Channel4,
}

pub struct TMCFlags(u8);

// Some constants for configuration.

const PI: f32 = 3.14159;

/// Gear ratio between the stepper motor shaft and the shaft actually being driven.
const GEAR_RATIO: u32 = 2;
/// Number of microsteps used. Must be a power of two no greater than 256.
const MICROSTEPS_PER_STEP: u32 = 1;
/// Number of microsteps per motor revolution.
const MICROSTEPS_PER_REV: u32 = 200 * MICROSTEPS_PER_STEP;
/// The number of stepper motors that the board is actually controlling, to avoid dealing with
/// motors that don't exist.
pub const NUM_STEPPERS: usize = 4;
/// Inverts the position convention for the motors.
const INVERT_STEPPER_DIR: [bool; 4] = [false, false, false, false];
/// Inverts the position convention for the encoders.
const INVERT_ENCODER_DIR: [bool; 4] = [true, false, false, false];
/// What encoder reading we interpret as an angular position of 0 for ROS purposes.
const ENCODER_ZERO: Radians = Radians(0.0);

// you can rename these to more helpfully refer to the physical function/location of each motor
const CHANNELS: [StepperChannel; 4] = [
    StepperChannel::Channel1,
    StepperChannel::Channel2,
    StepperChannel::Channel3,
    StepperChannel::Channel4,
];

/// Helper function to add some short delays around changes in CS pin state for SPI.
fn set_cs(pin: &mut gpio::AnyPin<gpio::Output>, high: bool) {
    for _ in 0..50 {
        if high {
            pin.set_high();
        } else {
            pin.set_low();
        }
    }
}

type EnnPin = gpio::PD2<gpio::Output>;
type ClkPin = gpio::PA8<gpio::Analog>; // MCO

type DiagPin = gpio::PA10<gpio::Input>;

pub struct StepperTempPins(pub gpio::PA0, pub gpio::PA1, pub gpio::PB2, pub gpio::PC5);
pub struct StepperCSPins(
    pub gpio::AnyPin<gpio::Output>,
    pub gpio::AnyPin<gpio::Output>,
    pub gpio::AnyPin<gpio::Output>,
    pub gpio::AnyPin<gpio::Output>,
);
pub struct EncoderCSPins(
    pub gpio::AnyPin<gpio::Output>,
    pub gpio::AnyPin<gpio::Output>,
    pub gpio::AnyPin<gpio::Output>,
    pub gpio::AnyPin<gpio::Output>,
);

// SCK, MISO, MOSI
type StepperSPIPins = (
    gpio::PC10<gpio::AF6>,
    gpio::PC11<gpio::AF6>,
    gpio::PC12<gpio::AF6>,
);
type EncoderSPIPins = (
    gpio::PB13<gpio::AF5>,
    gpio::PB14<gpio::AF5>,
    gpio::PB15<gpio::AF5>,
);

// the below two enums are just useful to have during development for reference/debugging
#[allow(non_camel_case_types, dead_code)]
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

#[allow(non_camel_case_types, dead_code)]
enum StepperRegister {
    // global
    GCONF = 0x00,
    GSTAT = 0x01,
    IFCNT = 0x02,
    NODECONF = 0x03,
    IOIN_OUTPUT = 0x04, // function depends on whether reading or writing
    X_COMPARE = 0x05,
    OTP_PROG = 0x06,
    OTP_READ = 0x07,
    FACTORY_CONF = 0x08,
    SHORT_CONF = 0x09,
    DRV_CONF = 0x0A,
    GLOBALSCALER = 0x0B,
    OFFSET_READ = 0x0C,

    TPOWERDOWN = 0x11,
    TPWM_THRS = 0x13,

    // velocity-dependent
    IHOLD_IRUN = 0x10,
    XACTUAL = 0x21,
    VSTART = 0x23,
    A1 = 0x24,
    V1 = 0x25,
    AMAX = 0x26,
    VMAX = 0x27,
    DMAX = 0x28,
    D1 = 0x2A,
    VSTOP = 0x2B,
    XTARGET = 0x2D,

    // ramp generator
    RAMPMODE = 0x20,
    CHOPCONF = 0x6C,
    COOLCONF = 0x6D,
    DRV_STATUS = 0x6F,
    // non-exhaustive, add any more registers you need
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
    pub fn new(
        adc_common: pac::ADC12_COMMON,
        adc2: pac::ADC2,
        spi2: pac::SPI2,
        spi3: pac::SPI3,
        uclock: &mut STM32G4xxGeneralClock,
        rcc: &mut Rcc,
        temp: StepperTempPins,
        enn: EnnPin,
        clk_pin: ClkPin,
        diag: DiagPin, // can be made mut
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
            rcc,
        );
        let adc = adc12.claim_and_configure(adc2, AdcConfig::default(), uclock);

        // initialise the MCO (CLK pin)
        // at 12 MHz
        let clk = clk_pin.mco(rcc::MCOSrc::HSE, rcc::Prescaler::Div2, rcc);
        // Pinout is wrong on Rev. A2 (DIAG and CLK swapped),
        // so just use the TMC5160's internal clock.
        clk.enable();

        // initialise the TMC5160 SPI bus, SPI3
        let step_spi = spi3.spi(stepper_spi_pins, spi::MODE_3, 2.MHz(), rcc);

        // initialise the AS5047P encoder SPI bus, SPI2
        let enc_spi = spi2.spi(encoder_spi_pins, spi::MODE_1, 2.MHz(), rcc);

        // Would like to do stepper and encoder configuration here but would have to rearrange the
        // code a fair bit to make this possible. Still, right now it's a bit too easy to pass
        // steppers to the main code unconfigured... TODO fix?

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

    // TMC5160 SPI interface sends data back on the *subsequent* read. A fun exercise would be to
    // write a pipelined API to optimise read chains using the Typestate pattern. However, we don't
    // actually do any reads in any significant quantity, so this probably wouldn't be worth it.
    #[allow(dead_code)]
    fn read_reg(
        &mut self,
        channel: StepperChannel,
        reg: StepperRegister,
    ) -> Result<(u32, TMCFlags), SPIError> {
        let cs = match channel {
            StepperChannel::Channel1 => &mut self.step_spi_cs.0,
            StepperChannel::Channel2 => &mut self.step_spi_cs.1,
            StepperChannel::Channel3 => &mut self.step_spi_cs.2,
            StepperChannel::Channel4 => &mut self.step_spi_cs.3,
        };
        set_cs(cs, false);
        // 40-bit (5 byte), first byte is address, MSB of first byte is 0 for read
        let address: u8 = reg as u8;
        let send_data = [address, 0u8, 0u8, 0u8, 0u8];
        let mut read_data = [0u8; 5];
        self.step_spi
            .write(&send_data)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        // embedded_hal gotcha: writes may return BEFORE they've actually written out all data,
        // so we need to flush before setting CS high again.
        // It is annoying that this line is as verbose as it is...
        <spi::Spi<pac::SPI3, StepperSPIPins> as SpiBus<u8>>::flush(&mut self.step_spi)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        set_cs(cs, false);
        self.step_spi
            .transfer(&mut read_data, &send_data)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        // reconstruct register
        Ok((
            (read_data[1] as u32) << 24
                | (read_data[2] as u32) << 16
                | (read_data[3] as u32) << 8
                | (read_data[4] as u32),
            TMCFlags(read_data[0]),
        ))
    }

    fn write_reg(
        &mut self,
        channel: StepperChannel,
        reg: StepperRegister,
        data: u32,
    ) -> Result<TMCFlags, SPIError> {
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
            (data & 0xFF) as u8,
        ];
        let mut read_data = [0u8; 5];
        set_cs(cs, false);
        self.step_spi
            .transfer(&mut read_data, &send_data)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        <spi::Spi<pac::SPI3, StepperSPIPins> as SpiBus<u8>>::flush(&mut self.step_spi)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        Ok(TMCFlags(read_data[0]))
    }

    pub fn init_steppers_config(&mut self) {
        // based on Evan's bringup code
        for i in 0..NUM_STEPPERS {
            let channel = CHANNELS[i];
            let do_invert = INVERT_STEPPER_DIR[i];

            // reset flags
            self.write_reg(channel, StepperRegister::GSTAT, 0x00000007)
                .unwrap();

            // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
            if do_invert {
                // shaft = 1
                self.write_reg(channel, StepperRegister::GCONF, 0x00000014)
                    .unwrap();
            } else {
                self.write_reg(channel, StepperRegister::GCONF, 0x00000004)
                    .unwrap();
            }
            // reduce current to 0x20/0x100 = 1/8th of maximum current (which is 15 A peak) = ~2A
            // this is the minimum we can reduce it to... still draws peaks above this!!!
            // self.write_reg(channel, StepperRegister::GLOBALSCALER, 0x00000020).unwrap();
            let gs = 64u32; // 32 to 255
            self.write_reg(channel, StepperRegister::GLOBALSCALER, gs)
                .unwrap();
            // SHORT_CONF: FET short detection moderate sensitivity, SHORTFILTER 1 us, normal shortdelay
            self.write_reg(channel, StepperRegister::SHORT_CONF, 0x00010606)
                .unwrap();
            // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)
            // ensure that short protection is enabled to prevent damage ー breaks with the bench PSU
            let microsteps_pattern = match MICROSTEPS_PER_STEP {
                1 => 0x08000000,
                2 => 0x07000000,
                4 => 0x06000000,
                8 => 0x05000000,
                16 => 0x04000000,
                32 => 0x03000000,
                64 => 0x02000000,
                128 => 0x01000000,
                256 => 0x00000000,
                _ => unreachable!("MICROSTEPS_PER_STEP is a power of two not greater than 256"),
            };
            self.write_reg(
                channel,
                StepperRegister::CHOPCONF,
                0x000100C3 | microsteps_pattern,
            )
            .unwrap();
            // COOLCONF: StallGuard minimum sensitivity
            // self.write_reg(channel, StepperRegister::COOLCONF, 0x003F0000).unwrap();
            // IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
            let ihold = 0u32;
            let irun = 31u32 << 8; // 0..=31
            let iholddelay = 6u32 << 16;
            self.write_reg(
                channel,
                StepperRegister::IHOLD_IRUN,
                ihold | irun | iholddelay,
            )
            .unwrap();
            // TPOWERDOWN=10: Delay before power down in stand still
            self.write_reg(channel, StepperRegister::TPOWERDOWN, 0x00000000)
                .unwrap();
            // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
            self.write_reg(channel, StepperRegister::TPWM_THRS, 0x000001F4)
                .unwrap();

            // re-zero position (XACTUAL, XTARGET)
            self.write_reg(channel, StepperRegister::RAMPMODE, 0x00000003)
                .unwrap(); // hold mode
            self.write_reg(channel, StepperRegister::XACTUAL, 0x00000000)
                .unwrap();
            self.write_reg(channel, StepperRegister::XTARGET, 0x00000000)
                .unwrap();
            self.write_reg(channel, StepperRegister::RAMPMODE, 0x00000000)
                .unwrap();

            // motion profiling
            // NOTE: A1 = 50, V1 = 500, AMAX = 30, VMAX = 2500, DMAX = 30, D1 = <nonzero>, VSTOP = 100
            // (with MICROSTEPS_PER_STEP multiplier for all except VSTOP)
            // was found to work pretty well for MICROSTEPS_PER_STEP = {1, 4}
            // on the basic lab power supply for the actual rover wheel (as of 7 Dec 2025) with no load,
            // given all the current settings (GLOBALSCALER, IRUN) were set sufficiently high.
            // Just documenting this for future tuning efforts.
            self.write_reg(channel, StepperRegister::A1, 50 * MICROSTEPS_PER_STEP)
                .unwrap();
            self.write_reg(channel, StepperRegister::V1, 500 * MICROSTEPS_PER_STEP)
                .unwrap();
            self.write_reg(channel, StepperRegister::AMAX, 30 * MICROSTEPS_PER_STEP)
                .unwrap();
            self.write_reg(channel, StepperRegister::VMAX, 2500 * MICROSTEPS_PER_STEP)
                .unwrap();
            self.write_reg(channel, StepperRegister::DMAX, 30 * MICROSTEPS_PER_STEP)
                .unwrap();
            self.write_reg(channel, StepperRegister::D1, 50 * MICROSTEPS_PER_STEP)
                .unwrap();
            self.write_reg(channel, StepperRegister::VSTOP, 100)
                .unwrap();
            self.write_reg(channel, StepperRegister::RAMPMODE, 0x00000000)
                .unwrap();
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
    #[allow(dead_code)]
    fn read_enc_reg(
        &mut self,
        channel: StepperChannel,
        reg: EncoderRegister,
    ) -> Result<u16, SPIError> {
        // TODO: detect parity errors and treat as SPIError
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
        self.enc_spi
            .write(&send_data)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        <spi::Spi<pac::SPI2, EncoderSPIPins> as SpiBus<u8>>::flush(&mut self.enc_spi)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        set_cs(cs, false);
        self.enc_spi
            .read(&mut read_data)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        // reconstruct register
        Ok((read_data[0] as u16) << 8 | (read_data[1] as u16))
    }

    /// Returns the old data
    fn write_enc_reg(
        &mut self,
        channel: StepperChannel,
        reg: EncoderRegister,
        data: u16,
    ) -> Result<u16, SPIError> {
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
        let send_data: [u8; 2] = [
            ((data & 0x3F00) >> 8 | data_parity) as u8,
            (data & 0xFF) as u8,
        ];
        let mut read_data = [0u8; 2];
        set_cs(cs, false);
        self.enc_spi
            .write(&addr_data)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        <spi::Spi<pac::SPI2, EncoderSPIPins> as SpiBus<u8>>::flush(&mut self.enc_spi)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        set_cs(cs, false);
        self.enc_spi
            .transfer(&mut read_data, &send_data)
            .map_err(STM32G4xxStepperDriver::map_spi_error)?;
        set_cs(cs, true);
        Ok((read_data[0] as u16) << 8 | (read_data[1] as u16))
    }

    /// Debug function: prints contents of the encoder's DIAAGC register
    #[allow(dead_code)]
    pub fn dbg_pretty_diaagc(&mut self, channel: StepperChannel) {
        let diaagc = self.read_enc_reg(channel, EncoderRegister::DIAAGC).unwrap();
        let magl = diaagc & (1 << 11) > 0;
        let magh = diaagc & (1 << 10) > 0;
        let cof = diaagc & (1 << 9) > 0;
        let lf = diaagc & (1 << 8) > 0;
        let agc = diaagc & 0xFF;
        hprintln
!(
            "MAGL {} MAGH {} COF {} LF {} AGC {}",
            magl,
            magh,
            cof,
            lf,
            agc
        );
    }

    /// Debug function: prints contents of the encoder's SETTINGS1 and SETTINGS2 registers
    #[allow(dead_code)]
    pub fn dbg_pretty_settings(&mut self, channel: StepperChannel) {
        let settings1 = self
            .read_enc_reg(channel, EncoderRegister::SETTINGS1)
            .unwrap();
        let dir = settings1 & 0x04 > 0;
        let daecdis = settings1 & 0x10 > 0;
        let datasel = settings1 & 0x40 > 0;
        hprintln!(
            "SETTINGS1 0x{:04x} = DIR {} DAECDIS {} DATASEL {}",
            settings1,
            dir,
            daecdis,
            datasel
        );
        let settings2 = self
            .read_enc_reg(channel, EncoderRegister::SETTINGS2)
            .unwrap();
        let hys = (settings2 & 0x18) >> 3;
        let abires = (settings2 & 0xE0) >> 5;
        hprintln!(
            "SETTINGS2 0x{:04x} = HYS {} ABIRES {}",
            settings2,
            hys,
            abires
        );
    }

    /// Debug function: prints contents of the encoder data registers (angle readings)
    #[allow(dead_code)]
    pub fn dbg_pretty_data(&mut self, channel: StepperChannel) {
        let mag = self.read_enc_reg(channel, EncoderRegister::MAG).unwrap();
        let angleunc = self
            .read_enc_reg(channel, EncoderRegister::ANGLEUNC)
            .unwrap();
        let anglecom = self
            .read_enc_reg(channel, EncoderRegister::ANGLECOM)
            .unwrap();
        hprintln!(
            "MAG 0x{:04x} ANGLEUNC 0x{:04x} ANGLECOM 0x{:04x}",
            mag & 0x3FFF,
            angleunc & 0x3FFF,
            anglecom & 0x3FFF
        );
    }

    pub fn init_encoders_config(&mut self) {
        for i in 0..NUM_STEPPERS {
            let channel = CHANNELS[i];
            let do_invert = INVERT_ENCODER_DIR[i];

            let mut settings1 = 0x01;
            // normally CW from topside. if do_invert, CCW from topside (CW from underside)
            if do_invert {
                settings1 |= 0x04;
            }
            // set hysteresis to minimum
            let settings2 = 0x18;
            self.write_enc_reg(channel, EncoderRegister::SETTINGS1, settings1)
                .unwrap();
            self.write_enc_reg(channel, EncoderRegister::SETTINGS2, settings2)
                .unwrap();
        }
    }

    pub fn enable_all(&mut self) {
        self.enn.set_low();
    }

    fn disable_all(&mut self) {
        self.enn.set_high();
    }

    pub fn set_position(
        &mut self,
        channel: StepperChannel,
        target: Radians,
    ) -> Result<(), SPIError> {
        if !self.position_mode {
            self.write_reg(channel, StepperRegister::RAMPMODE, 0x00000000)
                .unwrap();
            self.position_mode = true;
        }
        let target_usteps: i32 = Libm::<f32>::round(
            (target.0) * ((GEAR_RATIO * MICROSTEPS_PER_REV) as f32) / (PI * 2.0),
        ) as i32;
        hprintln!(
            "Moving to position {}, targeet: {}",
            target.0,
            target_usteps
        );

        self.write_reg(channel, StepperRegister::XTARGET, target_usteps as u32)
            .unwrap();
        Ok(())
    }

    // TODO: this yields very incorrect values, debug
    pub fn get_temperature(&mut self, channel: StepperChannel) -> Celsius {
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
        let t_reciprocal = B_RECIPROCAL
            * Libm::<f32>::log(R_FIXED * (reading_proportion_reciprocal - 1.0) / R_T_0)
            + T_0_RECIPROCAL;
        Celsius(1.0 / t_reciprocal - CELSIUS_OFFSET)
    }

    fn adjust(&mut self, channel: StepperChannel) -> Result<(), SPIError> {
        // how many steps XACTUAL must be out of place for a correction to take place
        const MIN_MICROSTEP_DIFFERENCE: i32 = 4;
        let xtarget = self.read_reg(channel, StepperRegister::XTARGET)?;
        let xactual = self.read_reg(channel, StepperRegister::XACTUAL)?;
        if xtarget.0 != xactual.0 {
            // the motor is still rotating to carry out a command
            return Ok(());
        }
        // Read position from encoder
        let raw_pos: f32 = self.read_enc_reg(channel, EncoderRegister::ANGLECOM)? as f32;
        let mut actual_pos = Radians(raw_pos * 2.0 * PI / (0x4000 as f32)) - ENCODER_ZERO;
        if actual_pos.0 > 2.0 * PI {
            actual_pos = Radians(actual_pos.0 - 2.0 * PI);
        }
        let expected_xactual: i32 =
            (actual_pos.0 * (MICROSTEPS_PER_REV as f32) * (GEAR_RATIO as f32) / (2.0 * PI)) as i32;
        let difference_usteps: i32 = expected_xactual - (xactual.0 as i32);
        if difference_usteps > MIN_MICROSTEP_DIFFERENCE
            || difference_usteps < -MIN_MICROSTEP_DIFFERENCE
        {
            self.write_reg(channel, StepperRegister::XACTUAL, expected_xactual as u32)
                .unwrap();
            hprintln!(
                "Corrected XACTUAL from {} to {}",
                xactual.0,
                expected_xactual
            );
        }
        Ok(())
    }

    fn is_busy(&mut self, channel: StepperChannel) -> Result<bool, SPIError> {
        let xtarget = self.read_reg(channel, StepperRegister::XTARGET)?;
        let xactual = self.read_reg(channel, StepperRegister::XACTUAL)?;
        Ok(xtarget.0 != xactual.0)
    }
}
