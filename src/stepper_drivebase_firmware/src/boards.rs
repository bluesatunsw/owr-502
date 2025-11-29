// Wraps implementations of board-specific low-level drivers for various interfaces into common
// abstract drivers for main.rs.

use canadensis::core::time as time;
use core::convert::From;
use cfg_if::cfg_if;

pub trait CyphalClock: time::Clock {
    fn start(&mut self);
}

pub trait GeneralClock {
    fn now(&self) -> time::Microseconds32;
}

#[derive(Copy, Clone)]
pub struct RGBLEDColor {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

#[derive(Copy, Clone)]
pub struct I2CAxis {
    pub x: f32,
    pub y: f32,
    pub z: f32
}

#[derive(Debug)]
pub enum I2CError {
    BusError,
    Timeout,
    InvalidAddress,
    WriteError,
    ReadError,
    ImuNotReady,
    InvalidRegister,
}

#[derive(Debug)]
pub enum SPIError {
    Overrun,
    Other,
}

impl RGBLEDColor {
    pub fn default() -> Self {
        RGBLEDColor {
            red: 0,
            green: 0,
            blue: 0
        }
    }
}

#[derive(Debug)]
pub struct Celsius(f32);

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

pub trait RGBLEDDriver {
    // n is zero-indexed. Panics if n is greater than the number of LEDs on the board.
    // This function does NOT change the display state. Call render() to actually send
    // the new color signals to the LEDs.
    fn set_nth_led(&mut self, n: usize, color: RGBLEDColor);

    // Syncs the LED display state with the internal color state.
    // There isn't any meaningful way we can tell if this fails.
    fn render(&mut self);

    // for convenience
    fn set_nth_led_and_render(&mut self, n: usize, color: RGBLEDColor);
}

#[allow(non_camel_case_types)]
pub enum ISM330Register {
    // temperature
    OUT_TEMP_L = 0x20,
    OUT_TEMP_H = 0x21,
    // Angular rate sensor pitch axis (X) angular rate output register
    OUTX_L_G = 0x22,
    OUTX_H_G = 0x23,
    // Angular rate sensor roll axis (Y) angular rate output register
    OUTY_L_G = 0x24,
    OUTY_H_G = 0x25,
    // Angular rate sensor pitch yaw (Z) angular rate output register
    OUTZ_L_G = 0x26,
    OUTZ_H_G = 0x27,
    // Linear acceleration sensor X-axis output register
    OUTX_L_A = 0x28,
    OUTX_H_A = 0x29,
    // Linear acceleration sensor Y-axis output register
    OUTY_L_A = 0x2A,
    OUTY_H_A = 0x2B,
    // Linear acceleration sensor Z-axis output register
    OUTZ_L_A = 0x2C,
    OUTZ_H_A = 0x2D,
    // TODO: the rest, or scrap entirely
}

pub trait I2CDriver {
    fn enable_fast_mode(&mut self);
    fn disable_fast_mode(&mut self);

    fn eeprom_read(&mut self, address: u16, data: &mut [u8]) -> Result<(), I2CError>;
    fn eeprom_write(&mut self, address: u16, data: &[u8]) -> Result<(), I2CError>;
    // Underlying operations: byte write, page write, current address read, random read, sequential
    // read. Identification page functions not implemented.

    fn imu_read_reg(&mut self, reg: ISM330Register) -> Result<u8, I2CError>;
    fn imu_write_reg(&mut self, reg: ISM330Register, data: u8) -> Result<u8, I2CError>;

    fn imu_read_16bitreg(&mut self, low_reg: ISM330Register, high_reg: ISM330Register) -> Result<f32, I2CError> {
        let low = self.imu_read_reg(low_reg)? as u16;
        let high = self.imu_read_reg(high_reg)? as u16;
        let raw = ((high << 8) | low) as i16;
        Ok(raw as f32)
    }

    // TODO: use bitfields internally
    fn imu_read_temperature(&mut self) -> Result<Celsius, I2CError> {
        let raw = self.imu_read_16bitreg(ISM330Register::OUT_TEMP_L, ISM330Register::OUT_TEMP_H)? as u16;
        Ok(Celsius(25.0 + (raw as f32) / 256.0))
    }

    fn imu_read_gyro(&mut self) -> Result<I2CAxis, I2CError> {
        let x = self.imu_read_16bitreg(ISM330Register::OUTX_L_G, ISM330Register::OUTX_H_G)?;
        let y = self.imu_read_16bitreg(ISM330Register::OUTY_L_G, ISM330Register::OUTY_H_G)?;
        let z = self.imu_read_16bitreg(ISM330Register::OUTZ_L_G, ISM330Register::OUTZ_H_G)?;
        Ok(I2CAxis {
            x: x,
            y: y,
            z: z
        })
    }

    fn imu_read_accel(&mut self) -> Result<I2CAxis, I2CError> {
        let x = self.imu_read_16bitreg(ISM330Register::OUTX_L_A, ISM330Register::OUTX_H_A)?;
        let y = self.imu_read_16bitreg(ISM330Register::OUTY_L_A, ISM330Register::OUTY_H_A)?;
        let z = self.imu_read_16bitreg(ISM330Register::OUTZ_L_A, ISM330Register::OUTZ_H_A)?;
        Ok(I2CAxis {
            x: x,
            y: y,
            z: z
        })
    }
}

pub trait QSPIDriver {
    // TODO
}

#[allow(non_camel_case_types)]
pub enum StepperRegister {
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
    GLOBAL_SCALER = 0x0B,
    OFFSET_READ = 0x0C,

    TPOWERDOWN = 0x11,
    TPWM_THRS = 0x13,

    // velocity-dependent
    IHOLD_IRUN = 0x10,
    XACTUAL = 0x21,
    A1 = 0x24,
    V1 = 0x25,
    AMAX = 0x26,
    VMAX = 0x27,
    DMAX = 0x28,
    D1 = 0x2A,
    VSTOP = 0x2B,
    XTARGET = 0x2D,

    // etc.
    
    // ramp generator
    RAMPMODE = 0x20,
    // TODO: the rest, or scrap this entirely
    CHOPCONF = 0x6C,
    COOLCONF = 0x6D,
}

impl Into<u8> for StepperRegister {
    fn into(self) -> u8 {
        self as u8
    }
}

// you can rename these to more helpfully refer to the physical function/location of each motor
pub enum StepperChannel {
    Channel1,
    Channel2,
    Channel3,
    Channel4
}

pub trait StepperDriver {
    // initialisation (not part of public interface): set up clock and send initialisation commands to steppers
    // SD_MODE = 0, SPI_MODE = 1

    // fn stepper_cfg(channel: u8, config: StepperConfig);

    fn enable_all(&mut self);
    fn disable_all(&mut self);

    // also want a config function for VMAX, AMAX etc.
    // could set a callback on position reached???
    fn set_position(&mut self, channel: StepperChannel, target: u32) -> Result<(), SPIError>; // in what units? TODO: proper type
    fn set_velocity(&mut self, channel: StepperChannel, velocity: i32) -> Result<(), SPIError>;

    fn read_reg(&mut self, channel: StepperChannel, reg: StepperRegister) -> Result<u32, SPIError>;
    fn write_reg(&mut self, channel: StepperChannel, reg: StepperRegister, data: u32) -> Result<(), SPIError>;

    fn get_temperature(&mut self, channel: StepperChannel) -> Celsius; // via ADC
    // calibration? boundary setting?
}

cfg_if! {
    if #[cfg(any(feature = "rev_a2", feature = "rev_b2", feature = "we_act_dev"))] {
        mod stm32g4xx;

        // these types need to be exposed to the layer that creates the canadensis/Cyphal node
        pub type CClock = stm32g4xx::STM32G4xxCyphalClock;
        pub type CanDriver = stm32g4xx::STM32G4xxCanDriver;

        pub fn init() -> (CClock, stm32g4xx::STM32G4xxGeneralClock, CanDriver, stm32g4xx::STM32G4xxLEDDriver, stm32g4xx::I2CDriver, stm32g4xx::STM32G4xxStepperDriver) {
            stm32g4xx::init()
        }
    } else if #[cfg(feature = "bigtree")] {
        mod stm32g0xx;

        // these types need to be exposed to the layer that creates the canadensis/Cyphal node
        pub type CClock = stm32g0xx::STM32G0xxCyphalClock;
        pub type CanDriver = stm32g0xx::STM32G0xxCanDriver;

        pub fn init() -> (CClock, stm32g0xx::STM32G0xxGeneralClock, CanDriver, stm32g0xx::STM32G0xxLEDDriver, stm32g0xx::I2CDriver, stm32g0xx::STM32G0xxStepperDriver) {
            stm32g0xx::init()
        }

    } else {
        compile_error!("The board that the firmware is being compiled for must be specified as a feature");
    }
}
