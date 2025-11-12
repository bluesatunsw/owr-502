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

impl RGBLEDColor {
    pub fn default() -> Self {
        RGBLEDColor {
            red: 0,
            green: 0,
            blue: 0
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

pub enum ASM330Register {
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

    fn imu_read_reg(&mut self, reg: ASM330Register) -> Result<u8, I2CError>;
    fn imu_write_reg(&mut self, reg: ASM330Register, data: u8) -> Result<u8, I2CError>;

    fn imu_read_axis(&mut self, low_reg: ASM330Register, high_reg: ASM330Register) -> Result<f32, I2CError> {
        let low = self.imu_read_reg(ASM330Register::low_reg)? as u16;
        let high = self.imu_read_reg(ASM330Register::high_reg)? as u16;
        let raw = ((high << 8) | low) as i16;
        Ok(raw as f32);
    }

    // TODO: use bitfields internally
    fn imu_read_temperature(&mut self) -> Result<f32, I2CError> {
        let low = self.imu_read_reg(ASM330Register::OUT_TEMP_L)? as u16;
        let high = self.imu_read_reg(ASM330Register::OUT_TEMP_H)? as u16;
        let raw = ((high << 8) | low) as i16;
        Ok(25.0 + (raw as f32) / 256.0)
    }

    fn imu_read_gyro(&mut self) -> Result<(f23, f32, f32), I2CError> {
        let x = self.imu_read_axis(ASM330Register::OUTX_L_G, ASM330Register::OUTX_H_G)?;
        let y = self.imu_read_axis(ASM330Register::OUTY_L_G, ASM330Register::OUTY_H_G)?;
        let z = self.imu_read_axis(ASM330Register::OUTZ_L_G, ASM330Register::OUTZ_H_G)?;
        Ok((x, y, z));
    }

    fn imu_read_accel(&mut self) -> Result<(), I2CError> {
        let x = self.imu_read_axis(ASM330Register::OUTX_L_A, ASM330Register::OUTX_H_A)?;
        let y = self.imu_read_axis(ASM330Register::OUTY_L_A, ASM330Register::OUTY_H_A)?;
        let z = self.imu_read_axis(ASM330Register::OUTZ_L_A, ASM330Register::OUTZ_H_A)?;
        Ok((x, y, z));
    }
}

pub trait QSPIDriver {
    // TODO
}

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

    // velocity-dependent
    IHOLD_IRUN = 0x10,
    // etc.
    
    // ramp generator
    RAMPMODE = 0x20,
    // TODO: the rest, or scrap this entirely
}

pub trait StepperDriver {
    // initialisation (not part of public interface): set up clock and send initialisation commands to steppers
    // SD_MODE = 0, SPI_MODE = 1

    // fn stepper_cfg(channel: u8, config: StepperConfig);

    fn enable_all();
    fn disable_all();

    // also want a config function for VMAX, AMAX etc.
    fn set_position(channel: u8, target: u32); // in what units? TODO: proper type
    fn set_velocity(channel: u8, velocity: i32);

    fn read_reg(channel: u8, reg: StepperRegister) -> u32;
    fn write_reg(channel: u8, reg: StepperRegister, data: u32);

    fn get_temperature(channel: u8) -> u16; // via ADC
    // calibration? boundary setting?
}

cfg_if! {
    if #[cfg(any(feature = "rev_a2", feature = "rev_b2", feature = "we_act_dev"))] {
        mod stm32g4xx;

        // these types need to be exposed to the layer that creates the canadensis/Cyphal node
        pub type CClock = stm32g4xx::STM32G4xxCyphalClock;
        pub type CanDriver = stm32g4xx::STM32G4xxCanDriver;

        pub fn init() -> (CClock, stm32g4xx::STM32G4xxGeneralClock, CanDriver, stm32g4xx::STM32G4xxLEDDriver, stm32g4xx::I2CDriver) {
            stm32g4xx::init()
        }
    } else {
        compile_error!("The board that the firmware is being compiling for must be specified as a feature");
    }
}
