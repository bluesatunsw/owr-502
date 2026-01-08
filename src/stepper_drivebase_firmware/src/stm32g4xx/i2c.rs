//! I2C driver, used to talk to the EEPROM (ZD24C128A) and IMU (ISM330).

use stm32g4xx_hal::{self as hal, i2c::I2cExt, prelude::I2c};

use hal::{pac, gpio, rcc::Rcc, time::RateExtU32, i2c};

use crate::boards::Celsius;

type SdaPin = gpio::PB7<gpio::AF4<gpio::OpenDrain>>;
type SclPin = gpio::PA15<gpio::AF4<gpio::OpenDrain>>;

const EEPROM_ADDR: u8 = 0b1010_000;
const IMU_ADDR: u8 = 0b1101_010;

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

#[derive(Copy, Clone)]
pub struct I2CAxis {
    pub x: f32,
    pub y: f32,
    pub z: f32
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

    // this list is non-exhaustive, add extra registers if you need
    WHO_AM_I = 0x0F,
}

pub struct STM32G4xxI2CDriver {
    i2c_bus: i2c::I2c<pac::I2C1, SdaPin, SclPin>,
}

impl STM32G4xxI2CDriver {
    pub fn new(i2c1: pac::I2C1, sda: SdaPin, scl: SclPin, rcc: &mut Rcc) -> Self {
        // there are config options for the analog/digital noise filters...? not gunna bother
        // try using fast mode unless it doesn't work for whatever reason
        // NOTE: may need to do custom timing config to make sure 1.3 us L / 0.6 ns H min times met?
        // will probably still work with pure square wave (1.25 us L)
        let i2c_bus = i2c1.i2c((sda, scl), 400.kHz(), rcc);

        Self {
            i2c_bus,
        }
    }

    fn map_i2c_err(_err: i2c::Error) -> I2CError {
        // i mean they're all technically bus errors, no?
        I2CError::BusError
    }
    
    fn eeprom_read(&mut self, address: u16, data: &mut [u8]) -> Result<(), I2CError> {
        // sequential read starting with random address read
        self.i2c_bus.write_read(EEPROM_ADDR, &[((address & 0x3F00) >> 8) as u8, (address & 0xFF) as u8], data)
            .map_err(STM32G4xxI2CDriver::map_i2c_err)
    }

    // TODO: test this
    fn eeprom_write(&mut self, address: u16, data: &[u8]) -> Result<(), I2CError> {
        const PAGE_SIZE: usize = 64;
        // pages are 64-byte aligned
        // (yes i know this is overkill since we're mostly peeking and poking in practice)
        let mut bytes_written: usize = 0;
        let address_words = [((address & 0x3F00) >> 8) as u8, (address & 0xFF) as u8];
        // write up to the next page boundary
        let bytes_left_in_page = PAGE_SIZE - ((address & 0x3F) as usize);
        if bytes_left_in_page >= data.len() {
            // all done
            let send_bytes = [&address_words, data].concat();
            return self.i2c_bus.write(EEPROM_ADDR, &send_bytes).map_err(STM32G4xxI2CDriver::map_i2c_err);
        } else {
            let send_bytes = [&address_words, &data[0..bytes_left_in_page]].concat();
            self.i2c_bus.write(EEPROM_ADDR, &send_bytes).map_err(STM32G4xxI2CDriver::map_i2c_err)?;
            bytes_written += bytes_left_in_page;
        }
        // write full pages
        let mut address_high: u8 = 1u8.wrapping_add(((address & 0x3F00) >> 8) as u8);
        while bytes_written + (PAGE_SIZE as usize) <= data.len() {
            let address_words = [address_high as u8, 0x00u8];
            let send_bytes = [&address_words, &data[bytes_written..(bytes_written + PAGE_SIZE)]].concat();
            self.i2c_bus.write(EEPROM_ADDR, &send_bytes).map_err(STM32G4xxI2CDriver::map_i2c_err)?;
            address_high = address_high.wrapping_add(1);
            bytes_written += PAGE_SIZE;
        }
        // write remaining bytes
        if bytes_written < data.len() {
            let address_words = [address_high as u8, 0x00u8];
            let send_bytes = [&address_words, &data[bytes_written..data.len()]].concat();
            self.i2c_bus.write(EEPROM_ADDR, &send_bytes).map_err(STM32G4xxI2CDriver::map_i2c_err)?;
        }
        Ok(())
    }

    // TODO: test these also
    fn imu_read_reg(&mut self, reg: ISM330Register) -> Result<u8, I2CError> {
        let mut data = [0u8];
        self.i2c_bus.write_read(IMU_ADDR, &[reg as u8], &mut data)
            .map_err(STM32G4xxI2CDriver::map_i2c_err)?;
        Ok(data[0])
    }

    fn imu_write_reg(&mut self, reg: ISM330Register, data: u8) -> Result<(), I2CError> {
        let send_data = [reg as u8, data];
        self.i2c_bus.write(IMU_ADDR, &send_data)
            .map_err(STM32G4xxI2CDriver::map_i2c_err)?;
        Ok(())
    }

    fn imu_read_16bitreg(&mut self, low_reg: ISM330Register, high_reg: ISM330Register) -> Result<u16, I2CError> {
        let low_reg = low_reg as u8;
        if low_reg + 1 != high_reg as u8 {
            panic!("Tried to read a pair of non-consecutive registers as a 16-bit register");
        }
        let mut data = [0u8, 0u8];
        self.i2c_bus.write_read(IMU_ADDR, &[low_reg], &mut data)
            .map_err(STM32G4xxI2CDriver::map_i2c_err)?;
        Ok((data[1] as u16) << (8 + data[0]))
    }

        fn imu_read_temperature(&mut self) -> Result<Celsius, I2CError> {
        let raw = self.imu_read_16bitreg(ISM330Register::OUT_TEMP_L, ISM330Register::OUT_TEMP_H)? as u16;
        Ok(Celsius(25.0 + (raw as f32) / 256.0))
    }

    fn imu_read_gyro(&mut self) -> Result<I2CAxis, I2CError> {
        let x = self.imu_read_16bitreg(ISM330Register::OUTX_L_G, ISM330Register::OUTX_H_G)? as i16;
        let y = self.imu_read_16bitreg(ISM330Register::OUTY_L_G, ISM330Register::OUTY_H_G)? as i16;
        let z = self.imu_read_16bitreg(ISM330Register::OUTZ_L_G, ISM330Register::OUTZ_H_G)? as i16;
        Ok(I2CAxis {
            x: x as f32,
            y: y as f32,
            z: z as f32
        })
    }

    fn imu_read_accel(&mut self) -> Result<I2CAxis, I2CError> {
        let x = self.imu_read_16bitreg(ISM330Register::OUTX_L_A, ISM330Register::OUTX_H_A)? as i16;
        let y = self.imu_read_16bitreg(ISM330Register::OUTY_L_A, ISM330Register::OUTY_H_A)? as i16;
        let z = self.imu_read_16bitreg(ISM330Register::OUTZ_L_A, ISM330Register::OUTZ_H_A)? as i16;
        Ok(I2CAxis {
            x: x as f32,
            y: y as f32,
            z: z as f32
        })
    }

}
