//! I2C driver, used to talk to the EEPROM (ZD24C128A) and IMU (ISM330).

use crate::boards::{
    I2CDriver, I2CError, ISM330Register
};

use hal::{pac, gpio, rcc::Rcc, time::RateExtU32, i2c};
use stm32g4xx_hal::{self as hal, i2c::I2cExt, prelude::I2c};

type SdaPin = gpio::PB7<gpio::AF4<gpio::OpenDrain>>;
type SclPin = gpio::PA15<gpio::AF4<gpio::OpenDrain>>;

const EEPROM_ADDR: u8 = 0b1010_000;
const IMU_ADDR: u8 = 0b1101_010;

pub struct STM32G4xxI2CDriver {
    i2c_bus: i2c::I2c<pac::I2C1, SdaPin, SclPin>,
}

impl STM32G4xxI2CDriver {
    pub fn new(i2c1: pac::I2C1, sda: SdaPin, scl: SclPin, rcc: &mut Rcc) -> Self {
        // there are config options for the analog/digital noise filters...? not gunna bother
        // try using fast mode unless it doesn't work for whatever reason
        // NOTE: may need to do custom timing config to make sure 1.3 us L / 0.6 ns H min times met?
        // will probably still work with pure square wave (1.25 us L)
        let i2c_bus = i2c1.i2c(sda, scl, 400.kHz(), rcc);

        Self {
            i2c_bus,
        }
    }

    fn map_i2c_err(_err: i2c::Error) -> I2CError {
        // i mean they're all technically bus errors, no?
        I2CError::BusError
    }
}

impl I2CDriver for STM32G4xxI2CDriver {
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
        Ok((data[1] as u16) << 8 + data[0])
    }
}
