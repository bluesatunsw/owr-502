use bitfield_struct::bitfield;
use cortex_m::asm::delay;
use stm32g4xx_hal::{
    gpio::{AnyPin, Output, AF6, PC10, PC11, PC12},
    pac::SPI3,
    prelude::SpiBus,
    rcc::Rcc,
    spi::{self, Spi, SpiExt, MODE_3},
    time::RateExtU32,
};

use crate::stm32g4xx::tmc_registers::Register;

#[derive(Debug, Clone, Copy)]
pub enum Channel {
    _0,
    _1,
    _2,
    _3,
}

#[bitfield(u8)]
pub struct SpiFlags {
    #[bits(1)]
    pub reset_flag: bool,

    #[bits(1)]
    pub driver_error: bool,

    #[bits(1)]
    pub sg2: bool,

    #[bits(1)]
    pub standstill: bool,

    #[bits(1)]
    pub velocity_reached: bool,

    #[bits(1)]
    pub position_reached: bool,

    #[bits(1)]
    pub status_stop_l: bool,

    #[bits(1)]
    pub status_stop_r: bool,
}

// SCK, MISO, MOSI
pub type StepperSpiPins = (PC10<AF6>, PC11<AF6>, PC12<AF6>);

pub struct StepperNcsPins(
    pub AnyPin<Output>,
    pub AnyPin<Output>,
    pub AnyPin<Output>,
    pub AnyPin<Output>,
);

pub struct StepperBus {
    spi_bus: Spi<SPI3, StepperSpiPins>,
    ncs_pins: StepperNcsPins,
}

impl StepperBus {
    pub const ALL_CHANNELS: [Channel; 4] = [Channel::_0, Channel::_1, Channel::_2, Channel::_3];

    pub fn new(
        spi_bus: SPI3,
        spi_pins: StepperSpiPins,
        ncs_pins: StepperNcsPins,
        rcc: &mut Rcc,
    ) -> Self {
        Self {
            spi_bus: spi_bus.spi(spi_pins, MODE_3, 1.MHz(), rcc),
            ncs_pins: ncs_pins,
        }
    }

    /// Helper to add some delay between CS transitions
    fn set_ncs(&mut self, channel: Channel, high: bool) {
        let pin = match channel {
            Channel::_0 => &mut self.ncs_pins.0,
            Channel::_1 => &mut self.ncs_pins.1,
            Channel::_2 => &mut self.ncs_pins.2,
            Channel::_3 => &mut self.ncs_pins.3,
        };
        delay(1024);
        if high {
            pin.set_high();
        } else {
            pin.set_low();
        }
        delay(1024);
    }

    fn pack_frame(addr: u8, value: u32) -> [u8; 5] {
        let mut frame = [0u8; 5];
        frame[0] = addr;
        frame[1..4].copy_from_slice(&value.to_le_bytes());
        frame
    }

    fn unpack_frame(frame: [u8; 5]) -> (u8, u32) {
        (
            frame[0],
            u32::from_le_bytes(*frame[1..4].as_array().unwrap()),
        )
    }

    pub fn write_reg<R: Register>(
        &mut self,
        channel: Channel,
        value: R,
    ) -> Result<SpiFlags, spi::Error> {
        self.set_ncs(channel, false);
        let mut res = [0u8; 5];
        // Set the MSB to indicate a write
        self.spi_bus
            .transfer(&mut res, &Self::pack_frame(R::ADDRESS & 0x80, value.into()))?;
        self.set_ncs(channel, true);

        Ok(SpiFlags(res[0]))
    }
    // TMC5160 SPI interface sends data back on the *subsequent* read. A fun exercise would be to
    // write a pipelined API to optimise read chains using the Typestate pattern. However, we don't
    // actually do any reads in any significant quantity, so this probably wouldn't be worth it.
    pub fn read_reg<R: Register>(&mut self, channel: Channel) -> Result<(SpiFlags, R), spi::Error> {
        self.set_ncs(channel, false);
        self.spi_bus.write(&Self::pack_frame(R::ADDRESS, 0))?;
        // embedded_hal gotcha: writes may return BEFORE they've actually written out all data,
        // so we need to flush before setting CS high again.
        // It is annoying that this line is as verbose as it is...
        <spi::Spi<SPI3, StepperSpiPins> as SpiBus<u8>>::flush(&mut self.spi_bus)?;
        self.set_ncs(channel, true);

        self.set_ncs(channel, false);
        let mut res = [0u8; 5];
        self.spi_bus.transfer(&mut res, &[0u8; 5])?;
        self.set_ncs(channel, true);

        let (flags, value) = Self::unpack_frame(res);
        Ok((SpiFlags(flags), R::from(value)))
    }
}
