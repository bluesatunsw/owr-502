use core::arch::breakpoint;

use embedded_common::dprintln;
use stm32g4xx_hal::flash::{FlashSize, FlashWriter, Parts};

use crate::chunk_flasher::Flash;

pub struct IflashSys<'a> {
    writer: FlashWriter<'a, 128>,
}

impl<'a> IflashSys<'a> {
    pub fn new(hw_flash: &'a mut Parts) -> Self {
        Self {
            writer: hw_flash.writer(FlashSize::Sz128K),
        }
    }
}

impl<'a> Flash for IflashSys<'a> {
    const BLOCK_SIZE: usize = 4096;

    fn write(&mut self, block: &[u8; Self::BLOCK_SIZE], addr: usize) {
        breakpoint();
        // Kinda bad since this is blocking
        self.writer.page_erase(addr as u32).unwrap();
        // The first 32K of flash is reserved by the bootloader
        self.writer.write(0x8000 + addr as u32, block, false).unwrap();
    }

    fn busy(&mut self) -> bool {
        false
    }

    fn enable_write(&mut self) {}

    fn disable_write(&mut self) {}
}
