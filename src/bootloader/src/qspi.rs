use core::convert::TryInto;

use stm32g4xx_hal::{quadspi::{ClockMode, Command, DdrMode, FlashMode, IoCommand, LineMode, Qspi, QuadSpiExt}, rcc::Rcc};
use stm32g4xx_hal::pac;

use crate::peripherals::*;

pub struct QspiSys {
    hw_swpi: Qspi,
}

impl QspiSys {
    pub fn new(
        qspi: pac::QUADSPI,
        rcc: &mut Rcc,
        ncs_pin: QspiNcsPin,
        clk_pin: QspiClkPin,
        io0_bank1_pin: QspiIo0Bank1Pin,
        io1_bank1_pin: QspiIo1Bank1Pin,
        io2_bank1_pin: QspiIo2Bank1Pin,
        io3_bank1_pin: QspiIo3Bank1Pin,
        io0_bank2_pin: QspiIo0Bank2Pin,
        io1_bank2_pin: QspiIo1Bank2Pin,
        io2_bank2_pin: QspiIo2Bank2Pin,
        io3_bank2_pin: QspiIo3Bank2Pin,
    ) -> QspiSys {
        let mut res = Self {
            hw_swpi: qspi.qspi(
                (
                    clk_pin.into_alternate::<10>(),
                    io0_bank1_pin.into_alternate::<10>(),
                    io1_bank1_pin.into_alternate::<10>(),
                    io2_bank1_pin.into_alternate::<10>(),
                    io3_bank1_pin.into_alternate::<10>(),
                    ncs_pin.into_alternate::<10>(),
                    io0_bank2_pin.into_alternate::<10>(),
                    io1_bank2_pin.into_alternate::<10>(),
                    io2_bank2_pin.into_alternate::<10>(),
                    io3_bank2_pin.into_alternate::<10>()
                ), 
                rcc,
                0, 8, false,
                21, 2, ClockMode::Mode0, FlashMode::Dual
            )
        };

        // Continuous Read Mode Reset
        res.hw_swpi.command(Command::new(DdrMode::Disabled)
            .with_instruction(LineMode::Single, 0xFF)
        );

        // Write Enable for Volatile Status Register
        res.hw_swpi.command(Command::new(DdrMode::Disabled)
            .with_instruction(LineMode::Single, 0x50)
        );

        // Write Status Register (EQ=1)
        res.hw_swpi.command(Command::new(DdrMode::Disabled)
            .with_instruction(LineMode::Single, 0x31)
            .with_alternate_bytes(LineMode::Single, [0x02])
        );

        // Enable QPI
        res.hw_swpi.command(Command::new(DdrMode::Disabled)
            .with_instruction(LineMode::Single, 0x38)
        );

        // Set Read Parameters (Dummy Cycles=10, Wrap Length=8)
        // Since we are running slightly faster than the recommended
        // max speed add additional dummy cycles
        res.hw_swpi.command(Command::new(DdrMode::Disabled)
            .with_instruction(LineMode::Quad, 0xC0)
            .with_alternate_bytes(LineMode::Quad, [0x00])
        );

        // Read Identification Sequence
        let mut id_seq: [u8; 6] = [0,0,0,0,0,0];
        res.hw_swpi.read(IoCommand::new(DdrMode::Disabled, LineMode::Quad)
            .with_instruction(LineMode::Quad, 0x9F),
            &mut id_seq
        );
        if id_seq != [0xBA,0xBA,0x60,0x60,0x15,0x15] {
            panic!("Invalid flash ID sequence: {:x?}", id_seq)
        }

        res
    }

    pub fn write_in_progress(&mut self) -> bool{
        let mut status: [u8; 2] = [0,0];
        self.hw_swpi.read(IoCommand::new(DdrMode::Disabled, LineMode::Quad)
            .with_instruction(LineMode::Quad, 0x05),
            &mut status
        );

        // Check WIP bit for both chips
        (status[0] & 0x01) == 0x01 || (status[1] & 0x01) == 0x01
    }

    pub fn chip_erase(&mut self) {
        // Write Enable
        self.hw_swpi.command(Command::new(DdrMode::Disabled)
            .with_instruction(LineMode::Quad, 0x06)
        );

        // Chip Erase
        self.hw_swpi.command(Command::new(DdrMode::Disabled)
            .with_instruction(LineMode::Quad, 0x60)
        );
    }

    pub fn page_program(&mut self, page_index: u16, data: &[u8; 512]) {
        // Write Enable
        self.hw_swpi.command(Command::new(DdrMode::Disabled)
            .with_instruction(LineMode::Quad, 0x06)
        );

        // Page Program
        self.hw_swpi.write(IoCommand::new(DdrMode::Disabled, LineMode::Quad)
            .with_instruction(LineMode::Quad, 0x02)
            .with_address(LineMode::Quad, [
                (page_index/256).try_into().unwrap(),
                (page_index%256).try_into().unwrap(),
                0
            ]),
            data
        );
    }

    pub fn enabled_mapping(&mut self) {
        self.hw_swpi.memory_mapped(IoCommand::new(DdrMode::Disabled, LineMode::Quad)
            .with_instruction(LineMode::Quad, 0x0B)
            .with_address(LineMode::Quad, [0,0,0])
            .with_dummy_cycles(10)
        );
    }
}
