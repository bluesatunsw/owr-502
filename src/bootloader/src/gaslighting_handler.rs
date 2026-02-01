use wzrd_core::FlashLocation;

use crate::{
    common::{FlashCommand, LocatedChunk},
    flash_data::{DATA_0, DATA_1, DATA_2, DATA_3},
};

pub struct GaslightingHandler {
    index: usize,
}

impl GaslightingHandler {
    pub fn new() -> Self {
        Self { index: 0 }
    }

    pub fn poll(&mut self) -> FlashCommand {
        self.index += 1;
        match self.index {
            1 => FlashCommand::Start,
            2 => FlashCommand::Write(LocatedChunk {
                data: DATA_0,
                location: FlashLocation::External,
                offset: 0x0000,
            }),
            3 => FlashCommand::Write(LocatedChunk {
                data: DATA_1,
                location: FlashLocation::External,
                offset: 0x1000,
            }),
            4 => FlashCommand::Write(LocatedChunk {
                data: DATA_2,
                location: FlashLocation::External,
                offset: 0x2000,
            }),
            5 => FlashCommand::Write(LocatedChunk {
                data: DATA_3,
                location: FlashLocation::Internal,
                offset: 0x0000,
            }),
            6 => FlashCommand::Finish,
            _ => FlashCommand::None,
        }
    }
}
