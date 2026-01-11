use core::usize;

use wzrd_core::{CHUNK_SIZE, FlashLocation};

use crate::{common::{Chunk, LocatedChunk}, qspi::QspiSys};

struct WorkingChunk {
    data: Chunk,
    base: usize,
    offset: usize,
}

impl WorkingChunk {
    pub fn null() -> Self {
        Self {
            data: [0; CHUNK_SIZE],
            base: 0,
            offset: usize::MAX,
        }
    }

    pub fn from_chunk(chunk: Chunk, address: usize) -> Self {
        Self {
            data: chunk,
            base: address,
            offset: 0,
        }
    }

    pub fn next<const N: usize>(&mut self) -> Option<([u8; N], usize)> {
        if self.done() {
            return None;
        }

        let res = Some((
            self.data[self.offset..self.offset+N].try_into().unwrap(),
            (self.base + self.offset) / N,
        ));

        self.offset += N;

        res
    }

    pub fn done(&self) -> bool {
        self.offset >= CHUNK_SIZE 
    }
}

pub struct FlashHandler<'a> {
    current_location: FlashLocation,
    current_chunk: WorkingChunk,
    qspi_sys: &'a mut QspiSys,
    disabled: bool,
    dirty: bool,
}

pub enum FlashHandlerState {
    Disabled,
    Idle,
    Busy,
}

impl<'a> FlashHandler<'a> {
    pub fn new(qspi_sys: &'a mut QspiSys) -> Self {
        let mut sys = Self {
            current_location: FlashLocation::Internal,
            current_chunk: WorkingChunk::null(),
            qspi_sys: qspi_sys,
            disabled: true,
            dirty: true,
        };
        sys.disable();
        sys
    }

    pub fn reset(&mut self) {
        if !self.dirty {
            return;
        }

        self.dirty = false;
        self.disabled = false;
        self.current_chunk = WorkingChunk::null();
        while self.write_in_progress() {}
        self.qspi_sys.chip_erase();
    }

    pub fn tick(&mut self) {
        if self.write_in_progress() {
            return;
        }

        match self.current_location {
            FlashLocation::Internal => { },
            FlashLocation::External => match self.current_chunk.next() {
                Some((data, index)) => {
                    self.qspi_sys.page_program(index.try_into().unwrap(), &data);
                },
                None => {}
            },
        }
    }

    pub fn feed(&mut self, chunk: LocatedChunk) {
        assert!(self.current_chunk.done());
        self.dirty = true;

        self.current_location = chunk.location;
        self.current_chunk = WorkingChunk::from_chunk(chunk.data, chunk.offset);
    }

    fn write_in_progress(&mut self) -> bool {
        self.qspi_sys.write_in_progress()
    }

    pub fn state(&mut self) -> FlashHandlerState {
        if self.disabled {
            FlashHandlerState::Disabled
        } else if self.current_chunk.done() && !self.write_in_progress() {
            FlashHandlerState::Idle
        } else {
            FlashHandlerState::Busy
        }
    }

    pub fn disable(&mut self) {
        self.disabled = true;
        // Wait for outstanding writes
        while self.write_in_progress() {}
        self.qspi_sys.enabled_mapping();
    }
}
