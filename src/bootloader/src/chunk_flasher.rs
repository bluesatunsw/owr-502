use core::arch::breakpoint;

use embedded_common::dprintln;
use wzrd_core::CHUNK_SIZE;

use crate::common::Chunk;

pub trait Flash {
    const BLOCK_SIZE: usize;

    fn write(&mut self, block: &[u8; Self::BLOCK_SIZE], addr: usize);
    fn busy(&mut self) -> bool;
    fn enable_write(&mut self);
    fn disable_write(&mut self);
}

pub struct ChunkFlasher<F: Flash>
where
    [(); F::BLOCK_SIZE]: Sized,
{
    flash: F,
    data: Chunk,
    base: usize,
    offs: usize,
}

impl<F: Flash> ChunkFlasher<F>
where
    [(); F::BLOCK_SIZE]: Sized,
{
    pub fn new(flash: F) -> Self {
        Self {
            flash,
            data: [0; CHUNK_SIZE],
            base: 0,
            offs: usize::MAX,
        }
    }

    pub fn write(&mut self, chunk: Chunk, addr: usize) {
        assert!(self.done());

        self.data = chunk;
        self.base = addr;
        self.offs = 0;
    }

    /// Returns true if ready
    pub fn tick(&mut self) -> bool {
        if self.flash.busy() {
            return false;
        }
        if self.offs >= CHUNK_SIZE {
            return true;
        }

        self.flash.write(
            self.data[self.offs..self.offs + F::BLOCK_SIZE]
                .try_into()
                .unwrap(),
            self.base + self.offs,
        );
        self.offs += F::BLOCK_SIZE;

        false
    }

    pub fn done(&mut self) -> bool {
        self.offs >= CHUNK_SIZE && !self.flash.busy()
    }

    pub fn enable_write(&mut self) {
        self.flash.enable_write();
    }

    pub fn disable_write(&mut self) {
        assert!(self.done());
        self.flash.disable_write();
    }
}
