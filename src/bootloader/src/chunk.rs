use canadensis::core::transport::TransferId;
use wzrd_core::CHUNK_SIZE;

use crate::common::Chunk;

pub const TRANSFER_SIZE: usize = 256;

#[derive(Default)]
enum ChunkletState<T: TransferId + PartialEq> {
    #[default]
    Empty,
    Transferring(T),
    Filled,
}

pub struct ChunkManager<T: TransferId + PartialEq> {
    data: Chunk,
    chunklets: [ChunkletState<T>; CHUNK_SIZE / TRANSFER_SIZE],
}

impl<T: TransferId + PartialEq> ChunkManager<T> {
    pub fn new() -> Self {
        Self {
            data: [0; CHUNK_SIZE],
            chunklets: Default::default(),
        }
    }

    pub fn complete(&self) -> bool {
        self.chunklets.iter().all(|x| {
            if let ChunkletState::Filled = x {
                true
            } else {
                false
            }
        })
    }

    pub fn start_transfer<F>(&mut self, cb: F) -> bool
    where
        F: FnOnce(usize) -> T,
    {
        for i in 0..self.chunklets.len() {
            if let ChunkletState::Empty = self.chunklets[i] {
                self.chunklets[i] = ChunkletState::Transferring(cb(i));
                return true;
            }
        }

        false
    }

    pub fn end_transfer(&mut self, id: &T, data: &[u8]) -> bool {
        for i in 0..self.chunklets.len() {
            if let ChunkletState::Transferring(x) = &self.chunklets[i]
                && x == id
            {
                let start_index = i * TRANSFER_SIZE;
                (&mut self.data[start_index..(start_index + TRANSFER_SIZE)])
                    .copy_from_slice(&data[0..TRANSFER_SIZE]);
                self.chunklets[i] = ChunkletState::Filled;
                return true;
            }
        }

        false
    }

    pub fn extract_data(&mut self) -> Option<Chunk> {
        if !self.complete() {
            return None;
        }

        for i in 0..self.chunklets.len() {
            self.chunklets[i] = ChunkletState::Empty;
        }
        return Some(self.data);
    }
}

pub struct DoubleBuffer<T> {
    state: bool,
    buf_a: T,
    buf_b: T,
}

impl<T> DoubleBuffer<T> {
    pub fn new(a: T, b: T) -> Self {
        Self {
            state: false,
            buf_a: a,
            buf_b: b,
        }
    }

    pub fn switch(&mut self) {
        self.state = !self.state;
    }

    pub fn front(&self) -> &T {
        if self.state { &self.buf_b } else { &self.buf_a }
    }

    pub fn front_mut(&mut self) -> &mut T {
        if self.state {
            &mut self.buf_b
        } else {
            &mut self.buf_a
        }
    }

    pub fn back(&self) -> &T {
        if self.state { &self.buf_a } else { &self.buf_b }
    }

    pub fn back_mut(&mut self) -> &mut T {
        if self.state {
            &mut self.buf_a
        } else {
            &mut self.buf_b
        }
    }
}
