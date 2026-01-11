use wzrd_core::{CHUNK_SIZE, FlashLocation};

pub type Chunk = [u8; CHUNK_SIZE];

pub struct LocatedChunk {
    pub data: Chunk,
    pub location: FlashLocation,
    pub offset: usize,
}
