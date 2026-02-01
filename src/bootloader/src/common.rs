use wzrd_core::{CHUNK_SIZE, FlashLocation, Header};

pub type Chunk = [u8; CHUNK_SIZE];

#[derive(Debug, Clone, Copy)]
pub struct LocatedChunk {
    pub data: Chunk,
    pub location: FlashLocation,
    pub offset: usize,
}

#[derive(Debug, Clone, Copy)]
pub enum FlashCommand {
    None,
    Start,
    Write(LocatedChunk),
    Finish,
}

const HEADER_ADDRESS: usize = 0x9000_0000;

pub unsafe fn get_header() -> Option<Header> {
    Header::deserialize(unsafe { &*(HEADER_ADDRESS as *const [u8; Header::SIZE]) })
}
