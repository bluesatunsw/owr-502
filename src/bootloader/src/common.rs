use wzrd_core::{CHUNK_SIZE, FlashLocation, Header};

pub type Chunk = [u8; CHUNK_SIZE];

pub struct LocatedChunk {
    pub data: Chunk,
    pub location: FlashLocation,
    pub offset: usize,
}

pub enum FlashCommand {
    None,
    Start,
    Write(LocatedChunk),
    Finish,
}

const HEADER_ADDRESS: usize = 0x9000_0000;

pub unsafe fn get_header() -> Option<Header> {
    Header::deserialize(unsafe {
        &*(HEADER_ADDRESS as *const [u8; Header::SIZE])
    })
}
