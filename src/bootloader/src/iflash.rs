use crate::flash_handler::Flash;

pub struct IflashSys {

}

impl Flash for IflashSys {
    const BLOCK_SIZE: usize = 4096;

    fn write(&mut self, block: &[u8; Self::BLOCK_SIZE], addr: usize) {
    }

    fn busy(&mut self) -> bool {
        false
    }

    fn enable_write(&mut self) {
        
    }

    fn disable_write(&mut self) {
    }
}
