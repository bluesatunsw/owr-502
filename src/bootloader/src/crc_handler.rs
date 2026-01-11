use core::cell::UnsafeCell;

use cortex_m::interrupt::Mutex;
use cortex_m_rt;
use stm32g4::stm32g474::{CRC, DMA1, interrupt};
use stm32g4xx_hal::dma::{channel::C, traits::Channel};

use cortex_m::interrupt::free as interrupt_free;

pub struct CrcHandler {
    target: u32,
}

#[derive(PartialEq)]
enum CrcResult {
    None,
    NotReady,
    Some(u32),
}

struct CrcState {
    hw_crc: CRC,
    dma_ちゃん: C<DMA1, 0>,
    address: usize,
    result: CrcResult,
}

const BLOCK_SIZE: usize = 32*1024;
// const INTERNAL_START: usize = 0;
const INTERNAL_END: usize = 0;
const EXTERNAL_START: usize = 0;
const EXTERNAL_END: usize = 0;

static STATE: Mutex<UnsafeCell<Option<CrcState>>> = Mutex::new(UnsafeCell::new(None));

#[cortex_m_rt::interrupt]
fn DMA1_CH1() {
    interrupt_free(|cs| unsafe {
        let state = &mut STATE.borrow(cs).as_mut_unchecked().as_mut().unwrap();
        assert!(state.result == CrcResult::NotReady);

        state.dma_ちゃん.disable();
        state.dma_ちゃん.clear_transfer_complete_interrupt();

        if state.address >= INTERNAL_END {
            state.address = EXTERNAL_START;
        }
        if state.address >= EXTERNAL_END {
            state.result = CrcResult::Some(state.hw_crc.dr().read().bits());
            return;
        }

        state.dma_ちゃん.set_memory_address(state.address.try_into().unwrap());
        state.dma_ちゃん.set_number_of_transfers((BLOCK_SIZE/4).try_into().unwrap());
        state.dma_ちゃん.enable();

        state.address += BLOCK_SIZE;
    });
}

pub enum CrcHandlerState {
    Unverified,
    Verifying,
    Verifed,
    Failed,
}

impl CrcHandler {
    pub fn new(target: u32) -> Self {
        Self { target }
    }

    pub fn start(&mut self) {
        interrupt_free(|cs| unsafe {
            let state = STATE.borrow(cs).as_mut_unchecked().as_mut().unwrap();
            state.result = CrcResult::NotReady;
        })
    }

    pub fn stop(&mut self) {
        interrupt_free(|cs| unsafe {
            let state = STATE.borrow(cs).as_mut_unchecked().as_mut().unwrap();
            state.dma_ちゃん.disable();
            state.hw_crc.cr().write(|w| w.reset().reset());
            state.result = CrcResult::None;
        })
    }

    /*
    pub fn result(&self) -> Option<u32> {
        interrupt_free(|cs| unsafe {
            match STATE.borrow(cs).as_mut_unchecked().as_mut().unwrap().result {
                CrcResult::Some(x) => Some(x),
                _ => None,
            }
        })
    } */

    pub fn state(&mut self) -> CrcHandlerState {
        interrupt_free(|cs| unsafe {
            match STATE.borrow(cs).as_mut_unchecked().as_mut().unwrap().result {
                CrcResult::None => CrcHandlerState::Unverified,
                CrcResult::NotReady => CrcHandlerState::Verifying,
                CrcResult::Some(x) => if x == self.target {
                    CrcHandlerState::Verifed
                } else {
                    CrcHandlerState::Failed
                },
            }
        })
    }
}
