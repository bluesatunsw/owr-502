use core::cell::UnsafeCell;

use cortex_m::interrupt::Mutex;
use cortex_m_rt;
use stm32g4::stm32g474::{CRC, interrupt};
use stm32g4xx_hal::dma::{traits::Channel};

use cortex_m::interrupt::free as interrupt_free;
use wzrd_core::CHUNK_SIZE;

use crate::{common::get_header, peripherals::DmaChannel};

pub struct CrcHandler { }

struct CrcState {
    hw_crc: CRC,
    dma_ちゃん: DmaChannel,
    address: usize,
    result: Option<u32>,
}

const EXTERNAL_START: usize = 0x9000_0000;
const INTERNAL_START: usize = 0x0800_8000;

static STATE: Mutex<UnsafeCell<Option<CrcState>>> = Mutex::new(UnsafeCell::new(None));

#[cortex_m_rt::interrupt]
fn DMA1_CH1() {
    interrupt_free(|cs| unsafe {
        let state = &mut STATE.borrow(cs).as_mut_unchecked().as_mut().unwrap();
        assert!(state.result.is_none());

        state.dma_ちゃん.disable();
        state.dma_ちゃん.clear_transfer_complete_interrupt();

        let header = get_header().unwrap();
        if state.address >= EXTERNAL_START + header.total_ext_length() {
            state.address = INTERNAL_START;
        }
        if state.address >= INTERNAL_START + header.ln_int as usize {
            state.result = Some(state.hw_crc.dr().read().bits());
            return;
        }

        state.dma_ちゃん.set_memory_address(state.address as u32);
        state.dma_ちゃん.set_number_of_transfers((CHUNK_SIZE/4) as u16);
        state.dma_ちゃん.enable();

        state.address += CHUNK_SIZE;
    });
}

impl CrcHandler {
    pub fn new(inst: CRC, mut ちゃん: DmaChannel) -> Self {
        interrupt_free(|cs| unsafe {
            // Peripheral is destination in mem-to-mem mode
            ちゃん.set_peripheral_address(inst.dr().as_ptr() as u32);
            ちゃん.set_peripheral_increment(false);
            ちゃん.set_peripheral_size(4);

            // Memory is source in mem-to-mem mode
            ちゃん.set_memory_increment(true);
            ちゃん.set_memory_size(4);

            *STATE.borrow(cs).as_mut_unchecked() = Some(CrcState {
                hw_crc: inst,
                dma_ちゃん: ちゃん,
                address: 0,
                result: None,
            })
        });
        Self { }
    }

    pub fn start(&mut self) {
        interrupt_free(|cs| unsafe {
            let state = STATE.borrow(cs).as_mut_unchecked().as_mut().unwrap();
            if get_header().is_none() {
                state.result = Some(0x0000_0000);
                return;
            }

            state.hw_crc.cr().write(|w| w.reset().reset());
            // Skip over target CRC
            state.dma_ちゃん.set_memory_address((EXTERNAL_START + 4) as u32);
            state.dma_ちゃん.set_number_of_transfers((CHUNK_SIZE/4 - 4) as u16);
            state.address = EXTERNAL_START;
            state.dma_ちゃん.enable();
        })
    }

    pub fn stop(&mut self) {
        interrupt_free(|cs| unsafe {
            let state = STATE.borrow(cs).as_mut_unchecked().as_mut().unwrap();
            state.dma_ちゃん.disable();
            state.result = None;
        })
    }

    pub fn valid(&mut self) -> Option<bool> {
        interrupt_free(|cs| unsafe {
            if let Some(header) = get_header() {
                STATE.borrow(cs).as_mut_unchecked().as_mut().unwrap()
                    .result.and_then(|x| Some(x == header.crc))
            } else {
                Some(false)
            }
        })
    }
}
