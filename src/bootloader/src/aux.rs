use core::{cell::UnsafeCell, mem::MaybeUninit};

use cortex_m::interrupt::Mutex;
use stm32g4xx_hal::rcc::Rcc;


#[unsafe(link_section = "AUX")]
pub static AUX_DATA: Mutex<UnsafeCell<MaybeUninit<AuxData>>> = Mutex::new(UnsafeCell::new(MaybeUninit::uninit()));

pub struct AuxData {
    pub rcc: Rcc,
}

impl AuxData {
    pub unsafe fn get() -> &'static mut Self {
        cortex_m::interrupt::free(|cs| {
            (*AUX_DATA.borrow(cs).get()).assume_init_mut()
        })
    }

    pub unsafe fn set(val: Self) {
        cortex_m::interrupt::free(|cs| {
            (*AUX_DATA.borrow(cs).get()).write(val);
        });
    }
}
