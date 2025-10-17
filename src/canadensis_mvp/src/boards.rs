//! Common board interface

use canadensis::core::time as time;

pub trait CyphalClock: time::Clock {
    fn start(&mut self);
}

pub trait GeneralClock {
    fn now(&self) -> time::Microseconds32;
}

// Specific boards

#[cfg(feature = "stm32f103")]
mod stm32f103;
#[cfg(feature = "stm32g431")]
mod stm32g431;

#[cfg(feature = "stm32f103")]
pub type CClock = stm32f103::CClock;
#[cfg(feature = "stm32f103")]
pub type GClock = stm32f103::GClock;
#[cfg(feature = "stm32f103")]
pub type CanDriver = stm32f103::CanDriver;
#[cfg(feature = "stm32g431")]
pub type CClock = stm32g431::CClock;
#[cfg(feature = "stm32g431")]
pub type GClock = stm32g431::GClock;
#[cfg(feature = "stm32g431")]
pub type CanDriver = stm32g431::CanDriver;

pub fn init() -> (CClock, GClock, CanDriver)
{
    #[cfg(feature = "stm32f103")]
    {
        stm32f103::init()
    }
    #[cfg(feature = "stm32g431")]
    {
        stm32g431::init()
    }
}
