//! Common board interface

// This is getting to the point that we might want to write macros

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
#[cfg(feature = "stm32g474")]
mod stm32g474;

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
#[cfg(feature = "stm32g474")]
pub type CClock = stm32g474::CClock;
#[cfg(feature = "stm32g474")]
pub type GClock = stm32g474::GClock;
#[cfg(feature = "stm32g474")]
pub type CanDriver = stm32g474::CanDriver;


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
    #[cfg(feature = "stm32g474")]
    {
        stm32g474::init()
    }
}
