//! Wraps implementations of board-specific low-level drivers for various interfaces into common
//! abstract drivers for main.rs.

use core::ops::Sub;

use crate::stm32g4xx;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Celsius(pub f32);

// these types need to be exposed to the layer that creates the canadensis/Cyphal node
pub type CClock = stm32g4xx::STM32G4xxCyphalClock;
pub type CanDriver = stm32g4xx::STM32G4xxCanDriver;

pub fn init() -> (
    CClock,
    stm32g4xx::STM32G4xxGeneralClock,
    CanDriver,
    stm32g4xx::STM32G4xxLEDDriver,
    stm32g4xx::STM32G4xxI2CDriver,
    stm32g4xx::STM32G4xxStepperDriver,
) {
    stm32g4xx::init()
}
