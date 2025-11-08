// Wraps implementations of board-specific low-level drivers for various interfaces into common
// abstract drivers for main.rs.

use canadensis::core::time as time;
use cfg_if::cfg_if;

pub trait CyphalClock: time::Clock {
    fn start(&mut self);
}

pub trait GeneralClock {
    fn now(&self) -> time::Microseconds32;
}

pub trait RGBLEDDriver {
    // n is zero-indexed. Panics if n is greater than the number of LEDs on the board.
    // This function does NOT change the display state. Call render() to actually send
    // the new color signals to the LEDs.
    fn set_nth_led(&mut self, n: usize, color: u32);

    // Syncs the LED display state with the internal color state.
    // Err value is an error message.
    fn render(&self) -> Result<(), &'static str>;

    // for convenience
    fn set_nth_led_and_render(&mut self, n: usize, color: u32) -> Result<(), &'static str>;
}

pub trait I2CDriver {

}

cfg_if! {
    if #[cfg(any(feature = "rev_a2", feature = "rev_b2", feature = "we_act_dev"))] {
        mod stm32g4xx;

        // these types need to be exposed to the layer that creates the canadensis/Cyphal node
        pub type CClock = stm32g4xx::STM32G4xxCyphalClock;
        pub type CanDriver = stm32g4xx::STM32G4xxCanDriver;

        pub fn init() -> (CClock, stm32g4xx::STM32G4xxGeneralClock, CanDriver, stm32g4xx::STM32G4xxLEDDriver, stm32g4xx::I2CDriver) {
            stm32g4xx::init()
        }
    } else {
        compile_error!("The board that the firmware is being compiling for must be specified as a feature");
    }
}
