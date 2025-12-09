use stm32g4xx_hal::{
    hal, 
    serial, 
    pac, 
    time,
    rcc::{Rcc}
};

use crate::{LedTxPin, NUM_LEDS};


pub struct RGBLEDColor {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

impl RGBLEDColor {

    // returns a default RGBLEDColour
    pub fn default() -> Self {
        RGBLEDColor { red: 255, green: 255, blue: 255 }
    }

}

impl From<u32> for RGBLEDColor {
    // Given classic hex code will convert to 3 u8s
    fn from(value: u32) -> Self {
        RGBLEDColor { 
            red: ((value & 0xFF0000) >> 16) as u8,
            green: ((value & 0xFF00) >> 8) as u8,
            blue: (value & 0xFF) as u8,
        }
    }
}

// Copied straight from jonah's code thanks jonah :)
pub trait RGBLEDDriver {
    /// n is zero-indexed. Panics if n is greater than the number of LEDs on the board.
    /// This function does NOT change the display state. Call render() to actually send
    /// the new color signals to the LEDs.
    fn set_nth_led(&mut self, n: usize, color: RGBLEDColor);

    /// Syncs the LED display state with the internal color state.
    /// There isn't any meaningful way we can tell if this fails.
    fn render(&mut self);

    /// For convenience.
    fn set_nth_led_and_render(&mut self, n: usize, color: RGBLEDColor);
}


// THIS IS THE THINGS FROM INSIDE THE STM32g4 MOD FILE

pub struct STM32G4xxLEDDriver {
    usart: stm32g4xx_hal::serial::Tx<pac::USART3, LedTxPin, serial::NoDMA>,
    colors: [RGBLEDColor; NUM_LEDS],
}

impl STM32G4xxLEDDriver {
    // Initialise the USART1 peripheral.
    fn new(usart1: pac::USART1, tx_pin: LedTxPin, rcc: &mut Rcc) -> Self 
    {
        const BAUDRATE: time::Bps = time::Bps(2_500_000);
        let config = stm32g4xx_hal::serial::config::FullConfig::default()
            .baudrate(BAUDRATE)
            .tx_invert()
            .wordlength_7()
            .stopbits(serial::config::StopBits::STOP1);
        let usart1 = usart1.usart_txonly(tx_pin, config, rcc).unwrap();

        Self {
            usart: usart1,
            colors: [RGBLEDColor::default(); NUM_LEDS]
        }
    }
}