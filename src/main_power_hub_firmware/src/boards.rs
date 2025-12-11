// For now this code is not split up into boards as there will probably only be
// the g4xx using this board
//
// If anything it will be the we-act using the board but that probably won't happen
// either

use stm32g4xx_hal::{
    prelude::*,
    serial,
    pwr::PwrExt,  
    pac, 
    time::{self, RateExtU32},
    rcc::*,
    gpio,
    pwm::PwmExt
};


use stm32g4xx_hal::serial::TxExt;

use embedded_io::Write;


////////////////////
// LED CODE BLOCK //
////////////////////

// Crude re-implementation of the LED driver from the stepper drivebase
pub const NUM_LEDS: usize = 6;

// This is for USART3_TX
type LedTxPin = gpio::PB9<gpio::AF7>;

#[derive(Clone, Copy)]
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
// ON THE STEPPER BOARD

pub struct STM32G4xxLEDDriver {
    usart: stm32g4xx_hal::serial::Tx<pac::USART3, LedTxPin, serial::NoDMA>,
    colors: [RGBLEDColor; NUM_LEDS],
}

impl STM32G4xxLEDDriver {
    // Initialise the USART1 peripheral.
    // This has to be public so that the other file can access it
    // it was private in stepper as it was initialised in the same file
    pub fn new(usart3: pac::USART3, tx_pin: LedTxPin, rcc: &mut Rcc) -> Self 
    {
        const BAUDRATE: time::Bps = time::Bps(2_500_000);
        let config = stm32g4xx_hal::serial::config::FullConfig::default()
            .baudrate(BAUDRATE)
            .tx_invert()
            .wordlength_7()
            .stopbits(serial::config::StopBits::STOP1);
        let usart3 = usart3.usart_txonly(tx_pin, config, rcc).unwrap();


        Self {
            usart: usart3,
            colors: [RGBLEDColor::default(); NUM_LEDS]
        }
    }
}

impl RGBLEDDriver for STM32G4xxLEDDriver {
    fn set_nth_led(&mut self, n: usize, color: RGBLEDColor) {
        self.colors[n] = color;
    }

    fn render(&mut self) {
        // we squeeze three WS2812 bits per USART byte
        const TRIBIT_LUT: [u8; 8] = [
            // 000 -> H_START L L H L L H L L_STOP
            // TX pin polarity inverted, so data actually 1101101
            // ...and then bits are sent in reverse, so 1011011.
            0b1011011,
            // 001 -> [H]LLHLLHH[L] -> 1101100 -> 0011011
            0b0011011,
            // 010 -> [H]LLHHLHL[L] -> 1100101 -> 1010011
            0b1010011,
            // 011 -> [H]LLHHLHH[L] -> 1100100 -> 0010011
            0b0010011,
            // etc.
            0b1011010,
            0b0011010,
            0b1010010,
            0b0010010
        ];
        for i in 0..NUM_LEDS {
            let mut packet = [0u8; 8];
            let color = self.colors[i];
            // required order for WS2812: G R B
            let shifted_color = ((color.green as u32) << 16) | ((color.red as u32) << 8) | (color.blue as u32);
            for j in 0..8 {
                let tribit = ((shifted_color & (0xE00000 >> (j * 3))) >> (21 - j * 3)) as usize;
                packet[j] = TRIBIT_LUT[tribit];
            }
            self.usart.write_all(&packet).unwrap();
        }
    }

    fn set_nth_led_and_render(&mut self, n: usize, color: RGBLEDColor) {
        self.set_nth_led(n, color);
        self.render();
    }
}


//////////////////////////////
// POWER CHANNEL CODE BLOCK //
//////////////////////////////

const NUM_CHANNELS: u32 = 4;

// TODO:
// Implement a struct which handles the power channels

// These channels should be marked physically so that we know which is which
pub enum PowerChannels {
    Channel0,
    Channel1,
    Channel2,
    Channel3,
}

pub struct PowerController {


}

// This function does all of the initialization for the board
// This will return all of the abstracted drivers to use in main
pub fn init() -> (
    STM32G4xxLEDDriver,
    (gpio::Pin<'C', 9, gpio::Output>, gpio::Pin<'C', 8, gpio::Output>, gpio::Pin<'C', 7, gpio::Output>, gpio::Pin<'C', 6, gpio::Output>)
) {

    let dp = stm32g4xx_hal::stm32::Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain().freeze();

    let mut rcc = dp.RCC.freeze(
        // enable HSE @ 24 MHz (stepper board)
        Config::pll()
            .pll_cfg(PllConfig {
                mux: PllSrc::HSE(24.MHz()),
                m: PllMDiv::DIV_3,
                n: PllNMul::MUL_32,
                r: Some(PllRDiv::DIV_2), // Why limit ourselves @Jonah? :p
                q: Some(PllQDiv::DIV_2),
                p: None,
            })
            .fdcan_src(FdCanClockSource::PLLQ),
        pwr
    );

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);

    // ARGB LED SETUP
    let led_tx_pin = gpiob.pb9.into_alternate();
    let usart3 = dp.USART3;
    let mut hled = STM32G4xxLEDDriver::new(usart3, led_tx_pin, &mut rcc);

    // Clock pin setup
    // This will be initialised and then not be used later
    let clock_pin: gpio::PB7<gpio::AF10> = gpiob.pb7.into_alternate();
    let mut pwm = dp.TIM3.pwm(clock_pin, 100.kHz(), &mut rcc);
    let _ = pwm.set_duty_cycle_percent(5);
    pwm.enable();

    // The power channel GPIO enables are initialised as a tuple
    // This means they can be directly called by eg:
    //          pwr_channel_enable.0.set_low();
    let mut pwr_channel_enable  = (
        // J8 CH0
        gpioc.pc9.into_push_pull_output(),
        // J9 CH1
        gpioc.pc8.into_push_pull_output(),
        // J2 CH2
        gpioc.pc7.into_push_pull_output(),
        // J4 CH3
        gpioc.pc6.into_push_pull_output()
    ); 


    (hled, pwr_channel_enable)

}