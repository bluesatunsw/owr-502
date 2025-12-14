// For now this code is not split up into boards as there will probably only be
// the g4xx using this board
//
// If anything it will be the we-act using the board but that probably won't happen
// either

use cortex_m::delay;
use cortex_m_semihosting::{hprint, hprintln};
use stm32g4::stm32g474::{ADC1, ADC2, ADC3, ADC4, adc12_common};
use stm32g4xx_hal::{
    adc::{self, Adc, AdcClaim, AdcCommonExt, config::{self, AdcConfig}}, gpio::{self, PB14}, pac, prelude::*, pwm::PwmExt, pwr::PwrExt, rcc::*, serial, time::{self, RateExtU32}
};


use stm32g4xx_hal::serial::TxExt;

use embedded_io::Write;


////////////////////
// LED CODE BLOCK //
////////////////////

// Crude re-implementation of the LED driver from the stepper drivebase
pub const NUM_LEDS: usize = 6;

// This is for USART1_TX
type LedTxPin = gpio::PA9<gpio::AF7>;

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
    usart: stm32g4xx_hal::serial::Tx<pac::USART1, LedTxPin, serial::NoDMA>,
    colors: [RGBLEDColor; NUM_LEDS],
}

impl STM32G4xxLEDDriver {
    // Initialise the USART1 peripheral.
    // This has to be public so that the other file can access it
    // it was private in stepper as it was initialised in the same file
    pub fn new(usart1: pac::USART1, tx_pin: LedTxPin, rcc: &mut Rcc) -> Self 
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
pub struct PowerChannel {
    enable: gpio::AnyPin<gpio::Output>,
    sense: gpio::AnyPin<gpio::Input>
}

impl PowerChannel {
    fn new(
        en_pin: gpio::AnyPin<gpio::Output>, 
        sense_pin: gpio::AnyPin<gpio::Input>
    ) -> Self {
        PowerChannel { enable: en_pin, sense: sense_pin }
    }
}


pub struct PowerController {
    pwr_channel: [gpio::AnyPin<gpio::Output>; 4],
}

impl PowerController {

    pub fn new(
        en_pin0: gpio::AnyPin<gpio::Output>,
        en_pin1: gpio::AnyPin<gpio::Output>,
        en_pin2: gpio::AnyPin<gpio::Output>,
        en_pin3: gpio::AnyPin<gpio::Output>,
        // sense_pin0: gpio::AnyPin<gpio::Input>,
        // sense_pin1: gpio::AnyPin<gpio::Input>,
        // sense_pin2: gpio::AnyPin<gpio::Input>,
        // sense_pin3: gpio::AnyPin<gpio::Input>
    ) -> Self {

        PowerController {
            pwr_channel: [
                // PowerChannel::new(en_pin0, sense_pin0),
                // PowerChannel::new(en_pin1, sense_pin1),
                // PowerChannel::new(en_pin2, sense_pin2),
                // PowerChannel::new(en_pin3, sense_pin3),
                en_pin0,
                en_pin1,
                en_pin2,
                en_pin3
            ]
        }


    }

    pub fn enable_all(&mut self) {
        
        // This enables all the output channels by letting going low
        // and letting the gate open on the NPN

        // Code for struct array
        // self.pwr_channel[0].enable.set_low();
        // self.pwr_channel[1].enable.set_low();
        // self.pwr_channel[2].enable.set_low();
        // self.pwr_channel[3].enable.set_low();
        
        self.pwr_channel[0].set_low();
        self.pwr_channel[1].set_low();
        self.pwr_channel[2].set_low();
        self.pwr_channel[3].set_low();

    }

    pub fn disable_all(&mut self) {
        
        // Similar but disabling instead via inverse

        // Code for struct array
        // self.pwr_channel[0].enable.set_high();
        // self.pwr_channel[1].enable.set_high();
        // self.pwr_channel[2].enable.set_high();
        // self.pwr_channel[3].enable.set_high();

        self.pwr_channel[0].set_high();
        self.pwr_channel[1].set_high();
        self.pwr_channel[2].set_high();
        self.pwr_channel[3].set_high();
    }


}

pub struct AdcController {
    pub adc1: Adc<ADC1, adc::Configured>,
    pub adc2: Adc<ADC2, adc::Configured>,
    pub adc3: Adc<ADC3, adc::Configured>,
    pub adc4: Adc<ADC4, adc::Configured>
}


// This function does all of the initialization for the board
// This will return all of the abstracted drivers to use in main
pub fn init() -> (
    STM32G4xxLEDDriver,
    PowerController,
    AdcController,
    gpio::Pin<'B', 14>
) {

    let dp = stm32g4xx_hal::stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
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
                p: Some(PllPDiv::DIV_8), // Has to be under 60MHz for it to work i think
            })
            .fdcan_src(FdCanClockSource::PLLQ),
        pwr
    );

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);

    // ARGB LED SETUP
    // let led_tx_pin = gpiob.pb9.into_open_drain_output().into_alternate();
    let led_tx_pin = gpioa.pa9.into_alternate();
    let usart1 = dp.USART1;
    let mut hled = STM32G4xxLEDDriver::new(usart1, led_tx_pin, &mut rcc);

    // Clock pin setup
    // This will be initialised and then not be used later
    let clock_pin: gpio::PB7<gpio::AF10> = gpiob.pb7.into_alternate();
    let mut pwm = dp.TIM3.pwm(clock_pin, 100.kHz(), &mut rcc);
    let _ = pwm.set_duty_cycle_percent(5);
    pwm.enable();

    // Sense setup
    // This initialises all of the ADCs and then sets all the pins to: into_analog()
    // This lets you poll it directly from the pins so no need to mess around with ADC in main
    let mut delay = cp.SYST.delay(&rcc.clocks);


    hprintln!("Configuring adc12!");
    let mut adc12_common = dp.ADC12_COMMON
    .claim(config::ClockMode::AdcKerCk { // Here instead of using the default use the enum
            prescaler: (config::Prescaler::Div_4), // Set the prescaler to /4
            src: (config::ClockSource::PllP) // set the clock to PLLP
        }, 
        &mut rcc
    );

    hprintln!("Configuring adc1");
    let mut adc1 = adc12_common
    .claim_and_configure(dp.ADC1, AdcConfig::default(), &mut delay);

    hprintln!("Configuring adc2");
    let mut adc2 = adc12_common
    .claim_and_configure(dp.ADC2, AdcConfig::default(), &mut delay);

    hprintln!("Configuring adc345!");
    let mut adc345_common = dp.ADC345_COMMON
    .claim(config::ClockMode::AdcKerCk {    // Same here
            prescaler: (config::Prescaler::Div_4),
            src: (config::ClockSource::PllP)
        }, 
        &mut rcc
    );

    hprintln!("Configuring adc3!");
    let mut adc3 = adc345_common
    .claim_and_configure(dp.ADC3, AdcConfig::default(), &mut delay);

    hprintln!("Configuring adc4!");
    let mut adc4 = adc345_common
    .claim_and_configure(dp.ADC4, AdcConfig::default(), &mut delay);

    // Pin configuration, not needed for now i think after they've been set
    let vsense_pin: gpio::Pin<'B', 14> = gpiob.pb14.into_analog();
    let ch0_sense_pin = gpioa.pa2.into_analog();
    let ch1_sense_pin = gpioa.pa6.into_analog();
    let ch2_sense_pin = gpiob.pb12.into_analog();
    let ch3_sense_pin = gpiob.pb1.into_analog();

    let adc_controller = AdcController { 
        adc1,
        adc2,
        adc3,
        adc4 
    };

    let power_controller = PowerController::new(
        // ENABLE PINS
            // J8 CH0
            gpioc.pc9.into_push_pull_output().into(),
            // J9 CH1
            gpioc.pc8.into_push_pull_output().into(),
            // J2 CH2
            gpioc.pc7.into_push_pull_output().into(),
            // J4 CH3
            gpioc.pc6.into_push_pull_output().into(),
        // SENSE PINS
        // ch0_sense_pin,
        // ch1_sense_pin.into(),
        // ch2_sense_pin.into(),
        // ch3_sense_pin.into()
    );



    (hled, power_controller, adc_controller, vsense_pin)

}