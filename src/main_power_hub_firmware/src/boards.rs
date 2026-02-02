// For now this code is not split up into boards as there will probably only be
// the g4xx using this board
//
// If anything it will be the we-act using the board but that probably won't happen
// either

use embedded_common::argb;
use stm32g4::stm32g474::USART1;
use stm32g4xx_hal::{
    gpio::{self, Alternate, Pin},
    prelude::*,
    pwm::PwmExt,
    pwr::{PwrExt, VoltageScale},
    rcc::*,
    time::RateExtU32,
};

//////////////////////////////
// POWER CHANNEL CODE BLOCK //
//////////////////////////////

pub const NUM_CHANNELS: usize = 4;

// These channels should be marked physically so that we know which is which
pub struct PowerChannel {
    pub enable: gpio::AnyPin<gpio::Output>,
    // sense: gpio::AnyPin<gpio::Input>,
}

impl PowerChannel {
    pub fn enable(&mut self) {
        self.enable.set_low();
    }

    pub fn disable(&mut self) {
        self.enable.set_high();
    }
}

/*
pub struct AdcController {
    pub adc1: Adc<ADC1, adc::Configured>,
    pub adc2: Adc<ADC2, adc::Configured>,
    pub adc3: Adc<ADC3, adc::Configured>,
    pub adc4: Adc<ADC4, adc::Configured>,
}
*/

// This function does all of the initialization for the board
// This will return all of the abstracted drivers to use in main
pub fn init() -> (
    argb::Controller<USART1, Pin<'A', 9, Alternate<7>>>,
    [PowerChannel; NUM_CHANNELS],
) {
    let dp = stm32g4xx_hal::stm32::Peripherals::take().unwrap();
    let pwr = dp
        .PWR
        .constrain()
        .vos(VoltageScale::Range1 { enable_boost: true })
        .freeze();

    let mut rcc = dp.RCC.freeze(
        // enable HSE @ 24 MHz (stepper board)
        Config::pll()
            .pll_cfg(PllConfig {
                mux: PllSrc::HSE(24.MHz()),
                m: PllMDiv::DIV_2,
                n: PllNMul::MUL_28,
                r: Some(PllRDiv::DIV_2), // Why limit ourselves @Jonah? :p
                q: Some(PllQDiv::DIV_2),
                p: Some(PllPDiv::DIV_8), // Has to be under 60MHz for it to work i think
            })
            .fdcan_src(FdCanClockSource::PLLQ),
        pwr,
    );

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    // ARGB LED SETUP
    let led_controller =
        argb::Controller::new(dp.USART1, gpioa.pa9.into_alternate(), 32, &mut rcc);

    // Charge pump clock pin setup
    // This will be initialised and then not be used later
    let clock_pin: gpio::PB7<gpio::AF10> = gpiob.pb7.into_alternate();
    let mut pwm = dp.TIM3.pwm(clock_pin, 100.kHz(), &mut rcc);
    let _ = pwm.set_duty_cycle_percent(5);
    pwm.enable();

    // Sense setup
    // This initialises all of the ADCs and then sets all the pins to: into_analog()
    // This lets you poll it directly from the pins so no need to mess around with ADC in main
    /*
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
    */

    let pwr_channels = [
        // J8 CH0
        PowerChannel {
            enable: gpioc.pc9.into_push_pull_output().into(),
        },
        // J9 CH1
        PowerChannel {
            enable: gpioc.pc8.into_push_pull_output().into(),
        },
        // J2 CH2
        PowerChannel {
            enable: gpioc.pc7.into_push_pull_output().into(),
        },
        // J4 CH3
        PowerChannel {
            enable: gpioc.pc6.into_push_pull_output().into(),
        },
    ];

    (led_controller, pwr_channels)
}
