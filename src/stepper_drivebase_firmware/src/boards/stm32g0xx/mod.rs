//! Implementations of some specific low-level drivers for the BIGTREETECH MMB CAN V2.0 (STM32G0B1).
//!
//! This board is primarily used to test the TMC5160s, so I'm not going to bother properly
//! implementing anything else (e.g. FDCAN) unless I get carried away.

use canadensis::core::{OutOfMemoryError, subscription::Subscription};
use canadensis_can::{driver, Frame, CanNodeId};

#[allow(unused)]
use cortex_m_semihosting::hprintln;

use crate::boards::{CyphalClock, RGBLEDDriver, RGBLEDColor, StepperDriver, StepperChannel, StepperRegister, SPIError, Celsius};
pub mod clock;
pub use clock::{STM32G0xxCyphalClock, STM32G0xxGeneralClock};

use hal::{
    prelude::*,
    time::RateExtU32,
    rcc::{Config, PllConfig, PLLSrc, PLLDiv, PLLMul, Rcc},
    gpio,
    pac,
    spi::{self, spi::SpiBus},
};
use stm32g0xx_hal as hal;

// FDCAN driver, not implemented.

#[derive(Debug)]
pub struct OverrunError {}

pub struct STM32G0xxCanDriver {}

impl driver::ReceiveDriver<STM32G0xxCyphalClock> for STM32G0xxCanDriver {
    type Error = OverrunError;

    fn receive(&mut self, clock: &mut clock::STM32G0xxCyphalClock) -> Result<Frame, nb::Error<OverrunError>> {
        Err(nb::Error::WouldBlock)
    }

    fn apply_filters<S>(
        &mut self,
        local_node: Option<CanNodeId>,
        subscriptions: S,
    )
       where S: IntoIterator<Item = Subscription> {}

    fn apply_accept_all(&mut self) {}
}

impl driver::TransmitDriver<clock::STM32G0xxCyphalClock> for STM32G0xxCanDriver {
    type Error = OverrunError;

    // this is literally a no-op
    fn try_reserve(&mut self, frames: usize) -> Result<(), OutOfMemoryError> {
        Err(OutOfMemoryError)
    }

    fn transmit(
        &mut self,
        frame: Frame,
        _clock: &mut clock::STM32G0xxCyphalClock,
    ) -> nb::Result<Option<Frame>, Self::Error> {
        Ok(None)
    }

    // busy-wait until all frames have been sent
    fn flush(&mut self, _clock: &mut clock::STM32G0xxCyphalClock) -> nb::Result<(), Self::Error> {
        Ok(())
    }
}



// RGB LED (WS2812) driver.
// The board has a WS2812 interface but no actual LEDs on-board.
// Implementing this is not a priority.

pub const NUM_LEDS: usize = 6;  // make sure this is set appropriately for the board

pub struct STM32G0xxLEDDriver {
    colors: [RGBLEDColor; NUM_LEDS],
}

impl STM32G0xxLEDDriver {
    fn new() -> Self 
    {
        Self {
            colors: [RGBLEDColor::default(); NUM_LEDS]
        }
    }
}
impl RGBLEDDriver for STM32G0xxLEDDriver {
    fn set_nth_led(&mut self, n: usize, color: RGBLEDColor) {
        self.colors[n] = color;
    }

    fn render(&mut self) {
        // lol. lmao, even
    }

    fn set_nth_led_and_render(&mut self, n: usize, color: RGBLEDColor) {
        self.set_nth_led(n, color);
        self.render();
    }
}

// I2C driver. Not implemented.

pub struct I2CDriver {}

// TMC5160 stepper driver (over SPI).
// This is the one thing we want to implement (test).
// Once this basic code is working we will copy it over to g4xx.

type EnnPins = (
    gpio::PD5<gpio::Output<gpio::PushPull>>,
    gpio::PD2<gpio::Output<gpio::PushPull>>,
    gpio::PC10<gpio::Output<gpio::PushPull>>,
    gpio::PC14<gpio::Output<gpio::PushPull>>,
);

struct StepperCSPins(
    gpio::Pin<gpio::Output<gpio::PushPull>>,
    gpio::Pin<gpio::Output<gpio::PushPull>>,
    gpio::Pin<gpio::Output<gpio::PushPull>>,
    gpio::Pin<gpio::Output<gpio::PushPull>>,
);

type StepperSPIPins = (
    gpio::PB13<gpio::DefaultMode>, // sck
    gpio::PB14<gpio::DefaultMode>, // miso
    gpio::PB15<gpio::DefaultMode> // mosi
);

pub struct STM32G0xxStepperDriver {
    enn: EnnPins,
    spi: spi::SpiBus<pac::SPI2, StepperSPIPins>,
    spi_cs: StepperCSPins,
}

impl STM32G0xxStepperDriver {
    fn new(
        spi2: pac::SPI2, rcc: &mut Rcc,
        enn: EnnPins, spi_pins: StepperSPIPins, spi_cs_pins: StepperCSPins
    ) -> Self {
        // initialise the SPI bus, SPI2
        // (using max allowable frequency)
        let spi = spi2.spi(spi_pins, spi::MODE_3, 4.MHz(), rcc);

        // do initial config of the steppers over SPI
        // TODO

        STM32G0xxStepperDriver {
            enn,
            spi,
            spi_cs: spi_cs_pins,
        }
    }

    fn map_spi_error(err: spi::Error) -> SPIError {
        match err {
            spi::Error::Overrun => SPIError::Overrun,
            _ => SPIError::Other,
        }
    }
}

impl StepperDriver for STM32G0xxStepperDriver {
    fn enable_all(&mut self) {
        self.enn.0.set_low().unwrap();
        self.enn.1.set_low().unwrap();
        self.enn.2.set_low().unwrap();
        self.enn.3.set_low().unwrap();
    }

    fn disable_all(&mut self) {
        self.enn.0.set_high().unwrap();
        self.enn.1.set_high().unwrap();
        self.enn.2.set_high().unwrap();
        self.enn.3.set_high().unwrap();
    }

    fn set_position(&mut self, channel: StepperChannel, target: u32) -> Result<(), SPIError> {
        // TODO
        Ok(())
    }

    fn set_velocity(&mut self, channel: StepperChannel, velocity: i32) -> Result<(), SPIError> {
        // TODO
        Ok(())
    }

    // TODO: TMC5160 SPI interface sends data back on the *subsequent* read, so we could possibly
    // set up a pipelined interface?
    // self.start_pipeline_read(reg).continue_pipeline_read/write(reg, data).end_pipeline(data)
    // need to fix annoying data allocation and ownership churn in above model
    fn read_reg(&mut self, channel: StepperChannel, reg: StepperRegister) -> Result<u32, SPIError> {
        let cs = match channel {
            StepperChannel::Channel1 => &mut self.spi_cs.0,
            StepperChannel::Channel2 => &mut self.spi_cs.1,
            StepperChannel::Channel3 => &mut self.spi_cs.2,
            StepperChannel::Channel4 => &mut self.spi_cs.3,
        };
        // 40-bit (5 byte), first byte is address, MSB of first byte is 0 for read
        let address: u8 = reg.into();
        let send_data = [address, 0u8, 0u8, 0u8, 0u8];
        let mut read_data = [0u8; 5];
        cs.set_low().unwrap();
        // send the data, ignore data returned
        self.spi.write(&send_data).map_err(STM32G0xxStepperDriver::map_spi_error)?;
        cs.set_high().unwrap();
        cs.set_low().unwrap();
        // read the data
        self.spi.read(&mut read_data).map_err(STM32G0xxStepperDriver::map_spi_error)?;
        cs.set_high().unwrap();
        // reconstruct register
        Ok((read_data[1] as u32) << 24 | (read_data[2] as u32) << 16 | (read_data[3] as u32) << 8 | (read_data[4] as u32))
    }

    fn write_reg(&mut self, channel: StepperChannel, reg: StepperRegister, data: u32) -> Result<(), SPIError> {
        let cs = match channel {
            StepperChannel::Channel1 => &mut self.spi_cs.0,
            StepperChannel::Channel2 => &mut self.spi_cs.1,
            StepperChannel::Channel3 => &mut self.spi_cs.2,
            StepperChannel::Channel4 => &mut self.spi_cs.3,
        };
        cs.set_low().unwrap();
        let address: u8 = reg.into();
        let send_data = [
            address,
            // terrible hack, make helper function
            ((data & 0xFF000000) >> 24) as u8,
            ((data & 0xFF0000) >> 16) as u8,
            ((data & 0xFF00) >> 8) as u8,
            (data & 0xFF) as u8
        ];
        self.spi.write(&send_data).map_err(STM32G0xxStepperDriver::map_spi_error)?;
        cs.set_high().unwrap();
        Ok(())
    }

    fn get_temperature(&mut self, channel: StepperChannel) -> Celsius {
        // not implemented
        Celsius(-273.15)
    }
}

pub fn init() -> (STM32G0xxCyphalClock, STM32G0xxGeneralClock, STM32G0xxCanDriver, STM32G0xxLEDDriver, I2CDriver, STM32G0xxStepperDriver) {
    // Embedded boilerplate...
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(
        // enable HSE @ 8 MHz (by visually inspecting the board)
        // should yield 64 MHz for SYSCLK and 128 MHz for PLLQ to be sent to FDCAN should it ever be implemented
        Config::pll()
            .pll_cfg(PllConfig {
                mux: PLLSrc::HSE(8.MHz()),
                m: PLLDiv::from(1),
                n: PLLMul::from(32),
                r: PLLDiv::from(4),
                q: Some(PLLDiv::from(2)),
                p: None,
            })
    );

    // Set up pins.
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);

    let mut cyphal_clock = STM32G0xxCyphalClock::new_singleton(dp.TIM2, &mut rcc);

    // FDCAN
    let can_driver = STM32G0xxCanDriver {};

    // ARGB LED
    let led_driver = STM32G0xxLEDDriver::new();

    // Steppers/SPI
    let en0 = gpiod.pd5.into_push_pull_output_in_state(PinState::High);
    let en1 = gpiod.pd2.into_push_pull_output_in_state(PinState::High);
    let en2 = gpioc.pc10.into_push_pull_output_in_state(PinState::High);
    let en3 = gpioc.pc14.into_push_pull_output_in_state(PinState::High);
    let enn_pins = (en0, en1, en2, en3);

    let sck = gpiob.pb13.into_analog();
    let miso = gpiob.pb14.into_analog();
    let mosi = gpiob.pb15.into_analog();
    let cs0 = gpiob.pb5.into_push_pull_output();
    let cs1 = gpiob.pb4.into_push_pull_output();
    let cs2 = gpiob.pb3.into_push_pull_output();
    let cs3 = gpiod.pd6.into_push_pull_output();
    let cs_pins = StepperCSPins(
        cs0.downgrade().downgrade(),
        cs1.downgrade().downgrade(),
        cs2.downgrade().downgrade(),
        cs3.downgrade().downgrade(),
    );
    let spi_pins = (sck, miso, mosi);

    cyphal_clock.start();

    let general_clock = STM32G0xxGeneralClock::new();

    let stepper_driver = STM32G0xxStepperDriver::new(dp.SPI2, &mut rcc, enn_pins, spi_pins, cs_pins);

    (cyphal_clock, general_clock, can_driver, led_driver, I2CDriver {}, stepper_driver)

    // TODO: remaining communication interfaces.
    // I2C1: pin PA15, PB7
    // SPI: PC10, PC11, PC12, 
}
