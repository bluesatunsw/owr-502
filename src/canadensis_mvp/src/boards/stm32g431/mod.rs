//! See boards README

use canadensis::core::time;
use canadensis_can::driver;
use canadensis_can::Frame;
use canadensis_core::OutOfMemoryError;
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;
use crate::boards::{CyphalClock, GeneralClock};

use stm32g4::stm32g431 as pac;

struct STM32G431CyphalClock {
    // TIM2 is a 32-bit timer on the G431
    hw_timer: pac::TIM2,
}

fn configure_clk(rcc: &mut pac::RCC) {
    // HSE is 8 MHz crystal on WeAct (NOTE: on the Nucleo it's 24 MHz!)
    // Set SYSCLK to 64 MHz.
    unsafe {
        rcc.pllcfgr().write(|w| {
            w
            .pllr().bits(0b11)      // set PLL multiplier to 8x
            .pllsrc().bits(0b11)    // set oscillator input to HSE (8 MHz)
        });
    }
    unsafe {
        // enable PLL
        rcc.cr().write(|w| w.pllon().bit(true));
        // set PLL as SYSCLK
        rcc.cfgr().write(|w| w.sw().bits(0b11));
    }
    // TIM2 is on APB1 (PCLK1)
    // SYSCLK -[AHB prescaler]-> HCLK -[APB1 prescaler]-> PCLK1 
    // don't bother prescaling here, default /1, /1 passthrough

    // set 64 MHz PCLK1 to be FDCAN clock source
    unsafe {
        rcc.ccipr().write(|w| w.fdcansel().bits(0b10));
        // other options: PLL "Q" (0b01), HSE (default on reset) (0b00)
    }
}

// Internal 1 MHz timer to provide time instants for Cyphal.
impl STM32G431CyphalClock {
    fn new(tim2: pac::TIM2, rcc: &mut pac::RCC) -> Self {
        // Reset and enable the timer peripheral.
        rcc.apb1rstr1().write(|w| w.tim2rst().bit(true));
        rcc.apb1rstr1().write(|w| w.tim2rst().bit(false));
        rcc.apb1enr1().write(|w| w.tim2en().bit(true));
        Self {
            hw_timer: tim2,
        }
    }

    // Configure timers: count up at 1 MHz, overflow automatically and silently.
    // this architecture just gives us a 32-bit timer so we don't need to do silly clock chaining yay
    fn start(&mut self) {
        // updates on overflow happen automatically
        // PCLK1 is 64MHz, so prescale by /64
        unsafe {
            // scale 64MHz to 1MHz
            self.hw_timer.psc().write(|w| w.psc().bits(0x003F));
            // disable updates and enable timer
            self.hw_timer.cr1().write(|w| w.udis().bit(true).cen().bit(true));
        }
    }
}

// make clock globally available, so we can write the interrupt handler for it
static G_CYPHAL_CLOCK: Mutex<RefCell<Option<STM32G431CyphalClock>>> =
    Mutex::new(RefCell::new(None));

pub struct CClock {}

impl CClock {
    fn new_singleton(tim2: pac::TIM2, rcc: &mut pac::RCC) -> Self {
        // takes in a few hardware peripherals
        cortex_m::interrupt::free(|cs| {
            *G_CYPHAL_CLOCK.borrow(cs).borrow_mut() = Some(STM32G431CyphalClock::new(tim2, rcc))
        });
        CClock {}
    }
}

// exists to be instantiable locally but passes everything through to global state
impl CyphalClock for CClock {
    fn start(&mut self) {
        cortex_m::interrupt::free(|cs| {
            G_CYPHAL_CLOCK
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .start()
        });
    }
}

// panics if CyphalClock not initialised and started
fn get_instant() -> time::Microseconds32 {
    cortex_m::interrupt::free(|cs| {
        if let Some(cyphal_clock) = G_CYPHAL_CLOCK.borrow(cs).borrow_mut().as_mut() {
            time::Microseconds32::from_ticks(
                cyphal_clock.hw_timer.cnt().read().bits()
            )
        } else {
            panic!("Could not borrow global cyphal clock")
        }
    })
}

impl time::Clock for CClock {
    fn now(&mut self) -> time::Microseconds32 {
        get_instant()
    }
}

pub struct GClock {}

impl GeneralClock for GClock {
    fn now(&self) -> time::Microseconds32 {
        get_instant()
    }
}

struct STM32G431CanDriver {
    fdcan: pac::FDCAN1,
}

impl STM32G431CanDriver {
    fn new_singleton(fdcan: pac::FDCAN1, gpioa: pac::GPIOA, rcc: &mut pac::RCC) -> Self {
        // Reset and enable the FDCAN1 peripheral.
        rcc.apb1enr1().write(|w| w.fdcanen().bit(true));
        rcc.apb1rstr1().write(|w| w.fdcanrst().bit(true));
        rcc.apb1rstr1().write(|w| w.fdcanrst().bit(false));

        // Reset and enable GPIOA.
        rcc.ahb2enr().write(|w| w.gpioaen().bit(true));
        rcc.ahb2rstr().write(|w| w.gpioarst().bit(true));
        rcc.ahb2rstr().write(|w| w.gpioarst().bit(false));

        unsafe {
            // Set PA11 and PA12 modes to alternate function
            // On G431 (and G474!), these are FDCAN_RX and _TX respectively...
            gpioa.moder().write(|w| w.moder11().bits(0b10).moder12().bits(0b10));
            // ...specifically, FDCAN1 (AF9 = 0b1001).
            gpioa.afrh().write(|w| w.afrh11().bits(0b1001).afrh12().bits(0b1001));
            // CHECKME: Set output modes to push-pull (interface with controller)
            gpioa.otyper().write(|w| w.ot11().bit(false).ot12().bit(false));
            // CHECKME: Set pin speed to medium; should be acceptable for 8MHz FDCAN data rate
            gpioa.ospeedr().write(|w| w.ospeedr11().bits(0b01).ospeedr12().bits(0b01));
        }

        // Configure FDCAN1.
        // Set CCE of CCCR (we just reset so INIT should be set)
        fdcan.cccr().write(|w| w.cce().bit(true));
        fdcan.cccr().write(|w|
            w.cce().bit(true)   // stay in configuration mode
            .pxhd().bit(true)   // disable protocol handling exception
            .brse().bit(true)   // enable bit rate switching for transmissions
            .fdoe().bit(true)   // enable FD operation
            .dar().bit(true)    // disable automatic retransmission
        );
        unsafe {
            // TODO: Calculate for data bit rate = 8 MHz, PCLK1 = 64 MHz
            fdcan.dbtp().write(|w|
                w.tdc().bit(false)      // disable transceiver delay compensation
                .dbrp().bits(0b1111)    // data bit rate prescaler
                .dtseg1().bits(0b1111)  // data time segment before sample point
                .dtseg2().bits(0b111)   // data time segment after sample point
                .dsjw().bits(0b111)     // synchronisation jump width
            );
            // TODO: Calculate for nominal bit rate = 1 MHz, PCLK1 = 64 MHz
            fdcan.nbtp().write(|w|
                w.nbrp().bits(0x1ff)    // nominal bit rate prescaler
                .ntseg1().bits(0xff)    // nominal time segment before sample point
                .ntseg2().bits(0x7f)    // nominal time segment after sample point
                .nsjw().bits(0x7f)      // nominal (re)synchronisation jump width
            );
            // TODO: How the heck does timestamping work
            // Desired: Timestamp with 1 us precision
            // FDCAN has internal 16-bit counter
            // We may need to make some way of translating this to the Cyphal time
            fdcan.tscc().write(|w| w.tcp().bits(0b1111).tss().bits(0b11));
            // Timeout modes: either manually reset timeout by writing to TOCV ("continuous mode"),
            // or automatically reset when a RX FIFO is cleared. Idea is to check for
            // liveness/FIFOs being processed. Let's not bother using it for now...
            //      fdcan.tocc().write(|w| w.top().bits(0xffff).tos().bits(0b11).etoc().bit(false));
            fdcan.tocc().write(|w| w.etoc().bit(false));
            // TODO: what is transmitter delay compensation
            // what the heck should these values be set to???
            fdcan.tdcr().write(|w| w.tdco().bits(0x7f).tdcf().bits(0x7f));
        }

        // Ignore following, not using interrupts for now. But might want set up
        // interrupt handler to buffer more messages between receive() calls in future?
        //      fdcan.ie().write(...);
        //      fdcan.ils().write(...);
        //      fdcan.ile().write(...);

        // Exit configuration mode and start.
        fdcan.cccr().write(|w| w.init().bit(false));
        STM32G431CanDriver {
            fdcan
        }
    }
}

#[derive(Debug)]
struct OverrunError {}

// TODO
// just use one of the RX FIFOs
impl driver::ReceiveDriver<CClock> for STM32G431CanDriver {
    type Error = OverrunError;

    // literally just returns a CAN frame if one is available
    fn receive(&mut self, clock: &mut CClock) -> Result<Frame, canadensis::nb::Error<OverrunError>> {
        // read some kind of register to check if CAN frames received
        let rxf0s = self.fdcan.rxf0s().read();
        //if rxf0s.
        // if so, create a Frame from the message data. calculate the memory address: 0x4000_A400,
        // add the index offset
        Ok(Frame::new(timestamp, id, data))
    }

    fn apply_filters<S>(
        &mut self,
        local_node: Option<CanNodeId>,
        subscriptions: S,
    )
       where S: IntoIterator<Item = Subscription> {

    }

    fn apply_accept_all(&mut self) {
        // set the filters to accept all frames
    }
}

// TODO
impl driver::TransmitDriver<CClock> for STM32G431CanDriver {
    // FIXME
    type Error = OverrunError;

    fn try_reserve(&mut self, frames: usize) -> Result<(), OutOfMemoryError> {
        Ok(())
    }

    fn transmit(
        &mut self,
        frame: Frame,
        clock: &mut CClock,
    ) -> Result<Option<Frame>, Self::Error> {

    }

    fn flush(&mut self, clock: &mut CClock) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub type CanDriver = STM32G431CanDriver;

pub fn init() -> (CClock, GClock, CanDriver)
{
    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC;

    configure_clk(&mut rcc);

    // set PB8 to output mode to limit power consumption on reset
    unsafe {
        dp.GPIOB.moder().write(|w| w.moder8().bits(0b01));
    }

    let cyphal_clock = CClock::new_singleton(dp.TIM2, &mut rcc);

    let hwdriver = STM32G431CanDriver::new_singleton(dp.FDCAN1, dp.GPIOA, &mut rcc);

    (cyphal_clock, GClock {}, hwdriver)
}
