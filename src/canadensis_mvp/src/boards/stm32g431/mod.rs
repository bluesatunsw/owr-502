//! See boards README

use canadensis::core::time;
use canadensis::core::nb;
use canadensis::core::time::Clock;
use canadensis_filter_config::Filter;
use core::slice;
use core::convert::TryFrom;
use canadensis_can::driver;
use canadensis_can::driver::optimize_filters;
use canadensis_can::{Frame, CanId, CanNodeId};
use canadensis_core::OutOfMemoryError;
use canadensis_core::subscription::Subscription;
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;
use crate::boards::{CyphalClock, GeneralClock};
use cortex_m_semihosting::hprintln;

use stm32g4::stm32g431 as pac;

// maybe use
// TIM3 and TIM4?? could get perfectly hardware-accurate frame timestamps...

struct STM32G431CyphalClock {
    // TIM2 is a 32-bit timer on the G431
    hw_timer: pac::TIM2,
}

fn configure_clk(rcc: &mut pac::RCC) {
    // HSE is 8 MHz crystal on WeAct (NOTE: on the Nucleo it's 24 MHz!)
    // Set SYSCLK to 64 MHz.
    rcc.cr().modify(|_, w| w.hseon().bit(true));       // enable hse (it's not on by default!)
    unsafe {
        rcc.pllcfgr().write(|w| {
            w
            .pllsrc().bits(0b11)    // set oscillator input to HSE (8 MHz)
            .plln().bits(16)        // set PLL multiplier to (16 / 1)
            .pllm().bits(0)         // "
            .pllr().bits(0b00)      // set PLLR division factor to 2
                                    // so PLLR = SYSCLK = 8 MHz * 16 / 2 = 64 MHz
            .pllren().bit(true)     // enable PLL output
        });
    }
    unsafe {
        // enable PLL
        rcc.cr().modify(|_, w| w.hseon().bit(true).pllon().bit(true));
        // set PLL as SYSCLK
        rcc.cfgr().modify(|_, w| w.sw().bits(0b11));
    }
    // TIM2 is on APB1 (PCLK1)
    // SYSCLK -[AHB prescaler]-> HCLK -[APB1 prescaler]-> PCLK1 
    // don't bother prescaling here, default /1, /1 passthrough

    // set 64 MHz PCLK1 to be FDCAN clock source
    unsafe {
        rcc.ccipr().modify(|_, w| w.fdcansel().bits(0b10));
        // other options: PLL "Q" (0b01), HSE (default on reset) (0b00)
    }
}

// Internal 1 MHz timer to provide time instants for Cyphal.
impl STM32G431CyphalClock {
    fn new(tim2: pac::TIM2, rcc: &mut pac::RCC) -> Self {
        // Reset and enable the timer peripheral.
        rcc.apb1rstr1().modify(|_, w| w.tim2rst().bit(true));
        rcc.apb1rstr1().modify(|_, w| w.tim2rst().bit(false));
        rcc.apb1enr1().modify(|_, w| w.tim2en().bit(true));
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
            self.hw_timer.psc().write(|w| w.psc().bits(64 - 1));
            // NOTE: do not disable updates, as the prescaler is loaded ON AN UPDATE EVENT
            // so the prescaler won't actually apply until e.g. counter overflows
            // (which we effectively set to happen immediately below)
            self.hw_timer.cnt().write(|w| w.cnt().bits(0xFFFFFFFF));
            // enable timer
            self.hw_timer.cr1().write(|w| w.cen().bit(true));
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

pub struct STM32G431CanDriver {
    fdcan: pac::FDCAN1,
}

impl STM32G431CanDriver {
    fn new_singleton(fdcan: pac::FDCAN1, gpioa: pac::GPIOA, rcc: &mut pac::RCC) -> Self {
        // this is used in two different places so it's set here
        // see the DBTP section for context
        const DATA_SAMPLE_PERIOD: u8 = 6;

        // Reset and enable the FDCAN1 peripheral.
        rcc.apb1enr1().modify(|_, w| w.fdcanen().bit(true));
        rcc.apb1rstr1().modify(|_, w| w.fdcanrst().bit(true));
        rcc.apb1rstr1().modify(|_, w| w.fdcanrst().bit(false));

        // Reset and enable GPIOA.
        rcc.ahb2enr().modify(|_, w| w.gpioaen().bit(true));
        rcc.ahb2rstr().modify(|_, w| w.gpioarst().bit(true));
        rcc.ahb2rstr().modify(|_, w| w.gpioarst().bit(false));

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
        fdcan.cccr().modify(|_, w| w.cce().bit(true));
        fdcan.cccr().modify(|_, w|
            w.pxhd().bit(true)   // CHECKME: disable protocol handling exception
            //.niso().bit(true)
            .brse().bit(true)   // enable bit rate switching for transmissions
            // keep EFBI (edge filtering) disabled, apparently there's errata
            .fdoe().bit(true)   // enable FD operation
            .dar().bit(true)    // CHECKME: disable automatic retransmission
        );
        unsafe {
            // For data bit rate = 8 MHz, PCLK1 = 64 MHz:
            // (note t_q is 15.625 ns = 1 PCLK1 period)
            // https://kvaser.com/support/calculators/can-fd-bit-timing-calculator/
            // nominal bit sample point 87.5%
            fdcan.dbtp().write(|w|
                w.tdc().bit(true)       // enable transceiver delay compensation
                .dbrp().bits(1 - 1)     // data bit rate prescaler
                                        // t_q = (0b0000 + 1) clock period(s)
                .dtseg1().bits(DATA_SAMPLE_PERIOD - 1)   // data time segment before sample point
                                        // = PROP_SEG + PHASE_SEG1 = 5 t_q
                .dtseg2().bits(1 - 1)   // data time segment after sample point
                                        // = PHASE_SEG2 = 2 t_q
                .dsjw().bits(1 - 1)     // synchronisation jump width
                                        // = 2 t_q
            );
            // For nominal bit rate = 1 MHz, PCLK1 = 64 MHz:
            // (note t_q is still 15.625 ns)
            // sample point 75%
            fdcan.nbtp().write(|w|
                w.nbrp().bits(1 - 1)    // nominal bit rate prescaler
                                        // = 1 (no scaling)
                .ntseg1().bits(55 - 1)  // nominal time segment before sample point
                                        // = PROP_SEG + PHASE_SEG1 = 55 t_q
                .ntseg2().bits(8 - 1)   // nominal time segment after sample point
                                        // = PHASE_SEG2 = 8 t_q
                .nsjw().bits(8 - 1)     // nominal (re)synchronisation jump width
                                        // = 8 t_q
            );

            // Transmitter delay compensation becomes necessary above a data rate of about 4.5 MHz
            fdcan.tdcr().write(|w| w.tdco().bits(DATA_SAMPLE_PERIOD - 1).tdcf().bits(3 - 1));

            // TXBC.TFQM is 0 (FIFO mode) on reset so don't change

            // CURRENTLY UNUSED CONFIGURATION REGISTERS:
            // Desired: Timestamp with 1 us precision
            // We may need to make some way of translating this to the Cyphal time
            // Can actually use the TIM3 value...? TSS = 0b10
            // Currently disabled (timestamp always 0)
            //      fdcan.tscc().write(|w| w.tcp().bits(0b0000).tss().bits(0b00));

            // Timeout modes: either manually reset timeout by writing to TOCV ("continuous mode"),
            // or automatically reset when a RX FIFO is cleared. Idea is to check for
            // liveness/FIFOs being processed. Let's not bother using it for now...
            //      fdcan.tocc().write(|w| w.top().bits(0xffff).tos().bits(0b11).etoc().bit(false));

            // Ignore following, not using interrupts for now. But might want set up
            // interrupt handler to buffer more messages between receive() calls in future?
            //      fdcan.ie().write(...);
            //      fdcan.ils().write(...);
            //      fdcan.ile().write(...);

            // Exit configuration mode and start.
            fdcan.cccr().modify(|_, w| w.init().bit(false));
        }

        STM32G431CanDriver {
            fdcan
        }
    }

    /// Set active filters to 0; accept all frames to FIFO 0
    fn reset_filters(&mut self) {
        unsafe {
            // put FDCAN back in configuration mode
            self.fdcan.cccr().modify(|_, w| w.cce().bit(true).init().bit(true));
            // block on INIT bit being set
            while self.fdcan.cccr().read().init().bit() == false {}

            // global filter configuration
            self.fdcan.rxgfc().modify(|_, w|
                w.lse().bits(0b0000)    // do not filter ExtMsgs
                .lss().bits(0b00000)    // do not filter StdMsgs
                //.f0om().bit(false)    // FIFO 0: set to block instead of overwrite
                //.f1om().bit(false)    // ditto for FIFO 1.
                                        // not documented in reference manual, but
                                        // from HAL, 0 = block, 1 = overwrite
                .anfs().bits(0b10)      // reject non-matching StdMsgs
                .anfe().bits(0b00)      // accept non-matching ExtMsgs to FIFO 0
                                        // (redundant but what the heck)
                .rrfs().bit(true)       // reject standard remote frames
            );

            // and back to normal mode
            self.fdcan.cccr().modify(|_, w| w.init().bit(false));
        }
    }

    /// Applies extended ID filters as produced for filtering Cyphal subscriptions
    fn do_apply_filters(&mut self, filters: &[Filter]) {
        if filters.len() > 8 {
            // can't have more than 8 extended filters applied in hardware,
            // so just accept everything
            self.reset_filters();
            return;
        }

        // fill message RAM with appropriate filter data
        let ext_filter_bank: &mut [ExtMsgFilterElement] = unsafe{slice::from_raw_parts_mut(FDCAN1_FLESA, 8)};
        for i in 0..filters.len() {
            let f = &filters[i];
            // 0b001: store in RX FIFO 0 if filter matches
            ext_filter_bank[i].f0 = (0b001 << 29) | (f.id());
            // 0b10: classic bit mask filter mode
            ext_filter_bank[i].f1 = (0b10 << 30) | (f.mask());
        }

        // do filter configuration again
        unsafe {
            // put FDCAN back in configuration mode
            self.fdcan.cccr().modify(|_, w| w.cce().bit(true).init().bit(true));
            // block on INIT bit being set
            while self.fdcan.cccr().read().init().bit() == false {}

            // global filter configuration
            self.fdcan.rxgfc().modify(|_, w|
                w.lse().bits(filters.len() as u8)    // apply ExtMsgs filters
                .lss().bits(0b00000)    // do not filter StdMsgs
                //.f0om().bit(false)    // FIFO 0: set to block instead of overwrite
                //.f1om().bit(false)    // ditto for FIFO 1.
                                        // not documented in reference manual, but
                                        // from HAL, 0 = block, 1 = overwrite
                .anfs().bits(0b10)      // reject non-matching StdMsgs
                .anfe().bits(0b10)      // reject non-matching ExtMsgs
                .rrfs().bit(true)       // reject standard remote frames
            );

            // and back to normal mode
            self.fdcan.cccr().modify(|_, w| w.init().bit(false));
        }
    }
}

#[derive(Debug)]
pub struct OverrunError {}

#[repr(C)]
struct FDCanRxFifoElement {
    r0: u32,
    r1: u32,
    data: [u8; 64],
}

#[repr(C)]
struct FDCanTxFifoElement {
    t0: u32,
    t1: u32,
    data: [u32; 16],
}

#[repr(C)]
struct ExtMsgFilterElement {
    f0: u32,
    f1: u32,
}

// const FDCAN1_FLSSA: *mut StdMsgFilterElement = (0x4000_A400 + 0x0000) as *mut StdMsgFilterElement;
const FDCAN1_FLESA: *mut ExtMsgFilterElement = (0x4000_A400 + 0x0070) as *mut ExtMsgFilterElement;
const FDCAN1_RXFIFO0: *const FDCanRxFifoElement = (0x4000_A400 + 0x00B0) as *const FDCanRxFifoElement;
const FDCAN1_TXFIFO: *mut FDCanTxFifoElement = (0x4000_A400 + 0x0278) as *mut FDCanTxFifoElement;
// const FDCAN1_RXFIFO1: *const FDCanFifoElement = (0x4000_A400 + 0x0188) as *const FDCanFifoElement;

struct RxFifo0 {}

impl RxFifo0 {
    // Make a frame from the element at index
    // idx should indicate a valid RX FIFO message
    fn make_frame(idx: usize, timestamp: time::Microseconds32) -> Frame {
        if idx >= 3 {
            panic!("Index {} out of bounds for the FDCAN1 Rx FIFO 0", idx);
        }
        let s: &[FDCanRxFifoElement] = unsafe{slice::from_raw_parts(FDCAN1_RXFIFO0, 3)};
        let element = &s[idx];
        let len: usize = ((element.r1 >> 16) & 0x0f) as usize;
        let ext_id: u32 = element.r0 & ((1 << 29) - 1);
        // TODO: god fucking damn it the fucking bytes are stored in little-endian order within
        // each word fuck my entire life
        Frame::new(timestamp, CanId::try_from(ext_id).unwrap(), &element.data[0..len])
    }
}

/// Extremely basic driver that just uses the hardware RX FIFO 0
impl driver::ReceiveDriver<CClock> for STM32G431CanDriver {
    type Error = OverrunError;

    // literally just returns a CAN frame if one is available
    fn receive(&mut self, clock: &mut CClock) -> Result<Frame, nb::Error<OverrunError>> {
        // read some kind of register to check if CAN frames received
        let rxf0s = self.fdcan.rxf0s().read();
        // TODO: check error status register? do more error checking, fault on the FIFO being full
        if rxf0s.f0fl().bits() == 0 {
            // fill level is 0, so receiving a message would block
            Err(nb::Error::WouldBlock)
        } else {
            // make the frame from the data in the message RAM
            let idx: u8 = rxf0s.f0gi().bits();
            let frame = RxFifo0::make_frame(rxf0s.f0gi().bits().into(), clock.now());
            // acknowledge message receipt
            unsafe { self.fdcan.rxf0a().write(|w| w.f0ai().bits(idx)) };
            Ok(frame)
        }
    }

    fn apply_filters<S>(
        &mut self,
        local_node: Option<CanNodeId>,
        subscriptions: S,
    )
       where S: IntoIterator<Item = Subscription> {
        if let Err(_) = optimize_filters(local_node, subscriptions, 8, |filters| self.do_apply_filters(filters)) {
            // OutOfMemory
            self.reset_filters();
        };
    }

    fn apply_accept_all(&mut self) {
        self.reset_filters();
    }
}

/// Extremely basic driver
impl driver::TransmitDriver<CClock> for STM32G431CanDriver {
    // operate the TX in FIFO (transmit in order of addition) instead of queue (transmit ordered by
    // message priority/ID) mode since the latter could lead to starvation
    // CHECKME: not sure above is the right choice?
    type Error = OverrunError;

    // this is basically a no-op
    fn try_reserve(&mut self, frames: usize) -> Result<(), OutOfMemoryError> {
        // the TX buffer has space for three messages
        if frames > 3 {
            Err(OutOfMemoryError)
        } else {
            Ok(())
        }
    }

    // add frame to the TX FIFO or return nb::WouldBlock if FIFO full
    // other parts of the codebase seem to expect this to displace lower-priority frames
    // so it returns Ok(None) if transmitted normally, Ok(Some(displaced_frame)) if transmitted
    // successfully but frame displaced while doing it, nb::Error::WouldBlock on WouldBlock,
    // other nb::Errors otherwise
    fn transmit(
        &mut self,
        frame: Frame,
        _clock: &mut CClock,
    ) -> nb::Result<Option<Frame>, Self::Error> {
        let txfqs = self.fdcan.txfqs();
        if txfqs.read().tfqf().bit() {
            // queue is full
            Err(nb::Error::WouldBlock)
        } else {
            // set data in TX buffer appropriately
            let put_index: usize = txfqs.read().tfqpi().bits().into();
            let s: &mut [FDCanTxFifoElement] = unsafe{slice::from_raw_parts_mut(FDCAN1_TXFIFO, 3)};
            //hprintln!("Trying to send id {:#0x}, DLC {}", u32::from(frame.id()), frame.dlc());
            //                ext. id
            s[put_index].t0 = (1 << 30) | u32::from(frame.id());
            //                CAN FD + BRS
            s[put_index].t1 = (0b11 << 20) | ((frame.dlc() as u32) << 16);
            let mut word: u32 = 0;
            for i in 0..frame.data().len() {
                word |= (frame.data()[i] as u32) << ((i % 4) * 8);
                if (i % 4) == 3 {
                    s[put_index].data[i / 4] = word;
                    word = 0;
                }
            }
            if frame.data().len() % 4 != 0 {
                s[put_index].data[frame.data().len() / 4] = word;
            }
            // add TX request
            unsafe {
                self.fdcan.txbar().modify(|r, w| w.ar().bits(r.bits() as u8 | ((1 << put_index) as u8)));
            }

            Ok(None)
        }
    }

    // busy-wait until all frames have been sent
    fn flush(&mut self, _clock: &mut CClock) -> nb::Result<(), Self::Error> {
        while self.fdcan.txfqs().read().tffl().bits() < 3 {}
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

    let mut cyphal_clock = CClock::new_singleton(dp.TIM2, &mut rcc);

    cyphal_clock.start();

    let hwdriver = STM32G431CanDriver::new_singleton(dp.FDCAN1, dp.GPIOA, &mut rcc);

    (cyphal_clock, GClock {}, hwdriver)
}
