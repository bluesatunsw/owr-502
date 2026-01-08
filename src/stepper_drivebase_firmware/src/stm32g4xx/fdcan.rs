//! FDCAN driver for canadensis.
//!
//! It has come to my attention that this was mostly wasted effort, as an FDCAN driver is in fact
//! part of the STM32G4 Rust HAL; it's just hidden behind a feature flag. Oh well.

use hal::{pac, gpio};
use stm32g4xx_hal as hal;

use canadensis::core::{time::{self, Clock}, OutOfMemoryError, subscription::Subscription};
use canadensis_can::{driver, Frame, CanId, CanNodeId};
use canadensis_filter_config;

use core::slice;
use core::convert::TryFrom;

use crate::stm32g4xx::clock;

// Helper function that converts an FDCAN DLC (data length code) to the size in bytes.
fn fddlc_to_bytes(dlc: u8) -> usize {
    match dlc {
        0..=8 => dlc as usize,
        9 => 12,
        10 => 16,
        11 => 20,
        12 => 24,
        13 => 32,
        14 => 48,
        15 => 64,
        16..=u8::MAX => unreachable!("DLC is always between 0 and 16 inclusive"),
    }
}

pub struct STM32G4xxCanDriver {
    fdcan: pac::FDCAN1,
    _pa11: gpio::PA11<gpio::AF9>,
    _pa12: gpio::PA12<gpio::AF9>,
}

impl STM32G4xxCanDriver {
    pub fn new_singleton(fdcan: pac::FDCAN1, pa11: gpio::PA11<gpio::AF9>, pa12: gpio::PA12<gpio::AF9>, rcc: &mut pac::RCC) -> Self {
        // this constant is used in two different places so it's set here
        // see the DBTP section for context
        const DATA_SAMPLE_PERIOD: u8 = 11;

        // Reset and enable the FDCAN1 peripheral.
        rcc.apb1enr1().modify(|_, w| w.fdcanen().bit(true));
        rcc.apb1rstr1().modify(|_, w| w.fdcanrst().bit(true));
        rcc.apb1rstr1().modify(|_, w| w.fdcanrst().bit(false));

        // Set speeds of pins to medium; should be acceptable for 8 MHz data rate.
        let pa11 = pa11.speed(gpio::Speed::Medium);
        let pa12 = pa12.speed(gpio::Speed::Medium);

        // Configure FDCAN1.
        // Set CCE of CCCR (we just reset so INIT should be set)
        fdcan.cccr().modify(|_, w| w.cce().bit(true));
        fdcan.cccr().modify(|_, w|
            w.pxhd().bit(true)   // disable protocol handling exception
            //.niso().bit(true)
            .brse().bit(true)   // enable bit rate switching for transmissions
            // keep EFBI (edge filtering) disabled, apparently there's errata
            .fdoe().bit(true)   // enable FD operation
            .dar().bit(true)    // disable automatic retransmission
        );
        unsafe {
            // For data bit rate = 8 MHz, FDCAN_CLK = 128 MHz:
            // https://kvaser.com/support/calculators/can-fd-bit-timing-calculator/
            // (actually, these are copied directly from the CAN-to-USB converter autoconfig)
            // nominal bit sample point 75%
            fdcan.dbtp().write(|w|
                w.tdc().bit(true)       // enable transceiver delay compensation
                .dbrp().bits(1 - 1)     // data bit rate prescaler
                                        // t_q = (0b0000 + 1) clock period(s)
                .dtseg1().bits(DATA_SAMPLE_PERIOD - 1)   // data time segment before sample point
                                        // = PROP_SEG + PHASE_SEG1
                .dtseg2().bits(4 - 1)   // data time segment after sample point
                                        // = PHASE_SEG2 = 4 t_q
                .dsjw().bits(2 - 1)     // synchronisation jump width
                                        // = 2 t_q
            );
            // For nominal bit rate = 1 MHz, FDCAN_CLK = 128 MHz:
            // sample point 75%
            fdcan.nbtp().write(|w|
                w.nbrp().bits(1 - 1)    // nominal bit rate prescaler
                                        // = 1 (no scaling)
                .ntseg1().bits(95 - 1)  // nominal time segment before sample point
                                        // = PROP_SEG + PHASE_SEG1 = 95 t_q
                .ntseg2().bits(32 - 1)   // nominal time segment after sample point
                                        // = PHASE_SEG2 = 32 t_q
                .nsjw().bits(16 - 1)     // nominal (re)synchronisation jump width
                                        // = 16 t_q
            );

            // Transmitter delay compensation becomes necessary above a data rate of about 4.5 MHz
            // I don't actually know what a good value to put in the TDCF is...
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

        STM32G4xxCanDriver {
            fdcan,
            // need to take ownership of pins
            _pa11: pa11,
            _pa12: pa12
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
    fn do_apply_filters(&mut self, filters: &[canadensis_filter_config::Filter]) {
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
    data: [u32; 16],
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

const FDCAN1_FLESA: *mut ExtMsgFilterElement = (0x4000_A400 + 0x0070) as *mut ExtMsgFilterElement;
const FDCAN1_TXFIFO: *mut FDCanTxFifoElement = (0x4000_A400 + 0x0278) as *mut FDCanTxFifoElement;
const FDCAN1_RXFIFO0: *const FDCanRxFifoElement = (0x4000_A400 + 0x00B0) as *const FDCanRxFifoElement;

struct RxFifo0 {}

impl RxFifo0 {
    // Make a frame from the element at index
    // idx should indicate a valid RX FIFO message
    fn make_frame(idx: usize, timestamp: time::Microseconds32) -> Frame {
        if idx >= 3 {
            panic!("Index {} out of bounds for the FDCAN1 Rx FIFO 0", idx);
        }
        let fifo: &[FDCanRxFifoElement] = unsafe{slice::from_raw_parts(FDCAN1_RXFIFO0, 3)};
        let element = &fifo[idx];
        let dlc: u8 = ((element.r1 >> 16) & 0x0f) as u8;
        let ext_id: u32 = element.r0 & ((1 << 29) - 1);
        let mut element_data_bytes: [u8; 64] = [0; 64];
        let mut word = 0; // bad C habits sorry
        for i in 0..fddlc_to_bytes(dlc) {
            // have to convert endianness
            if i % 4 == 0 {
                word = element.data[i / 4];
            }
            element_data_bytes[i] = ((word >> ((i % 4) * 8)) & 0xff) as u8;
        }
        Frame::new(timestamp, CanId::try_from(ext_id).unwrap(), &element_data_bytes[0..fddlc_to_bytes(dlc)])
    }
}

/// Extremely basic driver that just uses the hardware RX FIFO 0
impl driver::ReceiveDriver<clock::STM32G4xxCyphalClock> for STM32G4xxCanDriver {
    type Error = OverrunError;

    // literally just returns a CAN frame if one is available
    fn receive(&mut self, clock: &mut clock::STM32G4xxCyphalClock) -> Result<Frame, nb::Error<OverrunError>> {
        // read some kind of register to check if CAN frames received
        let rxf0s = self.fdcan.rxf0s().read();
        // TODO: check error status register? do more error checking, fault on the FIFO being full
        if rxf0s.f0fl().bits() == 0 {
            // fill level is 0, so receiving a message would block
            Err(nb::Error::WouldBlock)
        } else {
            // make the frame from the data in the message RAM
            let idx: u8 = rxf0s.f0gi().bits();
            let frame: Frame = RxFifo0::make_frame(rxf0s.f0gi().bits().into(), clock.now());
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
        if let Err(_) = driver::optimize_filters(local_node, subscriptions, 8, |filters| self.do_apply_filters(filters)) {
            // OutOfMemory
            self.reset_filters();
        };
    }

    fn apply_accept_all(&mut self) {
        self.reset_filters();
    }
}

/// Extremely basic driver
impl driver::TransmitDriver<clock::STM32G4xxCyphalClock> for STM32G4xxCanDriver {
    // operate the TX in FIFO (transmit in order of addition) instead of queue (transmit ordered by
    // message priority/ID) mode since the latter could lead to starvation
    // CHECKME: not sure above is the right choice?
    // TODO: update to pqueue
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
    // TODO: corroborate with queue mode above
    fn transmit(
        &mut self,
        frame: Frame,
        _clock: &mut clock::STM32G4xxCyphalClock,
    ) -> nb::Result<Option<Frame>, Self::Error> {
        let txfqs = self.fdcan.txfqs();
        if txfqs.read().tfqf().bit() {
            // queue is full
            Err(nb::Error::WouldBlock)
        } else {
            // set data in TX buffer appropriately
            let put_index: usize = txfqs.read().tfqpi().bits().into();
            let s: &mut [FDCanTxFifoElement] = unsafe{slice::from_raw_parts_mut(FDCAN1_TXFIFO, 3)};
            //                ext. id
            s[put_index].t0 = (1 << 30) | u32::from(frame.id());
            //                CAN FD + BRS
            s[put_index].t1 = (0b11 << 20) | ((frame.dlc() as u32) << 16);
            let mut word: u32 = 0;
            // TODO: ensure zero-padding until DLC length
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
    fn flush(&mut self, _clock: &mut clock::STM32G4xxCyphalClock) -> nb::Result<(), Self::Error> {
        // TODO: add timeout, since this blocks if there's an issue with the transceiver
        while self.fdcan.txfqs().read().tffl().bits() < 3 {}
        Ok(())
    }
}


