use core::{convert::{Infallible, TryInto}, hint::assert_unchecked, num::NonZero, slice::from_raw_parts};

use canadensis::core::{OutOfMemoryError, time::Clock};
use canadensis_can::{CanId, driver::{self, ReceiveDriver, TransmitDriver}};
use fdcan::{NormalOperationMode, config::*, filter::{Action, ExtendedFilter, FilterType}, frame::{FrameFormat, TxFrameHeader}, id::{ExtendedId, Id}};

use stm32g4xx_hal::{can::{Can, CanExt}, rcc::Rcc};
use fugit::ExtU32;

use crate::{clock::ClockSystem, peripherals::{CanInstance, CanRxPin, CanTxPin}};

pub struct CanSystem {
    hw_can: fdcan::FdCan<Can<CanInstance>, NormalOperationMode>,
}

impl CanSystem {
    pub fn new(instance: CanInstance, rx_pin: CanRxPin, tx_pin: CanTxPin, rcc: &mut Rcc) -> Self {
        let mut hw_can = instance.fdcan(tx_pin.into_alternate(), rx_pin.into_alternate(), rcc);
        hw_can.apply_config(FdCanConfig {
            nbtr: NominalBitTiming {
                prescaler: NonZero::new(1).unwrap(),
                seg1: NonZero::new(96).unwrap(),
                seg2: NonZero::new(32).unwrap(),
                sync_jump_width: NonZero::new(16).unwrap(),
            },
            dbtr: DataBitTiming {
                transceiver_delay_compensation: true,
                prescaler: NonZero::new(1).unwrap(),
                seg1: NonZero::new(11).unwrap(),
                seg2: NonZero::new(4).unwrap(),
                sync_jump_width: NonZero::new(2).unwrap(),
            },
            automatic_retransmit: false,
            transmit_pause: false,
            frame_transmit: FrameTransmissionConfig::AllowFdCanAndBRS,
            non_iso_mode: false,
            edge_filtering: false,
            protocol_exception_handling: false,
            clock_divider: ClockDivider::_1,
            interrupt_line_config: Interrupts::none(),
            timestamp_source: TimestampSource::None,
            global_filter: GlobalFilter::reject_all(),
        });

        Self {
            hw_can: hw_can.into_normal()
        }
    }
}

fn translate_id(id: Id) -> CanId {
    // SAFETY: the invarients of both types are the same
    unsafe { match id {
        fdcan::id::Id::Standard(standard_id) => standard_id.as_raw() as u32,
        fdcan::id::Id::Extended(extended_id) => extended_id.as_raw(),
    }.try_into().unwrap_unchecked() }
}

impl ReceiveDriver<ClockSystem> for CanSystem {
    type Error = Infallible;

    fn receive(&mut self, clock: &mut ClockSystem) -> nb::Result<canadensis_can::Frame, Self::Error> {
        let mut buf: [u8; 64] = [0; 64];

        Ok(canadensis_can::Frame::new(
            clock.now(),
            translate_id(self.hw_can.receive0(&mut buf)?.unwrap().id),
            &buf
        ))
    }

    fn apply_filters<S>(&mut self, local_node: Option<canadensis_can::CanNodeId>, subscriptions: S)
    where
        S: IntoIterator<Item = canadensis::core::subscription::Subscription> {
        driver::optimize_filters(local_node, subscriptions, 8, |filters| {
            let mut res = [ExtendedFilter::disable(); 8];
            // SAFETY: optimize filters obeys the max filters
            unsafe {assert_unchecked(filters.len() <= 8)};
            for i in 0..filters.len() {
                let flt = &filters[i];
                res[i] = ExtendedFilter {
                    filter: FilterType::BitMask { filter: flt.id(), mask: flt.mask() },
                    action: Action::StoreInFifo0,
                };
            }
            self.hw_can.set_extended_filters(&res);
        }).unwrap();
    }

    fn apply_accept_all(&mut self) {
        self.hw_can.set_extended_filters(&[ExtendedFilter::accept_all_into_fifo0(); 8]);
    }
}

impl TransmitDriver<ClockSystem> for CanSystem {
    type Error = Infallible;

    fn try_reserve(&mut self, frames: usize) -> Result<(), canadensis::core::OutOfMemoryError> {
       // the TX buffer has space for three messages
        if frames > 3 {
            Err(OutOfMemoryError)
        } else {
            Ok(())
        }
    }

    fn transmit(&mut self, frame: canadensis_can::Frame, clock: &mut ClockSystem) -> nb::Result<Option<canadensis_can::Frame>, Self::Error> {
        // Should always work since we just checked for room
        self.hw_can.transmit_preserve(TxFrameHeader {
            // SAFETY: Equivlent invarients
            len: unsafe { frame.data().len().try_into().unwrap_unchecked() },
            frame_format: FrameFormat::Fdcan,
            // SAFETY: Equivlent invarients
            id: unsafe { ExtendedId::new(u32::from(frame.id())).unwrap_unchecked().into() },
            bit_rate_switching: true,
            marker: None 
        }, frame.data(), &mut |_, fh, bf| {
            canadensis_can::Frame::new(
                // TODO: need to modify the API of the CAN HAL to return the mailbox
                // during normal transmission
                clock.now() + 10.millis(),
                translate_id(fh.id),
                unsafe {from_raw_parts(bf.as_ptr() as *const u8, bf.len() * 4)}
            )
        })
    }

    fn flush(&mut self, clock: &mut ClockSystem) -> nb::Result<(), Self::Error> {
        if self.hw_can.is_transmitter_idle() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
