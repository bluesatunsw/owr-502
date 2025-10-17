//! Cyphal (canadensis) demo program for the STM32F103.
//!
//! For the purpose of the MVP, this is a "motor controller":
//!     - Sends "speed" packets with dummy data at 100 Hz
//!         - NOTE: only sends at about 20 to 50 Hz
//!     - Listens for a file and blinks LED if checksum matches expected
//!         - NOTE: not yet implemented

#![no_std]
#![no_main]

use panic_semihosting as _;

extern crate alloc;

use canadensis::core::time as cyphal_time;
use canadensis::core::transfer::{MessageTransfer, ServiceTransfer};
use canadensis::core::transport::Transport;
use canadensis::node::data_types::Version;
use canadensis::node;
use canadensis::{Node, ResponseToken, TransferHandler};
use canadensis_can::{self, CanReceiver, CanTransmitter};
use canadensis_data_types::uavcan::node::get_info_1_0::GetInfoResponse;
use canadensis_data_types::uavcan::primitive::scalar::real64_1_0;
use canadensis_macro::types_from_dsdl;
use canadensis::encoding::Deserialize;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use embedded_alloc::LlffHeap as Heap;
use heapless;

pub mod boards;
use crate::boards::GeneralClock;

// Set up custom Cyphal data types.
types_from_dsdl! {
    package($CARGO_MANIFEST_DIR, "/dsdl")
    generate({
        // TODO: what are these?
        allow_utf8_and_byte: true,
        allow_saturated_bool: false,
    })
}
use crate::owr502::custom_speed_0_1;
use crate::owr502::custom_speed_0_1::CustomSpeed;
use crate::owr502::base_speed_1_0::BaseSpeed;

// The Cyphal node ID that this device operates as.
const NODE_ID: u8 = 1;
const CYPHAL_CONCURRENT_TRANSFERS: usize = 4;
const CYPHAL_NUM_TOPICS: usize = 8;
const CYPHAL_NUM_SERVICES: usize = 8;
const SPEED_PUBLISH_TIMEOUT_US: u32 = 1_000_000;

// Global allocator -- required by canadensis.
#[global_allocator]
static HEAP: Heap = Heap::empty();

fn initialise_allocator() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 6144;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
}

///////////
// Main. //
///////////

#[entry]
fn main() -> ! {
    // Embedded boilerplate.
    initialise_allocator();

    let (cyphal_clock, general_clock, hwdriver) = boards::init();

    // Set up Cyphal; refer to README.
    let id = canadensis_can::CanNodeId::from_truncating(NODE_ID);
    let transmitter = canadensis_can::CanTransmitter::new(canadensis_can::Mtu::CanFd64);
    let receiver = canadensis_can::CanReceiver::new(id);
    let core_node: node::CoreNode<
        boards::CClock,
        CanTransmitter<boards::CClock, boards::CanDriver>,
        CanReceiver<boards::CClock, boards::CanDriver>,
        canadensis::requester::TransferIdFixedMap<
            canadensis_can::CanTransport,
            CYPHAL_CONCURRENT_TRANSFERS,
        >,
        boards::CanDriver,
        CYPHAL_NUM_TOPICS,
        CYPHAL_NUM_SERVICES,
    > = node::CoreNode::new(cyphal_clock, id, transmitter, receiver, hwdriver);
    let info = GetInfoResponse {
        protocol_version: Version { major: 1, minor: 0 },
        hardware_version: Version { major: 1, minor: 0 },
        software_version: Version { major: 1, minor: 0 },
        software_vcs_revision_id: 0,
        unique_id: [
            // this is just garbage data
            0xFF, 0x55, 0x13, 0x37, 0x42, 0x69, 0x2A, 0xEE,
            0x78, 0x12, 0x99, 0x10, 0x00, 0x00, 0x00, 0x00,
        ],
        name: heapless::Vec::from_slice(b"org.bluesat.owr.demo").unwrap(),
        software_image_crc: heapless::Vec::new(),
        certificate_of_authenticity: heapless::Vec::new(),
    };
    let mut node = node::BasicNode::new(core_node, info).unwrap();

    // Initialise timestamps.
    let start_time: u64 = general_clock.now().ticks().into();
    let mut heartbeat_target_time_us: u64 = start_time + 1_000_000u64;
    let mut speed_time_us: u64 = start_time;
    hprintln!("start time is {}", start_time);

    // Variables for debugging only
    let mut _speed_packets_sent: u64 = 0;
    let mut _loop_iters: u32 = 0;

    // Start publishing!
    node.start_publishing(
        custom_speed_0_1::SUBJECT,
        cyphal_time::MicrosecondDuration32::from_ticks(SPEED_PUBLISH_TIMEOUT_US),
        canadensis::core::Priority::Nominal,
    ).unwrap();

    // leave a like and subscribe
    node.subscribe_message(
        canadensis::core::SubjectId::from_truncating(2),
        8, // max payload size. shouldn't this be a value we can pull from the type?
        cyphal_time::MicrosecondDuration32::from_ticks(1_000u32)
    ).unwrap();
    // If subscriptions fail with OutOfMemoryError, try upping the HEAP_SIZE in the allocator.

    loop {
        match node.receive(&mut RecvHandler) {
            Ok(_) => {}
            // TODO: handle overruns more robustly. Can currently get flooded by too many messages
            // if handling a message or message overrun itself takes too long, meaning we stop making any progress.
            //
            // Note that an overrun isn't a hardware error and isn't signalled in CAN_ESR, but
            // rather in either CAN_RF0R or CAN_RF1R. It is treated as an actual error by the bxcan
            // crate (OverrunError), though.
            //
            // Suggestion: just do receive()s in a loop when we detect OverrunError.
            //
            // TODO (stretch): use multiple hardware queues/mailboxes to optimise recv throughput? May need
            // to modify canadensis and/or bxcan drivers to do this.
            Err(e) => {
                hprintln!("recverr: {:?}", e);
            }
        }

        // Make an effort to avoid missing heartbeats
        let useconds: u64 = general_clock.now().ticks().into();
        let mut missed_heartbeats = 0;
        if useconds >= heartbeat_target_time_us {
            heartbeat_target_time_us += 1_000_000;
            while useconds >= heartbeat_target_time_us {
                missed_heartbeats += 1;
                heartbeat_target_time_us += 1_000_000;
            }
            while let Err(canadensis::core::nb::Error::WouldBlock) = node.run_per_second_tasks() {
                // block on handling heartbeat
            };
            node.flush().unwrap();
            if missed_heartbeats > 0 {
                hprintln!("WARNING: missed {} heartbeat(s)", missed_heartbeats);
            }
        }

        // Handle remaining tasks/messages on a best-effort basis
        if useconds >= speed_time_us + 10_000 {
            speed_time_us += 10_000;
            if useconds > speed_time_us {
                // in case we've fallen behind schedule
                speed_time_us = useconds + 10_000;
            }
            let speed_packet = CustomSpeed {
                rad_per_sec: BaseSpeed {
                    integer: 0x1337,
                    fraction: 0xcafe,
                },
                torque_mode: 0b1111,
            };
            match node.publish::<CustomSpeed>(custom_speed_0_1::SUBJECT, &speed_packet) {
                Ok(()) => {
                    _speed_packets_sent += 1;
                },
                Err(canadensis::core::nb::Error::WouldBlock) => {
                    // just skip publishing if blocking
                },
                Err(error) => hprintln!("Problem sending speed packet: {:?}", error),
            };
        }

        _loop_iters += 1;
    }
}

struct RecvHandler;

impl<T: Transport> TransferHandler<T> for RecvHandler {
    fn handle_message<N>(
        &mut self,
        _node: &mut N,
        transfer: &MessageTransfer<alloc::vec::Vec<u8>, T>,
    ) -> bool
    where
        N: Node<Transport = T>,
    {
        // Cast MessageTransfer to the appropriate type and get the value.
        // If using this one handler for multiple message subjects, match against transfer.header.subject.
        let recvd_real = real64_1_0::Real64::deserialize_from_bytes(transfer.payload.as_slice()).unwrap().value;
        hprintln!("Recv: {:?}", recvd_real);
        false
    }

    fn handle_request<N>(
        &mut self,
        _node: &mut N,
        _token: ResponseToken<T>,
        transfer: &ServiceTransfer<alloc::vec::Vec<u8>, T>,
    ) -> bool
    where
        N: Node<Transport = T>,
    {
        hprintln!("Got request {:?}", transfer);
        false
    }

    fn handle_response<N>(
        &mut self,
        _node: &mut N,
        transfer: &ServiceTransfer<alloc::vec::Vec<u8>, T>,
    ) -> bool
    where
        N: Node<Transport = T>,
    {
        hprintln!("Got response {:?}", transfer);
        false
    }
}
