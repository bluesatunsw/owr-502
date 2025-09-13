//! Cyphal (canadensis) demo program for the STM32F103.
//!
//! For the purpose of the MVP, this is a "motor controller":
//!     - Sends "speed" packets with dummy data at 100 Hz
//!     - Listens for a file and blinks LED if checksum matches expected

#![no_std]
#![no_main]

// TODO: Set a more stable panic behaviour.
use panic_semihosting as _;

extern crate alloc;

use bxcan;
use canadensis::core::time as cyphal_time;
use canadensis::core::transfer::{MessageTransfer, ServiceTransfer};
use canadensis::core::transport::Transport;
use canadensis::node::data_types::Version;
use canadensis::node;
use canadensis::{Node, ResponseToken, TransferHandler};
use canadensis_bxcan::{self as cbxcan, BxCanDriver};
use canadensis_can::{self, CanReceiver, CanTransmitter};
use canadensis_data_types::uavcan::node::get_info_1_0::GetInfoResponse;
use canadensis_macro::types_from_dsdl;
use core::panic;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use cyphal_time::Instant;
use cyphal_time::u48::U48;
use embedded_alloc::LlffHeap as Heap;
use heapless;
use nb::block;

#[cfg(feature = "stm32f103")]
use crate::boards::stm32f103::{CyphalClock, GeneralClock};

pub mod boards;

#[cfg(feature = "stm32f103")]
use stm32f1xx_hal::time as hw_time;
#[cfg(feature = "stm32f103")]
use stm32f1xx_hal::{
    can, pac,
    prelude::*,
    rcc,
    timer::Timer,
};

// Set up custom Cyphal data types.
types_from_dsdl! {
    package($CARGO_MANIFEST_DIR, "/dsdl")
    generate()
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
    const HEAP_SIZE: usize = 1024;
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

    let dp = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc: rcc::Rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        // STM32F103 board we're using has 8 MHz external clock; use that.
        .use_hse(hw_time::Hertz::MHz(8))
        .pclk1(hw_time::Hertz::MHz(8)) // for APB1, which CAN is on.
        .freeze(&mut flash.acr);

    // Set up microsecond clock for Cyphal.
    // TODO: Use singleton pattern so we can get an actual local object that we can pass through to
    // canadensis?
    let tim2 = dp.TIM2;
    let tim3 = dp.TIM3;
    let cyphal_clock = CyphalClock::new_singleton(tim2, tim3);
    cyphal_clock.start();

    // 1 Hz timer for debugging only
    let mut other_timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    other_timer.start(1.Hz()).unwrap();

    // Assemble and enable the CAN peripheral.
    let mut gpioa = dp.GPIOA.split();
    let can_tx = gpioa.pa12.into_alternate_open_drain(&mut gpioa.crh);
    let can_rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
    let can: can::Can<pac::CAN1> = can::Can::new(dp.CAN1, dp.USB);
    let mut afio = dp.AFIO.constrain();
    can.assign_pins((can_tx, can_rx), &mut afio.mapr);
    // Calculated for bit rate 1000 kbps, PCLK1 = 8 MHz, sample-point 87.5%, SJW = 1
    // See http://www.bittiming.can-wiki.info/, https://docs.rs/bxcan/0.7.0/bxcan/struct.CanBuilder.html
    let livecan = bxcan::Can::builder(can)
        .set_bit_timing(0x00050000)
        .set_loopback(false)
        .set_silent(false)
        .enable();

    // Set up Cyphal; refer to README.
    let id = canadensis_can::CanNodeId::from_truncating(NODE_ID);
    let transmitter = canadensis_can::CanTransmitter::new(canadensis_can::Mtu::Can8);
    let receiver = canadensis_can::CanReceiver::new(id, canadensis_can::Mtu::Can8);
    let driver = cbxcan::BxCanDriver::new(livecan);
    let core_node: node::CoreNode<
        CyphalClock,
        CanTransmitter<CyphalClock, BxCanDriver<CyphalClock, can::Can<pac::CAN1>>>,
        CanReceiver<CyphalClock, BxCanDriver<CyphalClock, can::Can<pac::CAN1>>>,
        canadensis::requester::TransferIdFixedMap<
            canadensis_can::CanTransport,
            CYPHAL_CONCURRENT_TRANSFERS,
        >,
        BxCanDriver<CyphalClock, can::Can<pac::CAN1>>,
        CYPHAL_NUM_TOPICS,
        CYPHAL_NUM_SERVICES,
    > = node::CoreNode::new(cyphal_clock, id, transmitter, receiver, driver);
    let info = GetInfoResponse {
        protocol_version: Version { major: 1, minor: 0 },
        hardware_version: Version { major: 1, minor: 0 },
        software_version: Version { major: 1, minor: 0 },
        software_vcs_revision_id: 0,
        unique_id: [
            0xFF, 0x55, 0x13, 0x37, 0x42, 0x69, 0x2A, 0xEE, 0x78, 0x12, 0x99, 0x10, 0x00, 0x00,
            0x00, 0x00,
        ],
        name: heapless::Vec::from_slice(b"org.bluesat.owr.demo").unwrap(),
        software_image_crc: heapless::Vec::new(),
        certificate_of_authenticity: heapless::Vec::new(),
    };
    let mut node = node::BasicNode::new(core_node, info).unwrap();

    // Initialise timestamps.
    let start_time: u64 = GeneralClock::now().as_microseconds().into();
    let mut heartbeat_target_time_us: u64 = start_time + 1_000_000u64;
    let mut speed_time_us: u64 = start_time;
    let mut speed_packets_sent: u64 = 0;
    let mut loop_iters: u32 = 0;
    let mut would_block: u32 = 0;
    hprintln!("start time is {}", start_time);

    // Start publishing!
    let publish_token = node.start_publishing::<CustomSpeed>(
        custom_speed_0_1::SUBJECT,
        cyphal_time::MicrosecondDuration48::new(U48::from(SPEED_PUBLISH_TIMEOUT_US)),
        canadensis::core::Priority::Nominal,
    ).unwrap();

    loop {
        match node.receive(&mut EmptyHandler) {
            Ok(_) => {}
            Err(e) => panic!("{:?}", e),
        }

        // Make an effort to avoid missing heartbeats
        let useconds: u64 = GeneralClock::now().as_microseconds().into();
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
            hprintln!("Once-per-second tasks run at {}", useconds);
            if missed_heartbeats > 0 {
                hprintln!("WARNING: missed {} heartbeat(s)", missed_heartbeats);
            }
            if (useconds / 1_000_000) % 5 == 0 {
                hprintln!("{} in {}; {} wb", speed_packets_sent, loop_iters, would_block);
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
            match node.publish::<CustomSpeed>(&publish_token, &speed_packet) {
                Ok(()) => {
                    speed_packets_sent += 1;
                },
                Err(canadensis::core::nb::Error::WouldBlock) => {
                    // just skip publishing if blocking
                    would_block += 1;
                },
                Err(error) => hprintln!("Problem sending speed packet: {:?}", error),
            };
        }

        loop_iters += 1;
    }
}

struct EmptyHandler;

impl<I: Instant, T: Transport> TransferHandler<I, T> for EmptyHandler {
    fn handle_message<N>(
        &mut self,
        _node: &mut N,
        transfer: &MessageTransfer<alloc::vec::Vec<u8>, I, T>,
    ) -> bool
    where
        N: Node<Transport = T>,
    {
        hprintln!("Got message {:?}", transfer);
        false
    }

    fn handle_request<N>(
        &mut self,
        _node: &mut N,
        _token: ResponseToken<T>,
        transfer: &ServiceTransfer<alloc::vec::Vec<u8>, I, T>,
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
        transfer: &ServiceTransfer<alloc::vec::Vec<u8>, I, T>,
    ) -> bool
    where
        N: Node<Transport = T>,
    {
        hprintln!("Got response {:?}", transfer);
        false
    }
}
