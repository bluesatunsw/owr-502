//! Cyphal (canadensis) demo program for the STM32F103.
//!
//! For the purpose of the MVP, this is a "motor controller":
//!     - Sends "speed" packets with dummy data at 100 Hz
//!     - Listens for a file and blinks LED if checksum matches expected

// Have to remove the following attribute since it's apparently incompatible with the
// embedded_alloc allocator.
//#![deny(unsafe_code)]

#![no_std]
#![no_main]

use canadensis::node::data_types::Version;
// TODO: Set a more stable panic behaviour.
use panic_halt as _;

extern crate alloc;

use bxcan;
use canadensis::core::time as cyphal_time;
use canadensis::core::transfer::{MessageTransfer, ServiceTransfer};
use canadensis::core::transport::Transport;
use canadensis::node;
use canadensis::{Node, ResponseToken, TransferHandler};
use canadensis_bxcan::{self as cbxcan, BxCanDriver};
use canadensis_can::{self, CanReceiver, CanTransmitter};
use canadensis_data_types::uavcan::node::get_info_1_0::GetInfoResponse;
use canadensis_macro::types_from_dsdl;
use core::cell::RefCell;
use core::panic;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use cyphal_time::Clock;
use cyphal_time::Instant;
use embedded_alloc::LlffHeap as Heap;
use heapless;
use nb::block;
use stm32f1xx_hal::pac::interrupt;
use stm32f1xx_hal::time as hw_time;
use stm32f1xx_hal::{
    can, pac,
    prelude::*,
    rcc,
    rcc::{Enable, Reset},
    timer,
    timer::{counter, Timer},
};

// Set up custom Cyphal data types.
types_from_dsdl! {
    package($CARGO_MANIFEST_DIR, "/dsdl")
    generate()
}
use crate::owr502::custom_speed_0_1;
use crate::owr502::custom_speed_0_1::CustomSpeed;

// The Cyphal node ID that this device operates as.
const NODE_ID: u8 = 1;
const CYPHAL_CONCURRENT_TRANSFERS: usize = 4;
const CYPHAL_NUM_TOPICS: usize = 8;
const CYPHAL_NUM_SERVICES: usize = 8;

// Global allocator -- required by canadensis.
#[global_allocator]
static HEAP: Heap = Heap::empty();

fn initialise_allocator() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
}

//////////////////////////////////////////////////////////////////////////////
// Implementations of platform-specific constructs that canadensis requires //
//////////////////////////////////////////////////////////////////////////////

struct STM32F103CyphalClock {
    // TIM2 allocated to the microsecond Cyphal timer, lower 16 bits
    hw_timer_lower: pac::TIM2,
    // TIM3 allocated to the microsecond Cyphal timer, upper 16 bits
    hw_timer_upper: pac::TIM3,
    // TODO: Add either interrupt handling (as per previous iteration of this program) or chain a
    // third timer to extend this timer to 48 bits.
}

// Internal 1 MHz timer to provide time instants for Cyphal.
impl STM32F103CyphalClock {
    fn new(tim2: pac::TIM2, tim3: pac::TIM3) -> Self {
        // Enable and reset the timer peripheral.
        unsafe {
            let rcc_ptr = &(*pac::RCC::ptr());
            pac::TIM2::enable(rcc_ptr);
            pac::TIM3::enable(rcc_ptr);
            pac::TIM2::reset(rcc_ptr);
            pac::TIM3::reset(rcc_ptr);
            Self {
                hw_timer_lower: tim2,
                hw_timer_upper: tim3,
            }
        }
    }

    // Configure timers: count up lower at 1 MHz, wraparound and update upper on overflow.
    fn start(&mut self) {
        // updates on overflow happen automatically
        // set frequency to external clock (8MHz)
        self.hw_timer_lower.smcr.write(|w| w.sms().bits(0b000)); // select CK_INT
        self.hw_timer_lower.psc.write(|w| w.psc().bits(0x0007)); // scale 8MHz to 1MHz

        // See STM RM0008 Section 15.3.15 Timer synchronization,
        // "Using one timer as prescaler for another timer" for hints on this section.
        // Configure lower timer to generate external trigger output on update (i.e. overflow).
        self.hw_timer_lower.cr2.write(|w| w.mms().bits(0b010));
        // Configure upper timer to be in external clock mode, clocked by ITR1
        // => TIM3 clocked by TIM2 updates (Table 86)
        unsafe {
            self.hw_timer_upper
                .smcr
                .write(|w| w.ts().bits(0b001).sms().bits(0b111));
        }
        // set counters to 0 initially
        self.hw_timer_lower.cnt.write(|w| w.cnt().bits(0));
        self.hw_timer_upper.cnt.write(|w| w.cnt().bits(0));
        // start timers
        self.hw_timer_lower
            .cr1
            .write(|w| w.urs().set_bit().cen().set_bit());
        self.hw_timer_upper.cr1.write(|w| w.cen().set_bit());
        // desired defaults:
        // DIR = 0 -> count up
        // CMS = 00 -> edge-aligned mode
        // OPM = 0
    }

    fn now(&self) -> u32 {
        // somewhat hacky purposes
        // TODO: need same edge case fix as for CyphalClock::now()
        (self.hw_timer_upper.cnt.read().bits() << 16) + self.hw_timer_lower.cnt.read().bits()
    }
}

// make clock globally available, so we can write the interrupt handler for it
static G_CYPHAL_CLOCK: Mutex<RefCell<Option<STM32F103CyphalClock>>> =
    Mutex::new(RefCell::new(None));

struct CyphalClock {}

// exists to be instantiable locally but passes everything through to global state
// (state has to be global to be modifiable by interrupt handler)
impl CyphalClock {
    fn new_singleton(tim2: pac::TIM2, tim3: pac::TIM3) -> Self {
        // takes in a few hardware peripherals
        cortex_m::interrupt::free(|cs| {
            *G_CYPHAL_CLOCK.borrow(cs).borrow_mut() = Some(STM32F103CyphalClock::new(tim2, tim3))
        });
        CyphalClock {}
    }

    fn start(&self) {
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

impl cyphal_time::Clock for CyphalClock {
    type Instant = cyphal_time::Microseconds48;

    fn now(&mut self) -> Self::Instant {
        // TODO: Handle edge case where timer overflows in between reading upper and lower bits
        cortex_m::interrupt::free(|cs| {
            if let Some(cyphal_clock) = G_CYPHAL_CLOCK.borrow(cs).borrow_mut().as_mut() {
                let lower_time = cyphal_clock.hw_timer_lower.cnt.read().bits();
                cyphal_time::Microseconds48::new(cyphal_time::u48::U48::from(
                    (cyphal_clock.hw_timer_upper.cnt.read().bits() << 16) + lower_time,
                ))
            } else {
                panic!("Could not borrow global cyphal clock")
            }
        })
    }
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
    let start_time = cortex_m::interrupt::free(|cs| {
        if let Some(cyphal_clock) = G_CYPHAL_CLOCK.borrow(cs).borrow_mut().as_mut() {
            cyphal_clock.now()
        } else {
            panic!("Could not borrow global cyphal clock")
        }
    });
    let mut heartbeat_time_s = 0;
    let mut speed_time_s = start_time;
    hprintln!("start time is {}", start_time);

    // Start publishing!
    let publish_token = node.start_publishing::<CustomSpeed>(
        custom_speed_0_1::SUBJECT,
        cyphal_time::MicrosecondDuration48::new(cyphal_time::u48::U48::from(1_000_000u32)), // I don't know what a sensible timeout should be.
                                               // One second?
        canadensis::core::Priority::Nominal,
    ).unwrap();

    loop {
        match node.receive(&mut EmptyHandler) {
            Ok(_) => {}
            Err(e) => panic!("{:?}", e),
        }

        let seconds = cortex_m::interrupt::free(|cs| {
            if let Some(cyphal_clock) = G_CYPHAL_CLOCK.borrow(cs).borrow_mut().as_mut() {
                cyphal_clock.now()
            } else {
                panic!("Could not borrow global cyphal clock")
            }
        });

        hprintln!("current time is {}", seconds);
        hprintln!("previous heartbeat target time is {}", heartbeat_time_s);

        if seconds >= speed_time_s + 10_000 {
            speed_time_s += 10_000;
            // send speed data packet
            // create type
            // serialise
            // send type
        }

        if seconds >= heartbeat_time_s + 1_000_000 {
            heartbeat_time_s += 1_000_000;
            hprintln!("running once-per-second tasks");
            node.run_per_second_tasks().unwrap();
            node.flush().unwrap();
            hprintln!("finished tasks");
        }
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
