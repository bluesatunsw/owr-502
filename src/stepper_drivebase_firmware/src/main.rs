//! Firmware for the stepper driver board on the rover drivebase.
//!
//! This mainly publishes sensor data over Cyphal/CAN (on the FDCAN) on a variety of subjects
//! and drives the stepper motors as instructed by Cyphal/CAN messages from the OBC.

#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use embedded_alloc::LlffHeap as Heap;
use canadensis::node::{self, data_types::Version};
use canadensis_can::{CanReceiver, CanTransmitter};
use canadensis::{Node, ResponseToken, TransferHandler};
use canadensis::core::SubjectId;
use canadensis::core::transfer::{MessageTransfer, ServiceTransfer};
use canadensis::core::transport::Transport;
use canadensis::core::time as cyphal_time;

// so many data types :O
use canadensis_data_types::uavcan::node::get_info_1_0::GetInfoResponse;
use canadensis_data_types::reg::udral::physics::dynamics::rotation::planar_0_1::Planar as PlanarTorque;
use canadensis_data_types::reg::udral::physics::kinematics::rotation::planar_0_1::Planar as Planar;
use canadensis_data_types::uavcan::si::unit::torque::scalar_1_0::Scalar as TorqueScalar;
use canadensis_data_types::uavcan::si::unit::angle::scalar_1_0::Scalar as AnglePosScalar;
use canadensis_data_types::uavcan::si::unit::angular_velocity::scalar_1_0::Scalar as AngleVelScalar;
use canadensis_data_types::uavcan::si::unit::angular_acceleration::scalar_1_0::Scalar as AngleAccelScalar;

extern crate alloc;

mod boards;
use crate::boards::{GeneralClock, RGBLEDDriver, RGBLEDColor, StepperDriver, StepperChannel, StepperRegister};

// NOTE: make sure these are configured to be the right values
const NODE_ID: u8 = 6;
const CYPHAL_CONCURRENT_TRANSFERS: usize = 4;
const CYPHAL_NUM_TOPICS: usize = 8;
const CYPHAL_NUM_SERVICES: usize = 8;
const TID_TIMEOUT_US: u32 = 1_000_000;

// Global allocator -- required by canadensis.
#[global_allocator]
static HEAP: Heap = Heap::empty();

fn initialise_allocator() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 16000;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
}

///////////
// Main. //
///////////

#[entry]
fn main() -> ! {
    initialise_allocator();

    // Initialise all communcation interfaces and hardware drivers, including those for canadensis.
    let (cyphal_clock, general_clock, hcan, mut hled, hi2c, mut hstepper) = boards::init();

    // Initialise canadensis node.
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
    > = node::CoreNode::new(cyphal_clock, id, transmitter, receiver, hcan);

    let hardware_version = if cfg!(feature = "rev_a2") {
        Version { major: 1, minor: 0 }
    } else if cfg!(feature = "rev_b2") {
        Version { major: 1, minor: 1 }
    } else {
        Version { major: 0, minor: 0 }
    };
    let info = GetInfoResponse {
        protocol_version: Version { major: 1, minor: 0 },
        hardware_version,
        // NOTE: remember to update this number!
        software_version: Version { major: 0, minor: 1 },
        software_vcs_revision_id: 0,
        // TODO: get this from the chip?
        unique_id: [
            // this is just garbage data for now
            0xFF, 0x55, 0x13, 0x31, 0x42, 0x69, 0x2A, 0xEE,
            0x78, 0x12, 0x99, 0x10, 0x00, 0x03, 0x00, 0x00,
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
    
    // Publish on the drivebase subjects.
    let planar_torque_subject = SubjectId::from_truncating(1337);
    node.start_publishing(
        // TODO: THIS IS PLACEHOLDER SUBJECT/PORT
        planar_torque_subject,
        // this is transmission timeout, make it the window of time within which each message is
        // still relevant VVVVVV
        cyphal_time::MicrosecondDuration32::from_ticks(1_000),
        canadensis::core::Priority::Nominal,
    ).unwrap();

    // Hit that subscribe button to the relevant subjects.
    node.subscribe_message(
        // TODO: THIS IS PLACEHOLDER SUBJECT/PORT
        canadensis::core::SubjectId::from_truncating(49),
        8, // max payload size. shouldn't this be a value we can pull from the type?
        cyphal_time::MicrosecondDuration32::from_ticks(TID_TIMEOUT_US)
    ).unwrap();
    // NOTE: If subscriptions fail with OutOfMemoryError, try upping the HEAP_SIZE in the allocator.

    hstepper.enable_all();
    hprintln!("Enabled steppers");

    let mut cycles = 0;
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
        // TODO: we may not actually care that much about this in a production context
        let useconds: u64 = general_clock.now().ticks().into();
        let mut missed_heartbeats = 0;
        if useconds >= heartbeat_target_time_us {
            hprintln!("whoa");
            heartbeat_target_time_us += 1_000_000;
            while useconds >= heartbeat_target_time_us {
                missed_heartbeats += 1;
                heartbeat_target_time_us += 1_000_000;
            }
            while let Err(canadensis::core::nb::Error::WouldBlock) = node.run_per_second_tasks() {
                // block on handling heartbeat
            };
            hprintln!("handled tasks");

            // RGB LED test routine!
            let led_color = RGBLEDColor {
                red: if cycles % 3 == 0 { 0xFF } else { 0x00 },
                blue: if cycles % 3 == 1 { 0xFF } else { 0x00 },
                green: if cycles % 3 == 1 { 0xFF } else { 0x00 },
            };
            let led_color_2: u32 = 0xF0F000 >> (8 * ((cycles + 1) % 3));
            cycles += 1;
            hled.set_nth_led(0, led_color);
            hled.set_nth_led_and_render(1, led_color_2.into());

            // Stepper Driver test routine!
            let chan1temp = hstepper.get_temperature(StepperChannel::Channel1);
            let chan2temp = hstepper.get_temperature(StepperChannel::Channel2);
            let chan3temp = hstepper.get_temperature(StepperChannel::Channel3);
            let chan4temp = hstepper.get_temperature(StepperChannel::Channel4);
            let gconf = hstepper.read_reg(StepperChannel::Channel1, StepperRegister::GCONF).unwrap();
            let gstat = hstepper.read_reg(StepperChannel::Channel1, StepperRegister::GSTAT).unwrap();
            let ifcnt = hstepper.read_reg(StepperChannel::Channel1, StepperRegister::IFCNT);
            let ioin = hstepper.read_reg(StepperChannel::Channel1, StepperRegister::IOIN_OUTPUT).unwrap();
            hprintln!("Temps: {:?} {:?} {:?} {:?} GCONF1 0x{:08x} IOIN 0x{:08x}", chan1temp, chan2temp, chan3temp, chan4temp, gconf, ioin);

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
            let planar_msg = PlanarTorque {
                kinematics: Planar {
                    angular_position: AnglePosScalar {
                        radian: -0.04,
                    },
                    angular_velocity: AngleVelScalar {
                        radian_per_second: 2.111,
                    },
                    angular_acceleration: AngleAccelScalar {
                        radian_per_second_per_second: 0.0,
                    }
                },
                torque: TorqueScalar {
                    newton_meter: 3.14,
                }
            };
            match node.publish::<PlanarTorque>(planar_torque_subject, &planar_msg) {
                Ok(()) => {},
                Err(canadensis::core::nb::Error::WouldBlock) => {
                    // just skip publishing if blocking
                },
                Err(error) => hprintln!("Problem sending planar message: {:?}", error),
            };
        }
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
        hprintln!("Got message {:?}", transfer);
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
