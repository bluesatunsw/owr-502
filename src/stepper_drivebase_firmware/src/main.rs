#![allow(internal_features)]
#![feature(never_type)]
#![feature(core_intrinsics)]
#![no_std]
#![no_main]

use canadensis::{
    core::{
        time::MicrosecondDuration32, transfer::MessageTransfer, transport::Transport, SubjectId,
    },
    encoding::Deserialize,
    node::{
        data_types::{GetInfoResponse, Version},
        BasicNode, CoreNode,
    },
    requester::TransferIdFixedMap,
    Node, TransferHandler,
};
use canadensis_can::{CanNodeId, CanReceiver, CanTransmitter, CanTransport, Mtu};
use canadensis_data_types::reg::udral::physics::kinematics::rotation::planar_0_1::Planar;
use core::panic::PanicInfo;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use embedded_alloc::LlffHeap as Heap;
use embedded_common::{
    argb::{self, Colour},
    can::CanDriver,
    clock,
};
use heapless::Vec;
use stm32g4xx_hal::{
    gpio::{GpioExt, PinState},
    pac::Peripherals,
    pwr::{PwrExt, VoltageScale},
    rcc::{
        Config, FdCanClockSource, PllConfig, PllMDiv, PllNMul, PllQDiv, PllRDiv, PllSrc, RccExt,
    },
    time::{ExtU32, RateExtU32},
};

use crate::{
    common::Channel, drivebase::Drivebase, encoder_bus::EncoderNcsPins,
    stepper_bus::StepperNcsPins, tmc_registers::TmcPosition,
};

extern crate alloc;

mod as_registers;
mod common;
mod drivebase;
mod encoder_bus;
mod stepper_bus;
mod tmc_registers;

// NOTE: make sure these are configured to be the right values
const NODE_ID: u8 = 6;
const CYPHAL_CONCURRENT_TRANSFERS: usize = 4;
const CYPHAL_NUM_TOPICS: usize = 8;
const CYPHAL_NUM_SERVICES: usize = 8;

const HEARTBEAT_PERIOD_US: u32 = 1_000_000;
const TID_TIMEOUT_US: u32 = 100_000;

// Message IDs
const SETPOINT_MESSAGE_CHAN_0_ID: SubjectId = SubjectId::from_truncating(3000);
const SETPOINT_MESSAGE_CHAN_1_ID: SubjectId = SubjectId::from_truncating(3010);
const SETPOINT_MESSAGE_CHAN_2_ID: SubjectId = SubjectId::from_truncating(3020);
const SETPOINT_MESSAGE_CHAN_3_ID: SubjectId = SubjectId::from_truncating(3030);

const RED: Colour = Colour { r: 255, g: 0, b: 0 };
const BLUE: Colour = Colour { r: 0, g: 0, b: 255 };

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    hprintln!("{}", _info.message());
    loop {}
}

// Global allocator -- required by canadensis.
#[global_allocator]
static HEAP: Heap = Heap::empty();

fn initialise_allocator() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 23000;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
}

///////////
// Main. //
///////////

#[entry]
fn main() -> ! {
    initialise_allocator();

    let dp = Peripherals::take().unwrap();
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
                r: Some(PllRDiv::DIV_2),
                q: Some(PllQDiv::DIV_2),
                p: None,
            })
            .fdcan_src(FdCanClockSource::PLLQ),
        pwr,
    );

    // Set up pins.
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);

    // Set PB8 to output mode to limit power consumption on reset
    gpiob.pb8.into_push_pull_output();

    let mut argb = argb::Controller::new(dp.USART1, gpiob.pb6.into_alternate(), 15, &mut rcc);
    let clock = clock::MicrosecondClock::new(dp.TIM2, &mut rcc);
    let can = CanDriver::new(
        dp.FDCAN1,
        gpioa.pa11.into_alternate(),
        gpioa.pa12.into_alternate(),
        &mut rcc,
    );

    let mut drivebase = Drivebase::new(
        gpiod.pd2.into_push_pull_output(),
        gpioa.pa8.into_analog(),
        gpioa.pa10.into_input(),
        (
            gpioc.pc10.into_alternate(),
            gpioc.pc11.into_alternate(),
            gpioc.pc12.into_alternate(),
        ),
        StepperNcsPins(
            gpiob
                .pb9
                .into_push_pull_output_in_state(PinState::High)
                .into(),
            gpioc
                .pc13
                .into_push_pull_output_in_state(PinState::High)
                .into(),
            gpioc
                .pc14
                .into_push_pull_output_in_state(PinState::High)
                .into(),
            gpioc
                .pc15
                .into_push_pull_output_in_state(PinState::High)
                .into(),
        ),
        /*
        (
            gpiob.pb13.into_alternate(),
            gpiob.pb14.into_alternate(),
            gpiob.pb15.into_alternate(),
        ),
        EncoderNcsPins(
            gpioc
                .pc6
                .into_push_pull_output_in_state(PinState::High)
                .into(),
            gpioc
                .pc7
                .into_push_pull_output_in_state(PinState::High)
                .into(),
            gpioc
                .pc8
                .into_push_pull_output_in_state(PinState::High)
                .into(),
            gpioc
                .pc9
                .into_push_pull_output_in_state(PinState::High)
                .into(),
        ),
        dp.SPI2,
        */
        dp.SPI3,
        &mut rcc,
    );

    let id = CanNodeId::from_truncating(NODE_ID);
    let transmitter = CanTransmitter::new(Mtu::CanFd64);
    let receiver = CanReceiver::new(id);
    let core_node: CoreNode<
        _,
        _,
        _,
        TransferIdFixedMap<CanTransport, CYPHAL_CONCURRENT_TRANSFERS>,
        _,
        CYPHAL_NUM_TOPICS,
        CYPHAL_NUM_SERVICES,
    > = CoreNode::new(clock, id, transmitter, receiver, can);

    let mut node = BasicNode::new(
        core_node,
        GetInfoResponse {
            protocol_version: Version { major: 1, minor: 0 },
            hardware_version: Version { major: 1, minor: 1 },
            software_version: Version { major: 0, minor: 2 },
            software_vcs_revision_id: 0,
            unique_id: [
                // this is just garbage data for now
                0xFF, 0x55, 0x13, 0x31, 0x42, 0x69, 0x2A, 0xEE, 0x78, 0x12, 0x99, 0x10, 0x00, 0x03,
                0x00, 0x00,
            ],
            name: Vec::from_slice(b"org.bluesat.owr.stepper_drivebase").unwrap(),
            software_image_crc: Vec::new(),
            certificate_of_authenticity: Vec::new(),
        },
    )
    .unwrap();

    node.subscribe_message(
        SETPOINT_MESSAGE_CHAN_0_ID,
        size_of::<Planar>(),
        MicrosecondDuration32::from_ticks(TID_TIMEOUT_US),
    )
    .unwrap();
    node.subscribe_message(
        SETPOINT_MESSAGE_CHAN_1_ID,
        size_of::<Planar>(),
        MicrosecondDuration32::from_ticks(TID_TIMEOUT_US),
    )
    .unwrap();
    node.subscribe_message(
        SETPOINT_MESSAGE_CHAN_2_ID,
        size_of::<Planar>(),
        MicrosecondDuration32::from_ticks(TID_TIMEOUT_US),
    )
    .unwrap();
    node.subscribe_message(
        SETPOINT_MESSAGE_CHAN_3_ID,
        size_of::<Planar>(),
        MicrosecondDuration32::from_ticks(TID_TIMEOUT_US),
    )
    .unwrap();

    drivebase.enable_all();
    let mut comms_handler = CommsHandler { drivebase };

    let mut tim_heartbeat = node.clock().now_const();

    let mut tim_argb = node.clock().now_const();
    let mut argb_phase = false;

    loop {
        node.receive(&mut comms_handler).unwrap();

        if node
            .clock()
            .advance_if_elapsed(&mut tim_heartbeat, HEARTBEAT_PERIOD_US.micros())
        {
            node.run_per_second_tasks().unwrap();
        }

        if node
            .clock()
            .advance_if_elapsed(&mut tim_argb, 200_000.micros())
        {
            if argb_phase {
                argb.display(&[BLUE, RED, BLUE, RED, BLUE, RED]);
            } else {
                argb.display(&[RED, BLUE, RED, BLUE, RED, BLUE]);
            }
            argb_phase = !argb_phase;
        }
    }
}

struct CommsHandler {
    drivebase: Drivebase,
}

impl<T: Transport> TransferHandler<T> for CommsHandler {
    fn handle_message<N: Node<Transport = T>>(
        &mut self,
        _node: &mut N,
        transfer: &MessageTransfer<alloc::vec::Vec<u8>, T>,
    ) -> bool {
        let op = match transfer.header.subject {
            SETPOINT_MESSAGE_CHAN_0_ID => Some(Channel::_0),
            SETPOINT_MESSAGE_CHAN_1_ID => Some(Channel::_1),
            SETPOINT_MESSAGE_CHAN_2_ID => Some(Channel::_2),
            SETPOINT_MESSAGE_CHAN_3_ID => Some(Channel::_3),
            _ => None,
        };
        if let Some(chan) = op {
            let msg = Planar::deserialize_from_bytes(&transfer.payload);
            self.drivebase
                .set_position(chan, TmcPosition(msg.unwrap().angular_position.radian))
                .unwrap();
            true
        } else {
            false
        }
    }
}
