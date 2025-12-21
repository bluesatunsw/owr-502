#![no_std]
#![no_main]

use core::{panic::PanicInfo, ptr::copy_nonoverlapping};
use stm32g4xx_hal::stm32::CorePeripherals;

use canadensis::{
    Node,
    encoding::DataType,
    node::{BasicNode, CoreNode, data_types::{GetInfoResponse, Version}},
    requester::TransferIdFixedMap
};
use canadensis_can::{CanNodeId, CanReceiver, CanTransmitter, CanTransport, Mtu};
use heapless::Vec;
// use panic_semihosting as _;

use cortex_m_rt::entry;
use embedded_alloc::LlffHeap as Heap;

use fugit::ExtU32;

use canadensis_data_types::uavcan::node::execute_command_1_3::{ExecuteCommandRequest, SERVICE as EXECUTE_COMMAND_SERVICE};

use crate::{aux::AuxData, can::CanSystem, chunk::Chunk, clock::ClockSystem, comms_handler::CommsHandler, flash_handler::FlashHandler, peripherals::Peripherals, qspi::QspiSys};

mod peripherals;
mod clock;
mod can;
mod qspi;
mod aux;
mod chunk;
mod comms_handler;
mod flash_handler;

extern crate alloc;

const CYPHAL_CONCURRENT_TRANSFERS: usize = 4;
const CYPHAL_NUM_TOPICS: usize = 4;
const CYPHAL_NUM_SERVICES: usize = 4;
const TID_TIMEOUT_US: u32 = 1_000_000;
const VECTOR_TABLE_ADDRESS: u32 = 0x9000_0000;

const UPDATE_TIMEOUT_US: u32 = 5_000_000;

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    loop {}
}

// Global allocator -- required by canadensis.
#[global_allocator]
static HEAP: Heap = Heap::empty();

fn initialise_allocator() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 0x4000;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
}

///////////
// Main. //
///////////

#[entry]
fn main() -> ! {
    initialise_allocator();

    let mut ps = Peripherals::take();

    let mut uuid: [u8; 16] = [0; 16];
    unsafe { copy_nonoverlapping(0x1FFF7590 as *const u8, uuid.as_mut_ptr(), 12); }

    let clock_sys = ClockSystem::new(ps.clock_tim, &mut ps.rcc);
    let can_sys = CanSystem::new(ps.can_instance, ps.can_rx_pin, ps.can_tx_pin, &mut ps.rcc);
    let mut qspi_sys = QspiSys::new(
        ps.qspi_instance,
        &mut ps.rcc,

        ps.qspi_ncs_pin,
        ps.qspi_clk_pin,

        ps.qspi_io0_bank1_pin,
        ps.qspi_io1_bank1_pin,
        ps.qspi_io2_bank1_pin,
        ps.qspi_io3_bank1_pin,

        ps.qspi_io0_bank2_pin,
        ps.qspi_io1_bank2_pin,
        ps.qspi_io2_bank2_pin,
        ps.qspi_io3_bank2_pin
    );

    let id = CanNodeId::from_truncating(0);
    let transmitter = CanTransmitter::new(Mtu::CanFd64);
    let receiver = CanReceiver::new(id);
    let core_node: CoreNode<
        _, _, _,
        TransferIdFixedMap<CanTransport, CYPHAL_CONCURRENT_TRANSFERS>,
        _, CYPHAL_NUM_TOPICS, CYPHAL_NUM_SERVICES
    > = CoreNode::new(clock_sys, id, transmitter, receiver, can_sys);
    let mut node = BasicNode::new(core_node, GetInfoResponse {
        protocol_version: Version { major: 1, minor: 0 },
        hardware_version: Version { major: 0, minor: 0 },
        software_version: Version { major: 0, minor: 0 },
        software_vcs_revision_id: 0,
        unique_id: uuid,
        name: Vec::from_slice(b"org.bluesat.owr.bootloader").unwrap(),
        software_image_crc: Vec::new(),
        certificate_of_authenticity: Vec::new()
    }).unwrap();

    node.subscribe_request(
        EXECUTE_COMMAND_SERVICE,
        ExecuteCommandRequest::EXTENT_BYTES.unwrap() as usize,
        10.millis()
    ).unwrap();

    let mut comms_handler = CommsHandler::new(&mut node);
    let mut flash_handler = FlashHandler::new();

    let mut current_chunk: Option<(Chunk, usize)> = None;

    qspi_sys.chip_erase();

    loop {
        node.receive(&mut comms_handler).unwrap();
        comms_handler.tick(&mut node);
        flash_handler.tick();

        if let Some((chunk, offset)) = current_chunk {
            qspi_sys.page_program(offset.try_into().unwrap(), &chunk[0..512].as_array().unwrap());
            current_chunk = None;
        } else {
            match comms_handler.extract() {
                comms_handler::ExtractResult::Some(chunk) => {
                    current_chunk = Some((chunk, 0));
                },
                comms_handler::ExtractResult::Wait => {},
                comms_handler::ExtractResult::Done => break,
            }
        }
    }

    // SAFETY: 😊
    unsafe {
        AuxData::set(AuxData { rcc: ps.rcc });
        CorePeripherals::take().unwrap_unchecked().SCB.vtor.write(VECTOR_TABLE_ADDRESS);
        cortex_m::asm::bootload(VECTOR_TABLE_ADDRESS as *const u32);
    }
}
