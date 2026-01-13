#![no_std]
#![no_main]
#![feature(ptr_as_ref_unchecked)]
#![feature(unsafe_cell_access)]
#![feature(naked_functions_rustic_abi)]

use core::{arch::naked_asm, hint::black_box, panic::PanicInfo, ptr::copy_nonoverlapping};

use canadensis::{
    Node,
    encoding::DataType,
    node::{BasicNode, CoreNode, data_types::{GetInfoResponse, Version}},
    requester::TransferIdFixedMap
};
use canadensis_can::{CanNodeId, CanReceiver, CanTransmitter, CanTransport, Mtu};
use cortex_m::asm::bkpt;
use cortex_m_semihosting::hprintln;
use heapless::Vec;
// use panic_semihosting as _;

use cortex_m_rt::{ExceptionFrame, entry, exception};
use embedded_alloc::LlffHeap as Heap;

use fugit::ExtU32;

use canadensis_data_types::uavcan::node::execute_command_1_3::{ExecuteCommandRequest, SERVICE as EXECUTE_COMMAND_SERVICE};

use crate::{argb::{ArgbSys, State}, can::CanSystem, clock::ClockSystem, comms_handler::{CommsHandler, CommsHandlerState}, crc_handler::{CrcHandler, CrcHandlerState}, flash_handler::{FlashHandler, FlashHandlerState}, peripherals::Peripherals, qspi::QspiSys};

mod peripherals;
mod clock;
mod can;
mod qspi;
mod chunk;
mod common;
mod argb;
mod comms_handler;
mod flash_handler;
mod crc_handler;

extern crate alloc;

const CYPHAL_CONCURRENT_TRANSFERS: usize = 4;
const CYPHAL_NUM_TOPICS: usize = 4;
const CYPHAL_NUM_SERVICES: usize = 4;
const TID_TIMEOUT_US: u32 = 1_000_000;

const UPDATE_TIMEOUT_US: u32 = 5_000_000;

const UID_ADDRESS: u32 = 0x1FFF_7590;

const RAM_START: *mut u32 = 0x2000_0000 as *mut u32;
const FLASH_START: *const u32 = 0x0800_0000 as *const u32;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    hprintln!("{}", info.message().as_str().unwrap_or_default());
    bkpt();
    loop {}
}

#[exception]
unsafe fn HardFault(_ef: &ExceptionFrame) -> ! {
    static mut DOUBLE_FAULT: bool = false;
    unsafe {
        if DOUBLE_FAULT {
            DOUBLE_FAULT = true;
            bkpt();
            DOUBLE_FAULT = false;
        }
    }
    loop {}
}

#[exception]
unsafe fn DefaultHandler(_x: i16) -> ! {
    bkpt();
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
    bkpt();
    initialise_allocator();
    let mut ps = Peripherals::take();

    // SAFETY: 😊
    unsafe {
        copy_nonoverlapping(FLASH_START, RAM_START, 0x2000);
        ps.sys.memrmp().write(|w| w.mem_mode().bits(0b011));
    }

    let mut uuid: [u8; 16] = [0; 16];
    // SAFETY: 😊
    unsafe { copy_nonoverlapping(UID_ADDRESS as *const u8, uuid.as_mut_ptr(), 12); }
    black_box(uuid);

    let clock_sys = ClockSystem::new(ps.clock_tim, &mut ps.rcc);
    let can_sys = CanSystem::new(ps.can_instance, ps.can_rx_pin, ps.can_tx_pin, &mut ps.rcc);
    let mut argb_sys = ArgbSys::new(ps.argb_instance, ps.argb_pin, &mut ps.rcc);
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
    let mut flash_handler = FlashHandler::new(&mut qspi_sys);
    let mut crc_handler = CrcHandler::new(ps.crc_instance, ps.dma_chan);

    loop {
        node.receive(&mut comms_handler).unwrap();
        comms_handler.tick(&mut node);
        flash_handler.tick();
        argb_sys.tick();

        let timer_expired: bool = true;
        match (comms_handler.state(), flash_handler.state(), crc_handler.state(), timer_expired) {
            (CommsHandlerState::Idle, FlashHandlerState::Disabled, _, false) => {
                // Wait for something to happen
                argb_sys.set_state(State::Idle);
            }

            (CommsHandlerState::BeginFlash, _, _, _) => {
                // Transition to flashing mode
                argb_sys.set_state(State::Flashing);
                crc_handler.stop();
                flash_handler.reset();
            }
            (CommsHandlerState::Loading, FlashHandlerState::Idle, CrcHandlerState::Unverified, _) => {
                // Try to feed flash handler
                if let Some(chunk) = comms_handler.extract() {
                    flash_handler.feed(chunk);
                }
            }
            (CommsHandlerState::Loading, FlashHandlerState::Busy, CrcHandlerState::Unverified, _) => {
                // Wait for flash handler to be free
            }

            (CommsHandlerState::Idle, FlashHandlerState::Idle, CrcHandlerState::Unverified, _) => {
                // Begin verifying
                argb_sys.set_state(State::Verifying);
                flash_handler.disable();
                crc_handler.start();
            }
            (CommsHandlerState::Idle, FlashHandlerState::Disabled, CrcHandlerState::Verifying, _) => {
                // Continue verifying
            }
            (CommsHandlerState::Idle, FlashHandlerState::Disabled, CrcHandlerState::Done, true) => {
                if crc_handler.result().unwrap() == 0 {
                    // Verification failed
                    argb_sys.set_state(State::Error);
                    // The normal opertunity to tick is missed
                    argb_sys.tick();
                    panic!()
                } else {
                    // Boot
                    argb_sys.set_state(State::Booting);
                    // The normal opertunity to tick is missed
                    argb_sys.tick();
                    // SAFETY: 😊
                    unsafe {
                        reset();
                    }
                }
            }
            _ => panic!()
        }
    }
}

#[unsafe(link_section = ".reset")]
#[unsafe(naked)]
unsafe extern "C" fn reset() -> ! {
    naked_asm!(
        // Map QUADSPI to 0x0000_0000 by
        // setting SYSCFG_MEMRMP = 0x0000_0040
        "mov    r0,    #0x0000",
        "movt   r0,    #0x4001",
        "mov    r1,    #0x04",
        "str    r1,    [r0]",

        // Use r0 as the zero register,
        // this can be used to access the info block
        "mov    r0,    #0x00",

        // Zero all of RAM except for the AUX data
        // from 0x2000_0000 to 0x2002_0000
        //
        // For all of the init loops
        // r1=dst, r2=end, r3=src
        "add    r1,     r0,    #0x20000000",
        "add    r2,     r1,    #0x00020000",

        "zero_loop:",
        "str    r0,    [r1],   #4",
        "cmp    r1,     r2",
        "blt    zero_loop",

        // Initialise CCM_SRAM
        "add    r1,     r0,    #0x10000000",
        // Stop after ccm_len bytes
        "ldr    r2,    [r0,#0x18]",
        "add    r2,     r2,     r1",
        // Start reading after the data block
        "mov    r3,    #0x40",

        "ccmr_loop:",
        "ldr    r4,    [r3],   #4",
        "str    r4,    [r1],   #4",
        "cmp    r1,     r2",
        "blt    ccmr_loop",

        // Initialise SRAM1 and set SP to the top of SRAM2
        "add    r1,     r0,    #0x20000000",
        // Set SP
        "add    r2,     r1,    #0x00018000",
        "mov    sp,     r2",
        // Stop after ram_len bytes
        "ldr    r2,    [r0,#0x1A]",
        "add    r2,     r2,     r1",

        "sram_loop:",
        "ldr    r4,    [r3],   #4",
        "str    r4,    [r1],   #4",
        "cmp    r1,     r2",
        "blt    sram_loop",

        // Update SCB_VTOR
        "mov    r1,    #0xED08",
        "movt   r1,    #0xE000",
        // Load vt_addr
        "ldr    r2,    [r0,#0x10]",
        "str    r2,    [r1]",

        // Set LR to the reset vector
        "add    lr,     r2,    #4",

        // Jump to entry point (ep_addr)
        "ldr    pc,    [r0,#0x14]",
    );
}
