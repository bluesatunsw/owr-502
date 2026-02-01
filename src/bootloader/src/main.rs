#![no_std]
#![no_main]
#![feature(ptr_as_ref_unchecked)]
#![feature(unsafe_cell_access)]
#![feature(naked_functions_rustic_abi)]
#![feature(never_type)]
#![feature(generic_const_exprs)]
#![feature(breakpoint)]

use core::{
    arch::{breakpoint, naked_asm},
    panic::PanicInfo,
    ptr::copy_nonoverlapping,
};

use canadensis::{
    Node,
    core::time::{Clock, Microseconds32},
    encoding::DataType,
    node::{
        BasicNode, CoreNode,
        data_types::{GetInfoResponse, Version},
    },
    requester::TransferIdFixedMap,
};
use canadensis_can::{CanNodeId, CanReceiver, CanTransmitter, CanTransport, Mtu};
use cortex_m_semihosting::hprintln;
use embedded_common::{can::CanDriver, clock::MicrosecondClock};
use heapless::Vec;
// use panic_semihosting as _;

use cortex_m_rt::{ExceptionFrame, entry, exception};
use embedded_alloc::LlffHeap as Heap;

use fugit::{ExtU32, MicrosDurationU32};

use canadensis_data_types::uavcan::node::execute_command_1_3::{
    ExecuteCommandRequest, SERVICE as EXECUTE_COMMAND_SERVICE,
};
use wzrd_core::FlashLocation;

use crate::{
    argb::{ArgbSys, State},
    chunk_flasher::{ChunkFlasher, Flash},
    common::{FlashCommand, get_header},
    comms_handler::CommsHandler,
    crc_handler::CrcHandler,
    iflash::IflashSys,
    peripherals::Peripherals,
    qspi::QspiSys,
};

mod argb;
mod chunk;
mod chunk_flasher;
mod common;
mod comms_handler;
mod crc_handler;
mod iflash;
mod interfaces;
mod peripherals;
mod qspi;

extern crate alloc;

const CYPHAL_CONCURRENT_TRANSFERS: usize = 4;
const CYPHAL_NUM_TOPICS: usize = 4;
const CYPHAL_NUM_SERVICES: usize = 4;

const HEARTBEAT_PERIOD_US: u32 = 1_000_000;
const UPDATE_TIMEOUT_US: u32 = 15_000_000;

const UID_ADDRESS: u32 = 0x1FFF_7590;

const RAM_START: *mut u32 = 0x2000_0000 as *mut u32;
const FLASH_START: *const u32 = 0x0800_0000 as *const u32;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    hprintln!("{}", info.message().as_str().unwrap_or_default());
    breakpoint();
    loop {}
}

#[exception]
unsafe fn HardFault(_ef: &ExceptionFrame) -> ! {
    static mut DOUBLE_FAULT: bool = false;
    unsafe {
        if DOUBLE_FAULT {
            DOUBLE_FAULT = true;
            breakpoint();
            DOUBLE_FAULT = false;
        }
    }
    loop {}
}

#[exception]
unsafe fn DefaultHandler(_x: i16) -> ! {
    breakpoint();
    loop {}
}

// Global allocator ー required by canadensis.
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
    breakpoint();
    initialise_allocator();
    let mut ps = Peripherals::take();

    // SAFETY: 😊
    unsafe {
        // Copy bootloader code into RAM to allow us to write to internal flash
        copy_nonoverlapping(FLASH_START, RAM_START, 0x2000);
        // Remap RAM to begin executing from RAM
        ps.sys.memrmp().write(|w| w.mem_mode().bits(0b011));
    }

    let mut uuid: [u8; 16] = [0x4C; 16];
    // SAFETY: 😊
    unsafe {
        copy_nonoverlapping(UID_ADDRESS as *const u8, uuid.as_mut_ptr(), 12);
    }

    let clock_sys = MicrosecondClock::new(ps.clock_tim, &mut ps.rcc);
    let can_sys = CanDriver::new(ps.can_instance, ps.can_rx_pin, ps.can_tx_pin, &mut ps.rcc);
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
        ps.qspi_io3_bank2_pin,
    );

    let id = CanNodeId::from_truncating(21);
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
    > = CoreNode::new(clock_sys, id, transmitter, receiver, can_sys);

    qspi_sys.disable_write();
    // SAFETY: at this point external flash is enabled
    let current_header = unsafe { get_header().unwrap_or_default() };

    let mut node = BasicNode::new(
        core_node,
        GetInfoResponse {
            protocol_version: Version { major: 1, minor: 0 },
            hardware_version: Version {
                major: current_header.hw_ver,
                minor: current_header.hw_rev,
            },
            software_version: Version {
                major: current_header.sw_maj,
                minor: current_header.sw_min,
            },
            software_vcs_revision_id: 0,
            unique_id: uuid,
            name: Vec::from_slice(b"org.bluesat.owr.bootloader").unwrap(),
            software_image_crc: Vec::from_array([current_header.crc as u64]),
            certificate_of_authenticity: Vec::new(),
        },
    )
    .unwrap();

    node.subscribe_request(
        EXECUTE_COMMAND_SERVICE,
        ExecuteCommandRequest::EXTENT_BYTES.unwrap() as usize,
        1000.millis(),
    )
    .unwrap();

    let mut comms_handler = CommsHandler::new(&mut node);
    let mut crc_handler = CrcHandler::new(ps.crc_instance, ps.dma_chan);

    let mut qspi_flasher = ChunkFlasher::new(qspi_sys);
    let mut iflash_flasher = ChunkFlasher::new(IflashSys::new(&mut ps.internal_flash));

    argb_sys.set_state(State::Idle);
    crc_handler.start();

    let mut next_hb = node.clock_mut().now();

    loop {
        if next_hb <= node.clock_mut().now() {
            node.run_per_second_tasks().unwrap();
            next_hb += MicrosDurationU32::from_ticks(HEARTBEAT_PERIOD_US);
        }

        node.receive(&mut comms_handler).unwrap();
        comms_handler.tick(&mut node);
        argb_sys.tick(comms_handler.identifying(), node.clock_mut().now());

        //  Do nothing if handlers are still busy
        if !qspi_flasher.tick() || !iflash_flasher.tick() {
            continue;
        }

        match comms_handler.poll() {
            FlashCommand::None => {
                if true || node.clock_mut().now() < Microseconds32::from_ticks(UPDATE_TIMEOUT_US) {
                    continue;
                }
                if let Some(valid) = crc_handler.valid() {
                    if valid {
                        argb_sys.set_state(State::Booting);
                        unsafe {
                            Peripherals::reset();
                            reset()
                        };
                    } else {
                        argb_sys.set_state(State::BadCrc);
                        panic!("Invalid CRC");
                    }
                }
            }
            FlashCommand::Start => {
                argb_sys.set_state(State::Flashing);
                crc_handler.stop();
                qspi_flasher.enable_write();
                iflash_flasher.enable_write();
            }
            FlashCommand::Write(chunk) => match chunk.location {
                FlashLocation::Internal => iflash_flasher.write(chunk.data, chunk.offset),
                FlashLocation::External => qspi_flasher.write(chunk.data, chunk.offset),
            },
            FlashCommand::Finish => {
                argb_sys.set_state(State::Idle);
                qspi_flasher.disable_write();
                iflash_flasher.disable_write();
                crc_handler.start();
            }
        }
    }
}

/// Initialise RAM and the memory layout for the Rust user program
///
/// This must be implemented in assembly since we are invalidating
/// the memory of the bootloader (by changing the memory remap to be
/// external QSPI and zeroing all of SRAM). Rust *really* doesn't
/// like this.
///
/// We place this function in a specific link section to force it
/// to have a stable address, which is necessary as the reset
/// vector of the user program should also point to this function.
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
        // Zero all of RAM from 0x2000_0000 to 0x2002_0000
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
        "ldr    r2,    [r0,#0x10]",
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
        "ldr    r2,    [r0,#0x14]",
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
        "ldr    r2,    [r0,#0x20]",
        "str    r2,    [r1]",
        // Set LR to the reset vector
        "add    lr,     r2,    #4",
        // Jump to entry point (ep_addr)
        "bkpt",
        "ldr    pc,    [r0,#0x24]",
    );
}
