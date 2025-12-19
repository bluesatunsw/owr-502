#![no_std]
#![no_main]

use core::{panic::PanicInfo, ptr::copy_nonoverlapping};

use canadensis::{
    Node, ResponseToken, ServiceToken, TransferHandler, core::{Priority, ServiceId, transfer::{MessageTransfer, ServiceTransfer}, transport::{TransferId, Transport}}, encoding::{DataType, Deserialize}, node::{BasicNode, CoreNode, data_types::{GetInfoResponse, Version}}, requester::TransferIdFixedMap
};
use canadensis_can::{CanNodeId, CanReceiver, CanTransmitter, CanTransport, Mtu};
use heapless::Vec;
// use panic_semihosting as _;

use canadensis_data_types::uavcan::{file::path_2_0::Path, node::execute_command_1_3 as ec};
use canadensis_data_types::uavcan::file::read_1_1 as read;

use cortex_m_rt::entry;
use embedded_alloc::LlffHeap as Heap;

use fugit::ExtU32;

use crate::{aux::AuxData, can::CanSystem, clock::ClockSystem, peripherals::Peripherals, qspi::QspiSys};

mod peripherals;
mod clock;
mod can;
mod qspi;
mod aux;

extern crate alloc;

const CYPHAL_CONCURRENT_TRANSFERS: usize = 4;
const CYPHAL_NUM_TOPICS: usize = 4;
const CYPHAL_NUM_SERVICES: usize = 4;
const TID_TIMEOUT_US: u32 = 1_000_000;

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
        ec::SERVICE,
        ec::ExecuteCommandRequest::EXTENT_BYTES.unwrap() as usize,
        10.millis()
    ).unwrap();

    let mut comms_handler = CommsHandler::new();
    let read_tok = node.start_sending_requests(
        read::SERVICE,
        10.millis(),
        read::ReadResponse::EXTENT_BYTES.unwrap() as usize,
        Priority::Low
    ).unwrap();

    loop {
        node.receive(&mut comms_handler).unwrap();

        if comms_handler.current_index != 0 && comms_handler.current_index % 2 == 0 {
            qspi_sys.page_program(comms_handler.current_index / 2, &comms_handler.flash_page);
        }
        if let Some(us) = comms_handler.update_server {
            comms_handler.transfer = FileTransferId::Valid(node.send_request(&read_tok, &read::ReadRequest {
                offset: (comms_handler.current_index as u64) * 256,
                path: Path::deserialize_from_bytes(&comms_handler.image_path).unwrap()
            }, us).unwrap());
        }
    }

    // 😊
    unsafe { AuxData::set(AuxData { rcc: ps.rcc }); }
    unsafe { cortex_m::asm::bootload(0x90000000 as *const u32) };
}

#[derive(Clone)]
enum FileTransferId<T: TransferId + PartialEq> {
    None,
    Invalid(T),
    Valid(T),
}

struct CommsHandler<T: Transport> where T::TransferId: PartialEq {
    pub image_path: Vec<u8, 255>,
    pub update_server: Option<T::NodeId>,
    pub current_index: u16,
    pub transfer: FileTransferId<T::TransferId>,
    pub flash_page: [u8; 512],
}

impl<T: Transport> CommsHandler<T> where T::TransferId: PartialEq {
    pub fn new() -> Self {
        Self {
            image_path: Vec::new(),
            update_server: None,
            current_index: 0,
            transfer: FileTransferId::None,
            flash_page: [0; 512],
        }
    }
}

impl<T: Transport> TransferHandler<T> for CommsHandler<T> where T::TransferId: PartialEq {
    fn handle_message<N>(
        &mut self,
        node: &mut N,
        transfer: &MessageTransfer<alloc::vec::Vec<u8>, T>,
    ) -> bool
    where
        N: Node<Transport = T>,
    {
        false
    }

    fn handle_request<N>(
        &mut self,
        node: &mut N,
        token: ResponseToken<T>,
        transfer: &ServiceTransfer<alloc::vec::Vec<u8>, T>,
    ) -> bool
    where
        N: Node<Transport = T>,
    {
        use ec::ExecuteCommandRequest as Req;
        use ec::ExecuteCommandResponse as Res;

        if transfer.header.service != ec::SERVICE {
            return false
        }

        let req = Req::deserialize_from_bytes(transfer.payload.as_slice()).unwrap();
        match req.command {
            Req::COMMAND_BEGIN_SOFTWARE_UPDATE => {
                if let FileTransferId::Valid(x) = self.transfer.clone() {
                    self.transfer = FileTransferId::Invalid(x);
                }

                self.image_path = req.parameter;
                self.current_index = 0;

                node.send_response(
                    token, 10.millis(),
                    &Res { status: Res::STATUS_SUCCESS, output: Vec::new() }
                ).unwrap()
            },
            Req::COMMAND_IDENTIFY => {
                node.send_response(
                    token, 10.millis(),
                    &Res { status: Res::STATUS_SUCCESS, output: Vec::new() }
                ).unwrap()
            },
            _ => {
                node.send_response(
                    token, 10.millis(),
                    &Res { status: Res::STATUS_BAD_COMMAND, output: Vec::new() }
                ).unwrap()
            },
        };

        true
    }

    fn handle_response<N>(
        &mut self,
        node: &mut N,
        transfer: &ServiceTransfer<alloc::vec::Vec<u8>, T>,
    ) -> bool
    where
        N: Node<Transport = T>,
    {
        match &self.transfer {
            FileTransferId::None => return false,
            FileTransferId::Invalid(x) => {
                if transfer.header.transfer_id != *x {
                    return false;
                }
            }
            FileTransferId::Valid(x) => {
                if transfer.header.transfer_id != *x {
                    return false;
                }
                if self.current_index % 2 == 0 {
                    self.flash_page[..256].copy_from_slice(&transfer.payload);
                } else {
                    self.flash_page[256..].copy_from_slice(&transfer.payload);
                }
            },
        }

        self.transfer = FileTransferId::None;
        true
    }
}
