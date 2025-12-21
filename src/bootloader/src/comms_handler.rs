use core::mem;

use canadensis::{Node, ResponseToken, ServiceToken, TransferHandler, core::{Priority, transfer::{MessageTransfer, ServiceTransfer}, transport::Transport}, encoding::{DataType, Deserialize}};
use canadensis_data_types::uavcan::{file::{path_2_0::Path, read_1_1::{ReadRequest, ReadResponse}}, node::execute_command_1_3::{ExecuteCommandRequest, ExecuteCommandResponse}};
use heapless::Vec;
use fugit::ExtU32;

use canadensis_data_types::uavcan::node::execute_command_1_3::SERVICE as EXECUTE_COMMAND_SERVICE;
use canadensis_data_types::uavcan::file::read_1_1::SERVICE as READ_SERVICE;

use crate::chunk::{Chunk, ChunkManager, DoubleBuffer, FLASH_CHUNK_SIZE, TRANSFER_SIZE};

const MAX_TRANSFERS: usize = 4;
const HEADER_SIZE: usize = 256;

#[repr(C)]
struct Header {
    pub version: u64,
    pub qspi_length: usize,

    _pad: [u8; 244],
}

impl Header {
    pub fn file_offset(&self, offset: usize) -> Option<usize> {
        if offset < self.qspi_length {
            Some(HEADER_SIZE + offset)
        } else {
            None
        }
    }
}

// assert_eq!(mem::sizeof::<Header>(), HEADER_SIZE);

#[derive(PartialEq)]
enum UpdateSourceState {
    Valid,
    Flushing,
}

struct UpdateSource<T: Transport> {
    pub state: UpdateSourceState,
    pub path: Vec<u8, 255>,
    pub server: T::NodeId,

    pub header: Option<Header>,
    pub section_offset: usize,
}

pub enum ExtractResult {
    Some(Chunk),
    Wait,
    Done,
}

pub struct CommsHandler<T: Transport> where T::TransferId: PartialEq {
    update_source: Option<UpdateSource<T>>,
    buffers: DoubleBuffer<ChunkManager<T::TransferId>>,
    read_token: ServiceToken<ReadRequest>,
}

impl<T: Transport> CommsHandler<T> where T::TransferId: PartialEq {
    pub fn new<N>(node: &mut N) -> Self where N: Node<Transport = T> {
        Self {
            update_source: None,
            buffers: DoubleBuffer::new(ChunkManager::new(), ChunkManager::new()),
            read_token: node.start_sending_requests::<ReadRequest>(
                READ_SERVICE,
                10.millis(),
                ReadResponse::EXTENT_BYTES.unwrap() as usize,
                Priority::Low.into()
            ).unwrap()
        }
    }

    pub fn tick<N>(&mut self, node: &mut N) where N: Node<Transport = T> {
        if let Some(src) = &mut self.update_source
        && src.state == UpdateSourceState::Valid
        && self.buffers.front().active_transfers() + self.buffers.back().active_transfers() < MAX_TRANSFERS
        && let Some(header) = &src.header {
            let tok = &self.read_token;
            let (buf, maybe_off) = if self.buffers.front().complete() {
                (self.buffers.back_mut(), header.file_offset(src.section_offset + FLASH_CHUNK_SIZE))
            } else {
                (self.buffers.front_mut(), header.file_offset(src.section_offset))
            };

            if let Some(off) = maybe_off {
                buf.start_transfer(|x|
                    node.send_request(&tok, &ReadRequest {
                        offset: (off + x*TRANSFER_SIZE) as u64,
                        path: Path::deserialize_from_bytes(src.path.as_slice()).unwrap() 
                    }, src.server.clone()).unwrap()
                );
            }
        }
    }

    pub fn extract(&mut self) -> ExtractResult {
        if let Some(src) = &mut self.update_source
        && let Some(header) = &src.header
        {
            if let Some(chunk) = self.buffers.front_mut().extract_data() {
                src.section_offset += FLASH_CHUNK_SIZE;
                ExtractResult::Some(chunk)
            } else if header.file_offset(src.section_offset).is_none() {
                ExtractResult::Done
            } else {
                ExtractResult::Wait
            }
        } else {
            ExtractResult::Wait
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
        if transfer.header.service != EXECUTE_COMMAND_SERVICE {
            return false
        }

        let req = ExecuteCommandRequest::deserialize_from_bytes(transfer.payload.as_slice()).unwrap();
        match req.command {
            ExecuteCommandRequest::COMMAND_BEGIN_SOFTWARE_UPDATE => {
                let src: UpdateSource<T> = UpdateSource {
                    state: if self.update_source.is_none() {
                        UpdateSourceState::Valid 
                    } else {
                        UpdateSourceState::Flushing 
                    },
                    path: req.parameter,
                    server: transfer.header.source.clone(),
                    header: None,
                    section_offset: 0,
                };

                node.send_response(
                    token, 10.millis(),
                    &ExecuteCommandResponse {
                        status: ExecuteCommandResponse::STATUS_SUCCESS,
                        output: Vec::new()
                    }
                ).unwrap();

                let tok = &self.read_token;
                node.send_request(&tok, &ReadRequest {
                    offset: 0,
                    path: Path::deserialize_from_bytes(src.path.as_slice()).unwrap() 
                }, src.server.clone()).unwrap();

                self.update_source = Some(src);
            },
            ExecuteCommandRequest::COMMAND_IDENTIFY => {
                node.send_response(
                    token, 10.millis(),
                    &ExecuteCommandResponse {
                        status: ExecuteCommandResponse::STATUS_SUCCESS,
                        output: Vec::new()
                    }
                ).unwrap();
            },
            _ => {
                node.send_response(
                    token, 10.millis(),
                    &ExecuteCommandResponse {
                        status: ExecuteCommandResponse::STATUS_BAD_COMMAND,
                        output: Vec::new()
                    }
                ).unwrap();
            },
        }

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
        if let Some(x) = &mut self.update_source {
            if x.header.is_none() {
                // SAFETY: the header only contains ints (no invariants)
                x.header = unsafe { Some(mem::transmute_copy::<[u8; HEADER_SIZE], Header>(
                    transfer.payload.as_slice()[0..HEADER_SIZE].as_array().unwrap()
                )) };

                return true;
            }

            // Flushing is complete when there are no more active transfers
            if (
                self.buffers.front_mut().end_transfer(
                    &transfer.header.transfer_id, transfer.payload.as_slice()
                ) || self.buffers.back_mut().end_transfer(
                    &transfer.header.transfer_id, transfer.payload.as_slice()
                )
            ) && (
                x.state == UpdateSourceState::Flushing
                && self.buffers.front().active_transfers() == 0
                && self.buffers.back().active_transfers() == 0
            ) {
                x.state = UpdateSourceState::Valid;
            }

            return true;
        }

        false
    }
}
