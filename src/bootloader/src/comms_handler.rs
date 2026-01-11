use canadensis::{Node, ResponseToken, ServiceToken, TransferHandler, core::{Priority, transfer::{MessageTransfer, ServiceTransfer}, transport::Transport}, encoding::{DataType, Deserialize}};
use canadensis_data_types::uavcan::{file::{path_2_0::Path, read_1_1::{ReadRequest, ReadResponse}}, node::execute_command_1_3::{ExecuteCommandRequest, ExecuteCommandResponse}};
use heapless::Vec;
use fugit::ExtU32;

use canadensis_data_types::uavcan::node::execute_command_1_3::SERVICE as EXECUTE_COMMAND_SERVICE;
use canadensis_data_types::uavcan::file::read_1_1::SERVICE as READ_SERVICE;
use wzrd_core::{CHUNK_SIZE, Header};

use crate::{chunk::{ChunkManager, DoubleBuffer, TRANSFER_SIZE}, common::LocatedChunk};

const MAX_TRANSFERS: usize = 4;

#[derive(PartialEq)]
enum UpdateSourceState {
    Valid,
    Flushing,
}

enum AsyncHeader<I: PartialEq> {
    Future(I),
    Some(Header),
}

struct UpdateSource<T: Transport> where T::TransferId: PartialEq {
    pub state: UpdateSourceState,
    pub path: Vec<u8, 255>,
    pub server: T::NodeId,

    pub header: AsyncHeader<T::TransferId>,
    pub file_offset: usize,
}

pub struct CommsHandler<T: Transport> where T::TransferId: PartialEq {
    update_source: Option<UpdateSource<T>>,
    buffers: DoubleBuffer<ChunkManager<T::TransferId>>,
    read_token: ServiceToken<ReadRequest>,
    identify: bool,
}

pub enum CommsHandlerState {
    Idle,
    BeginFlash,
    Loading,
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
            ).unwrap(),
            identify: false,
        }
    }

    pub fn tick<N>(&mut self, node: &mut N) where N: Node<Transport = T> {
        if let Some(src) = &mut self.update_source
        && src.state == UpdateSourceState::Valid
        && self.buffers.front().active_transfers() + self.buffers.back().active_transfers() < MAX_TRANSFERS
        && let AsyncHeader::Some(header) = &src.header {
            let tok = &self.read_token;
            let (buf, off) = if self.buffers.front().complete() {
                (self.buffers.back_mut(), src.file_offset + CHUNK_SIZE)
            } else {
                (self.buffers.front_mut(), src.file_offset)
            };

            if header.file_length() < off {
                buf.start_transfer(|x|
                    node.send_request(&tok, &ReadRequest {
                        offset: (off + x*TRANSFER_SIZE) as u64,
                        path: Path::deserialize_from_bytes(src.path.as_slice()).unwrap() 
                    }, src.server.clone()).unwrap()
                );
            }
        }
    }

    pub fn extract(&mut self) -> Option<LocatedChunk> {
        if let Some(src) = &mut self.update_source
        && let AsyncHeader::Some(header) = &src.header
        && let Some(chunk) = self.buffers.front_mut().extract_data() {
            let (location, offset) = header.to_flash_location(src.file_offset).unwrap();
            src.file_offset += CHUNK_SIZE;
            self.buffers.switch();
            
            if header.file_length() < src.file_offset {
                // Everything possible has been extracted, mark the update as complete
                self.update_source = None;
            }

            Some(LocatedChunk { data: chunk, location, offset })
        } else {
            None
        }
    }

    pub fn state(&self) -> CommsHandlerState {
        match &self.update_source {
            Some(s) => match s.header {
                AsyncHeader::Future(_) => CommsHandlerState::BeginFlash,
                AsyncHeader::Some(_) => CommsHandlerState::Loading,
            },
            None => CommsHandlerState::Idle,
        }
    }
}

impl<T: Transport> TransferHandler<T> for CommsHandler<T> where T::TransferId: PartialEq {
    fn handle_message<N>(
        &mut self,
        _node: &mut N,
        _transfer: &MessageTransfer<alloc::vec::Vec<u8>, T>,
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
                    path: req.parameter.clone(),
                    server: transfer.header.source.clone(),
                    header: AsyncHeader::Future(
                        node.send_request(&self.read_token, &ReadRequest {
                            offset: 0,
                            path: Path::deserialize_from_bytes(req.parameter.as_slice()).unwrap() 
                        }, transfer.header.source.clone()).unwrap()
                    ),
                    file_offset: Header::SIZE,
                };

                node.send_response(
                    token, 10.millis(),
                    &ExecuteCommandResponse {
                        status: ExecuteCommandResponse::STATUS_SUCCESS,
                        output: Vec::new()
                    }
                ).unwrap();

                self.update_source = Some(src);
            },
            ExecuteCommandRequest::COMMAND_IDENTIFY => {
                self.identify = true;

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
        _node: &mut N,
        transfer: &ServiceTransfer<alloc::vec::Vec<u8>, T>,
    ) -> bool
    where
        N: Node<Transport = T>,
    {
        if let Some(x) = &mut self.update_source {
            if let AsyncHeader::Future(tid) = &x.header
                && transfer.header.transfer_id == *tid {
                    x.header = AsyncHeader::Some(Header::deserialize(&transfer.payload.as_array().unwrap()));
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
