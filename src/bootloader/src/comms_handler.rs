use core::{arch::breakpoint, hint::black_box};

use canadensis::{
    Node, ResponseToken, ServiceToken, TransferHandler,
    core::{
        Priority,
        time::{Clock, Microseconds32},
        transfer::ServiceTransfer,
        transport::Transport,
    },
    encoding::{DataType, Deserialize},
};
use canadensis_data_types::uavcan::{
    file::{
        error_1_0::Error as FileError,
        path_2_0::Path,
        read_1_1::{ReadRequest, ReadResponse},
    },
    node::execute_command_1_3::{ExecuteCommandRequest, ExecuteCommandResponse},
};
use embedded_common::{debug::itm_hexdump, dprintln};
use fugit::ExtU32;
use heapless::Vec;

use canadensis_data_types::uavcan::file::read_1_1::SERVICE as READ_SERVICE;
use canadensis_data_types::uavcan::node::execute_command_1_3::SERVICE as EXECUTE_COMMAND_SERVICE;
use wzrd_core::{CHUNK_SIZE, Header};

const MAX_TRANSFERS: usize = 1;

use crate::{
    chunk::{ChunkManager, DoubleBuffer, TRANSFER_SIZE},
    common::{FlashCommand, LocatedChunk},
};

#[derive(Debug)]
enum AsyncHeader<I: PartialEq> {
    Future(I),
    Some(Header),
}

struct Update<T: Transport>
where
    T::TransferId: PartialEq,
{
    pub path: Vec<u8, 255>,
    pub server: T::NodeId,
    pub header: AsyncHeader<T::TransferId>,

    pub file_offset: usize,
    buffers: DoubleBuffer<ChunkManager<T::TransferId>>,
}

pub struct CommsHandler<T: Transport>
where
    T::TransferId: PartialEq,
{
    update: Option<Update<T>>,
    read_token: ServiceToken<ReadRequest>,
    start_flag: bool,
    finish_flag: bool,
    identify: Option<Microseconds32>,
}

impl<T: Transport> CommsHandler<T>
where
    T::TransferId: PartialEq,
{
    pub fn new<N: Node<Transport = T>>(node: &mut N) -> Self {
        Self {
            update: None,
            read_token: node
                .start_sending_requests::<ReadRequest>(
                    READ_SERVICE,
                    100.millis(),
                    ReadResponse::EXTENT_BYTES.unwrap() as usize,
                    Priority::Low.into(),
                )
                .unwrap(),
            start_flag: false,
            finish_flag: false,
            identify: None,
        }
    }

    pub fn tick<N: Node<Transport = T>>(&mut self, node: &mut N) {
        if let Some(itime) = self.identify
            && itime >= node.clock_mut().now()
        {
            self.identify = None
        }

        if let Some(update) = &mut self.update
            && let AsyncHeader::Some(header) = &update.header
            && (update.buffers.front().active_transfers()
                + update.buffers.back().active_transfers())
                < MAX_TRANSFERS
        {
            let tok = &self.read_token;
            let (buf, off) = if update.buffers.front().unfilled() {
                (update.buffers.front_mut(), update.file_offset)
            } else if update.buffers.back().unfilled() {
                (update.buffers.back_mut(), update.file_offset + CHUNK_SIZE)
            } else {
                return;
            };

            if off < header.image_length() {
                buf.start_transfer(|x| {
                    let file_offset = off + x * TRANSFER_SIZE;
                    dprintln!(0, "{}/{}", file_offset, header.image_length());
                    node.send_request(
                        &tok,
                        &ReadRequest {
                            offset: file_offset as u64,
                            path: Path {
                                path: update.path.clone(),
                            },
                        },
                        update.server.clone(),
                    )
                    .unwrap()
                });
            }
        }
    }

    pub fn poll(&mut self) -> FlashCommand {
        if self.finish_flag {
            self.finish_flag = false;
            return FlashCommand::Finish;
        }
        if self.start_flag {
            self.start_flag = false;
            return FlashCommand::Start;
        }

        if let Some(update) = &mut self.update
            && let AsyncHeader::Some(header) = &update.header
            && let Some(chunk) = update.buffers.front_mut().extract_data()
        {
            let (location, offset) = header.to_flash_location(update.file_offset).unwrap();
            update.file_offset += CHUNK_SIZE;
            update.buffers.switch();

            if header.image_length() <= update.file_offset {
                dprintln!(0, "Update compelete");
                self.update = None;
                self.finish_flag = true;
            }
            FlashCommand::Write(LocatedChunk {
                data: chunk,
                location,
                offset,
            })
        } else {
            FlashCommand::None
        }
    }

    pub fn identifying(&self) -> bool {
        self.identify.is_some()
    }
}

impl<T: Transport> TransferHandler<T> for CommsHandler<T>
where
    T::TransferId: PartialEq,
{
    fn handle_request<N: Node<Transport = T>>(
        &mut self,
        node: &mut N,
        token: ResponseToken<T>,
        transfer: &ServiceTransfer<alloc::vec::Vec<u8>, T>,
    ) -> bool {
        if transfer.header.service != EXECUTE_COMMAND_SERVICE {
            return false;
        }

        let req =
            ExecuteCommandRequest::deserialize_from_bytes(transfer.payload.as_slice()).unwrap();
        match req.command {
            ExecuteCommandRequest::COMMAND_BEGIN_SOFTWARE_UPDATE => {
                self.start_flag = true;
                if self.update.is_some() {
                    self.finish_flag = false;
                }

                let source = &transfer.header.source;
                self.update = Some(Update::<T> {
                    path: req.parameter.clone(),
                    server: source.clone(),
                    header: AsyncHeader::Future(
                        node.send_request(
                            &self.read_token,
                            &ReadRequest {
                                offset: 0,
                                path: Path {
                                    path: req.parameter,
                                },
                            },
                            source.clone(),
                        )
                        .unwrap(),
                    ),
                    file_offset: 0,
                    buffers: DoubleBuffer::new(ChunkManager::new(), ChunkManager::new()),
                });

                node.send_response(
                    token,
                    1000.millis(),
                    &ExecuteCommandResponse {
                        status: ExecuteCommandResponse::STATUS_SUCCESS,
                        output: Vec::new(),
                    },
                )
                .unwrap();
            }
            ExecuteCommandRequest::COMMAND_IDENTIFY => {
                self.identify = Some(node.clock_mut().now());

                node.send_response(
                    token,
                    1000.millis(),
                    &ExecuteCommandResponse {
                        status: ExecuteCommandResponse::STATUS_SUCCESS,
                        output: Vec::new(),
                    },
                )
                .unwrap();
            }
            _ => {
                node.send_response(
                    token,
                    1000.millis(),
                    &ExecuteCommandResponse {
                        status: ExecuteCommandResponse::STATUS_BAD_COMMAND,
                        output: Vec::new(),
                    },
                )
                .unwrap();
            }
        }

        true
    }

    fn handle_response<N: Node<Transport = T>>(
        &mut self,
        _node: &mut N,
        transfer: &ServiceTransfer<alloc::vec::Vec<u8>, T>,
    ) -> bool {
        if let Some(update) = &mut self.update {
            let tid = &transfer.header.transfer_id;
            let res = ReadResponse::deserialize_from_bytes(&transfer.payload).unwrap();

            if let AsyncHeader::Future(req_tid) = &update.header
                && *tid == *req_tid
            {
                update.header = AsyncHeader::Some(
                    Header::deserialize(
                        res.data.value.as_slice()[0..Header::SIZE]
                            .as_array()
                            .unwrap(),
                    )
                    .unwrap(),
                );
                dprintln!(0, "Header: {:?}", update.header);
                return true;
            }

            let chunk: &[u8; TRANSFER_SIZE] = res.data.value.as_slice()[0..TRANSFER_SIZE]
                .as_array()
                .unwrap();
            return update.buffers.front_mut().end_transfer(&tid, chunk)
                || update.buffers.back_mut().end_transfer(&tid, chunk);
        }

        false
    }
}
