//! REPL for controlling the rover drivebase.
//!
//! Usage: `deckard [SocketCAN interface name] [Node ID]`
//!
//! # Testing
//!
//! ## Create a virtual CAN device
//!
//! ```
//! sudo modprobe vcan
//! sudo ip link add dev vcan0 type vcan
//! sudo ip link set up vcan0
//! ```
//!
//! ## Start the node
//!
//! ```
//! basic_node vcan0 [node ID]
//! ```
//!
//! ## Interact with the node using Yakut
//!
//! To subscribe and print out Heartbeat messages:
//! `yakut --transport "CAN(can.media.socketcan.SocketCANMedia('vcan0',8),42)" subscribe uavcan.node.Heartbeat.1.0`
//!
//! To send a NodeInfo request:
//! `yakut --transport "CAN(can.media.socketcan.SocketCANMedia('vcan0',8),42)" call [Node ID of basic_node] uavcan.node.GetInfo.1.0 {}`
//!
//! In the above two commands, 8 is the MTU of standard CAN and 42 is the node ID of the Yakut node.

extern crate canadensis;
extern crate canadensis_can;
extern crate canadensis_linux;
extern crate rand;
extern crate simplelog;
extern crate socketcan;

use std::convert::TryFrom;
use std::env;
use std::time::Duration;
use std::io::{self, Write, ErrorKind};
use std::thread;
use std::collections::HashMap;
use std::sync::mpsc;

use half;
use socketcan::{CanFdSocket, CanSocket, Socket};

use canadensis::core::time::MicrosecondDuration32;
use canadensis::core::transfer::{MessageTransfer, ServiceTransfer};
use canadensis::core::transport::Transport;
use canadensis::core::{SubjectId, Priority};
use canadensis::node::{BasicNode, CoreNode};
use canadensis::requester::TransferIdFixedMap;
use canadensis::{Node, ResponseToken, TransferHandler};
use canadensis_can::queue::{ArrayQueue, SingleQueueDriver};
use canadensis_can::{CanNodeId, CanReceiver, CanTransmitter, CanTransport, Error, Mtu};
use canadensis::encoding::Deserialize;
use canadensis_data_types::uavcan::node::get_info_1_0::GetInfoResponse;
use canadensis_data_types::uavcan::node::version_1_0::Version;
use canadensis_data_types::reg::udral::service::actuator::common::sp::scalar_0_1::Scalar as VESCScalar;
use canadensis_data_types::reg::udral::physics::dynamics::rotation::planar_0_1::Planar as StepperPlanar;
use canadensis_data_types::reg::udral::physics::kinematics::rotation::planar_0_1::Planar;
use canadensis_data_types::uavcan::si::unit::angle::scalar_1_0 as angle_pos_scalar;
use canadensis_data_types::uavcan::si::unit::angular_velocity::scalar_1_0::Scalar as AVScalar;
use canadensis_data_types::uavcan::si::unit::angular_acceleration::scalar_1_0::Scalar as AAScalar;
use canadensis_data_types::uavcan::si::unit::torque::scalar_1_0::Scalar as TScalar;

use canadensis_linux::{LinuxCan, SystemClock};

mod cmd;
mod core;

// of the VESCs
// canonical ordering: FL, FR, BL, BR
const VESC_IDS: [u8; 4] = [10, 11, 12, 13];
const STEPPER_SETPOINT_SUB: [u16; 4] = [3000, 3010, 3020, 3030];
const STEPPER_POSITION_SUB: [u16; 4] = [3001, 3011, 3021, 3031];
const VESC_SPEED_SUB: [u16; 4] = [3050, 3060, 3070, 3080];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    simplelog::TermLogger::init(
        simplelog::LevelFilter::Warn,
        simplelog::Config::default(),
        simplelog::TerminalMode::Stderr,
        simplelog::ColorChoice::Auto,
    )
    .unwrap();

    let mut args = env::args().skip(1);
    let stepper_can_interface = args.next().expect("Expected CAN interface name");
    let vesc_can_interface = args.next().expect("Expected CAN interface name");
    let node_id = CanNodeId::try_from(
        args.next()
            .expect("Expected node ID")
            .parse::<u8>()
            .expect("Invalid node ID format"),
    )
    .expect("Node ID too large");

    println!(
        "Port list size: {} bytes",
        std::mem::size_of::<canadensis_data_types::uavcan::node::port::list_1_0::List>()
    );

    let stepper_can = CanFdSocket::open(&stepper_can_interface).expect("Failed to open CAN interface");
    stepper_can.set_read_timeout(Duration::from_millis(100))?;
    stepper_can.set_write_timeout(Duration::from_millis(100))?;
    let stepper_can = LinuxCan::new(stepper_can);

    let vesc_can = CanSocket::open(&vesc_can_interface).expect("Failed to open CAN interface");
    vesc_can.set_read_timeout(Duration::from_millis(100))?;
    vesc_can.set_write_timeout(Duration::from_millis(100))?;
    let vesc_can = LinuxCan::new(vesc_can);

    let stepper_node_info = GetInfoResponse {
        protocol_version: Version { major: 1, minor: 0 },
        hardware_version: Version { major: 0, minor: 0 },
        software_version: Version { major: 0, minor: 1 },
        software_vcs_revision_id: 0,
        unique_id: rand::random(),
        name: heapless::Vec::from_slice(b"org.bluesat.obc.all").unwrap(),
        software_image_crc: heapless::Vec::new(),
        certificate_of_authenticity: Default::default(),
    };
    let vesc_node_info = GetInfoResponse {
        protocol_version: Version { major: 1, minor: 0 },
        hardware_version: Version { major: 0, minor: 0 },
        software_version: Version { major: 0, minor: 1 },
        software_vcs_revision_id: 0,
        unique_id: rand::random(),
        name: heapless::Vec::from_slice(b"org.bluesat.obc.vesc").unwrap(),
        software_image_crc: heapless::Vec::new(),
        certificate_of_authenticity: Default::default(),
    };

    const QUEUE_CAPACITY: usize = 1210;
    type FDQueue = SingleQueueDriver<SystemClock, ArrayQueue<QUEUE_CAPACITY>, LinuxCan<CanFdSocket>>;
    type ClassicQueue = SingleQueueDriver<SystemClock, ArrayQueue<QUEUE_CAPACITY>, LinuxCan<CanSocket>>;
    let stepper_queue_driver: FDQueue = SingleQueueDriver::new(ArrayQueue::new(), stepper_can);
    let vesc_queue_driver: ClassicQueue = SingleQueueDriver::new(ArrayQueue::new(), vesc_can);

    const TRANSFER_IDS: usize = 16;
    const PUBLISHERS: usize = 16;
    const REQUESTERS: usize = 16;

    let transmitter = CanTransmitter::new(Mtu::CanFd64);
    let receiver = CanReceiver::new(node_id);
    let stepper_core_node: CoreNode<
        SystemClock,
        CanTransmitter<SystemClock, FDQueue>,
        CanReceiver<SystemClock, FDQueue>,
        TransferIdFixedMap<CanTransport, TRANSFER_IDS>,
        FDQueue,
        PUBLISHERS,
        REQUESTERS,
    > = CoreNode::new(
        SystemClock::new(),
        node_id,
        transmitter,
        receiver,
        stepper_queue_driver,
    );
    let mut stepper_node = BasicNode::new(stepper_core_node, stepper_node_info).unwrap();

    let transmitter = CanTransmitter::new(Mtu::Can8);
    let receiver = CanReceiver::new(node_id);
    let vesc_core_node: CoreNode<
        SystemClock,
        CanTransmitter<SystemClock, ClassicQueue>,
        CanReceiver<SystemClock, ClassicQueue>,
        TransferIdFixedMap<CanTransport, TRANSFER_IDS>,
        ClassicQueue,
        PUBLISHERS,
        REQUESTERS,
    > = CoreNode::new(
        SystemClock::new(),
        node_id,
        transmitter,
        receiver,
        vesc_queue_driver,
    );
    let mut vesc_node = BasicNode::new(vesc_core_node, vesc_node_info).unwrap();

    let (cmd_tx, cmd_rx) = mpsc::channel::<core::NodeCommand>();
    let (notif_tx, notif_rx) = mpsc::channel::<Result<(),()>>();

    for subject in VESC_SPEED_SUB {
        println!("Publishing on {}...", subject);
        vesc_node.start_publishing(
            SubjectId::from_truncating(subject),
            MicrosecondDuration32::millis(1_000),
            Priority::Nominal
        ).unwrap();
    }
    for subject in STEPPER_SETPOINT_SUB {
        println!("Publishing on {}...", subject);
        stepper_node.start_publishing(
            SubjectId::from_truncating(subject),
            MicrosecondDuration32::millis(1_000),
            Priority::Nominal
        ).unwrap();
    }
    for subject in STEPPER_POSITION_SUB {
        println!("Subscribing to {}...", subject);
        stepper_node.subscribe_message(
            SubjectId::from_truncating(subject),
            size_of::<angle_pos_scalar::Scalar>(),
            MicrosecondDuration32::millis(1_000)
        ).unwrap();
    }

    let start_time = std::time::Instant::now();
    let mut prev_seconds = 0;
    let mut notify_in_flight = false;
    let mut notif_op_time = std::time::Instant::now();
    let mut rover_internal = RoverInternal::new();
    // core thread
    // needs to handle node.receive and node.run_per_second_tasks
    // also handles other operations on the node via message passing from the REPL thread
    thread::spawn(move || loop {
        // node handler loop
        match vesc_node.receive(&mut rover_internal) {
            Ok(_) => {}
            Err(Error::Driver(e)) if e.kind() == ErrorKind::WouldBlock => {}
            Err(e) => panic!("{:?}", e),
        }
        match stepper_node.receive(&mut rover_internal) {
            Ok(_) => {}
            Err(Error::Driver(e)) if e.kind() == ErrorKind::WouldBlock => {}
            Err(e) => panic!("{:?}", e),
        }
        match cmd_rx.try_recv() {
            Ok(msg) => {
                match msg.op {
                    core::Operation::DriveStepper => {
                        let payload = StepperPlanar {
                            kinematics: Planar {
                                angular_position: angle_pos_scalar::Scalar { radian: msg.value },
                                angular_velocity: AVScalar { radian_per_second: f32::NAN },
                                angular_acceleration: AAScalar { radian_per_second_per_second: f32::NAN },
                            },
                            torque: TScalar {
                                newton_meter: f32::NAN
                            }
                        };
                        rover_internal.target_stepper_pos[msg.index] = msg.value;
                        // send the message thrice for redundancy
                        for _ in 0..3 {
                            stepper_node.publish(STEPPER_SETPOINT_SUB[msg.index].try_into().unwrap(), &payload).unwrap();
                        }
                    }
                    core::Operation::DriveVesc => {
                        let payload = VESCScalar {
                            value: half::f16::from_f32(msg.value)
                        };
                        // send the message thrice for redundancy
                        for _ in 0..3 {
                            vesc_node.publish(VESC_SPEED_SUB[msg.index].try_into().unwrap(), &payload).unwrap();
                        }
                    }
                    // this operation only makes sense in the context of the stepper
                    // to avoid deadlock, caller must NEVER call this twice in succession;
                    // it must do a blocking recv on the first call before calling again
                    core::Operation::NotifyWhenDone => {
                        println!("[BACKEND] Blocking on steering finishing...");
                        notify_in_flight = true;
                        notif_op_time = std::time::Instant::now();
                    }
                }
            }
            Err(_) => {}
        }
        if notify_in_flight {
            if rover_internal.is_idle() {
                notif_tx.send(Ok(())).unwrap();
                notify_in_flight = false;
                println!("[BACKEND] Steering resolved (motors reporting idle)");
            } else if std::time::Instant::now().duration_since(notif_op_time).as_secs() >= 3 {
                // 3-second timeout
                notif_tx.send(Ok(())).unwrap();
                notify_in_flight = false;
                println!("[BACKEND] Steering resolved (3-second timeout)");
            }
        }
        let seconds = std::time::Instant::now()
            .duration_since(start_time)
            .as_secs();
        if seconds != prev_seconds {
            prev_seconds = seconds;
            vesc_node.run_per_second_tasks().unwrap();
            vesc_node.flush().unwrap();
            stepper_node.run_per_second_tasks().unwrap();
            stepper_node.flush().unwrap();
        }
    });

    // assuming we start in this state...
    let mut rover = core::Rover {
        cmd_tx,
        notif_rx,
        wheels: core::WheelOrientation::Aligned(0.0),
    };

    let commands: [Box<dyn cmd::Cmd>; _] = [
        Box::new(cmd::Move {}),
        Box::new(cmd::Stop {}),
        Box::new(cmd::Help {}),
        Box::new(cmd::Quit {}),
    ];
    let mut keyword_lookup = HashMap::new();
    for i in 0..commands.len() {
        let command = &commands[i];
        for keyword in command.get_keywords() {
            keyword_lookup.insert(keyword, i);
        }
    }
    println!("Welcome to the new and improved (?) drivebase controller.");
    println!("Type 'help' for a list of commands, and 'help <command>' for command-specific help.");
    loop {
        print!("> ");
        io::stdout().flush().unwrap();

        let mut input = String::new();
        match io::stdin().read_line(&mut input) {
            Err(_) => {
                eprintln!("Error reading input");
                continue;
            }
            Ok(b) => {
                if b == 0 {
                    // EOF
                    break;
                }
            }
        }

        let input = input.trim();
        if input.is_empty() {
            continue;
        }

        let parts: Vec<&str> = input.split_whitespace().collect();
        let keyword = parts[0].to_lowercase();

        // special case: help command
        if keyword == "help" || keyword == "h" {
            if parts.len() < 2 {
                for command in commands.iter() {
                    println!("{}\t{}", command.get_keywords()[0], command.get_brief());
                }
                println!("Note commands and aliases are not case-sensitive.");
            } else {
                let help_keyword = parts[1].to_lowercase();
                if help_keyword == "help" || help_keyword == "h" {
                    println!("Usage: help\nOr:    help <command>");
                    println!("In the first form, lists available commands.");
                    println!("In the second form, provides more detailed information about a command.");
                }
                match keyword_lookup.get(help_keyword.as_str()) {
                    Some(cmd_idx) => {
                        let cmd = &commands[*cmd_idx];
                        println!("{}", cmd.get_help());
                    }
                    None => {
                        eprintln!("The word {} is not a recognised command or alias.", keyword);
                    }
                }
            }
        } else if keyword == "!" || keyword == "quit" || keyword == "exit" {
            // second special case: quit
            break;
        } else {
            match keyword_lookup.get(keyword.as_str()) {
                Some(cmd_idx) => {
                    let cmd = &commands[*cmd_idx];
                    match cmd.run(parts, &mut rover) {
                        Ok(_) => println!("Ok."),
                        Err(_) => println!("Error executing command.")
                    }
                }
                None => {
                    eprintln!("The word {} is not a recognised command or alias.", keyword);
                    eprintln!("Type 'help' for a list of commands.");
                }
            }
        }
    }
    println!("kthxbye");
    Ok(())
}

struct RoverInternal {
    curr_stepper_pos: [f32; 4],
    target_stepper_pos: [f32; 4],
}

impl RoverInternal {
    fn new() -> Self {
        Self {
            curr_stepper_pos: [0.0; 4],
            target_stepper_pos: [0.0; 4],
        }
    }
    
    fn is_idle(&self) -> bool {
        let mut is_idle = true;
        for i in 0..4 {
            // TODO: allow extra leeway here
            let delta = f32::abs(self.curr_stepper_pos[i] - self.target_stepper_pos[i]);
            const EPSILON: f32 = 0.001;
            if delta >= EPSILON {
                is_idle = false;
            }
        }
        is_idle
    }
}

impl<T: Transport> TransferHandler<T> for RoverInternal {
    fn handle_message<N>(&mut self, _node: &mut N, transfer: &MessageTransfer<Vec<u8>, T>) -> bool
    where
        N: Node<Transport = T>,
    {
        let Some(idx) = STEPPER_POSITION_SUB.iter().position(|&r| r == u16::from(transfer.header.subject)) else {
            // TODO: handle more messages here from nodes (Heartbeat?) for diagnostics
            return false;
        };
        // we're handling a stepper position message
        let Ok(stepper_pos) =
            angle_pos_scalar::Scalar::deserialize_from_bytes(&transfer.payload)
        else {
            return false;
        };
        self.curr_stepper_pos[idx] = stepper_pos.radian;
        true
    }

    fn handle_request<N>(
        &mut self,
        _node: &mut N,
        _token: ResponseToken<T>,
        transfer: &ServiceTransfer<Vec<u8>, T>,
    ) -> bool
    where
        N: Node<Transport = T>,
    {
        println!("Got request {:?}", transfer);
        false
    }

    fn handle_response<N>(&mut self, _node: &mut N, transfer: &ServiceTransfer<Vec<u8>, T>) -> bool
    where
        N: Node<Transport = T>,
    {
        println!("Got response {:?}", transfer);
        false
    }
}
