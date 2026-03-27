use std::sync::mpsc;

pub enum Operation {
    DriveStepper,
    DriveVesc,
    NotifyWhenDone
}

#[derive(PartialEq, Copy, Clone)]
pub enum WheelOrientation {
    Aligned(f32),
    RotateInPlace
}

/// A command that is handled by the internal Cyphal thread.
pub struct NodeCommand {
    pub op: Operation,
    pub values: [f32; 4]
}

/// The state of the rover as understood by the top layer, plus interfaces
/// with the lower layer.
pub struct Rover {
    pub is_stopped: bool,
    pub wheels: WheelOrientation,
    pub cmd_tx: mpsc::Sender<NodeCommand>,
    pub notif_rx: mpsc::Receiver<Result<(),()>>,
}
