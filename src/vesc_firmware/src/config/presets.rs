use crate::config::{CommsConfig, Config};
use canadensis::core::SubjectId;

pub const DRIVEBASE_FL_CONFIG: Config = Config {
    comms: CommsConfig {
        node_id: 10,
        ctrl_volt: SubjectId::from_truncating(3050),
    },
};

pub const DRIVEBASE_FR_CONFIG: Config = Config {
    comms: CommsConfig {
        node_id: 11,
        ctrl_volt: SubjectId::from_truncating(3060),
    },
};

pub const DRIVEBASE_BL_CONFIG: Config = Config {
    comms: CommsConfig {
        node_id: 12,
        ctrl_volt: SubjectId::from_truncating(3070),
    },
};

pub const DRIVEBASE_BR_CONFIG: Config = Config {
    comms: CommsConfig {
        node_id: 13,
        ctrl_volt: SubjectId::from_truncating(3080),
    },
};
