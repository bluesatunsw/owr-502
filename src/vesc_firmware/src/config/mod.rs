use canadensis::core::SubjectId;

pub mod presets;

pub struct AppConfig {
    pub comms: CommsConfig,
}

pub struct CommsConfig {
    pub node_id: u8,
    pub ctrl_volt: SubjectId,
}

impl Default for AppConfig {
    fn default() -> Self {
        AppConfig {
            comms: CommsConfig {
                node_id: 1,
                ctrl_volt: SubjectId::from_truncating(10),
            },
        }
    }
}
