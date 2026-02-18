use canadensis::core::SubjectId;

pub mod presets;

#[derive(Default)]
pub struct Config {
    pub comms: CommsConfig,
}

pub struct CommsConfig {
    pub node_id: u8,
    pub ctrl_volt: SubjectId,
}

impl Default for CommsConfig {
    fn default() -> Self {
        CommsConfig {
            node_id: 1,
            ctrl_volt: SubjectId::from_truncating(10),
        }
    }
}
