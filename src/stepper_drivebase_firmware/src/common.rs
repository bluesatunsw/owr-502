#[derive(Debug, Clone, Copy)]
pub enum Channel {
    CH0,
    CH1,
    CH2,
    CH3,
}

pub const ALL_CHANNELS: [Channel; 4] = [Channel::CH0, Channel::CH1, Channel::CH2, Channel::CH3];
