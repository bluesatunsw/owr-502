#[derive(Debug, Clone, Copy)]
pub enum Channel {
    _0,
    _1,
    _2,
    _3,
}

pub const ALL_CHANNELS: [Channel; 4] = [Channel::_0, Channel::_1, Channel::_2, Channel::_3];
