use core::fmt::Debug;

use bitfield_struct::bitfield;

pub trait Register: From<u16> + Into<u16> + Debug {
    const ADDRESS: u16;
}

#[bitfield(u16)]
pub struct Settings1 {
    #[default(0x1)]
    #[bits(2)]
    __: u32,

    #[bits(1)]
    pub dir: bool,

    #[bits(1)]
    pub uvw_abi: bool,

    #[bits(1)]
    pub daec_dis: bool,

    #[bits(1)]
    pub abi_bin: bool,

    #[bits(1)]
    pub data_select: bool,

    #[bits(1)]
    pub pwm_on: bool,

    #[bits(8)]
    __: u32,
}

impl Register for Settings1 {
    const ADDRESS: u16 = 0x0018;
}

#[bitfield(u16)]
pub struct Settings2 {
    #[bits(3)]
    pub uvw_pp: u8,

    #[bits(2)]
    pub hys: u8,

    #[bits(3)]
    pub abi_res: u8,

    #[bits(8)]
    __: u32,
}

impl Register for Settings2 {
    const ADDRESS: u16 = 0x0019;
}

#[bitfield(u16)]
pub struct DiaAcg {
    #[bits(8)]
    pub agc: u8,

    #[bits(1)]
    pub lf: bool,

    #[bits(1)]
    pub cof: bool,

    #[bits(1)]
    pub magh: bool,

    #[bits(1)]
    pub magl: bool,

    #[bits(4)]
    __: u32,
}

impl Register for DiaAcg {
    const ADDRESS: u16 = 0x3FFC;
}

macro_rules! value_register {
    ($name:ident, $addr:literal) => {
        #[derive(Debug)]
        pub struct $name(pub u16);
        impl Register for $name {
            const ADDRESS: u16 = $addr;
        }
        impl From<u16> for $name {
            fn from(value: u16) -> Self {
                Self { 0: value }
            }
        }
        impl Into<u16> for $name {
            fn into(self) -> u16 {
                self.0
            }
        }
    };
}

value_register!(Mag, 0x3FFD);
value_register!(AngleUnc, 0x3FFE);
value_register!(AngleCom, 0x3FFF);
