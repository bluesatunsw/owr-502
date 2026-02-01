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
    dir: bool,

    #[bits(1)]
    uvw_abi: bool,

    #[bits(1)]
    daec_dis: bool,

    #[bits(1)]
    abi_bin: bool,

    #[bits(1)]
    data_select: bool,

    #[bits(1)]
    pwm_on: bool,

    #[bits(8)]
    __: u32,
}

impl Register for Settings1 {
    const ADDRESS: u16 = 0x0018;
}

#[bitfield(u16)]
pub struct Settings2 {
    #[bits(3)]
    uvw_pp: u8,

    #[bits(2)]
    hys: u8,

    #[bits(3)]
    abi_res: u8,

    #[bits(8)]
    __: u32,
}

impl Register for Settings2 {
    const ADDRESS: u16 = 0x0019;
}

#[bitfield(u16)]
pub struct DiaAcg {
    #[bits(8)]
    agc: u8,

    #[bits(1)]
    lf: bool,

    #[bits(1)]
    cof: bool,

    #[bits(1)]
    magh: bool,

    #[bits(1)]
    magl: bool,

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
