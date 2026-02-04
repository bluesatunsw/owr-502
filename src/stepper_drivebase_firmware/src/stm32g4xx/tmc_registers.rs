use core::{f32, fmt::Debug, ops::Mul};

use bitfield_struct::bitfield;

/// Asume the highest microstepping resolution
const MICROSTEPS_PER_STEP: u32 = 256;
/// Number of microsteps per motor revolution
const MICROSTEPS_PER_REV: u32 = 200 * MICROSTEPS_PER_STEP;

pub trait Register: From<u32> + Into<u32> + Debug {
    const ADDRESS: u8;
}

#[derive(Debug, PartialEq, PartialOrd)]
pub struct TmcPosition(pub f32);

impl TmcPosition {
    const CONVERSION_FACTOR: f32 = MICROSTEPS_PER_REV as f32 / (2. * f32::consts::PI);

    pub fn from_bits(value: u32) -> Self {
        Self(value as i32 as f32 / Self::CONVERSION_FACTOR)
    }

    pub fn bits(&self) -> u32 {
        (self.0 * Self::CONVERSION_FACTOR) as i32 as u32
    }
}

impl Mul<f32> for TmcPosition {
    type Output = TmcPosition;

    fn mul(self, rhs: f32) -> Self::Output {
        TmcPosition(self.0 * rhs)
    }
}

#[derive(Debug, PartialEq, PartialOrd)]
pub struct TmcVelocity(f32);

impl TmcVelocity {
    const CONVERSION_FACTOR: f32 =
        MICROSTEPS_PER_REV as f32 / (2. * f32::consts::PI) / 12_000_000. * 2. * (1 << 23) as f32;

    pub fn from_bits(value: u32) -> Self {
        Self(value as i32 as f32 * Self::CONVERSION_FACTOR)
    }

    pub fn bits(&self) -> u32 {
        (self.0 * Self::CONVERSION_FACTOR) as i32 as u32
    }
}

#[derive(Debug, PartialEq, PartialOrd)]
pub struct TmcAcceleration(f32);

impl TmcAcceleration {
    const CONVERSION_FACTOR: f32 =
        MICROSTEPS_PER_REV as f32 / (2. * f32::consts::PI) / (12_000_000.) / (12_000_000.)
            * (512. * 256.)
            * (1 << 24) as f32;

    pub fn from_bits(value: u32) -> Self {
        Self(value as i32 as f32 * Self::CONVERSION_FACTOR)
    }

    pub fn bits(&self) -> u32 {
        (self.0 * Self::CONVERSION_FACTOR) as i32 as u32
    }
}

#[derive(Debug)]
pub struct TmcUnitless(i32);

impl TmcUnitless {
    pub fn from_bits(value: u32) -> Self {
        Self(value as i32)
    }

    pub fn bits(&self) -> u32 {
        self.0 as u32
    }
}

pub trait UnitsExt {
    fn rads(self) -> TmcPosition;
    fn rps(self) -> TmcVelocity;
    fn rps2(self) -> TmcAcceleration;
}

impl UnitsExt for f32 {
    fn rads(self) -> TmcPosition {
        TmcPosition(self)
    }

    fn rps(self) -> TmcVelocity {
        TmcVelocity(self)
    }

    fn rps2(self) -> TmcAcceleration {
        TmcAcceleration(self)
    }
}

pub trait UnitlessExt {
    fn ul(self) -> TmcUnitless;
}

impl UnitlessExt for i32 {
    fn ul(self) -> TmcUnitless {
        TmcUnitless(self)
    }
}

macro_rules! motion_register {
    ($name:ident, $type:ty, $addr:literal) => {
        #[derive(Debug)]
        pub struct $name(pub $type);
        impl Register for $name {
            const ADDRESS: u8 = $addr;
        }
        impl From<u32> for $name {
            fn from(value: u32) -> Self {
                Self {
                    0: <$type>::from_bits(value),
                }
            }
        }
        impl Into<u32> for $name {
            fn into(self) -> u32 {
                self.0.bits()
            }
        }
    };
}

#[bitfield(u32)]
pub struct GConf {
    #[bits(1)]
    pub recalibrate: bool,

    #[bits(1)]
    pub faststandstill: bool,

    #[bits(1)]
    pub en_pwm_mode: bool,

    #[bits(1)]
    __: u8,

    #[bits(1)]
    pub shaft: bool,

    #[bits(27)]
    __: u32,
}

impl Register for GConf {
    const ADDRESS: u8 = 0x00;
}

#[bitfield(u32)]
pub struct GStat {
    #[bits(1)]
    pub reset: bool,

    #[bits(1)]
    pub drv_err: bool,

    #[bits(1)]
    pub uv_cp: bool,

    #[bits(29)]
    __: u32,
}

impl Register for GStat {
    const ADDRESS: u8 = 0x01;
}

#[bitfield(u32)]
pub struct ShortConf {
    #[bits(4)]
    pub s2vs_level: u8,

    #[bits(4)]
    __: u32,

    #[bits(4)]
    pub s2g_level: u8,

    #[bits(4)]
    __: u32,

    #[bits(2)]
    pub short_filter: u8,

    #[bits(1)]
    pub short_delay: bool,

    #[bits(13)]
    __: u32,
}

impl Register for ShortConf {
    const ADDRESS: u8 = 0x09;
}

#[bitfield(u32)]
pub struct DrvConf {
    #[bits(5)]
    pub bbm_time: u8,

    #[bits(3)]
    __: u32,

    #[bits(4)]
    pub bbm_clks: u8,

    #[bits(4)]
    __: u32,

    #[bits(2)]
    pub ot_select: u8,

    #[bits(2)]
    pub drv_strength: u8,

    #[bits(2)]
    pub filt_isense: u8,

    #[bits(10)]
    __: u32,
}

impl Register for DrvConf {
    const ADDRESS: u8 = 0x0A;
}

motion_register!(GlobalScalar, TmcUnitless, 0x0B);

#[bitfield(u32)]
pub struct IHoldIRun {
    #[bits(5)]
    pub ihold: u8,

    #[bits(3)]
    __: u32,

    #[bits(5)]
    pub irun: u8,

    #[bits(3)]
    __: u32,

    #[bits(4)]
    pub ihold_delay: u8,

    #[bits(12)]
    __: u32,
}

impl Register for IHoldIRun {
    const ADDRESS: u8 = 0x10;
}

motion_register!(TPowerDown, TmcUnitless, 0x11);
motion_register!(TPwmThrs, TmcVelocity, 0x13);
motion_register!(TCoolThrs, TmcVelocity, 0x14);
motion_register!(THigh, TmcVelocity, 0x15);

motion_register!(RampMode, TmcUnitless, 0x20);
motion_register!(XActual, TmcPosition, 0x21);
motion_register!(VActual, TmcVelocity, 0x22);
motion_register!(VStart, TmcVelocity, 0x23);
motion_register!(A1, TmcAcceleration, 0x24);
motion_register!(V1, TmcVelocity, 0x25);
motion_register!(AMax, TmcAcceleration, 0x26);
motion_register!(VMax, TmcVelocity, 0x27);
motion_register!(DMax, TmcAcceleration, 0x28);
motion_register!(D1, TmcAcceleration, 0x2A);
motion_register!(VStop, TmcVelocity, 0x2B);
motion_register!(TZeroWait, TmcUnitless, 0x2C);
motion_register!(XTarget, TmcPosition, 0x2D);

motion_register!(VDcMin, TmcVelocity, 0x33);

#[bitfield(u32)]
pub struct ChopConf {
    #[bits(4)]
    pub toff: u8,

    #[bits(3)]
    pub hstrt: u8,

    #[bits(4)]
    pub hend: u8,

    #[bits(1)]
    pub fd3: u8,

    #[bits(1)]
    pub disfdcc: bool,

    #[bits(1)]
    __: u32,

    #[bits(1)]
    pub chm: bool,

    #[bits(2)]
    pub tbl: u8,

    #[bits(1)]
    __: u32,

    #[bits(1)]
    pub vhighfs: bool,

    #[bits(1)]
    pub vhighchm: bool,

    #[bits(4)]
    pub tpdf: u8,

    #[bits(4)]
    pub mres: u8,

    #[bits(1)]
    pub interpol: bool,

    #[bits(1)]
    pub dedge: bool,

    #[bits(1)]
    pub diss2g: bool,

    #[bits(1)]
    pub diss2vs: bool,
}

impl Register for ChopConf {
    const ADDRESS: u8 = 0x6C;
}

#[bitfield(u32)]
pub struct CoolConf {
    #[bits(4)]
    pub semin: u8,

    #[bits(1)]
    __: u32,

    #[bits(2)]
    pub seup: u8,

    #[bits(1)]
    __: u32,

    #[bits(4)]
    pub semax: u8,

    #[bits(1)]
    __: u32,

    #[bits(2)]
    pub sedn: u8,

    #[bits(1)]
    pub seimin: bool,

    #[bits(7)]
    pub sgt: i8,

    #[bits(1)]
    __: u32,

    #[bits(1)]
    pub sfilt: bool,

    #[bits(7)]
    __: u32,
}

impl Register for CoolConf {
    const ADDRESS: u8 = 0x6D;
}

#[bitfield(u32)]
pub struct DcCtrl {
    #[bits(10)]
    pub dc_time: u16,

    #[bits(6)]
    __: u32,

    #[bits(8)]
    pub dc_sg: u8,

    #[bits(8)]
    __: u32,
}

impl Register for DcCtrl {
    const ADDRESS: u8 = 0x6E;
}

#[bitfield(u32)]
pub struct DrvStatus {
    #[bits(10)]
    pub sg_result: u16,

    #[bits(2)]
    __: u32,

    #[bits(1)]
    pub s2vsa: bool,

    #[bits(1)]
    pub s2vsb: bool,

    #[bits(1)]
    pub stealth: bool,

    #[bits(1)]
    pub fsactive: bool,

    #[bits(5)]
    pub cs_actual: u8,

    #[bits(3)]
    __: u32,

    #[bits(1)]
    pub stall_guard: bool,

    #[bits(1)]
    pub ot: bool,

    #[bits(1)]
    pub otpw: bool,

    #[bits(1)]
    pub s2ga: bool,

    #[bits(1)]
    pub s2gb: bool,

    #[bits(1)]
    pub ola: bool,

    #[bits(1)]
    pub olb: bool,

    #[bits(1)]
    pub stst: bool,
}

impl Register for DrvStatus {
    const ADDRESS: u8 = 0x6F;
}

#[bitfield(u32)]
pub struct PwmConf {
    #[bits(8)]
    pub pwm_ofs: u8,

    #[bits(8)]
    pub pwm_grad: u8,

    #[bits(2)]
    pub pwm_freq: u8,

    #[bits(1)]
    pub pwm_autoscale: bool,

    #[bits(1)]
    pub pwm_autograd: bool,

    #[bits(2)]
    pub freewheel: u8,

    #[bits(2)]
    __: u32,

    #[bits(4)]
    pub pwm_reg: u8,

    #[bits(4)]
    pub pwm_lim: u8,
}

impl Register for PwmConf {
    const ADDRESS: u8 = 0x70;
}

#[bitfield(u32)]
pub struct PwmAuto {
    #[bits(8)]
    pub pwm_ofs_auto: u8,

    #[bits(8)]
    __: u32,

    #[bits(8)]
    pub pwm_grad_auto: u8,

    #[bits(8)]
    __: u32,
}

impl Register for PwmAuto {
    const ADDRESS: u8 = 0x72;
}
