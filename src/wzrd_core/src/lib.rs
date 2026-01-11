#![no_std]

pub const MAGIC_: u64 = 0x575A_5244;

/// The physical layout of fields in memory are as
/// follows, all multibyte fields are LE:
/// 00-03: FL_CRC
/// 04-07: MAGIC_
/// 
/// 08-09: HW_TYP
/// 0A-0A: HW_VER
/// 0B-0B: HW_REV
/// 
/// 0C-0C: SW_MAJ
/// 0D-0D: SW_MIN
/// 0E-0F: SW_BLD
/// 
/// 10-13: LN_INT
/// 14-17: LN_CCM
/// 18-1B: LN_RAM
/// 1C-1F: LN_EXT
/// 
/// 20-23: VT_ADR
/// 24-27: EP_ADR
/// 
/// 28-3F: RESV_0
#[derive(Clone, Copy, Debug)]
pub struct Header {
    pub crc: u32,

    pub hw_typ: u16,
    pub hw_ver: u8,
    pub hw_rev: u8,

    pub sw_maj: u8,
    pub sw_min: u8,
    pub sw_bld: u16,

    pub ln_int: u32,
    pub ln_ext: u32,
    pub ln_ccm: u32,
    pub ln_ram: u32,

    pub vt_adr: u32,
    pub ep_adr: u32,
}

pub const CHUNK_SIZE: usize = 4096;

pub enum FlashLocation {
    Internal,
    External,
}

impl Header {
    pub const SIZE: usize = 64;

    pub fn serialize(&self) -> [u8; Self::SIZE] {
        let mut res = [0; Self::SIZE];

        res[0x00..0x03].copy_from_slice(&self.crc.to_le_bytes());
        res[0x04..0x07].copy_from_slice(&MAGIC_.to_le_bytes());

        res[0x08..0x09].copy_from_slice(&self.hw_typ.to_le_bytes());
        res[0x0A..0x0A].copy_from_slice(&self.hw_ver.to_le_bytes());
        res[0x0B..0x0B].copy_from_slice(&self.hw_rev.to_le_bytes());

        res[0x0C..0x0C].copy_from_slice(&self.sw_maj.to_le_bytes());
        res[0x0D..0x0D].copy_from_slice(&self.sw_min.to_le_bytes());
        res[0x0E..0x0F].copy_from_slice(&self.sw_bld.to_le_bytes());

        res[0x10..0x13].copy_from_slice(&self.ln_int.to_le_bytes());
        res[0x14..0x17].copy_from_slice(&self.ln_ccm.to_le_bytes());
        res[0x18..0x1B].copy_from_slice(&self.ln_ram.to_le_bytes());
        res[0x1C..0x1F].copy_from_slice(&self.ln_ext.to_le_bytes());

        res[0x20..0x23].copy_from_slice(&self.vt_adr.to_le_bytes());
        res[0x24..0x27].copy_from_slice(&self.ep_adr.to_le_bytes());

        res
    }

    pub fn deserialize(raw: &[u8; Self::SIZE]) -> Self {
        assert_eq!(u64::from_le_bytes(*raw[0x04..0x07].as_array().unwrap()), MAGIC_);
        assert_eq!(*raw[0x28..0x3F].as_array().unwrap(), [0; 24]);

        Self {
            crc: u32::from_le_bytes(*raw[0x00..0x03].as_array().unwrap()),

            hw_typ: u16::from_le_bytes(*raw[0x08..0x09].as_array().unwrap()),
            hw_ver: u8::from_le_bytes(*raw[0x0A..0x0A].as_array().unwrap()),
            hw_rev: u8::from_le_bytes(*raw[0x0B..0x0B].as_array().unwrap()),

            sw_maj: u8::from_le_bytes(*raw[0x0C..0x0C].as_array().unwrap()),
            sw_min: u8::from_le_bytes(*raw[0x0D..0x0D].as_array().unwrap()),
            sw_bld: u16::from_le_bytes(*raw[0x0E..0x0F].as_array().unwrap()),

            ln_int: u32::from_le_bytes(*raw[0x10..0x13].as_array().unwrap()),
            ln_ccm: u32::from_le_bytes(*raw[0x14..0x17].as_array().unwrap()),
            ln_ram: u32::from_le_bytes(*raw[0x18..0x1B].as_array().unwrap()),
            ln_ext: u32::from_le_bytes(*raw[0x1C..0x1F].as_array().unwrap()),

            vt_adr: u32::from_le_bytes(*raw[0x20..0x23].as_array().unwrap()),
            ep_adr: u32::from_le_bytes(*raw[0x24..0x27].as_array().unwrap()),
        }
    }

    pub fn file_length(&self) -> usize {
        Self::SIZE
            + self.ln_int as usize
            + self.ln_ext as usize
            + self.ln_ccm as usize
            + self.ln_ram as usize
    }

    pub fn to_flash_location(&self, mut offset: usize) -> Option<(FlashLocation, usize)> {
        use FlashLocation::*;

        if offset < Self::SIZE {
            return None;
        }
        offset -= Self::SIZE;

        if offset < self.ln_int as usize {
            return Some((Internal, offset));
        }
        offset -= self.ln_int as usize;

        if offset < self.ln_ext as usize + self.ln_ccm  as usize + self.ln_ram as usize {
            Some((External, offset))
        } else {
            None
        }
    }
}
