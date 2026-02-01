#![no_std]

pub const MAGICW: u32 = 0x4452_5A57;

/// The physical layout of fields in memory are as
/// follows, all multibyte fields are LE:
/// 00-03: FL_CRC
/// 04-07: MAGICW
/// 
/// 08-09: HW_TYP
/// 0A-0A: HW_VER
/// 0B-0B: HW_REV
/// 
/// 0C-0C: SW_MAJ
/// 0D-0D: SW_MIN
/// 0E-0F: SW_BLD
/// 
/// 10-13: LN_CCM
/// 14-17: LN_RAM
/// 18-1B: LN_EXT
/// 1C-1F: LN_INT
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

    pub ln_ext: u32,
    pub ln_ccm: u32,
    pub ln_ram: u32,
    pub ln_int: u32,

    pub vt_adr: u32,
    pub ep_adr: u32,
}

pub const CHUNK_SIZE: usize = 4096;

#[derive(Debug, Clone, Copy)]
pub enum FlashLocation {
    External,
    Internal,
}

impl Header {
    pub const SIZE: usize = 64;

    pub fn serialize(&self) -> [u8; Self::SIZE] {
        let mut res = [0; Self::SIZE];

        res[0x00..0x04].copy_from_slice(&self.crc.to_le_bytes());
        res[0x04..0x08].copy_from_slice(&MAGICW.to_le_bytes());

        res[0x08..0x0A].copy_from_slice(&self.hw_typ.to_le_bytes());
        res[0x0A..0x0B].copy_from_slice(&self.hw_ver.to_le_bytes());
        res[0x0B..0x0C].copy_from_slice(&self.hw_rev.to_le_bytes());

        res[0x0C..0x0D].copy_from_slice(&self.sw_maj.to_le_bytes());
        res[0x0D..0x0E].copy_from_slice(&self.sw_min.to_le_bytes());
        res[0x0E..0x10].copy_from_slice(&self.sw_bld.to_le_bytes());

        res[0x10..0x14].copy_from_slice(&self.ln_ccm.to_le_bytes());
        res[0x14..0x18].copy_from_slice(&self.ln_ram.to_le_bytes());
        res[0x18..0x1C].copy_from_slice(&self.ln_ext.to_le_bytes());
        res[0x1C..0x20].copy_from_slice(&self.ln_int.to_le_bytes());

        res[0x20..0x24].copy_from_slice(&self.vt_adr.to_le_bytes());
        res[0x24..0x28].copy_from_slice(&self.ep_adr.to_le_bytes());

        res
    }

    pub fn deserialize(raw: &[u8; Self::SIZE]) -> Option<Self> {
        if u32::from_le_bytes(*raw[0x04..0x08].as_array()?) != MAGICW {
            return None;
        }
        if *raw[0x28..0x40].as_array()? !=  [0; 24] {
            return None
        }

        Some(Self {
            crc: u32::from_le_bytes(*raw[0x00..0x04].as_array().unwrap()),

            hw_typ: u16::from_le_bytes(*raw[0x08..0x0A].as_array().unwrap()),
            hw_ver: u8::from_le_bytes(*raw[0x0A..0x0B].as_array().unwrap()),
            hw_rev: u8::from_le_bytes(*raw[0x0B..0x0C].as_array().unwrap()),

            sw_maj: u8::from_le_bytes(*raw[0x0C..0x0D].as_array().unwrap()),
            sw_min: u8::from_le_bytes(*raw[0x0D..0x0E].as_array().unwrap()),
            sw_bld: u16::from_le_bytes(*raw[0x0E..0x10].as_array().unwrap()),

            ln_ccm: u32::from_le_bytes(*raw[0x10..0x14].as_array().unwrap()),
            ln_ram: u32::from_le_bytes(*raw[0x14..0x18].as_array().unwrap()),
            ln_ext: u32::from_le_bytes(*raw[0x18..0x1C].as_array().unwrap()),
            ln_int: u32::from_le_bytes(*raw[0x1C..0x20].as_array().unwrap()),

            vt_adr: u32::from_le_bytes(*raw[0x20..0x24].as_array().unwrap()),
            ep_adr: u32::from_le_bytes(*raw[0x24..0x28].as_array().unwrap()),
        })
    }

    pub fn total_ext_length(&self) -> usize {
        Self::SIZE
            + self.ln_ccm as usize
            + self.ln_ram as usize
            + self.ln_ext as usize
    }

    pub fn image_length(&self) -> usize {
        self.total_ext_length() + self.ln_int as usize
    }

    pub fn to_flash_location(&self, mut offset: usize) -> Option<(FlashLocation, usize)> {
        use FlashLocation::*;

        if offset < Self::SIZE {
            return None;
        }
        offset -= Self::SIZE;

        if offset < self.total_ext_length() {
            return Some((External, offset));
        }
        offset -= self.total_ext_length();

        if offset < self.ln_int as usize {
            Some((External, offset))
        } else {
            None
        }
    }
}

impl Default for Header {
    fn default() -> Self {
        Self {
            crc: 0x4C4C,

            hw_typ: 0xFFFF,
            hw_ver: 0x00,
            hw_rev: 0x00,

            sw_maj: 0x00,
            sw_min: 0x00,
            sw_bld: 0x00,

            ln_ccm: 0x0000_0000,
            ln_ram: 0x0000_0000,
            ln_ext: 0x0000_0000,
            ln_int: 0x0000_0000,

            vt_adr: 0x4C4C_4C4C,
            ep_adr: 0x4C4C_4C4C,
        }
    }
}
