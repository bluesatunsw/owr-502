use std::{fmt::Debug, fs, path::PathBuf};

use clap::Parser;
use crc::{CRC_32_ISO_HDLC, Crc};
use elf::{ElfBytes, endian::AnyEndian};
use wzrd_core::Header;

const CRC: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

const ABOUT: &str = "
A utility to convert an ELF binary into WIZ image which can \
be remotely deployed to the DNM bootloader.

WARNING: this program does far fewer checks than it ought to \
and will produce a malformed image without warning if the \
ELF binary is not produced using the correct linker script. \
Refer to the embedded_common crate for more information. \
";

#[derive(Parser, Debug)]
#[command(version, about=ABOUT, long_about=ABOUT)]
struct Args {
    /// Path to the ELF binary
    #[arg(short, long)]
    in_file: PathBuf,

    /// Path to the resulting WIZ image
    #[arg(short, long)]
    out_file: PathBuf,
}

fn get_section(elf: &ElfBytes<AnyEndian>, name: &str) -> Vec<u8> {
    let shdr = elf
        .section_header_by_name(name)
        .expect("Failed to parse section table")
        .expect(format!("Section {} does not exist, ensure that the correct linker script is being used", name).as_str());

    elf.section_data(&shdr)
        .expect(format!("Failed to get data from section {}", name).as_str())
        .0.to_vec()
}

fn get_symbol<T: TryFrom<u64>>(elf: &ElfBytes<AnyEndian>, name: &str) -> T where <T as TryFrom<u64>>::Error: Debug {
    // Reparsing every time, but I am too lazy to fix this

    let common = elf.find_common_data().expect("Failed to parse shdrs");
    let (dynsyms, strtab) = (common.dynsyms.unwrap(), common.dynsyms_strs.unwrap());

    common.sysv_hash.unwrap()
        .find(name.as_bytes(), &dynsyms, &strtab)
        .expect(format!("Failed to find symbol {}", name).as_str()).unwrap()
        .1.st_value
        .try_into().unwrap()
}

pub fn main() {
    let args = Args::parse();

    let file_data = fs::read(args.in_file).expect("Could not read ELF file");
    let elf = ElfBytes::<AnyEndian>::minimal_parse(file_data.as_slice()).expect("Failed to parse ELF file");

    let mut res = vec![];

    let int = get_section(&elf, ".int");
    let ccm = get_section(&elf, ".ccm");
    let ram = get_section(&elf, ".ram");
    let ext = get_section(&elf, ".ext");

    res.extend_from_slice(&Header {
        crc: 0,

        hw_typ: get_symbol(&elf, "hw_typ"),
        hw_ver: get_symbol(&elf, "hw_ver"),
        hw_rev: get_symbol(&elf, "hw_rev"),

        sw_maj: get_symbol(&elf, "hw_maj"),
        sw_min: get_symbol(&elf, "hw_min"),
        sw_bld: get_symbol(&elf, "hw_bld"),

        ln_int: int.len().try_into().unwrap(),
        ln_ccm: ccm.len().try_into().unwrap(),
        ln_ram: ram.len().try_into().unwrap(),
        ln_ext: ext.len().try_into().unwrap(),

        vt_adr: get_symbol(&elf, "__vector_table"),
        ep_adr: get_symbol(&elf, "main"),
    }.serialize());

    res.extend(int);
    res.extend(ccm);
    res.extend(ram);
    res.extend(ext);

    let checksum = CRC.checksum(&res).to_le_bytes();
    res[0..4].copy_from_slice(&checksum);

    fs::write(args.out_file, res).expect("Failed to write to output file");
}
