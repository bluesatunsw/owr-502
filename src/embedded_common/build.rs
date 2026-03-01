use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::env;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let link_x = include_bytes!("bs_link.x");

    let mut f = File::create(out.join("bs_link.x")).unwrap();
    f.write_all(link_x).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
}
