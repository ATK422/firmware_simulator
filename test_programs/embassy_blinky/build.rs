//! Same memory map and linker scripts as titan_firmware_rs's `titan_build`.
use std::{env, fs, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(
        out.join("memory.x"),
        "MEMORY {\n    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100\n    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100\n    RAM   : ORIGIN = 0x20000000, LENGTH = 256K\n}\n",
    )
    .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rerun-if-changed=build.rs");
}
