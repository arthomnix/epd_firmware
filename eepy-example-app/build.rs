use std::env;
use std::path::Path;

fn main() {
    if env::var("TARGET").unwrap() != "thumbv6m-none-eabi" {
        return;
    }

    let linker_script = std::fs::read_to_string("program.x").unwrap();
    let slot = env::var("EPD_PROG_SLOT").unwrap();
    let out_script = format!("SLOT_N = {slot};\n\n{linker_script}");

    let out_dir = env::var_os("OUT_DIR").unwrap();
    let out_dir = Path::new(&out_dir);
    let out_file = out_dir.join("program.x");
    std::fs::write(out_file, &out_script).unwrap();

    println!("cargo::rerun-if-changed={}", out_dir.join("program.x").to_str().unwrap());
    println!("cargo::rustc-link-search={}", out_dir.to_str().unwrap());
    println!("cargo::rustc-link-arg=-Tprogram.x");
    println!("cargo::rustc-link-arg=--nmagic");
}