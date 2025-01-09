#![no_std]

pub use tp370pgh01::IMAGE_BYTES;

#[repr(C)]
pub struct ProgramFunctionTable {
    write_epd_image: extern "C" fn(&[u8; IMAGE_BYTES]),
}