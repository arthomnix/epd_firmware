#![no_std]

#[cfg(feature = "embedded-graphics")]
pub mod eg;

pub use tp370pgh01::IMAGE_BYTES;

#[repr(C)]
#[derive(Copy, Clone)]
pub struct ProgramFunctionTable {
    pub write_image: extern "C" fn(&[u8; IMAGE_BYTES]),
    pub refresh: extern "C" fn(),
    pub refresh_fast: extern "C" fn(),
}