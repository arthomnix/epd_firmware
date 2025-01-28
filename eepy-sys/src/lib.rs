#![no_std]

pub mod header;
pub mod syscall;
pub mod misc;
pub mod image;
pub mod input;
pub mod usb;
pub mod exec;

pub use tp370pgh01::IMAGE_BYTES;

/// FFI-safe version of the standard `Option` type.
///
/// Convert to/from a standard `Option` using `.into()`.
#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SafeOption<T> {
    None,
    Some(T),
}

impl<T> From<Option<T>> for SafeOption<T> {
    fn from(value: Option<T>) -> Self {
        match value {
            None => SafeOption::None,
            Some(v) => SafeOption::Some(v),
        }
    }
}

impl<T> From<SafeOption<T>> for Option<T> {
    fn from(value: SafeOption<T>) -> Self {
        match value {
            SafeOption::None => None,
            SafeOption::Some(v) => Some(v),
        }
    }
}