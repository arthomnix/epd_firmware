#![no_std]

#[cfg(all(target_os = "none", target_arch = "arm"))]
pub mod header;
#[cfg(all(target_os = "none", target_arch = "arm"))]
pub mod syscall;
#[cfg(all(target_os = "none", target_arch = "arm"))]
pub mod misc;
#[cfg(all(target_os = "none", target_arch = "arm"))]
pub mod image;
#[cfg(all(target_os = "none", target_arch = "arm"))]
pub mod input;
#[cfg(all(target_os = "none", target_arch = "arm"))]
pub mod usb;
#[cfg(all(target_os = "none", target_arch = "arm"))]
pub mod exec;
#[cfg(all(target_os = "none", target_arch = "arm"))]
pub mod critical_section;
#[cfg(all(target_os = "none", target_arch = "arm"))]
pub mod flash;
#[cfg(all(target_os = "none", target_arch = "arm"))]
pub mod kv_store;

pub mod input_common;


#[cfg(feature = "critical-section-impl")]
#[cfg(all(target_os = "none", target_arch = "arm"))]
mod critical_section_impl;

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

#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SafeResult<T, E> {
    Ok(T),
    Err(E),
}

impl<T, E> From<SafeResult<T, E>> for Result<T, E> {
    fn from(value: SafeResult<T, E>) -> Self {
        match value {
            SafeResult::Ok(v) => Ok(v),
            SafeResult::Err(e) => Err(e),
        }
    }
}

impl<T, E> From<Result<T, E>> for SafeResult<T, E> {
    fn from(value: Result<T, E>) -> Self {
        match value {
            Ok(v) => SafeResult::Ok(v),
            Err(e) => SafeResult::Err(e),
        }
    }
}