use core::mem::MaybeUninit;
use crate::{syscall, SafeResult};
use crate::syscall::SyscallNumber;

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum KvStoreSyscall {
    ReadKey = 0,
    AppendKey = 1,
    PutKey = 2,
    InvalidateKey = 3,
    ZeroiseKey = 4,
}

#[repr(C)]
pub struct ReadKeyArgs {
    pub key_len: usize,
    pub key_ptr: *const u8,
    pub buf_len: usize,
    pub buf_ptr: *mut u8,
}

#[repr(C)]
pub struct WriteKeyArgs {
    pub key_len: usize,
    pub key_ptr: *const u8,
    pub value_len: usize,
    pub value_ptr: *const u8,
}

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ReadKeyError {
    FlashCorrupt,
    InvalidChecksum,
    NotFound,
    BufferTooSmall(usize),
}

#[cfg(feature = "tickv")]
impl TryFrom<tickv::ErrorCode> for ReadKeyError {
    type Error = tickv::ErrorCode;

    fn try_from(value: tickv::ErrorCode) -> Result<Self, Self::Error> {
        use tickv::ErrorCode as E;

        match value {
            E::CorruptData => Ok(Self::FlashCorrupt),
            E::InvalidCheckSum => Ok(Self::InvalidChecksum),
            E::KeyNotFound => Ok(Self::NotFound),
            E::BufferTooSmall(size) => Ok(Self::BufferTooSmall(size)),
            _ => Err(value),
        }
    }
}

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum AppendKeyError {
    FlashCorrupt,
    AlreadyExists,
    RegionFull,
    FlashFull,
}

#[cfg(feature = "tickv")]
impl TryFrom<tickv::ErrorCode> for AppendKeyError {
    type Error = tickv::ErrorCode;

    fn try_from(value: tickv::ErrorCode) -> Result<Self, Self::Error> {
        use tickv::ErrorCode as E;

        match value {
            E::CorruptData => Ok(Self::FlashCorrupt),
            E::KeyAlreadyExists => Ok(Self::AlreadyExists),
            E::RegionFull => Ok(Self::RegionFull),
            E::FlashFull => Ok(Self::FlashFull),
            _ => Err(value),
        }
    }
}

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PutKeyError {
    FlashCorrupt,
    RegionFull,
    FlashFull,
}

#[cfg(feature = "tickv")]
impl TryFrom<tickv::ErrorCode> for PutKeyError {
    type Error = tickv::ErrorCode;

    fn try_from(value: tickv::ErrorCode) -> Result<Self, Self::Error> {
        use tickv::ErrorCode as E;

        match value {
            E::CorruptData => Ok(Self::FlashCorrupt),
            E::RegionFull => Ok(Self::RegionFull),
            E::FlashFull => Ok(Self::FlashFull),
            _ => Err(value),
        }
    }
}

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum DeleteKeyError {
    FlashCorrupt,
    NotFound,
}

#[cfg(feature = "tickv")]
impl TryFrom<tickv::ErrorCode> for DeleteKeyError {
    type Error = tickv::ErrorCode;

    fn try_from(value: tickv::ErrorCode) -> Result<Self, Self::Error> {
        use tickv::ErrorCode as E;

        match value {
            E::CorruptData => Ok(Self::FlashCorrupt),
            E::KeyNotFound => Ok(Self::NotFound),
            _ => Err(value),
        }
    }
}

pub fn get(key: &[u8], buf: &mut [u8]) -> Result<usize, ReadKeyError> {
    let args = ReadKeyArgs {
        key_len: key.len(),
        key_ptr: key.as_ptr(),
        buf_len: buf.len(),
        buf_ptr: buf.as_mut_ptr(),
    };
    let mut res: MaybeUninit<SafeResult<usize, ReadKeyError>> = MaybeUninit::uninit();

    unsafe {
        syscall!(
            SyscallNumber::KvStore,
            in KvStoreSyscall::ReadKey,
            in &raw const args,
            in res.as_mut_ptr(),
        );

        res.assume_init().into()
    }
}

pub fn append(key: &[u8], value: &[u8]) -> Result<(), AppendKeyError> {
    let args = WriteKeyArgs {
        key_len: key.len(),
        key_ptr: key.as_ptr(),
        value_len: value.len(),
        value_ptr: value.as_ptr(),
    };
    let mut res: MaybeUninit<SafeResult<(), AppendKeyError>> = MaybeUninit::uninit();

    unsafe {
        syscall!(
            SyscallNumber::KvStore,
            in KvStoreSyscall::AppendKey,
            in &raw const args,
            in res.as_mut_ptr(),
        );

        res.assume_init().into()
    }
}

pub fn put(key: &[u8], value: &[u8]) -> Result<(), PutKeyError> {
    let args = WriteKeyArgs {
        key_len: key.len(),
        key_ptr: key.as_ptr(),
        value_len: value.len(),
        value_ptr: value.as_ptr(),
    };
    let mut res: MaybeUninit<SafeResult<(), PutKeyError>> = MaybeUninit::uninit();

    unsafe {
        syscall!(
            SyscallNumber::KvStore,
            in KvStoreSyscall::PutKey,
            in &raw const args,
            in res.as_mut_ptr(),
        );

        res.assume_init().into()
    }
}

pub fn invalidate(key: &[u8]) -> Result<(), DeleteKeyError> {
    let mut res: MaybeUninit<SafeResult<(), DeleteKeyError>> = MaybeUninit::uninit();

    unsafe {
        syscall!(
            SyscallNumber::KvStore,
            in KvStoreSyscall::InvalidateKey,
            in key.as_ptr(),
            in key.len(),
            in res.as_mut_ptr(),
        );

        res.assume_init().into()
    }
}

pub fn zeroise(key: &[u8]) -> Result<(), DeleteKeyError> {
    let mut res: MaybeUninit<SafeResult<(), DeleteKeyError>> = MaybeUninit::uninit();

    unsafe {
        syscall!(
            SyscallNumber::KvStore,
            in KvStoreSyscall::ZeroiseKey,
            in key.as_ptr(),
            in key.len(),
            in res.as_mut_ptr(),
        );

        res.assume_init().into()
    }
}