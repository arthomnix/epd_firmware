use tp370pgh01::IMAGE_BYTES;
use crate::syscall;
use crate::syscall::SyscallNumber;

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ImageSyscall {
    Refresh = 0,
    MaybeRefresh = 1,
}

pub fn refresh(image: &[u8; IMAGE_BYTES], fast_refresh: bool) {
    unsafe {
        syscall!(
            SyscallNumber::Image,
            in ImageSyscall::Refresh,
            in fast_refresh,
            in &raw const *image,
        );
    }
}

pub fn maybe_refresh(image: &[u8; IMAGE_BYTES], fast_refresh: bool) {
    unsafe {
        syscall!(
            SyscallNumber::Image,
            in ImageSyscall::MaybeRefresh,
            in fast_refresh,
            in &raw const *image,
        );
    }
}