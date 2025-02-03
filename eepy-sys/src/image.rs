use tp370pgh01::IMAGE_BYTES;
use crate::syscall;
use crate::syscall::SyscallNumber;

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ImageSyscall {
    WriteImage = 0,
    Refresh = 1,
}

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RefreshBlockMode {
    NonBlocking = 0,
    BlockAcknowledge = 1,
    BlockFinish = 2,
}

pub fn write_image(image: &[u8; IMAGE_BYTES]) {
    unsafe {
        syscall!(
            SyscallNumber::Image,
            in ImageSyscall::WriteImage,
            in &raw const *image,
        );
    }
}

pub fn refresh(fast_refresh: bool, refresh_block_mode: RefreshBlockMode) {
    unsafe {
        syscall!(
            SyscallNumber::Image,
            in ImageSyscall::Refresh,
            in fast_refresh,
            in refresh_block_mode,
        );
    }
}