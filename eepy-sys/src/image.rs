use tp370pgh01::IMAGE_BYTES;
use crate::syscall;
use crate::syscall::SyscallNumber;

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ImageSyscall {
    WriteImage = 0,
    Refresh = 1,
}

impl TryFrom<usize> for ImageSyscall {
    type Error = ();

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            x if x == ImageSyscall::WriteImage as usize => Ok(ImageSyscall::WriteImage),
            x if x == ImageSyscall::Refresh as usize => Ok(ImageSyscall::Refresh),
            _ => Err(()),
        }
    }
}

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RefreshBlockMode {
    NonBlocking,
    BlockAcknowledge,
    BlockFinish,
}

impl TryFrom<usize> for RefreshBlockMode {
    type Error = ();

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            x if x == RefreshBlockMode::NonBlocking as usize => Ok(RefreshBlockMode::NonBlocking),
            x if x == RefreshBlockMode::BlockAcknowledge as usize => Ok(RefreshBlockMode::BlockAcknowledge),
            x if x == RefreshBlockMode::BlockFinish as usize => Ok(RefreshBlockMode::BlockFinish),
            _ => Err(()),
        }
    }
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