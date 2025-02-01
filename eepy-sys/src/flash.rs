use crate::syscall;
use crate::syscall::SyscallNumber;

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashSyscall {
    Erase = 0,
    Program = 1,
    EraseAndProgram = 2,
    InvalidateCache = 3,
}

impl TryFrom<usize> for FlashSyscall {
    type Error = ();

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            x if x == FlashSyscall::Erase as usize => Ok(FlashSyscall::Erase),
            x if x == FlashSyscall::Program as usize => Ok(FlashSyscall::Program),
            x if x == FlashSyscall::EraseAndProgram as usize => Ok(FlashSyscall::EraseAndProgram),
            x if x == FlashSyscall::InvalidateCache as usize => Ok(FlashSyscall::InvalidateCache),
            _ => Err(()),
        }
    }
}

pub unsafe fn erase(start_addr: u32, len: u32) {
    syscall!(
        SyscallNumber::Flash,
        in FlashSyscall::Erase,
        in start_addr,
        in len,
    );
}

pub unsafe fn program(start_addr: u32, data: &[u8]) {
    syscall!(
        SyscallNumber::Flash,
        in FlashSyscall::Program,
        in start_addr,
        in data.len(),
        in data.as_ptr(),
    );
}

pub unsafe fn erase_and_program(start_addr: u32, data: &[u8]) {
    syscall!(
        SyscallNumber::Flash,
        in FlashSyscall::EraseAndProgram,
        in start_addr,
        in data.len(),
        in data.as_ptr(),
    );
}

pub unsafe fn invalidate_cache() {
    syscall!(
        SyscallNumber::Flash,
        in FlashSyscall::InvalidateCache,
    );
}