use crate::syscall;
use crate::syscall::SyscallNumber;

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashSyscall {
    Erase = 0,
    Program = 1,
    EraseAndProgram = 2,
}

pub unsafe fn erase(start_addr: u32, len: u32) { unsafe {
    syscall!(
        SyscallNumber::Flash,
        in FlashSyscall::Erase,
        in start_addr,
        in len,
    );
}}

pub unsafe fn program(start_addr: u32, data: &[u8]) { unsafe {
    syscall!(
        SyscallNumber::Flash,
        in FlashSyscall::Program,
        in start_addr,
        in data.len(),
        in data.as_ptr(),
    );
}}

pub unsafe fn erase_and_program(start_addr: u32, data: &[u8]) { unsafe {
    syscall!(
        SyscallNumber::Flash,
        in FlashSyscall::EraseAndProgram,
        in start_addr,
        in data.len(),
        in data.as_ptr(),
    );
}}