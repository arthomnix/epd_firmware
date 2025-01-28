use core::hint::unreachable_unchecked;
use crate::syscall;
use crate::syscall::SyscallNumber;

pub fn exec(slot: u8) -> ! {
    unsafe {
        syscall!(
            SyscallNumber::Exec,
            in slot,
        );

        unreachable_unchecked()
    }
}

pub fn exit() -> ! {
    exec(0);
}