use crate::critical_section::CsSyscall;
use crate::syscall;
use crate::syscall::SyscallNumber;

struct EepyCs;
critical_section::set_impl!(EepyCs);

unsafe impl critical_section::Impl for EepyCs {
    unsafe fn acquire() -> bool {
        let mut state: usize;
        syscall!(
            SyscallNumber::CriticalSection,
            out state in CsSyscall::Acquire,
        );
        state != 0
    }

    unsafe fn release(state: bool) {
        syscall!(
            SyscallNumber::CriticalSection,
            in CsSyscall::Release,
            in state,
        );
    }
}