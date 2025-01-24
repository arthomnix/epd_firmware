use core::arch::{asm, global_asm};
use cortex_m_rt::exception;
use defmt::debug;
use defmt::export::panic;
use eepy_sys::syscall::{MiscSyscall, SyscallNumber};
use crate::SERIAL_NUMBER;

global_asm!(include_str!("syscall.s"));

/// Main syscall (SVC) handler.
///
/// This function is called by the assembly trampoline code in `syscall.s`.
///
/// `sp` contains the caller's stack pointer value (taken from either MSP or PSP depending on which
/// stack was active). This is where the register values are pushed when the exception is triggered,
/// so this is where we need to read/write to interface with the caller.
/// This stack location contains (in order): r0, r1, r2, r3, r12, lr, pc, xPSR.
/// The pc value from the stack can be used to extract the literal operand from the SVC instruction
/// which triggered the syscall.
#[no_mangle]
extern "C" fn syscall(sp: *mut usize) {
    // Stack contains R0, R1, R2, R3, R12, LR, ReturnAddress, xPSR
    let stack_values = unsafe { core::slice::from_raw_parts_mut(sp, 8) };
    let syscall_num = unsafe { (stack_values[6] as *const u8).sub(2).read() };

    match SyscallNumber::try_from(syscall_num) {
        Ok(SyscallNumber::Misc) => match MiscSyscall::try_from(stack_values[0]) {
            Ok(MiscSyscall::GetSerial) => {
                let serial = SERIAL_NUMBER.get().unwrap();
                stack_values[0] = (&raw const *serial) as usize;
            },
            _ => panic!("illegal syscall"),
        },
        _ => panic!("illegal syscall"),
    }
}