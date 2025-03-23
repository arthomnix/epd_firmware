mod kv_store;
mod flash;
mod cs;
mod input;
mod image;
mod misc;
mod exec;

use core::arch::global_asm;
use defmt::trace;
use eepy_sys::syscall::SyscallNumber;
use crate::exception::StackFrame;

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
#[unsafe(no_mangle)]
extern "C" fn handle_syscall(sp: *mut StackFrame, using_psp: bool) {
    // Stack contains R0, R1, R2, R3, R12, LR, ReturnAddress, xPSR
    let stack_values = unsafe { &mut *sp };
    let syscall_num = unsafe { *stack_values.pc.sub(2) };

    trace!("syscall: sp={} imm={:x} {}", sp, syscall_num, stack_values);

    match SyscallNumber::from_repr(syscall_num) {
        Some(SyscallNumber::Misc) => misc::handle_misc(stack_values),
        Some(SyscallNumber::Image) => image::handle_image(stack_values),
        Some(SyscallNumber::Input) => input::handle_input(stack_values),
        Some(SyscallNumber::Usb) => crate::usb::handle_usb(stack_values),
        Some(SyscallNumber::Exec) => exec::handle_exec(stack_values, using_psp),
        Some(SyscallNumber::CriticalSection) => cs::handle_cs(stack_values),
        Some(SyscallNumber::Flash) => flash::handle_flash(stack_values),
        Some(SyscallNumber::KvStore) => kv_store::handle_kv_store(stack_values),
        None => panic!("illegal syscall"),
    }
}