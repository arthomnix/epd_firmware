use core::arch::{asm, global_asm};
use defmt::{debug, trace, Formatter};
use eepy_sys::exec::exec;
use eepy_sys::header::slot;
use eepy_sys::syscall::SyscallNumber;
use crate::SRAM_END;

global_asm!(include_str!("syscall.s"));

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
struct StackFrame {
    r0: usize,
    r1: usize,
    r2: usize,
    r3: usize,
    r12: usize,
    lr: *const u8,
    pc: *const u8,
    xpsr: usize,
}

impl defmt::Format for StackFrame {
    fn format(&self, fmt: Formatter) {
        defmt::write!(
            fmt,
            "r0=0x{:x} r1=0x{:x} r2=0x{:x} r3=0x{:x} r12=0x{:x} lr={:x} pc={:x} xpsr=0x{:x}",
            self.r0,
            self.r1,
            self.r2,
            self.r3,
            self.r12,
            self.lr,
            self.pc,
            self.xpsr,
        )
    }
}

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
extern "C" fn handle_syscall(sp: *mut StackFrame, using_psp: bool) {
    // Stack contains R0, R1, R2, R3, R12, LR, ReturnAddress, xPSR
    let stack_values = unsafe { &mut *sp };
    let syscall_num = unsafe { *stack_values.pc.sub(2) };

    trace!("syscall: sp={} imm={:x} {}", sp, syscall_num, stack_values);

    match SyscallNumber::try_from(syscall_num) {
        Ok(SyscallNumber::Misc) => misc::handle_misc(stack_values),
        Ok(SyscallNumber::Image) => image::handle_image(stack_values),
        Ok(SyscallNumber::Input) => input::handle_input(stack_values),
        Ok(SyscallNumber::Usb) => todo!("usb syscalls"),
        Ok(SyscallNumber::Exec) => handle_exec(stack_values, using_psp),
        Err(_) => panic!("illegal syscall"),
    }
}

fn handle_exec(stack_values: &mut StackFrame, using_psp: bool) {
    // disable privileged mode
    unsafe {
        asm!(
            "msr control, {control_0b11}",
            "isb",
            control_0b11 = in(reg) 0b11,
        );
    }

    let slot_n = stack_values.r0 as u8;
    unsafe {
        let program = slot(slot_n);
        if !(*program).is_valid() {
            panic!("tried to exec invalid program");
        }

        (*program).load();
        stack_values.pc = core::mem::transmute((*program).entry);
        stack_values.lr = program_return_handler as *const u8;

        // Move the saved registers to the top of program memory
        // This makes sure all programs start with an empty program stack
        if using_psp {
            let ptr: *mut StackFrame = SRAM_END.sub(size_of::<StackFrame>()).cast();
            ptr.write(*stack_values);
            asm!(
                "msr psp, {ptr}",
                ptr = in(reg) ptr,
            );
        } else {
            // If the saved registers are on the main stack, just set the PSP to
            // the top of program memory
            asm!(
                "msr psp, {ptr}",
                ptr = in(reg) SRAM_END,
            )
        }
    }
}

// NOTE: this function runs in unprivileged thread mode
extern "C" fn program_return_handler() {
    exec(0);
}

mod misc {
    use eepy_sys::misc::MiscSyscall;
    use crate::SERIAL_NUMBER;
    use super::StackFrame;

    pub(super) fn handle_misc(stack_values: &mut StackFrame) {
        match MiscSyscall::try_from(stack_values.r0) {
            Ok(MiscSyscall::GetSerial) => handle_get_serial(stack_values),
            _ => panic!("illegal syscall"),
        }
    }

    fn handle_get_serial(stack_values: &mut StackFrame) {
        stack_values.r0 = (&raw const *SERIAL_NUMBER.get().unwrap()) as usize;
    }
}

mod image {
    use core::sync::atomic::Ordering;
    use eepy_sys::image::{ImageSyscall, RefreshBlockMode};
    use tp370pgh01::IMAGE_BYTES;
    use crate::{DO_REFRESH, FAST_REFRESH, IMAGE_BUFFER, REFRESHING};
    use super::StackFrame;

    pub(super) fn handle_image(stack_values: &mut StackFrame) {
        match ImageSyscall::try_from(stack_values.r0) {
            Ok(ImageSyscall::WriteImage) => handle_write_image(stack_values),
            Ok(ImageSyscall::Refresh) => handle_refresh(stack_values),
            _ => panic!("illegal syscall"),
        }
    }

    fn handle_write_image(stack_values: &mut StackFrame) {
        let image: &[u8; IMAGE_BYTES] = unsafe { &*(stack_values.r1 as *const [u8; IMAGE_BYTES]) };
        critical_section::with(|cs| IMAGE_BUFFER.borrow_ref_mut(cs).copy_from_slice(image));
    }

    fn handle_refresh(stack_values: &mut StackFrame) {
        let fast_refresh = stack_values.r1 != 0;
        let blocking_mode = RefreshBlockMode::try_from(stack_values.r2).expect("illegal refresh blocking mode");

        DO_REFRESH.store(true, Ordering::Relaxed);
        FAST_REFRESH.store(fast_refresh, Ordering::Relaxed);
        cortex_m::asm::sev();

        if matches!(blocking_mode, RefreshBlockMode::BlockAcknowledge | RefreshBlockMode::BlockFinish) {
            while DO_REFRESH.load(Ordering::Relaxed) {}
        }

        if matches!(blocking_mode, RefreshBlockMode::BlockFinish) {
            while REFRESHING.load(Ordering::Relaxed) {}
        }
    }
}

mod input {
    use core::sync::atomic::Ordering;
    use eepy_sys::input::{Event, InputSyscall};
    use eepy_sys::SafeOption;
    use crate::{EVENT_QUEUE, TOUCH_ENABLED};
    use super::StackFrame;

    pub(super) fn handle_input(stack_values: &mut StackFrame) {
        match InputSyscall::try_from(stack_values.r0) {
            Ok(InputSyscall::NextEvent) => handle_next_event(stack_values),
            Ok(InputSyscall::SetTouchEnabled) => handle_set_touch_enabled(stack_values),
            Ok(InputSyscall::HasEvent) => handle_has_event(stack_values),
            _ => panic!("illegal syscall"),
        }
    }

    fn handle_next_event(stack_values: &mut StackFrame) {
        let next_event = critical_section::with(|cs| EVENT_QUEUE.borrow_ref_mut(cs).pop());
        let ptr = stack_values.r1 as *mut SafeOption<Event>;
        unsafe { ptr.write(next_event.into()) };
    }

    fn handle_set_touch_enabled(stack_values: &mut StackFrame) {
        let enable = stack_values.r1 != 0;
        TOUCH_ENABLED.store(enable, Ordering::Relaxed);
        if !enable {
            critical_section::with(|cs| EVENT_QUEUE.borrow_ref_mut(cs).clear());
        }
    }

    fn handle_has_event(stack_values: &mut StackFrame) {
        let empty = critical_section::with(|cs| EVENT_QUEUE.borrow_ref(cs).is_empty());
        stack_values.r0 = (!empty) as usize;
    }
}