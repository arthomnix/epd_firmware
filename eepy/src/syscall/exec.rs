use core::arch::asm;
use eepy_sys::exec::exec;
use eepy_sys::header::slot;
use crate::exception::StackFrame;
use crate::SRAM_END;

pub(super) fn handle_exec(stack_values: &mut StackFrame, using_psp: bool) {
    // cleanup previous program's USB
    crate::usb::handle_uninit();
    crate::usb::handle_clear_handler();

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

        (*program).load();
    }
}

// NOTE: this function runs in unprivileged thread mode
extern "C" fn program_return_handler() -> ! {
    exec(0);
}