use core::arch::{asm, global_asm};
use defmt::trace;
use eepy_sys::misc::get_serial;
use eepy_sys::usb::UsbSyscall;
use fw16_epd_bsp::pac;
use fw16_epd_bsp::pac::interrupt;
use crate::exception::StackFrame;

global_asm!(include_str!("usb.s"));

extern "C" {
    fn usb_ret();
}

extern "C" fn example_usb_handler() {
    // Do a syscall so it shows up in the kernel logs
    // (we should be in unprivileged thread mode here so we can't use defmt)
    core::hint::black_box(get_serial());
}

#[no_mangle]
extern "C" fn handle_usb_irq(sp: *mut StackFrame, using_psp: bool) {
    trace!("fake USBCTRL_IRQ");

    if !using_psp {
        return;
    }

    unsafe {
        let new_sp = sp.sub(1);
        new_sp.write(StackFrame {
            r0: 0,
            r1: 0,
            r2: 0,
            r3: 0,
            r12: 0,
            lr: usb_ret as *const u8,
            pc: example_usb_handler as *const u8,
            xpsr: 0x1000000,
        });

        asm!(
            "msr psp, {new_psp}",
            new_psp = in(reg) new_sp,
        );
    }
}

pub(crate) fn handle_usb(stack_values: &mut StackFrame) {
    match UsbSyscall::try_from(stack_values.r0) {
        Ok(UsbSyscall::UsbRet) => handle_usb_ret(stack_values),
        Err(_) => {},
    }
}

fn handle_usb_ret(stack_values: &mut StackFrame) {
    trace!("usb_ret");
    unsafe {
        let new_sp = (&raw mut *stack_values).add(1);
        asm!(
            "msr psp, {new_psp}",
            new_psp = in(reg) new_sp,
        );
    }
}