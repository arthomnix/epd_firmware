use core::sync::atomic::Ordering;
use eepy_sys::critical_section::CsSyscall;
use fw16_epd_bsp::pac;
use fw16_epd_bsp::pac::interrupt;
use crate::exception::StackFrame;

pub(super) fn handle_cs(stack_values: &mut StackFrame) {
    match CsSyscall::from_repr(stack_values.r0) {
        Some(CsSyscall::Acquire) => handle_acquire(stack_values),
        Some(CsSyscall::Release) => handle_release(stack_values),
        None => panic!("illegal syscall"),
    }
}

// USBCTRL_IRQ is the only interrupt that might cause anything to happen
// with program memory

fn handle_acquire(stack_values: &mut StackFrame) {
    core::sync::atomic::compiler_fence(Ordering::SeqCst);
    stack_values.r0 = pac::NVIC::is_enabled(interrupt::USBCTRL_IRQ) as usize;
    pac::NVIC::mask(interrupt::USBCTRL_IRQ);
    core::sync::atomic::compiler_fence(Ordering::SeqCst);
}

fn handle_release(stack_values: &mut StackFrame) {
    core::sync::atomic::compiler_fence(Ordering::SeqCst);
    if stack_values.r1 != 0 {
        unsafe {
            pac::NVIC::unmask(interrupt::USBCTRL_IRQ);
        }
    }
    core::sync::atomic::compiler_fence(Ordering::SeqCst);
}