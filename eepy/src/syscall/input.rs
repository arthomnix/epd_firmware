use core::sync::atomic::Ordering;
use eepy_sys::input::InputSyscall;
use eepy_sys::input_common::Event;
use eepy_sys::SafeOption;
use crate::{EVENT_QUEUE, TOUCH_ENABLED};
use super::StackFrame;

pub(super) fn handle_input(stack_values: &mut StackFrame) {
    match InputSyscall::from_repr(stack_values.r0) {
        Some(InputSyscall::NextEvent) => handle_next_event(stack_values),
        Some(InputSyscall::SetTouchEnabled) => handle_set_touch_enabled(stack_values),
        Some(InputSyscall::HasEvent) => handle_has_event(stack_values),
        None => panic!("illegal syscall"),
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