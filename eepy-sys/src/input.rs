use core::arch::asm;
use core::mem::MaybeUninit;
use crate::{syscall, SafeOption};
use crate::input_common::Event;
use crate::syscall::SyscallNumber;

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InputSyscall {
    NextEvent = 0,
    SetTouchEnabled = 1,
    HasEvent = 2,
}

pub fn next_event() -> Option<Event> {
    let mut event: MaybeUninit<SafeOption<Event>> = MaybeUninit::uninit();

    unsafe {
        syscall!(
            SyscallNumber::Input,
            in InputSyscall::NextEvent,
            in event.as_mut_ptr(),
        );

        event.assume_init().into()
    }
}

pub fn set_touch_enabled(enabled: bool) {
    unsafe {
        syscall!(
            SyscallNumber::Input,
            in InputSyscall::SetTouchEnabled,
            in enabled,
        );
    }
}

pub fn has_event() -> bool {
    let mut has_event: usize;

    unsafe {
        syscall!(
            SyscallNumber::Input,
            out has_event in InputSyscall::HasEvent,
        );
    }

    has_event != 0
}

/// If there are no events remaining in the event queue, allow the processor to enter a sleep state
/// until there is a new event ready.
#[inline(always)]
pub fn eep() {
    if !has_event() {
        // SAFETY: "wfe" only (maybe) puts the processor to sleep temporarily and doesn't access any
        // memory so this specific instruction is safe
        // has_event() is a syscall. The SVCall exception is a WFE wakeup event, so we need two
        // WFEs so we don't immediately wake up.
        unsafe { asm!("wfe", "wfe"); }
    }
}