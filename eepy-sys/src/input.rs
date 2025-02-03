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