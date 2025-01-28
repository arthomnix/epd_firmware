use core::fmt::{Display, Formatter};
use core::mem::MaybeUninit;
use crate::{syscall, SafeOption};
use crate::syscall::SyscallNumber;

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InputSyscall {
    NextEvent = 0,
    SetTouchEnabled = 1,
    HasEvent = 2,
}

impl TryFrom<usize> for InputSyscall {
    type Error = ();

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            x if x == InputSyscall::NextEvent as usize => Ok(InputSyscall::NextEvent),
            x if x == InputSyscall::SetTouchEnabled as usize => Ok(InputSyscall::SetTouchEnabled),
            x if x == InputSyscall::HasEvent as usize => Ok(InputSyscall::HasEvent),
            _ => Err(()),
        }
    }
}

#[repr(C)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Event {
    Touch(TouchEvent),
    RefreshFinished,
}

#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum TouchEventType {
    Down,
    Up,
    Move,
}

#[repr(C)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TouchEvent {
    pub ev_type: TouchEventType,
    pub x: u16,
    pub y: u16,
}

impl TouchEvent {
    pub const fn new() -> Self {
        Self {
            ev_type: TouchEventType::Down,
            x: u16::MAX,
            y: u16::MAX,
        }
    }

    #[cfg(feature = "embedded-graphics")]
    pub fn eg_point(&self) -> embedded_graphics::prelude::Point {
        embedded_graphics::prelude::Point::new(self.x as i32, self.y as i32)
    }
}

impl Display for TouchEvent {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        let ty = match self.ev_type {
            TouchEventType::Down => "Down",
            TouchEventType::Up => "Up",
            TouchEventType::Move => "Move",
        };
        write!(f, "{ty} @ ({}, {})", self.x, self.y)
    }
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