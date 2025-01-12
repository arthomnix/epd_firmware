#![no_std]

use core::fmt::{Display, Formatter};
pub use tp370pgh01::IMAGE_BYTES;

/// Option type with stable ABI.
#[repr(C)]
pub enum SafeOption<T> {
    None,
    Some(T),
}

impl<T> From<Option<T>> for SafeOption<T> {
    fn from(value: Option<T>) -> Self {
        match value {
            None => SafeOption::None,
            Some(v) => SafeOption::Some(v),
        }
    }
}

impl<T> From<SafeOption<T>> for Option<T> {
    fn from(value: SafeOption<T>) -> Self {
        match value {
            SafeOption::None => None,
            SafeOption::Some(v) => Some(v),
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct ProgramFunctionTable {
    pub write_image: extern "C" fn(&[u8; IMAGE_BYTES]),
    pub refresh: extern "C" fn(bool, bool),
    pub next_touch_event: extern "C" fn() -> SafeOption<TouchEvent>,
    pub set_touch_enabled: unsafe extern "C" fn(bool),
    pub serial_number: extern "C" fn() -> &'static [u8; 16]
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, defmt::Format)]
pub enum TouchEventType {
    Down,
    Up,
    Move,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, defmt::Format)]
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