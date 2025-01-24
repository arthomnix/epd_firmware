#![no_std]

pub mod header;
pub mod syscall;

use core::fmt::{Display, Formatter};
pub use tp370pgh01::IMAGE_BYTES;

/// Option type with stable ABI.
#[repr(C)]
#[derive(Debug, serde::Serialize, serde::Deserialize)]
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
pub enum RefreshBlockMode {
    NonBlocking,
    BlockAcknowledge,
    BlockFinish,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct ProgramFunctionTable {
    pub write_image: extern "C" fn(&[u8; IMAGE_BYTES]),
    pub refresh: extern "C" fn(bool, RefreshBlockMode),
    pub next_event: extern "C" fn() -> SafeOption<Event>,
    pub set_touch_enabled: unsafe extern "C" fn(bool),
    pub serial_number: extern "C" fn() -> &'static [u8; 16]
}

impl ProgramFunctionTable {
    pub fn serial_number_str(&self) -> &'static str {
        unsafe { core::str::from_utf8_unchecked((self.serial_number)()) }
    }
}

#[repr(C)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Event {
    Null,
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