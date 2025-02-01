use core::fmt::{Display, Formatter};

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