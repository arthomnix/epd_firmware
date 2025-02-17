#![no_std]

pub use eepy_sys::input_common::Event;

use core::fmt::{Display, Formatter};

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, strum::FromRepr)]
pub enum SerialCommand {
    /// Refresh the screen. Must be followed by exactly 12480 bytes of image data.
    RefreshNormal = 0,
    /// Fast refresh the screen. Must be followed by exactly 12480 bytes of image data.
    RefreshFast = 1,

    MaybeRefreshNormal = 2,
    MaybeRefreshFast = 3,

    /// Enter Host App mode. In this mode, events will not be processed by the builtin UI and can
    /// instead be retrieved by host programs using the [SerialCommand::NextEvent] command. This
    /// mode should be used by all host programs writing images to the display.
    EnterHostApp = 4,
    /// Exit Host App mode (see [SerialCommand::EnterHostApp]).
    ExitHostApp = 5,

    /// Get the next event. Only works in Host App mode.
    NextEvent = 6,

    /// Disable touch. Touch events will no longer be added to the event queue. Only works in Host
    /// App mode.
    DisableTouch = 7,
    /// Enable touch.
    EnableTouch = 8,

    /// Get the program slot that will be used to store the next uploaded program. This command is
    /// only available when not in Host App mode.
    GetProgramSlot = 9,
    /// Upload a program. The program will be stored in the slot returned by the previous
    /// GetProgramSlot call. The program uploaded must be linked correctly for the
    /// slot. Only available when not in Host App mode.
    UploadProgram = 10,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, strum::FromRepr)]
pub enum Response {
    UnknownCommand = 0x00,
    IncorrectMode = 0x01,
    ProgramSlotsFull = 0x02,
    NoProgramSlot = 0x03,
    Ack = 0xff,
}

impl From<Response> for Result<(), SerialError> {
    fn from(value: Response) -> Self {
        match value {
            Response::UnknownCommand => Err(SerialError::UnknownCommand),
            Response::IncorrectMode => Err(SerialError::IncorrectMode),
            Response::ProgramSlotsFull => Err(SerialError::ProgramSlotsFull),
            Response::NoProgramSlot => Err(SerialError::NoProgramSlot),
            Response::Ack => Ok(()),
        }
    }
}

impl Display for Response {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Response::UnknownCommand => write!(f, "Invalid command"),
            Response::IncorrectMode => write!(f, "Incorrect mode"),
            Response::ProgramSlotsFull => write!(f, "No program slot is available"),
            Response::NoProgramSlot => write!(f, "Call GetProgramSlot before UploadProgram"),
            Response::Ack => write!(f, "Success"),
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SerialError {
    UnknownCommand,
    IncorrectMode,
    ProgramSlotsFull,
    NoProgramSlot,
}

impl Display for SerialError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            SerialError::UnknownCommand => write!(f, "Invalid command"),
            SerialError::IncorrectMode => write!(f, "Incorrect mode"),
            SerialError::ProgramSlotsFull => write!(f, "No program slot is available"),
            SerialError::NoProgramSlot => write!(f, "Call GetProgramSlot before UploadProgram"),
        }
    }
}

impl core::error::Error for SerialError {}