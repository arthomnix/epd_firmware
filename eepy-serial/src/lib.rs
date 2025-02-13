#![no_std]

pub use eepy_sys::input_common::Event;

use core::fmt::{Display, Formatter};

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SerialCommand {
    /// Refresh the screen. Must be followed by exactly 12480 bytes of image data.
    RefreshNormal = 0,
    /// Fast refresh the screen. Must be followed by exactly 12480 bytes of image data.
    RefreshFast = 1,

    /// Enter Host App mode. In this mode, events will not be processed by the builtin UI and can
    /// instead be retrieved by host programs using the [SerialCommand::NextEvent] command. This
    /// mode should be used by all host programs writing images to the display.
    EnterHostApp = 2,
    /// Exit Host App mode (see [SerialCommand::EnterHostApp]).
    ExitHostApp = 3,

    /// Get the next event. Only works in Host App mode.
    NextEvent = 4,

    /// Disable touch. Touch events will no longer be added to the event queue. Only works in Host
    /// App mode.
    DisableTouch = 5,
    /// Enable touch.
    EnableTouch = 6,

    /// Get the program slot that will be used to store the next uploaded program. This command is
    /// only available when not in Host App mode.
    GetProgramSlot = 7,
    /// Upload a program. The program will be stored in the slot returned by the previous
    /// GetProgramSlot call. The program uploaded must be linked correctly for the
    /// slot. Only available when not in Host App mode.
    UploadProgram = 8,
}

impl TryFrom<u8> for SerialCommand {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            x if x == SerialCommand::RefreshNormal as u8 => Ok(SerialCommand::RefreshNormal),
            x if x == SerialCommand::RefreshFast as u8 => Ok(SerialCommand::RefreshFast),
            x if x == SerialCommand::EnterHostApp as u8 => Ok(SerialCommand::EnterHostApp),
            x if x == SerialCommand::ExitHostApp as u8 => Ok(SerialCommand::ExitHostApp),
            x if x == SerialCommand::NextEvent as u8 => Ok(SerialCommand::NextEvent),
            x if x == SerialCommand::DisableTouch as u8 => Ok(SerialCommand::DisableTouch),
            x if x == SerialCommand::EnableTouch as u8 => Ok(SerialCommand::EnableTouch),
            x if x == SerialCommand::GetProgramSlot as u8 => Ok(SerialCommand::GetProgramSlot),
            x if x == SerialCommand::UploadProgram as u8 => Ok(SerialCommand::UploadProgram),

            _ => Err(()),
        }
    }
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
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

impl TryFrom<u8> for Response {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            x if x == Response::UnknownCommand as u8 => Ok(Response::UnknownCommand),
            x if x == Response::IncorrectMode as u8 => Ok(Response::IncorrectMode),
            x if x == Response::ProgramSlotsFull as u8 => Ok(Response::ProgramSlotsFull),
            x if x == Response::NoProgramSlot as u8 => Ok(Response::NoProgramSlot),
            x if x == Response::Ack as u8 => Ok(Response::Ack),

            _ => Err(()),
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