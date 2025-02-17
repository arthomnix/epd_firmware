pub mod image;
pub mod input;
pub mod program_upload;

use std::fmt::{Display, Formatter};
use std::io;
use std::io::{Read, Write};
use std::marker::PhantomData;
use std::time::Duration;
use serialport::SerialPort;
use eepy_serial::{Response, SerialCommand, SerialError};

#[derive(Debug)]
pub enum Error {
    InvalidResponse,
    DeserializeFailed(postcard::Error),
    Serial(SerialError),
    Io(io::Error),
}

impl From<SerialError> for Error {
    fn from(value: SerialError) -> Self {
        Self::Serial(value)
    }
}

impl From<io::Error> for Error {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

impl From<postcard::Error> for Error {
    fn from(value: postcard::Error) -> Self {
        Self::DeserializeFailed(value)
    }
}

impl Display for Error {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidResponse => write!(f, "received invalid response"),
            Self::DeserializeFailed(e) => e.fmt(f),
            Self::Serial(e) => e.fmt(f),
            Self::Io(e) => e.fmt(f),
        }
    }
}

impl std::error::Error for Error {}

pub trait SerialState {}
pub trait CanEnter<S: SerialState>: SerialState {}

pub enum Unknown {}
impl SerialState for Unknown {}
impl CanEnter<Normal> for Unknown {}
impl CanEnter<HostApp> for Unknown {}

pub enum Normal {}
impl SerialState for Normal {}
impl CanEnter<HostApp> for Normal {}

pub enum HostApp {}
impl SerialState for HostApp {}
impl CanEnter<Normal> for HostApp {}

pub struct Serial<S: SerialState> {
    port: Box<dyn SerialPort>,
    _marker: PhantomData<S>,
}

impl<T: SerialState> Serial<T> {
    pub(crate) fn write(&mut self, command: SerialCommand, data: &[u8]) -> Result<(), Error> {
        self.port.write_all(&[command as u8])?;
        self.port.write_all(data)?;
        let mut response_buf = [0u8];
        self.port.read_exact(&mut response_buf)?;
        Response::from_repr(response_buf[0])
            .map(|resp| Result::<(), SerialError>::from(resp).map_err(|e| Error::Serial(e)))
            .ok_or(Error::InvalidResponse)?
    }
}

impl Serial<Unknown> {
    pub fn new(port: &str) -> Result<Self, serialport::Error> {
        let port = serialport::new(port, 0)
            .timeout(Duration::from_secs(60))
            .open()?;

        Ok(Self {
            port,
            _marker: PhantomData::default(),
        })
    }
}

impl<T: CanEnter<Normal>> Serial<T> {
    pub fn normal(mut self) -> Result<Serial<Normal>, Error> {
        let res = self.write(SerialCommand::ExitHostApp, &[]);
        match res {
            Ok(()) | Err(Error::Serial(SerialError::IncorrectMode)) => {
                Ok(Serial {
                    port: self.port,
                    _marker: PhantomData::default(),
                })
            },
            Err(e) => Err(e),
        }
    }
}

impl<T: CanEnter<HostApp>> Serial<T> {
    pub fn host_app(mut self) -> Result<Serial<HostApp>, Error> {
        let res = self.write(SerialCommand::EnterHostApp, &[]);
        match res {
            Ok(()) | Err(Error::Serial(SerialError::IncorrectMode)) => {
                Ok(Serial {
                    port: self.port,
                    _marker: PhantomData::default(),
                })
            },
            Err(e) => Err(e),
        }
    }
}