use eepy_serial::{Event, SerialCommand};
use crate::{Error, HostApp, Serial};

impl Serial<HostApp> {
    pub fn set_touch_enabled(&mut self, enabled: bool) -> Result<(), Error> {
        let cmd = if enabled { SerialCommand::EnableTouch } else { SerialCommand::DisableTouch };
        self.write(cmd, &[])
    }

    pub fn next_event(&mut self) -> Result<Option<Event>, Error> {
        self.write(SerialCommand::NextEvent, &[])?;
        let mut event_buf = [0u8; 32];
        self.port.read(&mut event_buf)?;
        Ok(postcard::from_bytes(&event_buf)?)
    }
}