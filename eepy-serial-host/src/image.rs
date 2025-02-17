use eepy_serial::SerialCommand;
use crate::{Error, HostApp, Serial};

pub const IMAGE_BYTES: usize = (240 * 416) / 8;

impl Serial<HostApp> {
    pub fn refresh(&mut self, fast: bool, image: &[u8; IMAGE_BYTES]) -> Result<(), Error> {
        let cmd = if fast { SerialCommand::RefreshFast } else { SerialCommand::RefreshNormal };
        self.write(cmd, image)
    }

    pub fn maybe_refresh(&mut self, fast: bool, image: &[u8; IMAGE_BYTES]) -> Result<(), Error> {
        let cmd = if fast { SerialCommand::MaybeRefreshFast } else { SerialCommand::MaybeRefreshNormal };
        self.write(cmd, image)
    }
}