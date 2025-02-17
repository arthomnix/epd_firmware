use std::io::Read;
use eepy_serial::SerialCommand;
use crate::{Error, Normal, Serial};

impl Serial<Normal> {
    pub fn get_slot(&mut self) -> Result<u8, Error> {
        self.write(SerialCommand::GetProgramSlot, &[])?;
        let mut slot_n = [0u8];
        self.port.read_exact(&mut slot_n)?;
        Ok(slot_n[0])
    }

    pub fn upload(&mut self, program: &[u8]) -> Result<(), Error> {
        self.write(SerialCommand::UploadProgram, &program)
    }
}