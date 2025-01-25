use eepy_sys::header::*;

pub(crate) const unsafe fn slot(id: u8) -> *const ProgramSlotHeader {
    if id < 1 || id > 31 {
        panic!("slot ID must be between 1 and 31");
    }

    XIP_BASE.add(SLOT_SIZE * id as usize).cast::<ProgramSlotHeader>()
}


pub(crate) struct Programs {
    id: u8,
}

impl Programs {
    pub(crate) fn new() -> Self {
        Self { id: 0 }
    }
}

impl Iterator for Programs {
    type Item = *const ProgramSlotHeader;

    fn next(&mut self) -> Option<Self::Item> {
        return None;
        /*
        loop {
            self.id += 1;

            if self.id == 32 {
                return None;
            }

            let s = unsafe { slot(self.id) };

            if s.is_valid_program() {
                debug!("Found program {} version {} in slot {}", s.name().unwrap(), s.version_string().unwrap(), self.id);
                return Some(s);
            } else {
                debug!("No program found in slot {}", self.id);
            }

        }*/
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (0, Some(32 - self.id as usize))
    }
}