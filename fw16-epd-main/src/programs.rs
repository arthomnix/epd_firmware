use core::marker::PhantomData;
use fw16_epd_program_interface::header::*;

pub(crate) const unsafe fn slot<'a>(id: u8) -> &'a ProgramSlotHeader {
    if id < 1 || id > 31 {
        panic!("slot ID must be between 1 and 31");
    }

    &*XIP_BASE.add(SLOT_SIZE * id as usize).cast::<ProgramSlotHeader>()
}


pub(crate) struct Programs<'a> {
    id: u8,
    _phantom: PhantomData<&'a ProgramSlotHeader>,
}

impl<'a> Programs<'a> {
    pub(crate) fn new() -> Self {
        Self { id: 0, _phantom: PhantomData }
    }
}

impl<'a> Iterator for Programs<'a> {
    type Item = &'a ProgramSlotHeader;

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