use core::marker::PhantomData;
use core::str::Utf8Error;
use defmt::debug;
use fw16_epd_program_interface::ProgramFunctionTable;

#[link_section = ".prog1"]
#[used]
static PROG1: [u32; 11] = [
    0, // block_erase_cycles
    1419452448, // crc
    44, // len
    4, // name_len
    32, // name_offset
    5, // version_len
    36, // version_offset
    0, // entry_offset
    0x74736554,
    0x2E302E30,
    0x00000032,
];

const XIP_BASE: *const u8 = 0x10000000 as *const u8;

const SLOT_SIZE: usize = 0x80000;

pub(crate) const unsafe fn slot<'a>(id: u8) -> &'a ProgramSlotHeader {
    if id < 1 || id > 31 {
        panic!("slot ID must be between 1 and 31");
    }

    &*XIP_BASE.add(SLOT_SIZE * id as usize).cast::<ProgramSlotHeader>()
}

#[repr(C)]
pub(crate) struct ProgramSlotHeader {
    pub(crate) block_erase_cycles: usize,
    pub(crate) crc: u32,
    pub(crate) len: usize,
    pub(crate) name_len: usize,
    pub(crate) name_offset: usize,
    pub(crate) version_len: usize,
    pub(crate) version_offset: usize,
    pub(crate) entry_offset: usize,
}

impl ProgramSlotHeader {
    fn ptr(&self) -> *const u8 {
        (&raw const *self).cast()
    }

    pub(crate) fn is_valid_program(&self) -> bool {
        self.len != 0
            && self.check_crc()
            && self.name_offset.saturating_add(self.name_len) <= self.len
            && self.version_offset.saturating_add(self.version_len) <= self.len
            && self.entry_offset < self.len
            && self.name().is_ok()
            && self.version_string().is_ok()
    }

    pub(crate) fn check_crc(&self) -> bool {
        if self.len >= SLOT_SIZE || self.len < size_of::<ProgramSlotHeader>() {
            return false;
        }

        let ptr = unsafe { self.ptr().add(8) };
        let slice = unsafe { core::slice::from_raw_parts(ptr, self.len - 8) };
        let crc = crc32fast::hash(slice);
        crc == self.crc
    }

    pub(crate) fn name(&self) -> Result<&str, Utf8Error> {
        let ptr = unsafe { self.ptr().add(self.name_offset) };
        let slice = unsafe { core::slice::from_raw_parts(ptr, self.name_len) };
        core::str::from_utf8(slice)
    }

    pub(crate) fn version_string(&self) -> Result<&str, Utf8Error> {
        let ptr = unsafe { self.ptr().add(self.version_offset) };
        let slice = unsafe { core::slice::from_raw_parts(ptr, self.version_len) };
        core::str::from_utf8(slice)
    }

    pub(crate) unsafe fn entry(&self) -> unsafe extern "C" fn(&ProgramFunctionTable) -> () {
        let ptr = self.ptr().add(self.entry_offset);
        core::mem::transmute(ptr)
    }
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

        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (0, Some(32 - self.id as usize))
    }
}