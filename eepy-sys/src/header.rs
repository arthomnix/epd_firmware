use core::str::Utf8Error;

pub const XIP_BASE: *const u8 = 0x10000000 as *const u8;
pub const PROGRAM_RAM_AREA_BASE: *mut u8 = 0x20020000 as *mut u8;

pub const SLOT_SIZE: usize = 0x80000;

#[repr(C)]
pub struct ProgramSlotHeader {
    pub block_erase_cycles: usize,
    pub crc: u32,
    pub len: usize,

    pub data_len: usize,
    pub data_lma: *const u8,
    pub data_vma: *const u8,

    pub bss_len: usize,
    pub bss_vma: *const u8,

    pub name_len: usize,
    pub name_ptr: *const u8,

    pub version_len: usize,
    pub version_ptr: *const u8,

    pub entry: unsafe extern "C" fn(),
}

unsafe impl Sync for ProgramSlotHeader {}

impl ProgramSlotHeader {
    pub const fn partial(name: &'static str, version: &'static str, entry: unsafe extern "C" fn()) -> Self {
        Self {
            block_erase_cycles: 0,
            crc: 0,
            len: 0,
            data_len: 0,
            data_lma: core::ptr::null(),
            data_vma: core::ptr::null(),
            bss_len: 0,
            bss_vma: core::ptr::null(),
            name_len: name.len(),
            name_ptr: name.as_ptr(),
            version_len: version.len(),
            version_ptr: version.as_ptr(),
            entry,
        }
    }

    pub fn check_crc(&self) -> bool {
        if self.len >= SLOT_SIZE || self.len < size_of::<ProgramSlotHeader>() {
            return false;
        }

        let ptr = unsafe { (&raw const *self).cast::<u8>().add(8) };
        let slice = unsafe { core::slice::from_raw_parts(ptr, self.len - 8) };
        let crc = crc32fast::hash(slice);
        crc == self.crc
    }

    pub fn name(&self) -> Result<&str, Utf8Error> {
        unsafe {
            core::str::from_utf8(core::slice::from_raw_parts(self.name_ptr, self.name_len))
        }
    }

    pub fn version(&self) -> Result<&str, Utf8Error> {
        unsafe {
            core::str::from_utf8(core::slice::from_raw_parts(self.version_ptr, self.version_len))
        }
    }
}