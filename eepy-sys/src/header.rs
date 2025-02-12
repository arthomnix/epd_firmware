use core::str::Utf8Error;

#[cfg(feature = "defmt")]
use defmt::{warn, debug};

pub const XIP_BASE: *const u8 = 0x10000000 as *const u8;
pub const PROGRAM_RAM_AREA_BASE: *mut u8 = 0x20020000 as *mut u8;

pub const SLOT_SIZE: usize = 0x80000;

pub const unsafe fn slot_ptr(id: u8) -> *const u8 {
    if id > 31 {
        panic!("slot ID must be between 0 and 31");
    }

    if id == 0 {
        // "Slot 0" is used for the launcher, which is stored 128K into the flash
        XIP_BASE.add(128 * 1024)
    } else {
        XIP_BASE.add(SLOT_SIZE * id as usize)
    }
}

pub const unsafe fn slot(id: u8) -> *const ProgramSlotHeader {
    slot_ptr(id).cast()
}


pub struct Programs {
    id: u8,
}

impl Programs {
    pub fn new() -> Self {
        Self { id: 0 }
    }
}

impl Iterator for Programs {
    type Item = *const ProgramSlotHeader;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            self.id += 1;

            if self.id > 31 {
                return None;
            }

            let s = unsafe { slot(self.id) };

            unsafe {
                if (*s).is_valid() {
                    #[cfg(feature = "defmt")]
                    debug!("Found program {} version {} in slot {}", (*s).name().unwrap(), (*s).version().unwrap(), self.id);
                    return Some(s);
                } else {
                    #[cfg(feature = "defmt")]
                    debug!("No program found in slot {}", self.id);
                }
            }
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (0, Some(32 - self.id as usize))
    }
}

#[repr(C)]
pub struct ProgramSlotHeader {
    pub block_erase_cycles: usize,
    pub crc: u32,
    pub len: usize,

    pub data_len: usize,
    pub data_lma: *const u8,
    pub data_vma: *mut u8,

    pub bss_len: usize,
    pub bss_vma: *mut u8,

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
            data_vma: core::ptr::null_mut(),
            bss_len: 0,
            bss_vma: core::ptr::null_mut(),
            name_len: name.len(),
            name_ptr: name.as_ptr(),
            version_len: version.len(),
            version_ptr: version.as_ptr(),
            entry,
        }
    }

    pub fn is_valid(&self) -> bool {
        // Erased flash contains all 1s
        if self.len == 0 || self.len == usize::MAX {
            return false;
        }

        if self.len > SLOT_SIZE {
            #[cfg(feature = "defmt")]
            warn!("Program header has invalid size");
            return false;
        }

        if !self.check_crc() {
            #[cfg(feature = "defmt")]
            warn!("Program has invalid CRC");
            return false;
        }

        if unsafe { self.name() }.is_err() {
            #[cfg(feature = "defmt")]
            warn!("Program name is not valid UTF-8");
            return false;
        }

        if unsafe { self.version() }.is_err() {
            #[cfg(feature = "defmt")]
            warn!("Program version string is not valid UTF-8");
            return false;
        }

        let slot_min = (&raw const *self) as usize;
        let slot_max = slot_min + SLOT_SIZE;
        let slot_range = slot_min..slot_max;
        let ram_min = PROGRAM_RAM_AREA_BASE as usize;
        let ram_max = ram_min + 136 * 1024;
        let ram_range = ram_min..ram_max;

        if self.data_len > 0 {
            if !slot_range.contains(&(self.data_lma as usize)) || !slot_range.contains(&(self.data_lma as usize + self.data_len - 1)) {
                #[cfg(feature = "defmt")]
                warn!("Program has invalid data section addresses");
                return false;
            }

            if !ram_range.contains(&(self.data_vma as usize)) || !ram_range.contains(&(self.data_vma as usize + self.data_len - 1)) {
                #[cfg(feature = "defmt")]
                warn!("Program has invalid data section load addresses");
                return false;
            }
        }

        if self.bss_len > 0 {
            if !ram_range.contains(&(self.bss_vma as usize)) || !ram_range.contains(&(self.bss_vma as usize + self.bss_len - 1)) {
                #[cfg(feature = "defmt")]
                warn!("Program has invalid bss section addresses");
                return false;
            }
        }

        true
    }

    pub fn slot(&self) -> u8 {
        (((&raw const *self as usize) - XIP_BASE as usize) / SLOT_SIZE) as u8
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

    pub unsafe fn name(&self) -> Result<&str, Utf8Error> {
        unsafe {
            core::str::from_utf8(core::slice::from_raw_parts(self.name_ptr, self.name_len))
        }
    }

    pub unsafe fn version(&self) -> Result<&str, Utf8Error> {
        unsafe {
            core::str::from_utf8(core::slice::from_raw_parts(self.version_ptr, self.version_len))
        }
    }

    pub unsafe fn load(&self) {
        if self.data_len > 0 {
            core::ptr::copy_nonoverlapping(self.data_lma, self.data_vma, self.data_len);
        }

        if self.bss_len > 0 {
            self.bss_vma.write_bytes(0, self.bss_len);
        }
    }
}