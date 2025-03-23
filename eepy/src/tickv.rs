use once_cell::unsync::OnceCell;
use defmt::trace;
use siphasher::sip::SipHasher;
use tickv::{ErrorCode, FlashController, TicKV, MAIN_KEY};

const TICKV_START_XIP: *const u8 = 0x10040000 as *const u8;
const TICKV_START_FLASH: usize = 0x40000;
const TICKV_SIZE: usize = 256 * 1024;

pub(crate) struct EepyFlashController;

impl FlashController<4096> for EepyFlashController {
    fn read_region(&self, region_number: usize, buf: &mut [u8; 4096]) -> Result<(), ErrorCode> {
        trace!("kv read_region");

        unsafe {
            let start_addr = TICKV_START_XIP.add(region_number * 4096);
            let slice = core::slice::from_raw_parts(start_addr, 4096);
            buf.copy_from_slice(slice);
            Ok(())
        }
    }

    fn write(&self, mut address: usize, mut buf: &[u8]) -> Result<(), ErrorCode> {
        trace!("kv write");

        // If the address is unaligned, read-modify-write the first part
        if address % 256 != 0 {
            let write_len = usize::min(buf.len(), 256 - (address % 256));
            let block_start = address & !(256 - 1);

            let mut write_buf = [0u8; 256];
            unsafe {
                write_buf.copy_from_slice(core::slice::from_raw_parts(TICKV_START_XIP.add(block_start), 256));
            }
            write_buf[address % 256..address % 256 + write_len].copy_from_slice(&buf[..write_len]);

            unsafe {
                crate::flash::program((TICKV_START_FLASH + block_start) as u32, &write_buf);
            }

            buf = &buf[write_len..];
            address += write_len;
        }

        let blocks = buf.len() / 256;
        if blocks > 0 {
            unsafe {
                crate::flash::program((TICKV_START_FLASH + address) as u32, &buf[..blocks * 256]);
            }
        }
        address += blocks * 256;
        buf = &buf[blocks * 256..];

        if buf.len() > 0 {
            let mut write_buf = [0u8; 256];
            write_buf[..buf.len()].copy_from_slice(buf);
            unsafe {
                write_buf[buf.len()..].copy_from_slice(core::slice::from_raw_parts(TICKV_START_XIP.add(address), 256 - buf.len()));
            }

            unsafe {
                crate::flash::program((TICKV_START_FLASH + address) as u32, &write_buf);
            }
        }

        Ok(())
    }

    fn erase_region(&self, region_number: usize) -> Result<(), ErrorCode> {
        trace!("kv erase_region");

        unsafe {
            crate::flash::erase((TICKV_START_FLASH + region_number * 4096) as u32, 4096);
        }
        Ok(())
    }
}

static mut TICKV_BUFFER: [u8; 4096] = [0u8; 4096];
static mut TICKV: OnceCell<TicKV<EepyFlashController, 4096>> = OnceCell::new();

/// SAFETY: this must only be called once
unsafe fn init_tickv() -> TicKV<'static, EepyFlashController, 4096> {
    // SAFETY: this is only called once, so we can't have aliased mutable references
    #[allow(static_mut_refs)]
    let read_buffer = unsafe { &mut TICKV_BUFFER };

    let tickv = TicKV::new(EepyFlashController, read_buffer, TICKV_SIZE);

    let hasher = SipHasher::new();
    tickv.initialise(hasher.hash(MAIN_KEY)).expect("Failed to initialise TicKV");
    tickv
}

#[allow(static_mut_refs)]
pub(crate) unsafe fn with_tickv<R>(f: impl FnOnce(&TicKV<EepyFlashController, 4096>) -> R) -> R { unsafe {
    f(TICKV.get_or_init(|| init_tickv()))
}}