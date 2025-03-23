use defmt::{trace, warn};
use eepy_sys::header::XIP_BASE;
use fw16_epd_bsp::hal::Sio;
use fw16_epd_bsp::pac;
use crate::core1::{ToCore0Message, ToCore1Message};

fn flash_op(f: impl FnOnce()) {
    let mut fifo = Sio::new(unsafe { pac::Peripherals::steal() }.SIO).fifo;

    fifo.write_blocking(ToCore1Message::FlashWait as u32);
    // Wait until core1 has acknowledged that it is now in RAM code
    while fifo.read_blocking() != ToCore0Message::FlashAck as u32 {
        warn!("received incorrect message");
    }

    trace!("core1 is in safe state, performing flash operation...");
    cortex_m::interrupt::free(|_cs| f());

    // Wake up core1
    fifo.write_blocking(ToCore1Message::FlashFinished as u32);
}

fn flush_cache_range(start_addr: u32, len: u32) {
    // RP2040 Datasheet 2.6.3.2:
    // "A write to the 0x10… mirror will look up the addressed location in the cache, and delete
    // any matching entry found. Writing to all word-aligned locations in an address range (e.g.
    // a flash sector that has just been erased and reprogrammed) therefore eliminates the
    // possibility of stale cached data in this range, without suffering the effects of a
    // complete cache flush."

    unsafe {
        let start_ptr: *mut u32 = XIP_BASE.add(start_addr as usize).cast_mut().cast();
        let words = len / 4;
        for _ in 0..words {
            start_ptr.add(words as usize).write_volatile(0);
        }
    }
}

pub(crate) unsafe fn erase(start_addr: u32, len: u32) { unsafe {
    trace!("flash erasing start {} len {}", start_addr, len);
    flash_op(|| rp2040_flash::flash::flash_range_erase(start_addr, len, true));
    flush_cache_range(start_addr, len);
}}

pub(crate) unsafe fn program(start_addr: u32, data: &[u8]) { unsafe {
    trace!("flash programming start {} len {}", start_addr, data.len());
    flash_op(|| rp2040_flash::flash::flash_range_program(start_addr, data, true));
    flush_cache_range(start_addr, data.len() as u32);
}}

pub(crate) unsafe fn erase_and_program(start_addr: u32, data: &[u8]) { unsafe {
    trace!("flash programming and erasing start {} len {}", start_addr, data.len());
    flash_op(|| rp2040_flash::flash::flash_range_erase_and_program(start_addr, data, true));
    flush_cache_range(start_addr, data.len() as u32);
}}