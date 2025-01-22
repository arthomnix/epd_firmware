use core::sync::atomic::Ordering;
use crate::{interrupt, FLASHING, FLASHING_ACK};
use defmt::{debug, trace};
use usbd_serial::SerialPort;
use fw16_epd_bsp::hal::usb::UsbBus;
use fw16_epd_bsp::pac;
use fw16_epd_program_interface::header::ProgramSlotHeader;
use fw16_epd_program_interface::RefreshBlockMode;
use fw16_epd_serial_common::{Response, SerialCommand};
use tp370pgh01::IMAGE_BYTES;
use crate::{refresh, write_image, GLOBAL_USB_DEVICE, GLOBAL_USB_SERIAL};
use crate::programs::Programs;

#[derive(Copy, Clone, Debug, defmt::Format)]
enum SerialState {
    ReadyForCommand,

    ReceivingImage {
        fast_refresh: bool,
        index: usize,
    },

    FlashingProgram {
        index: usize,
        page: usize,
        num_pages: Option<usize>,
        remainder: Option<usize>,
    },
}

fn write_all(serial: &mut SerialPort<UsbBus>, mut buf: &[u8]) {
    while !buf.is_empty() {
        let _ = serial.write(buf).map(|len| buf = &buf[len..]);
    }
}

/// Safety:
///
/// This function takes care of the main safety requirements of flashing, but the
/// caller must ensure that the `slot` and `page` parameters are valid and do
/// not produce an address outside the flash's range. Additionally, do not write
/// to slot 0 as this contains the firmware.
unsafe fn write_flash(buf: &[u8], slot: u8, page: usize) {
    debug!("Begin write slot {} page {}", slot, page);

    // Make sure core1 is running code from RAM with interrupts disabled
    FLASHING.store(true, Ordering::Relaxed);
    cortex_m::asm::sev();
    // Wait until core1 has acknowledged that it is now in RAM code
    while !FLASHING_ACK.load(Ordering::Relaxed) {}
    // Disable interrupts on this core
    cortex_m::interrupt::disable();

    unsafe {
        rp2040_flash::flash::flash_range_erase_and_program(
            (slot as u32) * 512 * 1024 + (page as u32) * 4096,
            buf,
            true
        );
    }

    // Enable interrupts
    unsafe { cortex_m::interrupt::enable() }
    // Wake up core1
    FLASHING.store(false, Ordering::Relaxed);
    cortex_m::asm::sev();

    debug!("End write slot {} page {}", slot, page);
}

#[interrupt]
fn USBCTRL_IRQ() {
    static mut STATE: SerialState = SerialState::ReadyForCommand;

    // Receive buffer. Size equal to IMAGE_BYTES so it can store an entire frame; also used for
    // receiving flash applications.
    static mut BUF: [u8; IMAGE_BYTES] = [0; IMAGE_BYTES];

    trace!("USBCTRL_IRQ");

    // Safety: These are only accessed within this interrupt handler, or in main() before the
    // interrupt is enabled.
    #[allow(static_mut_refs)]
    let usb_dev = unsafe { GLOBAL_USB_DEVICE.as_mut().unwrap() };
    #[allow(static_mut_refs)]
    let serial = unsafe { GLOBAL_USB_SERIAL.as_mut().unwrap() };

    if usb_dev.poll(&mut [serial]) {
        match STATE {
            SerialState::ReadyForCommand => {
                let mut cmd_buf = [0u8];
                if let Ok(count) = serial.read(&mut cmd_buf) {
                    if count == 0 {
                        return;
                    }

                    match SerialCommand::try_from(cmd_buf[0]) {
                        Ok(SerialCommand::RefreshNormal) => *STATE = SerialState::ReceivingImage { fast_refresh: false, index: 0 },
                        Ok(SerialCommand::RefreshFast) => *STATE = SerialState::ReceivingImage { fast_refresh: true, index: 0 },
                        Ok(SerialCommand::UploadProgram) => *STATE = SerialState::FlashingProgram { index: 0, page: 0, num_pages: None, remainder: None },
                        Ok(_) => write_all(serial, &[Response::UnknownCommand as u8]),
                        Err(_) => write_all(serial, &[Response::UnknownCommand as u8]),
                    }
                }
            }

            SerialState::ReceivingImage { fast_refresh, index } => {
                if let Ok(count) = serial.read(&mut BUF[*index..]) {
                    *index += count;
                    if *index == IMAGE_BYTES {
                        write_image(BUF);
                        refresh(*fast_refresh, RefreshBlockMode::NonBlocking);
                        write_all(serial, &[Response::Ack as u8]);
                        *STATE = SerialState::ReadyForCommand;
                    }
                }
            }

            SerialState::FlashingProgram { index, page, num_pages, remainder } => {
                debug!("Flashing program - page {}", *page);
                // Write page 0 last - this is the header, so we only want to write it once everything
                // else is written successfully
                // Keep page 0 in the first 4096 bytes of BUF for the end
                debug!("{} {} {} {}", index, page, num_pages, remainder);
                if *page == 0 {
                    if let Ok(count) = serial.read(&mut BUF[*index..4096]) {
                        *index += count;

                        if num_pages.is_none() && *index >= 12 {
                            let mut b = [0u8; 4];
                            b.copy_from_slice(&BUF[8..12]);
                            let num_bytes = usize::from_le_bytes(b);
                            debug!("Program is {} bytes ({} pages) long", num_bytes, num_bytes.div_ceil(4096));
                            *num_pages = Some(num_bytes.div_ceil(4096));
                            *remainder = Some(num_bytes % 4096);
                        }

                        if *index == 4096 {
                            *index = 0;
                            *page += 1;
                        }
                    }
                } else {
                    if let Ok(count) = serial.read(&mut BUF[(4096 + *index)..8192]) {
                        *index += count;

                        let num_pages = num_pages.unwrap();
                        let remainder = remainder.unwrap();
                        if *index == 4096 || (*page == num_pages - 1 && *index == remainder) {
                            *index = 0;

                            // Actually write the flash page
                            // TODO: get next slot instead of always using slot 1
                            // TODO: wear levelling
                            unsafe { write_flash(&BUF[4096..8192], 1, *page) };

                            *page += 1;
                            // If this is the last page, also flash the first page which we didn't
                            // do at the start
                            if *page == num_pages {
                                unsafe { write_flash(&BUF[0..4096], 1, 0) };

                                // Invalidate the XIP cache, in case something from the flash area
                                // we just wrote is in there
                                unsafe {
                                    // FIXME: steal
                                    let xip = pac::Peripherals::steal().XIP_CTRL;
                                    xip.flush().write(|w| w.flush().set_bit());
                                    xip.flush().read();
                                };

                                let program = unsafe { &*(0x10080000 as *const ProgramSlotHeader) };
                                debug!("{} {}", program.name().unwrap(), program.version().unwrap());

                                *STATE = SerialState::ReadyForCommand;
                            }
                        }
                    }
                }
            }
        }
    }
}