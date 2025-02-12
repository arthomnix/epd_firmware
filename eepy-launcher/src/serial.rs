use core::sync::atomic::Ordering;
use core::fmt::Write;
use portable_atomic::{AtomicBool, AtomicU8};
use usb_device::device::UsbDevice;
use usbd_serial::SerialPort;
use eepy_serial::{Response, SerialCommand};
use eepy_sys::flash::erase_and_program;
use eepy_sys::header::{slot, slot_ptr, Programs};
use eepy_sys::image::refresh;
use eepy_sys::{header, IMAGE_BYTES};
use eepy_sys::input::{next_event, set_touch_enabled};
use eepy_sys::misc::{debug, info, trace};
use eepy_sys::usb::UsbBus;
use crate::{delete_program, USB_DEVICE, USB_SERIAL};
use crate::ui::flashing::draw_flashing_ui;

#[derive(Copy, Clone, Debug)]
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

fn erase_cycles(slot: u8) -> u32 {
    let c = unsafe { u32::from_ne_bytes(*slot_ptr(slot).cast()) };
    if c == u32::MAX {
        0
    } else {
        c
    }
}

fn best_slot() -> Option<u8> {
    (1u8..=31)
        .filter(|s| unsafe { !(*slot(*s)).is_valid() })
        .map(|s| (s, erase_cycles(s)))
        .min_by_key(|(_s, e)| *e)
        .map(|(s, _e)| s)
}

unsafe fn write_flash(buf: &[u8], slot: u8, page: usize) {
    erase_and_program((slot as u32) * 512 * 1024 + (page as u32) * 4096, buf);
}

fn write_all(serial: &mut SerialPort<UsbBus>, mut buf: &[u8]) {
    while !buf.is_empty() {
        let _ = serial.write(buf).map(|len| buf = &buf[len..]);
    }
}

pub(crate) static NEEDS_REFRESH: AtomicBool = AtomicBool::new(false);
pub(crate) static NEEDS_REFRESH_PROGRAMS: AtomicBool = AtomicBool::new(false);
pub(crate) static HOST_APP: AtomicBool = AtomicBool::new(false);

static PROG_SLOT: AtomicU8 = AtomicU8::new(0);

pub(crate) extern "C" fn usb_handler() {
    trace("USB handler");

    static mut STATE: SerialState = SerialState::ReadyForCommand;
    #[allow(static_mut_refs)]
    let state = unsafe { &mut STATE };

    #[allow(static_mut_refs)]
    let dev: &mut UsbDevice<UsbBus> = unsafe { USB_DEVICE.as_mut().unwrap() };
    #[allow(static_mut_refs)]
    let serial: &mut SerialPort<UsbBus> = unsafe { USB_SERIAL.as_mut().unwrap() };

    // Receive buffer. Size equal to IMAGE_BYTES so it can store an entire frame; also used for
    // receiving flash applications.
    static mut BUF: [u8; IMAGE_BYTES] = [0; IMAGE_BYTES];
    #[allow(static_mut_refs)]
    let buf = unsafe { &mut BUF };

    if dev.poll(&mut [serial]) {
        let mut s = heapless::String::<100>::new();
        write!(s, "{state:?}").unwrap();
        debug(&s);

        match state {
            SerialState::ReadyForCommand => {
                let mut cmd_buf = [0u8];
                if let Ok(count) = serial.read(&mut cmd_buf) {
                    if count == 0 {
                        return;
                    }

                    if HOST_APP.load(Ordering::Relaxed) {
                        match SerialCommand::try_from(cmd_buf[0]) {
                            Ok(SerialCommand::RefreshNormal) => *state = SerialState::ReceivingImage { fast_refresh: false, index: 0 },
                            Ok(SerialCommand::RefreshFast) => *state = SerialState::ReceivingImage { fast_refresh: true, index: 0 },
                            Ok(SerialCommand::ExitHostApp) => {
                                set_touch_enabled(true);
                                HOST_APP.store(false, Ordering::Relaxed);
                                NEEDS_REFRESH.store(true, Ordering::Relaxed);
                                write_all(serial, &[Response::Ack as u8]);
                            },
                            Ok(SerialCommand::NextEvent) => {
                                write_all(serial, &[Response::Ack as u8]);
                                write_all(serial, &postcard::to_vec::<_, 32>(&next_event()).unwrap());
                            },
                            Ok(SerialCommand::EnableTouch) => {
                                set_touch_enabled(true);
                                write_all(serial, &[Response::Ack as u8]);
                            },
                            Ok(SerialCommand::DisableTouch) => {
                                set_touch_enabled(false);
                                write_all(serial, &[Response::Ack as u8]);
                            },
                            Ok(SerialCommand::EnterHostApp | SerialCommand::GetProgramSlot | SerialCommand::UploadProgram) => {
                                write_all(serial, &[Response::IncorrectMode as u8]);
                            }
                            Err(_) => write_all(serial, &[Response::UnknownCommand as u8]),
                        }
                    } else {
                        match SerialCommand::try_from(cmd_buf[0]) {
                            Ok(SerialCommand::GetProgramSlot) => {
                                if let Some(slot) = best_slot() {
                                    write_all(serial, &[Response::Ack as u8, slot]);
                                    PROG_SLOT.store(slot, Ordering::Relaxed);
                                } else {
                                    write_all(serial, &[Response::ProgramSlotsFull as u8]);
                                }
                            },
                            Ok(SerialCommand::UploadProgram) => {
                                if PROG_SLOT.load(Ordering::Relaxed) == 0 {
                                    write_all(serial, &[Response::NoProgramSlot as u8]);
                                } else {
                                    set_touch_enabled(false);
                                    *state = SerialState::FlashingProgram { index: 0, page: 0, num_pages: None, remainder: None };
                                    write_all(serial, &[Response::Ack as u8]);
                                }
                            },
                            Ok(SerialCommand::EnterHostApp) => {
                                HOST_APP.store(true, Ordering::Relaxed);
                                refresh(&[0u8; IMAGE_BYTES], false);
                                set_touch_enabled(false);
                                write_all(serial, &[Response::Ack as u8]);
                            },
                            Ok(
                                SerialCommand::RefreshNormal
                                | SerialCommand::RefreshFast
                                | SerialCommand::ExitHostApp
                                | SerialCommand::NextEvent
                                | SerialCommand::DisableTouch
                                | SerialCommand::EnableTouch
                            ) => write_all(serial, &[Response::IncorrectMode as u8]),
                            Err(_) => write_all(serial, &[Response::UnknownCommand as u8]),
                        }
                    }
                }
            }

            SerialState::ReceivingImage { fast_refresh, index } => {
                if let Ok(count) = serial.read(&mut buf[*index..]) {
                    *index += count;
                    if *index == IMAGE_BYTES {
                        refresh(buf, *fast_refresh);
                        write_all(serial, &[Response::Ack as u8]);
                        *state = SerialState::ReadyForCommand;
                    }
                }
            }

            SerialState::FlashingProgram { index, page, num_pages, remainder } => {
                let slot = PROG_SLOT.load(Ordering::Relaxed);

                // Write page 0 last - this is the header, so we only want to write it once everything
                // else is written successfully
                // Keep page 0 in the first 4096 bytes of BUF for the end
                if *page == 0 {
                    draw_flashing_ui(*page, None);
                    debug("receiving page 0");
                    if let Ok(count) = serial.read(&mut buf[*index..4096]) {
                        *index += count;

                        if num_pages.is_none() && *index >= 12 {
                            let mut b = [0u8; 4];
                            b.copy_from_slice(&buf[8..12]);
                            let num_bytes = usize::from_le_bytes(b);
                            *num_pages = Some(num_bytes.div_ceil(4096));
                            *remainder = Some(num_bytes % 4096);
                        }

                        if *index == 4096 {
                            *index = 0;
                            *page += 1;
                        }
                    }
                } else {
                    draw_flashing_ui(*page, *num_pages);
                    if let Ok(count) = serial.read(&mut buf[(4096 + *index)..8192]) {
                        let mut message = heapless::String::<32>::new();
                        write!(message, "receiving page {page}").unwrap();
                        debug(&message);

                        *index += count;

                        let num_pages = num_pages.unwrap();
                        let remainder = remainder.unwrap();
                        if *index == 4096 || (*page == num_pages - 1 && *index == remainder) {
                            *index = 0;

                            // Actually write the flash page
                            debug("writing page");
                            unsafe { write_flash(&buf[4096..8192], slot, *page) };

                            *page += 1;
                            // If this is the last page, also flash the first page which we didn't
                            // do at the start
                            if *page == num_pages {
                                debug("finalising");

                                let erase_cycles = erase_cycles(slot);
                                buf[0..4].copy_from_slice(&(erase_cycles + 1).to_ne_bytes());

                                unsafe { write_flash(&buf[0..4096], slot, 0) };

                                let this_header = unsafe { header::slot(slot) };
                                let this_name = unsafe { core::slice::from_raw_parts((*this_header).name_ptr, (*this_header).name_len) };

                                // If there is an old program with the same name, delete it
                                Programs::new()
                                    .filter_map(|prog| unsafe {
                                        let name = core::slice::from_raw_parts((*prog).name_ptr, (*prog).name_len);
                                        if (*prog).slot() != slot && name == this_name {
                                            Some((*prog).slot())
                                        } else {
                                            None
                                        }
                                    })
                                    .for_each(|slot| unsafe { delete_program(slot) });

                                PROG_SLOT.store(0, Ordering::Relaxed);

                                NEEDS_REFRESH_PROGRAMS.store(true, Ordering::Relaxed);
                                set_touch_enabled(true);
                                info("Finished writing program");

                                *state = SerialState::ReadyForCommand;
                            }
                        }
                    }
                }
            }
        }
    }
}