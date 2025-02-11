use core::mem::offset_of;
use eepy_gui::element::button::Button;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_gui::element::{Gui, DEFAULT_TEXT_STYLE};
use eepy_sys::flash;
use eepy_sys::header::{slot, slot_ptr, ProgramSlotHeader};
use eepy_sys::input_common::Event;
use crate::ui::MainGui;
use crate::ui::page::main::MainPage;

pub(crate) struct AppInfoPage {
    slot: u8,
    back_button: Button<'static>,
    delete_button: Button<'static>,
}

impl AppInfoPage {
    pub(crate) fn new(slot: u8) -> Self {
        Self {
            slot,
            back_button: Button::with_default_style_auto_sized(Point::new(10, 386), "Back", true),
            delete_button: Button::with_default_style_auto_sized(Point::new(10, 60), "Delete program", true),
        }
    }
}

impl Gui for AppInfoPage {
    type Output = Option<MainGui>;

    fn draw_init(&self, target: &mut EpdDrawTarget) {
        let header = unsafe { slot(self.slot) };

        Text::new("Program: ", Point::new(10, 20), DEFAULT_TEXT_STYLE)
            .draw(target)
            .unwrap();

        if let Ok(name) = unsafe { (*header).name() } {
            Text::new(name, Point::new(120, 20), DEFAULT_TEXT_STYLE)
                .draw(target)
                .unwrap();
        } else {
            Text::new("<invalid>", Point::new(120, 20), DEFAULT_TEXT_STYLE)
                .draw(target)
                .unwrap();
        }


        Text::new("Version: ", Point::new(10, 40), DEFAULT_TEXT_STYLE)
            .draw(target)
            .unwrap();

        if let Ok(vers) = unsafe { (*header).version() } {
            Text::new(vers, Point::new(120, 40), DEFAULT_TEXT_STYLE)
                .draw(target)
                .unwrap();
        } else {
            Text::new("<invalid>", Point::new(120, 40), DEFAULT_TEXT_STYLE)
                .draw(target)
                .unwrap();
        }

        self.back_button.draw_init(target);
        self.delete_button.draw_init(target);
    }

    fn tick(&mut self, target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        let b = self.back_button.tick(target, ev);
        if b.clicked {
            return Some(MainGui::MainPage(MainPage::new()));
        }

        let d = self.delete_button.tick(target, ev);
        if d.clicked {
            let ptr = unsafe { slot_ptr(self.slot) };
            let mut buf = [0u8; 256];
            unsafe { buf.copy_from_slice(core::slice::from_raw_parts(ptr, 256)) };
            let offset = offset_of!(ProgramSlotHeader, len);
            buf[offset..offset + 4].copy_from_slice(&[0; 4]);

            unsafe { flash::program(self.slot as u32 * 512 * 1024, &buf) };
            return Some(MainGui::MainPage(MainPage::new()));
        }

        None
    }
}