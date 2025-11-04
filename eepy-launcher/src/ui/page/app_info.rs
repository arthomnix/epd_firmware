use core::fmt::Write;
use eepy_gui::element::button::Button;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::Text;
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_gui::element::{Gui, DEFAULT_TEXT_STYLE};
use eepy_sys::header::slot;
use eepy_sys::input_common::Event;
use crate::{delete_program, get_autostart, set_autostart};
use crate::ui::MainGui;
use crate::ui::page::main::MainPage;

pub(crate) struct AppInfoPage {
    slot: u8,
    back_button: Button<'static>,
    delete_button: Button<'static>,
    autostart_button: Button<'static>,
}

impl AppInfoPage {
    fn autostart_label(slot: u8) -> &'static str {
        if get_autostart() == Some(slot) {
            "Disable autostart"
        } else {
            "Enable autostart"
        }
    }

    pub(crate) fn new(slot: u8) -> Self {
        Self {
            slot,
            back_button: Button::with_default_style_auto_sized(Point::new(10, 386), "Back", true),
            delete_button: Button::with_default_style_auto_sized(Point::new(10, 80), "Delete program", true),
            autostart_button: Button::with_default_style(
                Rectangle::new(Point::new(10, 110), Size::new((10 * "Disable autostart".len() + 5) as u32, 20)),
                Self::autostart_label(slot),
                true
            ),
        }
    }
}

impl Gui for AppInfoPage {
    type Output = Option<MainGui>;

    fn draw_init(&self, target: &mut EpdDrawTarget) {
        let header = unsafe { slot(self.slot) };

        Text::new("Program:", Point::new(10, 20), DEFAULT_TEXT_STYLE)
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


        Text::new("Version:", Point::new(10, 40), DEFAULT_TEXT_STYLE)
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

        Text::new("Slot:", Point::new(10, 60), DEFAULT_TEXT_STYLE)
            .draw(target)
            .unwrap();
        let mut slot_s = heapless::String::<2>::new();
        write!(slot_s, "{}", self.slot).unwrap();
        Text::new(&slot_s, Point::new(120, 60), DEFAULT_TEXT_STYLE)
            .draw(target)
            .unwrap();

        self.back_button.draw_init(target);
        self.delete_button.draw_init(target);
        self.autostart_button.draw_init(target);
    }

    fn tick(&mut self, target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        let mut refresh = false;

        let b = self.back_button.tick(target, ev);
        if b.clicked {
            return Some(MainGui::MainPage(MainPage::new()));
        }
        refresh |= b.needs_refresh;

        let d = self.delete_button.tick(target, ev);
        if d.clicked {
            unsafe { delete_program(self.slot) };
            return Some(MainGui::MainPage(MainPage::new()));
        }
        refresh |= d.needs_refresh;

        let a = self.autostart_button.tick(target, ev);
        if a.clicked {
            if get_autostart() == Some(self.slot) {
                set_autostart(None);
            } else {
                set_autostart(Some(self.slot));
            }

            self.autostart_button.label = Self::autostart_label(self.slot);
            self.autostart_button.draw_init(target);
            refresh = true;
        }
        refresh |= a.needs_refresh;

        if refresh {
            target.refresh(true);
        }

        None
    }
}