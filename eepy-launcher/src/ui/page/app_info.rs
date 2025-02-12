use eepy_gui::element::button::Button;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_gui::element::{Gui, DEFAULT_TEXT_STYLE};
use eepy_sys::header::slot;
use eepy_sys::input_common::Event;
use crate::delete_program;
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

        if refresh {
            target.refresh(true);
        }

        None
    }
}