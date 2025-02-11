use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_gui::element::Gui;
use eepy_sys::input_common::Event;
use crate::ui::page::app_info::AppInfoPage;
use crate::ui::page::main::MainPage;
use crate::ui::page::scratchpad::ScratchpadPage;

mod page;
pub(crate) mod flashing;

pub(crate) enum MainGui {
    MainPage(MainPage),
    ScratchpadPage(ScratchpadPage),
    AppInfoPage(AppInfoPage),
}

impl MainGui {
    pub(crate) fn new() -> Self {
        Self::MainPage(MainPage::new())
    }

    fn get_current_page(&self) -> &dyn Gui<Output = Option<Self>> {
        match self {
            MainGui::MainPage(page) => page,
            MainGui::ScratchpadPage(page) => page,
            MainGui::AppInfoPage(page) => page,
        }
    }

    fn get_current_page_mut(&mut self) -> &mut dyn Gui<Output = Option<Self>> {
        match self {
            MainGui::MainPage(page) => page,
            MainGui::ScratchpadPage(page) => page,
            MainGui::AppInfoPage(page) => page,
        }
    }
}

impl Gui for MainGui {
    type Output = ();

    fn draw_init(&self, draw_target: &mut EpdDrawTarget) {
        self.get_current_page().draw_init(draw_target);
    }

    fn tick(&mut self, draw_target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        if let Some(gui) = self.get_current_page_mut().tick(draw_target, ev) {
            *self = gui;

            draw_target.clear(BinaryColor::Off).unwrap();
            self.draw_init(draw_target);
            draw_target.refresh(false);
        }
    }
}