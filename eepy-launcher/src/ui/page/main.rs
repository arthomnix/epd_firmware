use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_gui::element::button::Button;
use eepy_gui::element::Gui;
use eepy_sys::exec::exec;
use eepy_sys::header::Programs;
use eepy_sys::input_common::Event;
use crate::ui::MainGui;
use crate::ui::page::about::AboutPage;
use crate::ui::page::app_info::AppInfoPage;
use crate::ui::page::scratchpad::ScratchpadPage;

pub(crate) struct MainPage {
    scratchpad_button: Button<'static>,
    about_button: Button<'static>,
    app_buttons: [Option<(Button<'static>, u8)>; 32],
}

impl MainPage {
    fn grid_button(x: usize, y: usize, label: &'static str, touch_feedback_immediate_release: bool) -> Button<'static> {
        let x_coord = 10 + 115 * x as i32;
        let y_coord = 8 + 25 * y as i32;
        Button::with_default_style(
            Rectangle::new(Point::new(x_coord, y_coord), Size::new(105, 20)),
            label,
            touch_feedback_immediate_release,
        )
    }

    fn app_button(x: usize, y: usize, label: &'static str) -> Button<'static> {
        Self::grid_button(x, y, label, false)
    }

    pub(crate) fn refresh_buttons(&mut self) {
        let mut programs = Programs::new();

        for y in 0..16 {
            for x in 0..2 {
                // Reserve the first space for the scratchpad
                if x == 0 && y == 0 {
                    continue;
                }

                if let Some(prog) = programs.next() {
                    let bi = y * 2 + x;
                    let button = Self::app_button(x, y, unsafe { (*prog).name().unwrap_or("<invalid>") });
                    let slot_num = unsafe { (&*prog).slot() };
                    self.app_buttons[bi] = Some((button, slot_num))
                }
            }
        }
    }

    pub(crate) fn new() -> Self {
        let mut res = Self {
            scratchpad_button: Self::grid_button(0, 0, "Scratchpad", true),
            about_button: Self::grid_button(1, 15, "About", true),
            app_buttons: [const { None }; 32],
        };
        res.refresh_buttons();
        res
    }
}

impl Gui for MainPage {
    type Output = Option<MainGui>;

    fn draw_init(&self, draw_target: &mut EpdDrawTarget) {
        self.scratchpad_button.draw_init(draw_target);

        if self.app_buttons[31].is_none() {
            self.about_button.draw_init(draw_target);
        }

        for b in &self.app_buttons {
            if let Some((button, _)) = b {
                button.draw_init(draw_target);
            }
        }
    }

    fn tick(&mut self, draw_target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        let mut needs_refresh = false;

        let sp_out = self.scratchpad_button.tick(draw_target, ev);
        if sp_out.clicked {
            return Some(MainGui::ScratchpadPage(ScratchpadPage::new()));
        } else if sp_out.needs_refresh {
            draw_target.refresh(true);
        }

        let about_out = self.about_button.tick(draw_target, ev);
        if about_out.clicked {
            return Some(MainGui::AboutPage(AboutPage::default()));
        } else if about_out.needs_refresh {
            draw_target.refresh(true);
        }


        for b in &mut self.app_buttons[1..] {
            if let Some((button, s)) = b {
                let response = button.tick(draw_target, ev);

                if response.long_clicked {
                    return Some(MainGui::AppInfoPage(AppInfoPage::new(*s)));
                } else if response.clicked {
                    exec(*s);
                }

                needs_refresh |= response.needs_refresh;
            }
        }

        if needs_refresh {
            draw_target.refresh(true);
        }

        None
    }
}