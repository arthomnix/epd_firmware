use embedded_graphics::Drawable;
use embedded_graphics::mono_font::{ascii, MonoTextStyle};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Point;
use embedded_graphics::text::Text;
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_gui::element::button::Button;
use eepy_gui::element::{Gui, DEFAULT_TEXT_STYLE};
use eepy_sys::input_common::Event;
use eepy_sys::misc::get_serial;
use crate::ui::MainGui;
use crate::ui::page::main::MainPage;

const SMALL_ITALIC_TEXT_STYLE: MonoTextStyle<BinaryColor> = MonoTextStyle::new(&ascii::FONT_6X13_ITALIC, BinaryColor::On);

pub(crate) struct AboutPage {
    back_button: Button<'static>,
}

impl Default for AboutPage {
    fn default() -> Self {
        Self {
            back_button: Button::with_default_style_auto_sized(Point::new(10, 10), "Back", true),
        }
    }
}

impl Gui for AboutPage {
    type Output = Option<MainGui>;

    fn draw_init(&self, target: &mut EpdDrawTarget) {
        Text::new(concat!("eepyOS (Launcher ", env!("CARGO_PKG_VERSION"), ")"), Point::new(5, 190), DEFAULT_TEXT_STYLE)
            .draw(target)
            .unwrap();
        Text::new(concat!("built ", env!("EEPY_LAUNCHER_BUILD_DATE")), Point::new(27, 210), SMALL_ITALIC_TEXT_STYLE)
            .draw(target)
            .unwrap();
        Text::new("Serial number: ", Point::new(27, 240), SMALL_ITALIC_TEXT_STYLE)
            .draw(target)
            .unwrap();
        Text::new(get_serial(), Point::new(27 + 6 * "Serial number: ".len() as i32, 240), SMALL_ITALIC_TEXT_STYLE)
            .draw(target)
            .unwrap();
        Text::new("(c) 2025 arthomnix - MIT licensed", Point::new(18, 384), SMALL_ITALIC_TEXT_STYLE)
            .draw(target)
            .unwrap();
        Text::new("https://tangled.org/@arthomnix.dev/eepy", Point::new(3, 400), SMALL_ITALIC_TEXT_STYLE)
            .draw(target)
            .unwrap();
        self.back_button.draw_init(target);
    }

    fn tick(&mut self, target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        let bb_out = self.back_button.tick(target, ev);
        if bb_out.clicked {
            return Some(MainGui::MainPage(MainPage::new()));
        } else if bb_out.needs_refresh {
            target.refresh(true);
        }

        None
    }
}