use defmt::debug;
use embedded_graphics::geometry::AnchorPoint;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Line;
use fw16_epd_gui::draw_target::EpdDrawTarget;
use fw16_epd_gui::element::button::Button;
use fw16_epd_gui::element::{GuiElement, DEFAULT_PRIMITIVE_STYLE};
use fw16_epd_program_interface::{SafeOption, TouchEvent, TouchEventType};
use crate::{next_touch_event, set_touch_enabled};

struct MainPage {
    scratchpad_button: Button<'static>,
}

impl MainPage {
    fn new() -> Self {
        Self {
            scratchpad_button: Button::with_default_style_auto_sized(Point::new(10, 10), "Scratchpad"),
        }
    }
}

impl GuiElement for MainPage {
    fn draw_element(&self, target: &mut EpdDrawTarget) {
        self.scratchpad_button.draw_element(target);
    }

    fn handle_touch(&mut self, ev: TouchEvent) {
        self.scratchpad_button.handle_touch(ev);
    }
}

struct ScratchpadPage {
    exit_button: Button<'static>,
    clear_button: Button<'static>,
    prev_pos: Option<Point>,
}

impl ScratchpadPage {
    fn new() -> Self {
        let exit_button = Button::with_default_style_auto_sized(Point::new(10, 416 - 10 - 20), "Exit");
        let next_pos = exit_button
            .rect()
            .translate(Point::new(10, 0))
            .anchor_point(AnchorPoint::TopRight);
        let clear_button = Button::with_default_style_auto_sized(next_pos, "Clear");

        Self {
            exit_button,
            clear_button,
            prev_pos: None,
        }
    }
}

impl GuiElement for ScratchpadPage {
    fn draw_element(&self, target: &mut EpdDrawTarget) {
        self.exit_button.draw_element(target);
        self.clear_button.draw_element(target);
    }

    fn handle_touch(&mut self, ev: TouchEvent) {
        self.exit_button.handle_touch(ev);
        self.clear_button.handle_touch(ev);
        if matches!(ev.ev_type, TouchEventType::Down | TouchEventType::Move) {
            self.prev_pos = Some(ev.eg_point());
        }
    }
}

enum Gui {
    MainPage(MainPage),
    ScratchpadPage(ScratchpadPage),
}

impl Gui {
    fn tick(&mut self, target: &mut EpdDrawTarget, ev: TouchEvent) {
        match self {
            Gui::MainPage(page) => {
                page.handle_touch(ev);

                if page.scratchpad_button.clicked(true) {
                    let scratchpad = ScratchpadPage::new();
                    target.clear(BinaryColor::Off).unwrap();
                    scratchpad.draw_element(target);
                    target.refresh(false, true);
                    *self = Gui::ScratchpadPage(scratchpad);
                    return;
                }
            }

            Gui::ScratchpadPage(page) => {
                if matches!(ev.ev_type, TouchEventType::Move | TouchEventType::Up) {
                    if let Some(prev) = page.prev_pos {
                        Line::new(prev, ev.eg_point())
                            .into_styled(DEFAULT_PRIMITIVE_STYLE)
                            .draw(target)
                            .unwrap();
                        page.draw_element(target);
                        target.refresh(true, false);
                    }
                }

                page.handle_touch(ev);

                if page.exit_button.clicked(true) {
                    let main = MainPage::new();
                    target.clear(BinaryColor::Off).unwrap();
                    main.draw_element(target);
                    target.refresh(false, true);
                    *self = Gui::MainPage(main);
                    return;
                }

                if page.clear_button.clicked(true) {
                    target.clear(BinaryColor::Off).unwrap();
                    page.draw_element(target);
                    target.refresh(false, false);
                }
            }
        }
    }
}

pub(crate) fn gui_main(mut draw_target: EpdDrawTarget) -> ! {
    debug!("gui_main");

    unsafe { set_touch_enabled(true) };
    let main = MainPage::new();
    main.draw_element(&mut draw_target);
    draw_target.refresh(false, true);
    let mut gui = Gui::MainPage(main);

    loop {
        while let SafeOption::Some(ev) = next_touch_event() {
            gui.tick(&mut draw_target, ev);
        }
    }
}