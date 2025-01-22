use defmt::debug;
use embedded_graphics::geometry::AnchorPoint;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, Rectangle};
use fw16_epd_gui::draw_target::EpdDrawTarget;
use fw16_epd_gui::element::button::Button;
use fw16_epd_gui::element::Gui;
use fw16_epd_gui::element::slider::Slider;
use fw16_epd_program_interface::{RefreshBlockMode, SafeOption, Event, TouchEventType};
use crate::{next_event, set_touch_enabled};

enum Page {
    MainPage,
    ScratchpadPage,
}

struct MainPage {
    scratchpad_button: Button<'static>,
    test_buttons: [Button<'static>; 16],
    s1: Slider,
    s2: Slider,
}

impl MainPage {
    fn new() -> Self {
        let test_buttons = [
            Button::with_default_style(Rectangle::new(Point::new(10, 50), Size::new(40, 40)), "0", false),
            Button::with_default_style(Rectangle::new(Point::new(60, 50), Size::new(40, 40)), "1", false),
            Button::with_default_style(Rectangle::new(Point::new(110, 50), Size::new(40, 40)), "2", false),
            Button::with_default_style(Rectangle::new(Point::new(160, 50), Size::new(40, 40)), "3", false),
            Button::with_default_style(Rectangle::new(Point::new(10, 100), Size::new(40, 40)), "4", false),
            Button::with_default_style(Rectangle::new(Point::new(60, 100), Size::new(40, 40)), "5", false),
            Button::with_default_style(Rectangle::new(Point::new(110, 100), Size::new(40, 40)), "6", false),
            Button::with_default_style(Rectangle::new(Point::new(160, 100), Size::new(40, 40)), "7", false),
            Button::with_default_style(Rectangle::new(Point::new(10, 150), Size::new(40, 40)), "8", false),
            Button::with_default_style(Rectangle::new(Point::new(60, 150), Size::new(40, 40)), "9", false),
            Button::with_default_style(Rectangle::new(Point::new(110, 150), Size::new(40, 40)), "A", false),
            Button::with_default_style(Rectangle::new(Point::new(160, 150), Size::new(40, 40)), "B", false),
            Button::with_default_style(Rectangle::new(Point::new(10, 200), Size::new(40, 40)), "C", false),
            Button::with_default_style(Rectangle::new(Point::new(60, 200), Size::new(40, 40)), "D", false),
            Button::with_default_style(Rectangle::new(Point::new(110, 200), Size::new(40, 40)), "E", false),
            Button::with_default_style(Rectangle::new(Point::new(160, 200), Size::new(40, 40)), "F", false),
        ];


        Self {
            scratchpad_button: Button::with_default_style_auto_sized(Point::new(10, 10), "Scratchpad", true),
            test_buttons,
            s1: Slider::with_default_style(Point::new(10, 300), 220, 1, 20, 10),
            s2: Slider::with_default_style(Point::new(10, 350), 220, 1, 20, 10),
        }
    }
}

impl Gui for MainPage {
    type Output = Option<Page>;

    fn draw_init(&self, draw_target: &mut EpdDrawTarget) {
        self.scratchpad_button.draw_init(draw_target);
        self.s1.draw_init(draw_target);
        self.s2.draw_init(draw_target);
        for button in &self.test_buttons {
            button.draw_init(draw_target);
        }
    }

    fn tick(&mut self, draw_target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        let mut needs_refresh = false;

        let s = self.scratchpad_button.tick(draw_target, ev);
        if s.clicked {
            return Some(Page::ScratchpadPage);
        } else if s.needs_refresh {
            draw_target.refresh(true, RefreshBlockMode::BlockAcknowledge);
        }

        for button in &mut self.test_buttons {
            needs_refresh |= button.tick(draw_target, ev).needs_refresh;
        }

        if self.s1.tick(draw_target, ev) {
            needs_refresh = true;
            draw_target.fill_solid(&self.s2.bounding_box(), BinaryColor::Off).unwrap();
            self.s2.marker_radius = self.s1.value;
            self.s2.draw_init(draw_target);
        }

        if self.s2.tick(draw_target, ev) {
            needs_refresh = true;
            draw_target.fill_solid(&self.s1.bounding_box(), BinaryColor::Off).unwrap();
            self.s1.marker_radius = self.s2.value;
            self.s1.draw_init(draw_target);
        }

        if needs_refresh {
            draw_target.refresh(true, RefreshBlockMode::NonBlocking);
        }

        None
    }
}

struct ScratchpadPage {
    exit_button: Button<'static>,
    clear_button: Button<'static>,
    toggle_button: Button<'static>,
    slider: Slider,
    eraser: bool,
    prev_pos: Option<Point>,
}

impl ScratchpadPage {
    fn new() -> Self {
        let exit_button = Button::with_default_style_auto_sized(Point::new(10, 416 - 10 - 20), "Exit", true);

        let next_pos = exit_button
            .bounding_box()
            .translate(Point::new(10, 0))
            .anchor_point(AnchorPoint::TopRight);
        let clear_button = Button::with_default_style_auto_sized(next_pos, "Clear", true);

        let next_pos = clear_button
            .bounding_box()
            .translate(Point::new(10, 0))
            .anchor_point(AnchorPoint::TopRight);
        let toggle_button = Button::with_default_style_auto_sized(next_pos, "Eraser", true);

        let slider = Slider::with_default_style(Point::new(20, 20), 100, 1, 10, 2);

        Self {
            exit_button,
            clear_button,
            toggle_button,
            slider,
            eraser: false,
            prev_pos: None,
        }
    }
}

impl Gui for ScratchpadPage {
    type Output = Option<Page>;

    fn draw_init(&self, draw_target: &mut EpdDrawTarget) {
        self.exit_button.draw_init(draw_target);
        self.clear_button.draw_init(draw_target);
        self.toggle_button.draw_init(draw_target);
        self.slider.draw_init(draw_target);
    }

    fn tick(&mut self, draw_target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        let mut refresh: Option<RefreshBlockMode> = None;
        let mut handle_drawing: bool = true;

        let e = self.exit_button.tick(draw_target, ev);
        if e.clicked {
            return Some(Page::MainPage);
        }
        if e.needs_refresh {
            refresh = Some(RefreshBlockMode::BlockAcknowledge);
        }

        let c = self.clear_button.tick(draw_target, ev);
        if c.clicked {
            draw_target.clear(BinaryColor::Off).unwrap();
            self.draw_init(draw_target);
            draw_target.refresh(false, RefreshBlockMode::BlockAcknowledge);
            return None;
        }
        if c.needs_refresh {
            refresh = Some(RefreshBlockMode::BlockAcknowledge);
        }

        let t = self.toggle_button.tick(draw_target, ev);
        if t.clicked {
            self.eraser = !self.eraser;
            self.toggle_button.label = if self.eraser { "Pen" } else { "Eraser" };
            refresh = Some(RefreshBlockMode::NonBlocking);
        }
        if t.needs_refresh {
            refresh = Some(RefreshBlockMode::BlockAcknowledge);
        }

        if self.slider.tick(draw_target, ev) {
            refresh = Some(RefreshBlockMode::NonBlocking);
            debug!("stroke width = {}", self.slider.value);
            handle_drawing = false;
        }

        if let Event::Touch(ev) = ev {
            if handle_drawing && matches!(ev.ev_type, TouchEventType::Move | TouchEventType::Up) {
                if let Some(prev) = self.prev_pos {
                    let style = PrimitiveStyle::with_stroke(BinaryColor::from(!self.eraser), self.slider.value as u32);
                    Line::new(prev, ev.eg_point())
                        .into_styled(style)
                        .draw(draw_target)
                        .unwrap();
                    // Draw a circle at each end of the line
                    let circle_style = PrimitiveStyle::with_fill(BinaryColor::from(!self.eraser));
                    Circle::with_center(prev, self.slider.value as u32 - 1)
                        .into_styled(circle_style)
                        .draw(draw_target)
                        .unwrap();
                    Circle::with_center(ev.eg_point(), self.slider.value as u32 - 1)
                        .into_styled(circle_style)
                        .draw(draw_target)
                        .unwrap();

                    self.draw_init(draw_target);

                    if refresh.is_none() {
                        refresh = Some(RefreshBlockMode::NonBlocking);
                    }
                }
            }

            if matches!(ev.ev_type, TouchEventType::Down | TouchEventType::Move) {
                self.prev_pos = Some(ev.eg_point());
            }
        }

        if let Some(mode) = refresh {
            draw_target.refresh(true, mode);
        }

        None
    }
}

struct MainGui {
    current_page: Page,
    main_page: MainPage,
    scratchpad_page: ScratchpadPage,
}

impl MainGui {
    fn new() -> Self {
        Self {
            current_page: Page::MainPage,
            main_page: MainPage::new(),
            scratchpad_page: ScratchpadPage::new(),
        }
    }

    fn get_current_page(&self) -> &dyn Gui<Output = Option<Page>> {
        match self.current_page {
            Page::MainPage => &self.main_page,
            Page::ScratchpadPage => &self.scratchpad_page,
        }
    }

    fn get_current_page_mut(&mut self) -> &mut dyn Gui<Output = Option<Page>> {
        match self.current_page {
            Page::MainPage => &mut self.main_page,
            Page::ScratchpadPage => &mut self.scratchpad_page,
        }
    }
}

impl Gui for MainGui {
    type Output = ();

    fn draw_init(&self, draw_target: &mut EpdDrawTarget) {
        self.get_current_page().draw_init(draw_target);
    }

    fn tick(&mut self, draw_target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        if let Some(page) = self.get_current_page_mut().tick(draw_target, ev) {
            self.current_page = page;
            draw_target.clear(BinaryColor::Off).unwrap();
            self.draw_init(draw_target);
            draw_target.refresh(false, RefreshBlockMode::BlockAcknowledge);
        }
    }
}

pub(crate) fn gui_main(mut draw_target: EpdDrawTarget) -> ! {
    debug!("gui_main");

    unsafe { set_touch_enabled(true) };
    let mut gui = MainGui::new();
    gui.draw_init(&mut draw_target);
    draw_target.refresh(false, RefreshBlockMode::BlockAcknowledge);

    loop {
        while let SafeOption::Some(ev) = next_event() {
            gui.tick(&mut draw_target, ev);
        }
    }
}