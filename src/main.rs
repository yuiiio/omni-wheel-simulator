use eframe::{egui, App, Frame};
use egui::{Pos2, Shape, Stroke};
use std::f32::consts::PI;

#[derive(Default)]
struct OmniApp {
    pos: [f32; 2],
    vel: [f32; 2],
    ux: f32,
    uy: f32,
    time: f32,
    time_max: f32,
    running: bool,
    show_trail: bool,
    trail: Vec<[f32; 2]>,
    scale: f32,
}

impl OmniApp {
    fn new() -> Self {
        Self {
            pos: [0.0, 0.0],
            vel: [0.0, 0.0],
            ux: 0.0,
            uy: 0.0,
            time: 0.0,
            time_max: 10.0,
            running: false,
            show_trail: true,
            trail: Vec::new(),
            scale: 200.0,
        }
    }

    fn world_to_screen(&self, center: Pos2, p: [f32; 2]) -> Pos2 {
        Pos2::new(center.x + p[0] * self.scale, center.y - p[1] * self.scale)
    }

    fn compute_accel(&self, t: f32) -> (f32, f32) {
        // 時間依存加速度 (例)
        let ux = (t).sin() * 2.0;
        let uy = (t * 0.5).cos() * 2.0;
        (ux, uy)
    }

    fn simulate(&mut self, dt: f32) {
        let steps = 5;
        let sub_dt = dt / steps as f32;
        for _ in 0..steps {
            let (ux, uy) = self.compute_accel(self.time);
            self.ux = ux;
            self.uy = uy;

            self.vel[0] += ux * sub_dt;
            self.vel[1] += uy * sub_dt;
            self.pos[0] += self.vel[0] * sub_dt;
            self.pos[1] += self.vel[1] * sub_dt;
            self.time += sub_dt;

            if self.show_trail {
                self.trail.push(self.pos);
                if self.trail.len() > 2000 {
                    self.trail.remove(0);
                }
            }
        }
    }
}

impl App for OmniApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut Frame) {
        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Omni-wheel time simulation");
                if ui.button(if self.running { "Pause" } else { "Run" }).clicked() {
                    self.running = !self.running;
                }
                if ui.button("Reset").clicked() {
                    self.pos = [0.0, 0.0];
                    self.vel = [0.0, 0.0];
                    self.trail.clear();
                    self.time = 0.0;
                }
                ui.checkbox(&mut self.show_trail, "Show trail");
            });

            ui.add(egui::Slider::new(&mut self.time, 0.0..=self.time_max).text("time (s)"));
            let (ux, uy) = self.compute_accel(self.time);
            ui.label(format!("t = {:.2} s, ux = {:.2} m/s², uy = {:.2} m/s²", self.time, ux, uy));
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let available = ui.available_rect_before_wrap();
            let center = available.center();
            let (response, painter) = ui.allocate_painter(available.size(), egui::Sense::hover());

            if self.running {
                self.simulate(1.0 / 60.0);
                if self.time > self.time_max {
                    self.running = false;
                }
            }

            // 座標軸
            let axis_len = 1.0 * self.scale;
            painter.line_segment(
                [Pos2::new(center.x - axis_len, center.y), Pos2::new(center.x + axis_len, center.y)],
                Stroke::new(1.0, egui::Color32::LIGHT_GRAY),
            );
            painter.line_segment(
                [Pos2::new(center.x, center.y - axis_len), Pos2::new(center.x, center.y + axis_len)],
                Stroke::new(1.0, egui::Color32::LIGHT_GRAY),
            );

            // 軌跡
            if self.trail.len() >= 2 {
                let points: Vec<Pos2> = self.trail.iter().map(|&p| self.world_to_screen(center, p)).collect();
                painter.add(Shape::line(points, Stroke::new(1.0, egui::Color32::LIGHT_BLUE)));
            }

            // 本体
            let body_center = self.world_to_screen(center, self.pos);
            painter.circle_filled(body_center, 10.0, egui::Color32::from_rgb(200, 120, 80));

            // 速度・加速度ベクトル
            let vel_end = Pos2::new(body_center.x + self.vel[0] * self.scale, body_center.y - self.vel[1] * self.scale);
            painter.line_segment([body_center, vel_end], Stroke::new(2.0, egui::Color32::GREEN));

            let acc_end = Pos2::new(body_center.x + self.ux * self.scale * 0.2, body_center.y - self.uy * self.scale * 0.2);
            painter.line_segment([body_center, acc_end], Stroke::new(2.0, egui::Color32::RED));

            painter.text(
                available.left_top() + egui::vec2(10.0, 10.0),
                egui::Align2::LEFT_TOP,
                format!(
                    "pos = [{:.2}, {:.2}] vel = [{:.2}, {:.2}]",
                    self.pos[0], self.pos[1], self.vel[0], self.vel[1]
                ),
                egui::FontId::proportional(14.0),
                egui::Color32::BLACK,
            );
        });

        ctx.request_repaint();
    }
}

fn main() {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "Omni-wheel time simulator",
        options,
        Box::new(|_cc| Box::new(OmniApp::new())),
    );
}

