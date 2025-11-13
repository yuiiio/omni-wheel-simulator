use eframe::{egui, App, Frame};
use egui::{Pos2, Shape, Stroke};

#[derive(Default, Clone)]
struct State {
    pos: [f32; 2],
    vel: [f32; 2],
    time: f32,
}

#[derive(Default)]
struct OmniApp {
    traj: Vec<State>,
    time: f32,
    time_max: f32,
    scale: f32,
    show_trail: bool,
    playing: bool,
    // 新しいパラメータ
    a1: f32,
    a2: f32,
    a3: f32,
    a4: f32,
}

impl OmniApp {
    fn new() -> Self {
        Self {
            traj: Vec::new(),
            time: 0.0,
            time_max: 10.0,
            scale: 200.0,
            show_trail: true,
            playing: false,
            a1: 1.0,
            a2: 0.8,
            a3: 0.0,
            a4: 0.5,
        }
    }

    fn compute_accel(&self, t: f32) -> (f32, f32) {
        // phai3, phai4 の式に基づく加速度
        let phai3 = self.a1 * t + self.a3;
        let phai4 = self.a2 * t + self.a4;
        let h1 = (phai3 * phai3 + phai4 * phai4).sqrt().max(1e-6);
        let ux = phai3 / h1;
        let uy = phai4 / h1;
        (ux, uy)
    }

    fn precompute(&mut self) {
        self.traj.clear();
        let mut s = State::default();
        let dt = 1.0 / 120.0;
        for _ in 0..(self.time_max / dt) as usize {
            let (ux, uy) = self.compute_accel(s.time);
            s.vel[0] += ux * dt;
            s.vel[1] += uy * dt;
            s.pos[0] += s.vel[0] * dt;
            s.pos[1] += s.vel[1] * dt;
            s.time += dt;
            self.traj.push(s.clone());
        }
    }

    fn get_state_at(&self, t: f32) -> State {
        if self.traj.is_empty() {
            return State::default();
        }
        if t <= 0.0 {
            return self.traj[0].clone();
        }
        if t >= self.time_max {
            return self.traj.last().unwrap().clone();
        }
        let idx = ((t / self.time_max) * (self.traj.len() - 1) as f32) as usize;
        self.traj[idx].clone()
    }

    fn world_to_screen(&self, center: Pos2, p: [f32; 2]) -> Pos2 {
        Pos2::new(center.x + p[0] * self.scale, center.y - p[1] * self.scale)
    }
}

impl App for OmniApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut Frame) {
        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            ui.heading("Omni-wheel 2D trajectory (parametric accel)");
            ui.separator();

            ui.horizontal(|ui| {
                ui.label("a1"); ui.add(egui::Slider::new(&mut self.a1, -2.0..=2.0));
                ui.label("a2"); ui.add(egui::Slider::new(&mut self.a2, -2.0..=2.0));
                ui.label("a3"); ui.add(egui::Slider::new(&mut self.a3, -2.0..=2.0));
                ui.label("a4"); ui.add(egui::Slider::new(&mut self.a4, -2.0..=2.0));
            });

            ui.horizontal(|ui| {
                if ui.button(if self.playing { "Pause" } else { "Play" }).clicked() {
                    if !self.playing && self.traj.is_empty() {
                        self.precompute();
                    }
                    self.playing = !self.playing;
                }
                if ui.button("Recompute").clicked() {
                    self.precompute();
                }
                ui.checkbox(&mut self.show_trail, "Show trail");
            });

            ui.add(egui::Slider::new(&mut self.time, 0.0..=self.time_max).text("time (s)"));

            let (ux, uy) = self.compute_accel(self.time);
            let state = self.get_state_at(self.time);
            ui.label(format!(
                "t = {:.2}s | pos = [{:.2}, {:.2}] | vel = [{:.2}, {:.2}] | acc = [{:.2}, {:.2}]",
                state.time, state.pos[0], state.pos[1], state.vel[0], state.vel[1], ux, uy
            ));
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let rect = ui.available_rect_before_wrap();
            let center = rect.center();
            let painter = ui.painter();

            // 軌道線
            if self.show_trail && !self.traj.is_empty() {
                let points: Vec<Pos2> = self
                    .traj
                    .iter()
                    .take_while(|s| s.time <= self.time)
                    .map(|s| self.world_to_screen(center, s.pos))
                    .collect();
                if points.len() > 1 {
                    painter.add(Shape::line(points, Stroke::new(1.5, egui::Color32::LIGHT_BLUE)));
                }
            }

            // 現在位置
            let s = self.get_state_at(self.time);
            let pos_screen = self.world_to_screen(center, s.pos);
            painter.circle_filled(pos_screen, 10.0, egui::Color32::from_rgb(200, 100, 60));

            // 速度ベクトル
            let vel_end = Pos2::new(pos_screen.x + s.vel[0] * self.scale * 0.5, pos_screen.y - s.vel[1] * self.scale * 0.5);
            painter.line_segment([pos_screen, vel_end], Stroke::new(2.0, egui::Color32::GREEN));

            // 加速度ベクトル
            let (ux, uy) = self.compute_accel(self.time);
            let acc_end = Pos2::new(pos_screen.x + ux * self.scale * 0.3, pos_screen.y - uy * self.scale * 0.3);
            painter.line_segment([pos_screen, acc_end], Stroke::new(2.0, egui::Color32::RED));

            // 軸
            let axis_len = self.scale;
            painter.line_segment(
                [Pos2::new(center.x - axis_len, center.y), Pos2::new(center.x + axis_len, center.y)],
                Stroke::new(1.0, egui::Color32::GRAY),
            );
            painter.line_segment(
                [Pos2::new(center.x, center.y - axis_len), Pos2::new(center.x, center.y + axis_len)],
                Stroke::new(1.0, egui::Color32::GRAY),
            );
        });

        // 再生モード
        if self.playing {
            self.time += 1.0 / 60.0;
            if self.time > self.time_max {
                self.time = 0.0;
                self.playing = false;
            }
        }

        ctx.request_repaint();
    }
}

fn main() {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "Omni-wheel 2D trajectory",
        options,
        Box::new(|_cc| Box::new(OmniApp::new())),
    );
}

