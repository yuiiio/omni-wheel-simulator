use eframe::{egui, App, Frame};
use egui::{Pos2, Shape, Stroke, Vec2};

#[derive(Default, Clone)]
struct State {
    pos: [f32; 2],
    vel: [f32; 2],
    time: f32,
    pos_compute_2: [f32; 2],
    vel_compute_2: [f32; 2],
}

#[derive(Default)]
struct OmniApp {
    traj: Vec<State>,
    time: f32,
    time_max: f32,
    scale: f32,
    show_trail: bool,
    playing: bool,
    // パラメータ
    a1: f32,
    a2: f32,
    a3: f32,
    a4: f32,
    vx0: f32,
    vy0: f32,
    need_recalc: bool,
}

impl OmniApp {
    fn new() -> Self {
        Self {
            traj: Vec::new(),
            time: 0.0,
            time_max: 10.0,
            scale: 100.0,
            show_trail: true,
            playing: false,
            a1: 1.1,
            a2: -0.7,
            a3: 1.0,
            a4: -0.5,
            vx0: -1.2,
            vy0: 1.3,
            need_recalc: false,
        }
    }

    fn compute_accel(&self, t: f32) -> (f32, f32, f32) {
        let phai3 = self.a1 * t + self.a3;
        let phai4 = self.a2 * t + self.a4;
        let h1 = (phai3 * phai3 + phai4 * phai4).sqrt().max(1e-6);
        let ux = phai3 / h1;
        let uy = phai4 / h1;
        (ux, uy, h1)
    }

    fn precompute(&mut self) {
        let p = self.a1 * self.a1 + self.a2 * self.a2;
        let sqrt_p = p.sqrt();
        let q_div2 = self.a1 * self.a3 + self.a2 * self.a4;
        let h1_t0 = (self.a3 * self.a3 + self.a4 * self.a4).sqrt();
        let log_t0 = (q_div2 / sqrt_p + h1_t0).ln();

        self.traj.clear();
        let mut s = State::default();
        s.vel[0] = self.vx0;
        s.vel[1] = self.vy0;
        let dt = 1.0 / 120.0;
        for _ in 0..(self.time_max / dt) as usize {
            let (ux, uy, h1) = self.compute_accel(s.time);
            s.vel[0] += ux * dt;
            s.vel[1] += uy * dt;
            s.pos[0] += s.vel[0] * dt;
            s.pos[1] += s.vel[1] * dt;
            s.time += dt;

            let log_clause = (sqrt_p * s.time + q_div2 / sqrt_p + h1).ln();

            s.vel_compute_2[0] = self.vx0 
                + (self.a1 / p) * (h1 - h1_t0)
                + (log_clause - log_t0) * (self.a3 * p - self.a1 * q_div2) / (sqrt_p * p);

            s.vel_compute_2[1] = self.vy0 
                + (self.a2 / p) * (h1 - h1_t0)
                + (log_clause - log_t0) * (self.a4 * p - self.a2 * q_div2) / (sqrt_p * p);

            // WIP also integral position
            s.pos_compute_2[0] += s.vel_compute_2[0] * dt;
            s.pos_compute_2[1] += s.vel_compute_2[1] * dt;

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
        // 🌞 明るいテーマ設定
        ctx.set_visuals(egui::Visuals::light());

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            ui.add_space(8.0);
            ui.vertical_centered(|ui| {
                ui.heading("🌀 Omni-wheel 2D Trajectory Simulator");
            });
            ui.add_space(8.0);
            ui.separator();

            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().slider_width = 200.0;
                ui.label("a1"); self.need_recalc |= ui.add(egui::Slider::new(&mut self.a1, -2.0..=2.0).text("a1")).changed();
                ui.label("a2"); self.need_recalc |= ui.add(egui::Slider::new(&mut self.a2, -2.0..=2.0).text("a2")).changed();
                ui.label("a3"); self.need_recalc |= ui.add(egui::Slider::new(&mut self.a3, -2.0..=2.0).text("a3")).changed();
                ui.label("a4"); self.need_recalc |= ui.add(egui::Slider::new(&mut self.a4, -2.0..=2.0).text("a4")).changed();
            });
            ui.separator();
            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().slider_width = 200.0;
                ui.label("Initial Velocity");
                self.need_recalc |= ui.add(egui::Slider::new(&mut self.vx0, -5.0..=5.0).text("vx0")).changed();
                self.need_recalc |= ui.add(egui::Slider::new(&mut self.vy0, -5.0..=5.0).text("vy0")).changed();
            });

            ui.add_space(6.0);
            ui.horizontal(|ui| {
                if ui.add(egui::Button::new(if self.playing { "⏸ Pause" } else { "▶ Play" }).min_size(Vec2::new(80.0, 32.0))).clicked() {
                    if !self.playing && self.traj.is_empty() {
                        self.precompute();
                    }
                    self.playing = !self.playing;
                }
                if ui.add(egui::Button::new("🔁 Recompute").min_size(Vec2::new(100.0, 32.0))).clicked() || self.need_recalc {
                    self.precompute();
                    self.need_recalc = false;
                }
                ui.checkbox(&mut self.show_trail, "Show trail");
            });

            ui.add_space(10.0);

            // 時間スライダーを大きくする
            ui.style_mut().spacing.slider_width = 500.0;
            let slider = egui::Slider::new(&mut self.time, 0.0..=self.time_max)
                .text("⏱ time (s)")
                .trailing_fill(true);
            ui.add_sized(Vec2::new(520.0, 28.0), slider);

            let (ux, uy, _h1) = self.compute_accel(self.time);
            let state = self.get_state_at(self.time);
            ui.add_space(5.0);
            ui.label(format!(
                "t = {:.2}s   pos = [{:.2}, {:.2}]   vel = [{:.2}, {:.2}]   acc = [{:.2}, {:.2}]",
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
                    painter.add(Shape::line(points, Stroke::new(2.0, egui::Color32::LIGHT_BLUE)));
                }
                let points_compute_2: Vec<Pos2> = self
                    .traj
                    .iter()
                    .take_while(|s| s.time <= self.time)
                    .map(|s| self.world_to_screen(center, s.pos_compute_2))
                    .collect();
                if points_compute_2.len() > 1 {
                    painter.add(Shape::line(points_compute_2, Stroke::new(2.0, egui::Color32::LIGHT_RED)));
                }
            }

            const VEL_ARROW_SCALE: f32 = 0.5;

            // 現在位置
            let s = self.get_state_at(self.time);
            let pos_screen = self.world_to_screen(center, s.pos);
            painter.circle_filled(pos_screen, 10.0, egui::Color32::from_rgb(255, 140, 0));

            // 速度ベクトル
            let vel_end = Pos2::new(pos_screen.x + s.vel[0] * self.scale * VEL_ARROW_SCALE, pos_screen.y - s.vel[1] * self.scale * VEL_ARROW_SCALE);
            painter.line_segment([pos_screen, vel_end], Stroke::new(2.5, egui::Color32::GREEN));

            // 加速度ベクトル
            let (ux, uy, _h1) = self.compute_accel(self.time);
            let acc_end = Pos2::new(pos_screen.x + ux * self.scale * 0.3, pos_screen.y - uy * self.scale * 0.3);
            painter.line_segment([pos_screen, acc_end], Stroke::new(2.5, egui::Color32::RED));

            // 軸
            let axis_len = self.scale * 3.0;
            painter.line_segment(
                [Pos2::new(center.x - axis_len, center.y), Pos2::new(center.x + axis_len, center.y)],
                Stroke::new(1.0, egui::Color32::from_gray(150)),
            );
            painter.line_segment(
                [Pos2::new(center.x, center.y - axis_len), Pos2::new(center.x, center.y + axis_len)],
                Stroke::new(1.0, egui::Color32::from_gray(150)),
            );

            // 初速度ベクトル
            let initial_vel = self.world_to_screen(center, [self.vx0 * VEL_ARROW_SCALE, self.vy0 * VEL_ARROW_SCALE]);
            painter.line_segment([Pos2::new(center.x, center.y), initial_vel], Stroke::new(2.5, egui::Color32::DARK_GREEN));
        });

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
        "Omni-wheel 2D Trajectory Simulator (Bright UI)",
        options,
        Box::new(|_cc| Box::new(OmniApp::new())),
    );
}
