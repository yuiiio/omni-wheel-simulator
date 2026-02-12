use eframe::{egui, App, Frame};
use egui::{Pos2, Shape, Stroke, Vec2};
use nalgebra::{Matrix4, Vector4};

const ACCEL_NORM: f32 = 1.0;

#[derive(Default, Clone)]
struct State {
    accel: [f32; 2],
    pos: [f32; 2],
    vel: [f32; 2],
    state_time: f32,
    /*
    pos_func_2: [f32; 2],
    vel_func_2: [f32; 2],
    */
    jacobian: Matrix4<f32>,
}

#[derive(Default)]
struct OmniApp {
    traj: Vec<State>,
    time_max: f32,
    scale: f32,
    show_trail: bool,
    // 入力
    target_pos: [f32; 2],
    target_vel: [f32; 2],
    vx0: f32,
    vy0: f32,
    gn_active: bool,
    gn_max_iter: usize,
    parametor: Vector4<f32>,
    
    dragging_v0: bool,
    drag_start_screen: egui::Pos2,
}

impl OmniApp {
    fn new() -> Self {
        Self {
            traj: Vec::new(),
            time_max: 100.0,
            scale: 10.0,
            show_trail: true,
            vx0: -1.2,
            vy0: 1.3,
            target_pos: [0.0, 0.0],
            target_vel: [0.0, 0.0],
            parametor: Vector4::new(0.0, 0.0, 0.0, 0.0),
            gn_active: false,
            gn_max_iter: 200,
            dragging_v0: false,
            drag_start_screen: egui::Pos2::new(0.0, 0.0),
        }
    }

    fn compute_residual(&self) -> Vector4<f32> {
        let s = self.traj.last().unwrap();

        Vector4::new(
            s.pos[0] - self.target_pos[0],
            s.pos[1] - self.target_pos[1],
            s.vel[0] - self.target_vel[0],
            s.vel[1] - self.target_vel[1],
        )
    }

    fn compute_accel(&self, t: f32) -> (f32, f32, f32, f32, f32) {
        let phai3 = self.parametor[0] * t + self.parametor[2];
        let phai4 = self.parametor[1] * t + self.parametor[3];

        let h1 = (phai3 * phai3 + phai4 * phai4).sqrt().max(1e-6);
        
        let ux = (phai3 / h1 ) * ACCEL_NORM;
        let uy = (phai4 / h1 ) * ACCEL_NORM;
        (ux, uy, phai3, phai4, h1)
    }

    fn gauss_newton_step(&mut self) {
        println!("run gn step\n");
        let s = self.traj.last().unwrap();
        let r = self.compute_residual();
        let j = s.jacobian;

        let lambda: f32 = 1e-2;
        let identity = Matrix4::identity();

        let jt = j.transpose();
        let h = (jt * j) + lambda * identity; // 近似ヘッセ
        let g = jt * r;              // 勾配

        if let Some(delta) = h.lu().solve(&g) {
            self.parametor -= delta;
            println!("|r| = {}", r.norm());
            println!("|delta| = {}", delta.norm());
            //println!("parametor: {}", self.parametor);
        } else {
            println!("failed to solve delta, reset parametor");
            self.parametor[0] = 0.0;
            self.parametor[1] = 0.0;
            self.parametor[2] = self.target_pos[0] * 0.1;
            self.parametor[3] = self.target_pos[1] * 0.1;
        }
    }

    fn precompute(&mut self) {

        // by PMP ceory
        // phai_3 = a1 * t + a3;
        // phai_4 = a2 * t + a4;
        // accel_x = phi3 / sqrt(phi3^2 + phi4^2)
        // accel_y = phi4 / sqrt(phi3^2 + phi4^2)
        
        // 数値積分と、積分関数の結果があっていることを確かめます。
        /*
        let alpha = self.a1.powi(2) + self.a2.powi(2);
        let beta = 2.0 * (self.a1 * self.a3 + self.a2 * self.a4);
        let gamma = self.a3.powi(2) + self.a4.powi(2);

        let square_p = (4.0 * alpha * gamma - beta.powi(2)) / (4.0 * alpha.powi(2));
        let p = square_p.sqrt();
        let u_t0 = beta / (2.0 * alpha);
        let norm_t0 = (u_t0.powi(2) + square_p).sqrt();
        // let log_t0 = (u_t0 + norm_t0).ln();
        //log(u + sqrt(u^2 + p^2)) = arcsinh(u/p)
        let arcsisnh_t0 = (u_t0 / p).asinh();
        let log_coef_x = self.a3 - (beta * self.a1) / (2.0 * alpha);
        let log_coef_y = self.a4 - (beta * self.a2) / (2.0 * alpha);
        */

        self.traj.clear();
        let mut s = State::default();
        s.vel[0] = self.vx0;
        s.vel[1] = self.vy0;
        let dt = 1.0 / 120.0;
        for _ in 0..(self.time_max / dt) as usize {
            let (ux, uy, phai3, phai4, r) 
                = self.compute_accel(s.state_time);
            s.accel[0] = ux;
            s.accel[1] = uy;

            //数値積分
            //速度、位置は一方向の加算なので、オイラー法でよい
            s.vel[0] += ux * dt;
            s.vel[1] += uy * dt;
            s.pos[0] += s.vel[0] * dt;
            s.pos[1] += s.vel[1] * dt;

            // State: S(t) { x, y, vx, vy }
            // parametor: P { a1, a2, a3, a4 }
            // Jacobian: J(t,P) = ∂S(t)/∂P (縦行x,y,vx,vy: 横列∂a1,∂a2,∂a3,∂a4)
            // S(t)はPに依存しているので、
            // ∂{∂S(t)/∂P}/∂t = ∂{∂S(t)/∂t}/∂P
            // = ({∂S(t)/∂t}/∂S) * J(t,P) + ({∂S(t)/∂t}/∂P)
            // 
            //  {∂S(t)/∂t}/∂S = {vx, vy, ux, uy}/∂S = [ 0, 0, 1, 0,
            //                                          0, 0, 0, 1,
            //                                          0, 0, 0, 0,
            //                                          0, 0, 0, 0, ]
            // Pはux,uyに影響し、=> vx,vy,=> x,yは状態遷移行列で更新されるため、
            //  入力項として
            // {∂S(t)/∂t}/∂P = {vx, vy, ux, uy}/∂P
            //  = [ 0, 0, 0, 0,
            //      0, 0, 0, 0,
            //      ∂ux/∂a1,  ∂ux/∂a2, ∂ux/∂a3, ∂ux/∂a4,
            //      ∂uy/∂a1,  ∂uy/∂a2, ∂uy/∂a3, ∂uy/∂a4,
            //      ]
            //
            // R = sqrt(phai3^2 + phai4^2)
            // ∂ux/∂a1 = t/R - (phai3^2 * t)/R^3
            // ∂ux/∂a2 = 0
            // ∂ux/∂a3 = 1/R - (phai3^2 * 1)/R^3
            // ∂ux/∂a4 = 0
            //
            // ∂uy/∂a1 = 0
            // ∂uy/∂a2 = t/R - (phai4^2 * t)/R^3
            // ∂uy/∂a3 = 0
            // ∂uy/∂a4 = 1/R - (phai4^2 * 1)/R^3

            let transition_matrix = Matrix4::new(
                0.0, 0.0, 1.0, 0.0,             
                0.0, 0.0, 0.0, 1.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,);

            let xa1 = (s.state_time / r) - (phai3.powi(2) * s.state_time / r.powi(3));
            let xa3 = (1.0 / r) - (phai3.powi(2) * 1.0 / r.powi(3));
            let ya2 = (s.state_time / r) - (phai4.powi(2) * s.state_time / r.powi(3));
            let ya4 = (1.0 / r) - (phai4.powi(2) * 1.0 / r.powi(3));
            
            let input_matrix = Matrix4::new(
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                xa1, 0.0, xa3, 0.0,
                0.0, ya2, 0.0, ya4);

            //とりあえずオイラー法
            //ヤコビアンが^2以降0なので、状態行列は解析的に解ける
            s.jacobian += (transition_matrix * s.jacobian + input_matrix )*dt;

            //積分関数 
            //実験結果=> 特に位置の2階積分が、不安定
            //sqrt(alpha)が分母(1,a3が0に近いとき、加速度は変化しない)
            // log,arcsinhの実装依存や浮動小数点の精度限界ぽい
            /*
            let u_now = u_t0 + s.state_time;
            let norm_now = (u_now.powi(2) + square_p).sqrt();
            // let log_now = (u_now + norm_now).ln();
            let arcsisnh_now = (u_now / p).asinh();

            s.vel_func_2[0] = self.vx0 +
                ( self.a1 * (norm_now - norm_t0) + log_coef_x * (arcsisnh_now - arcsisnh_t0) ) / alpha.sqrt();
            s.vel_func_2[1] = self.vy0 +
                ( self.a2 * (norm_now - norm_t0) + log_coef_y * (arcsisnh_now - arcsisnh_t0) ) / alpha.sqrt();
            /*
            s.pos_func_2[0] += s.vel_func_2[0] * dt;
            s.pos_func_2[1] += s.vel_func_2[1] * dt;
            */
            s.pos_func_2[0] = 0.0 + self.vx0 * s.state_time +
                ( (self.a1 / 2.0) * ((u_now * norm_now - u_t0 * norm_t0) + (square_p * (arcsisnh_now - arcsisnh_t0))) +
                    log_coef_x * ((u_now * arcsisnh_now - u_t0 * arcsisnh_t0) - (norm_now - norm_t0))
                ) / alpha.sqrt();
            s.pos_func_2[1] = 0.0 + self.vy0 * s.state_time +
                ( (self.a2 / 2.0) * ((u_now * norm_now - u_t0 * norm_t0) + (square_p * (arcsisnh_now - arcsisnh_t0))) +
                    log_coef_y * ((u_now * arcsisnh_now - u_t0 * arcsisnh_t0) - (norm_now - norm_t0))
                ) / alpha.sqrt();
            */

            s.state_time += dt;
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
    
    fn screen_to_world(&self, center: egui::Pos2, p: egui::Pos2) -> egui::Vec2 {
        egui::Vec2 {
            x: (p.x - center.x) / self.scale,
            y: -(p.y - center.y) / self.scale,
        }
    }
}

impl App for OmniApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut Frame) {
        //ctx.set_visuals(egui::Visuals::light());

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            ui.add_space(8.0);
            ui.vertical_centered(|ui| {
                ui.heading("🌀 Omni-wheel 2D Trajectory Simulator");
            });
            ui.add_space(8.0);
            ui.separator();

            let mut input_changed = false;

            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().slider_width = 200.0;
                ui.label("Initial Velocity");
                input_changed |= ui.add(egui::Slider::new(&mut self.vx0, -10.0..=10.0).text("vx0")).changed();
                input_changed |= ui.add(egui::Slider::new(&mut self.vy0, -10.0..=10.0).text("vy0")).changed();
            });

            ui.label("Target");
            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().slider_width = 200.0;
                input_changed |= ui.add(egui::Slider::new(&mut self.target_pos[0], -50.0..=50.0).text("x")).changed();
                input_changed |= ui.add(egui::Slider::new(&mut self.target_pos[1], -50.0..=50.0).text("y")).changed();
            });
            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().slider_width = 200.0;
                input_changed |= ui.add(egui::Slider::new(&mut self.target_vel[0], -50.0..=50.0).text("vx")).changed();
                input_changed |= ui.add(egui::Slider::new(&mut self.target_vel[1], -50.0..=50.0).text("vy")).changed();
            });
            ui.separator();

            ui.style_mut().spacing.slider_width = 500.0;
            let slider = egui::Slider::new(&mut self.time_max, 0.1..=100.0)
                .text("time_max (T)")
                .trailing_fill(true);
            ui.add_sized(Vec2::new(520.0, 28.0), slider);
            ui.add_space(10.0);

            if input_changed {
                self.parametor[0] = 0.0;
                self.parametor[1] = 0.0;
                self.parametor[2] = self.target_pos[0] * 0.1;
                self.parametor[3] = self.target_pos[1] * 0.1;
            }

            for _ in 0..self.gn_max_iter {
                self.precompute();

                let err = self.compute_residual().norm();
                if err < 1e-4 {
                    self.gn_active = false;
                    break;
                }
                self.gauss_newton_step();
            }
            self.precompute();              // 最終結果反映
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let rect = ui.available_rect_before_wrap();
            let center = rect.center();
            let painter = ui.painter();

            const VEL_ARROW_SCALE: f32 = 0.5;

            // --- 0.5 秒ごとの状態描画（軌道線の上に追加） ---
            let mut t_mark = 0.0;
            while t_mark <= self.time_max {
                if let Some(state) = self
                    .traj
                        .iter()
                        .min_by(|a, b| (a.state_time - t_mark).abs().partial_cmp(&(b.state_time - t_mark).abs()).unwrap())
                {
                    let pos_screen = self.world_to_screen(center, state.pos);
                    painter.circle_filled(pos_screen, 10.0, egui::Color32::from_rgb(255, 140, 0));

                    // 速度ベクトル
                    let vel_end = Pos2::new(pos_screen.x + state.vel[0] * self.scale * VEL_ARROW_SCALE, pos_screen.y - state.vel[1] * self.scale * VEL_ARROW_SCALE);
                    painter.line_segment([pos_screen, vel_end], Stroke::new(2.5, egui::Color32::DARK_GREEN));

                    // 加速度ベクトル
                    let [ux, uy] = state.accel;
                    let acc_end = Pos2::new(pos_screen.x + ux * self.scale * 0.3, pos_screen.y - uy * self.scale * 0.3);
                    painter.line_segment([pos_screen, acc_end], Stroke::new(2.5, egui::Color32::RED));

                }
                t_mark += 0.5;
            }

            // 軌道線
            if self.show_trail && !self.traj.is_empty() {
                let points: Vec<Pos2> = self
                    .traj
                    .iter()
                    .map(|s| self.world_to_screen(center, s.pos))
                    .collect();
                if points.len() > 1 {
                    painter.add(Shape::line(points, Stroke::new(2.0, egui::Color32::LIGHT_BLUE)));
                }

                /*
                let points_func_2: Vec<Pos2> = self
                    .traj
                    .iter()
                    .take_while(|s| s.state_time <= self.time)
                    .map(|s| self.world_to_screen(center, s.pos_func_2))
                    .collect();
                if points_func_2.len() > 1 {
                    painter.add(Shape::line(points_func_2, Stroke::new(2.0, egui::Color32::LIGHT_RED)));
                }
                */
            }

            /*
            if let Some(current_state) = self
                .traj
                    .iter()
                    .min_by(|a, b| {
                        (a.state_time - self.time)
                            .abs()
                            .partial_cmp(&(b.state_time - self.time).abs())
                            .unwrap()
                    })
            {
                println!("Current time: {:.3}", current_state.state_time);
                println!("Jacobian:\n{}", current_state.jacobian);
            }
            */

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
            painter.line_segment([Pos2::new(center.x, center.y), initial_vel], Stroke::new(2.5, egui::Color32::DARK_BLUE));
        });

        ctx.request_repaint();
    }
}

fn main() {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "Omni-wheel 2D Trajectory Simulator (Bright UI)",
        options,
        Box::new(|_cc| Ok(Box::new(OmniApp::new()))),
    ).ok();
}
