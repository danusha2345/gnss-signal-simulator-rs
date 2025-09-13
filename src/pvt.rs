use crate::constants::LIGHT_SPEED;
use crate::types::*;
use crate::coordinate::{ecef_to_lla, gps_sat_pos_speed_eph, gps_clock_correction};

#[derive(Clone, Copy)]
struct SatMeas {
    pos: KinematicInfo,
    rho: f64,       // псевдодальность (м)
}

fn build_meas_for_system(
    system: GnssSystem,
    cur_time: GnssTime,
    eph_list: &[Option<GpsEphemeris>],
    max_sat: usize,
) -> Vec<SatMeas> {
    let mut out = Vec::new();
    let mut count = 0;
    for eph_opt in eph_list.iter() {
        if let Some(mut eph) = *eph_opt {
            if (eph.valid & 1) == 0 || eph.health != 0 { continue; }
            let mut sat_pos = KinematicInfo::default();
            let tx = (cur_time.MilliSeconds as f64) / 1000.0; // секунды недели соответствующей шкалы
            if gps_sat_pos_speed_eph(system, tx, &mut eph, &mut sat_pos, None) {
                // Псевдодальность: геометрическая + поправка часов спутника
                let range = ((sat_pos.x).powi(2) + (sat_pos.y).powi(2) + (sat_pos.z).powi(2)).sqrt();
                let clk = gps_clock_correction(&eph, tx);
                let rho = range - LIGHT_SPEED * clk; // без приёмо‑часов, они оценятся
                out.push(SatMeas { pos: sat_pos, rho });
                count += 1;
                if count >= max_sat { break; }
            }
        }
    }
    out
}

pub fn solve_pvt_wls(
    cur_time_gps: GnssTime,
    cur_time_bds: GnssTime,
    cur_time_gal: GnssTime,
    gps_eph: &[Option<GpsEphemeris>],
    bds_eph: &[Option<GpsEphemeris>],
    gal_eph: &[Option<GpsEphemeris>],
    init_pos: KinematicInfo,
) -> Option<(KinematicInfo, f64, f64)> {
    // Сконструируем измерения из всех систем (по 8 макс. на систему)
    let mut meas = Vec::new();
    meas.extend(build_meas_for_system(GnssSystem::GpsSystem, cur_time_gps, gps_eph, 12));
    meas.extend(build_meas_for_system(GnssSystem::BdsSystem, cur_time_bds, bds_eph, 12));
    meas.extend(build_meas_for_system(GnssSystem::GalileoSystem, cur_time_gal, gal_eph, 12));

    if meas.len() < 4 { return None; }

    // Инициализация
    let mut x = init_pos.x;
    let mut y = init_pos.y;
    let mut z = init_pos.z;
    let mut cb = 0.0; // сдвиг часов приёмника (сек)

    for _ in 0..8 {
        // Сборка линейной системы H * dx = v
        let mut h11 = 0.0; let mut h12 = 0.0; let mut h13 = 0.0; let mut h14 = 0.0;
        let mut h22 = 0.0; let mut h23 = 0.0; let mut h24 = 0.0;
        let mut h33 = 0.0; let mut h34 = 0.0; let mut h44 = 0.0;
        let mut b1 = 0.0; let mut b2 = 0.0; let mut b3 = 0.0; let mut b4 = 0.0;

        for m in &meas {
            let dx = m.pos.x - x;
            let dy = m.pos.y - y;
            let dz = m.pos.z - z;
            let r = (dx*dx + dy*dy + dz*dz).sqrt();
            if r <= 1.0 { continue; }
            let ux = dx / r; let uy = dy / r; let uz = dz / r;
            // Предсказанная псевдодальность
            let rho_hat = r + LIGHT_SPEED * cb;
            let v = m.rho - rho_hat; // невязка
            // Аккумулируем нормальные уравнения (H^T H) и H^T v
            h11 += ux*ux; h12 += ux*uy; h13 += ux*uz; h14 += ux*LIGHT_SPEED;
            h22 += uy*uy; h23 += uy*uz; h24 += uy*LIGHT_SPEED;
            h33 += uz*uz; h34 += uz*LIGHT_SPEED;
            h44 += LIGHT_SPEED*LIGHT_SPEED;
            b1 += ux * v; b2 += uy * v; b3 += uz * v; b4 += LIGHT_SPEED * v;
        }

        // Решим 4x4 (симметричная) — простая Гауссова элиминация на нормальных уравнениях
        // Матрица:
        // [h11 h12 h13 h14]
        // [h12 h22 h23 h24]
        // [h13 h23 h33 h34]
        // [h14 h24 h34 h44]
        let mut a = [[h11, h12, h13, h14],
                     [h12, h22, h23, h24],
                     [h13, h23, h33, h34],
                     [h14, h24, h34, h44]];
        let mut b = [b1, b2, b3, b4];

        // Простейшая элиминация без частичного выбора (достаточно для устойчивых систем)
        for i in 0..4 {
            // pivot
            let piv = a[i][i];
            if piv.abs() < 1e-12 { return None; }
            for j in i..4 { a[i][j] /= piv; }
            b[i] /= piv;
            for k in 0..4 {
                if k==i { continue; }
                let f = a[k][i];
                for j in i..4 { a[k][j] -= f * a[i][j]; }
                b[k] -= f * b[i];
            }
        }

        let dx = b[0]; let dy = b[1]; let dz = b[2]; let dcb = b[3];
        x += dx; y += dy; z += dz; cb += dcb;
        if dx*dx + dy*dy + dz*dz < 1e-6 { break; }
    }

    let sol = KinematicInfo { x, y, z, ..Default::default() };
    let lla = ecef_to_lla(&sol);
    Some((sol, cb, lla.alt))
}

