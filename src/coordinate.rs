//----------------------------------------------------------------------
// coordinate.rs:
//   Implementation of coordinate related functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::types::*;
use crate::constants::*;
use std::f64::consts::PI;

const COARSE_STEP: f64 = 30.0;
const COS_5: f64 = 0.99619469809174553;
const SIN_5: f64 = 0.087155742747658173559;
const REL_HUMI: f64 = 0.7;

/// GPS clock correction calculation
pub fn gps_clock_correction(eph: &GpsEphemeris, transmit_time: f64) -> f64 {
    let mut time_diff = transmit_time - eph.toc as f64;
    
    // protection for time ring back at week end
    if time_diff > 302400.0 {
        time_diff -= 604800.0;
    }
    if time_diff < -302400.0 {
        time_diff += 604800.0;
    }
    
    let mut clock_adj = eph.af0 + (eph.af1 + eph.af2 * time_diff) * time_diff;
    clock_adj *= 1.0 - eph.af1; // adjustment to time
    
    clock_adj
}

/// GLONASS clock correction calculation
pub fn glonass_clock_correction(eph: &GlonassEphemeris, transmit_time: f64) -> f64 {
    let mut time_diff = transmit_time - eph.tb as f64;
    
    if time_diff > 43200.0 {
        time_diff -= 86400.0;
    } else if time_diff < -43200.0 {
        time_diff += 86400.0;
    }
    
    -eph.tn + eph.gamma * time_diff
}

/// GPS satellite position and velocity calculation from ephemeris
pub fn gps_sat_pos_speed_eph(
    system: GnssSystem,
    transmit_time: f64,
    eph: &mut GpsEphemeris,
    pos_vel: &mut KinematicInfo,
    mut acc: Option<&mut [f64; 3]>,
) -> bool {
    // calculate time difference
    let mut delta_t = transmit_time - eph.toe as f64;
    
    // protection for time ring back at week end
    if delta_t > 302400.0 {
        delta_t -= 604800.0;
    }
    if delta_t < -302400.0 {
        delta_t += 604800.0;
    }
    
    // get Ek from Mk with recursive algorithm
    let alpha = eph.delta_n_dot * delta_t;
    let mk = eph.M0 + (eph.n + alpha / 2.0) * delta_t;
    let mut ek = mk;
    let mut ek1 = ek;
    
    for _ in 0..10 {
        ek = mk + eph.ecc * ek.sin();
        if (ek - ek1).abs() < 1e-14 {
            break;
        }
        ek1 = ek;
    }
    eph.Ek = ek;
    
    // assign ek1 as 1-e*cos(Ek)
    ek1 = 1.0 - (eph.ecc * ek.cos());
    
    // get phi(k) with atan2
    let phi = (eph.root_ecc * ek.sin()).atan2(ek.cos() - eph.ecc) + eph.w;
    let sin_temp = (phi + phi).sin();
    let cos_temp = (phi + phi).cos();
    
    // get u(k), r(k) and i(k)
    let mut uk = phi;
    let mut rk = (eph.axis + eph.axis_dot * delta_t) * ek1;
    let mut ik = eph.i0 + (eph.idot * delta_t);
    
    // apply 2nd order correction to u(k), r(k) and i(k)
    let duk = (eph.cuc * cos_temp) + (eph.cus * sin_temp);
    let drk = (eph.crc * cos_temp) + (eph.crs * sin_temp);
    let dik = (eph.cic * cos_temp) + (eph.cis * sin_temp);
    uk += duk;
    rk += drk;
    ik += dik;
    
    // calculate derivatives of r(k) and u(k)
    eph.Ek_dot = (eph.n + alpha) / ek1;
    let mut uk_dot = eph.Ek_dot * eph.root_ecc / ek1;
    let phi_dot = uk_dot * 2.0;
    let mut rk_dot = eph.axis * eph.ecc * ek.sin() * eph.Ek_dot + eph.axis_dot * ek1;
    let drk_dot = ((eph.crs * cos_temp) - (eph.crc * sin_temp)) * phi_dot;
    let duk_dot = ((eph.cus * cos_temp) - (eph.cuc * sin_temp)) * phi_dot;
    let dik_dot = ((eph.cis * cos_temp) - (eph.cic * sin_temp)) * phi_dot;
    rk_dot += drk_dot;
    uk_dot += duk_dot;
    let mut ik_dot = eph.idot + dik_dot;
    
    // calculate intermediate variables for acceleration
    let (rk_dot2, uk_dot2, ik_dot2) = if acc.is_some() {
        let ek_dot2 = -eph.Ek_dot * eph.Ek_dot * eph.ecc * ek.sin() / ek1;
        let phi_dot2 = 2.0 * ek_dot2 * eph.root_ecc / ek1;
        let alpha_acc = 2.0 * phi_dot2 / phi_dot; // phi_dot2/phi_dot
        let beta = phi_dot * phi_dot; // 4*phi_dot^2
        let rk_dot2 = eph.axis * eph.ecc * (ek.sin() * ek_dot2 + ek.cos() * eph.Ek_dot * eph.Ek_dot)
            + alpha_acc * drk_dot - beta * drk;
        let uk_dot2 = phi_dot2 + alpha_acc * duk_dot - beta * duk;
        let ik_dot2 = alpha_acc * dik_dot - beta * dik;
        (rk_dot2, uk_dot2, ik_dot2)
    } else {
        (0.0, 0.0, 0.0)
    };
    
    // calculate Xp and Yp and corresponding derivatives
    let sin_temp = uk.sin();
    let cos_temp = uk.cos();
    let xp = rk * cos_temp;
    let yp = rk * sin_temp;
    let xp_dot = rk_dot * cos_temp - yp * uk_dot;
    let yp_dot = rk_dot * sin_temp + xp * uk_dot;
    
    // calculate intermediate variables for acceleration
    let (xp_dot2, yp_dot2) = if acc.is_some() {
        let xp_dot2 = rk_dot2 * uk.cos() - 2.0 * uk_dot * rk_dot * uk.sin() - uk_dot * uk_dot * xp - uk_dot2 * yp;
        let yp_dot2 = rk_dot2 * uk.sin() + 2.0 * uk_dot * rk_dot * uk.cos() - uk_dot * uk_dot * yp + uk_dot2 * xp;
        (xp_dot2, yp_dot2)
    } else {
        (0.0, 0.0)
    };
    
    // get final position and speed in ECEF coordinate
    let omega = eph.omega_t + eph.omega_delta * delta_t;
    let sin_temp = omega.sin();
    let cos_temp = omega.cos();
    let phi_sin = ik.sin();
    pos_vel.z = yp * phi_sin;
    pos_vel.vz = yp_dot * phi_sin;
    
    let phi_cos = ik.cos();
    pos_vel.x = xp * cos_temp - yp * phi_cos * sin_temp;
    pos_vel.y = xp * sin_temp + yp * phi_cos * cos_temp;
    
    // phi_dot assign as yp_dot * cos(ik) - z * ik_dot
    let phi_dot_final = yp_dot * phi_cos - pos_vel.z * ik_dot;
    pos_vel.vx = xp_dot * cos_temp - phi_dot_final * sin_temp;
    pos_vel.vy = xp_dot * sin_temp + phi_dot_final * cos_temp;
    pos_vel.vx -= pos_vel.y * eph.omega_delta;
    pos_vel.vy += pos_vel.x * eph.omega_delta;
    pos_vel.vz += yp * ik_dot * phi_cos;
    
    // calculate acceleration if given valid array pointer
    if acc.is_some() {
        let acc_array = acc.as_mut().unwrap();
        let alpha_final = pos_vel.vz * ik_dot + pos_vel.z * ik_dot2 - xp_dot * eph.omega_delta
            + yp_dot * ik_dot * phi_sin - yp_dot2 * phi_cos;
        let beta_final = xp_dot2 + pos_vel.z * ik_dot * eph.omega_delta - yp_dot * eph.omega_delta * phi_cos;
        acc_array[0] = -pos_vel.vy * eph.omega_delta + alpha_final * sin_temp + beta_final * cos_temp;
        acc_array[1] = pos_vel.vx * eph.omega_delta - alpha_final * cos_temp + beta_final * sin_temp;
        acc_array[2] = (yp_dot2 - yp * ik_dot * ik_dot) * phi_sin + (yp * ik_dot2 + 2.0 * yp_dot * ik_dot) * phi_cos;
    }
    
    // BDS GEO satellite special handling
    if system == GnssSystem::BdsSystem && eph.svid <= 5 {
        // first rotate -5 degree
        let yp_temp = pos_vel.y * COS_5 - pos_vel.z * SIN_5; // rotated y
        pos_vel.z = pos_vel.z * COS_5 + pos_vel.y * SIN_5; // rotated z
        let yp_dot_temp = pos_vel.vy * COS_5 - pos_vel.vz * SIN_5; // rotated vy
        pos_vel.vz = pos_vel.vz * COS_5 + pos_vel.vy * SIN_5; // rotated vz
        
        // rotate delta_t * CGCS2000_OMEGDOTE
        let omega_rot = CGCS2000_OMEGDOTE * delta_t;
        let sin_temp = omega_rot.sin();
        let cos_temp = omega_rot.cos();
        pos_vel.y = yp_temp * cos_temp - pos_vel.x * sin_temp;
        pos_vel.x = pos_vel.x * cos_temp + yp_temp * sin_temp;
        pos_vel.vy = yp_dot_temp * cos_temp - pos_vel.vx * sin_temp;
        pos_vel.vx = pos_vel.vx * cos_temp + yp_dot_temp * sin_temp;
        
        // earth rotate compensation on velocity
        pos_vel.vx += pos_vel.y * CGCS2000_OMEGDOTE;
        pos_vel.vy -= pos_vel.x * CGCS2000_OMEGDOTE;
        
        if acc.is_some() {
            let acc_array = acc.as_mut().unwrap();
            // first rotate -5 degree
            let yp_acc = acc_array[1] * COS_5 - acc_array[2] * SIN_5; // rotated ay
            acc_array[2] = acc_array[2] * COS_5 + acc_array[1] * SIN_5; // rotated az
            acc_array[1] = yp_acc * cos_temp - acc_array[0] * sin_temp;
            acc_array[0] = acc_array[0] * cos_temp + yp_acc * sin_temp;
            
            // earth rotate compensation on acceleration
            acc_array[0] += pos_vel.vy * CGCS2000_OMEGDOTE;
            acc_array[1] -= pos_vel.vx * CGCS2000_OMEGDOTE;
        }
    }
    
    // if ephemeris expire, return false
    if delta_t.abs() > 7200.0 {
        false
    } else if delta_t.abs() > 3600.0 && system == GnssSystem::BdsSystem {
        false
    } else {
        true
    }
}

/// GLONASS satellite position and velocity calculation from ephemeris
pub fn glonass_sat_pos_speed_eph(
    transmit_time: f64,
    eph: &mut GlonassEphemeris,
    pos_vel: &mut KinematicInfo,
    acc: Option<&mut [f64; 3]>,
) -> bool {
    let mut delta_t = transmit_time - eph.tb as f64;
    
    if delta_t > 43200.0 {
        delta_t -= 86400.0;
    } else if delta_t < -43200.0 {
        delta_t += 86400.0;
    }
    
    let delta_t1 = if (eph.flag & 0x2) == 0 {
        // satellite position and velocity in CIS coordinate
        let mut state = [
            eph.x, eph.y, eph.z,
            eph.vx - PZ90_OMEGDOTE * eph.y,
            eph.vy + PZ90_OMEGDOTE * eph.x,
            eph.vz,
            eph.ax, eph.ay, eph.az,
        ];
        
        let step_number = (delta_t / COARSE_STEP) as i32;
        if step_number >= 0 {
            for _ in 0..step_number {
                runge_kutta(COARSE_STEP, &mut state);
            }
        } else {
            for _ in step_number..0 {
                runge_kutta(-COARSE_STEP, &mut state);
            }
        }
        
        // Update ephemeris state
        eph.PosVelT.x = state[0];
        eph.PosVelT.y = state[1];
        eph.PosVelT.z = state[2];
        eph.PosVelT.vx = state[3];
        eph.PosVelT.vy = state[4];
        eph.PosVelT.vz = state[5];
        
        delta_t - (step_number as f64) * COARSE_STEP
    } else {
        // prediction from tc
        let mut state = [
            eph.PosVelT.x, eph.PosVelT.y, eph.PosVelT.z,
            eph.PosVelT.vx, eph.PosVelT.vy, eph.PosVelT.vz,
            eph.ax, eph.ay, eph.az,
        ];
        
        let mut delta_t1 = transmit_time - eph.tc;
        if delta_t1 > 43200.0 {
            delta_t1 -= 86400.0;
        } else if delta_t1 < -43200.0 {
            delta_t1 += 86400.0;
        }
        
        runge_kutta(delta_t1, &mut state);
        
        // Update ephemeris state
        eph.PosVelT.x = state[0];
        eph.PosVelT.y = state[1];
        eph.PosVelT.z = state[2];
        eph.PosVelT.vx = state[3];
        eph.PosVelT.vy = state[4];
        eph.PosVelT.vz = state[5];
        
        delta_t1
    };
    
    eph.tc = transmit_time;
    eph.flag |= 0x2; // can predict from eph.tc instead of eph.tb
    
    // CIS to CTS(PZ-90) conversion
    cis_to_cts(&eph.PosVelT, delta_t, pos_vel, acc);
    
    true
}

/// Convert ECEF to LLA coordinates
pub fn ecef_to_lla(ecef_pos: &KinematicInfo) -> LlaPosition {
    let p = (ecef_pos.x * ecef_pos.x + ecef_pos.y * ecef_pos.y).sqrt();
    
    if p < 1e-10 {
        // north or south pole
        return LlaPosition {
            lon: 0.0,
            lat: PI / 2.0,
            alt: if ecef_pos.z > 0.0 {
                ecef_pos.z - WGS_AXIS_B
            } else {
                -ecef_pos.z - WGS_AXIS_B
            },
        };
    }
    
    let theta = (ecef_pos.z * WGS_AXIS_A / (p * WGS_AXIS_B)).atan();
    let lat = ((ecef_pos.z + WGS_E2_SQR * WGS_AXIS_B * theta.sin().powi(3))
        / (p - WGS_E1_SQR * WGS_AXIS_A * theta.cos().powi(3))).atan();
    let lon = ecef_pos.y.atan2(ecef_pos.x);
    
    let n = WGS_AXIS_A / (1.0 - WGS_E1_SQR * lat.sin() * lat.sin()).sqrt();
    let alt = p / lat.cos() - n;
    
    LlaPosition { lat, lon, alt }
}

/// Convert LLA to ECEF coordinates
pub fn lla_to_ecef(lla_pos: &LlaPosition) -> KinematicInfo {
    let n = WGS_AXIS_A / (1.0 - WGS_E1_SQR * lla_pos.lat.sin() * lla_pos.lat.sin()).sqrt();
    
    KinematicInfo {
        x: (n + lla_pos.alt) * lla_pos.lat.cos() * lla_pos.lon.cos(),
        y: (n + lla_pos.alt) * lla_pos.lat.cos() * lla_pos.lon.sin(),
        z: (n * (1.0 - WGS_E1_SQR) + lla_pos.alt) * lla_pos.lat.sin(),
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
    }
}

/// Calculate conversion matrix from ECEF position
pub fn calc_conv_matrix_from_ecef(position: &KinematicInfo) -> ConvertMatrix {
    let pos_lla = ecef_to_lla(position);
    calc_conv_matrix_lla(&pos_lla)
}

/// Calculate conversion matrix from LLA position
pub fn calc_conv_matrix_lla(position: &LlaPosition) -> ConvertMatrix {
    ConvertMatrix {
        x2e: -position.lon.sin(),
        y2e: position.lon.cos(),
        x2n: -position.lat.sin() * position.lon.cos(),
        y2n: -position.lat.sin() * position.lon.sin(),
        z2n: position.lat.cos(),
        x2u: position.lat.cos() * position.lon.cos(),
        y2u: position.lat.cos() * position.lon.sin(),
        z2u: position.lat.sin(),
    }
}

pub fn calc_up_vector(position: &LlaPosition) -> [f64; 3] {
    [
        position.lat.cos() * position.lon.cos(),
        position.lat.cos() * position.lon.sin(),
        position.lat.sin(),
    ]
}

/// Convert ENU speed to course and speed
pub fn speed_enu_to_course(speed: &mut LocalSpeed) {
    speed.speed = (speed.ve * speed.ve + speed.vn * speed.vn).sqrt();
    speed.course = speed.ve.atan2(speed.vn);
    if speed.course < 0.0 {
        speed.course += PI2;
    }
}

/// Convert course and speed to ENU speed
pub fn speed_course_to_enu(speed: &mut LocalSpeed) {
    speed.ve = speed.speed * speed.course.sin();
    speed.vn = speed.speed * speed.course.cos();
}

/// Convert ECEF speed to local speed
pub fn speed_ecef_to_local(
    convert_matrix: &ConvertMatrix,
    pos_vel: &KinematicInfo,
    speed: &mut LocalSpeed,
) {
    speed.ve = pos_vel.vx * convert_matrix.x2e + pos_vel.vy * convert_matrix.y2e;
    speed.vn = pos_vel.vx * convert_matrix.x2n + pos_vel.vy * convert_matrix.y2n + pos_vel.vz * convert_matrix.z2n;
    speed.vu = pos_vel.vx * convert_matrix.x2u + pos_vel.vy * convert_matrix.y2u + pos_vel.vz * convert_matrix.z2u;
    speed_enu_to_course(speed);
}

/// Convert local speed to ECEF speed
pub fn speed_local_to_ecef(
    convert_matrix: &ConvertMatrix,
    speed: &LocalSpeed,
    pos_vel: &mut KinematicInfo,
) {
    pos_vel.vx = speed.ve * convert_matrix.x2e + speed.vn * convert_matrix.x2n + speed.vu * convert_matrix.x2u;
    pos_vel.vy = speed.ve * convert_matrix.y2e + speed.vn * convert_matrix.y2n + speed.vu * convert_matrix.y2u;
    pos_vel.vz = speed.vn * convert_matrix.z2n + speed.vu * convert_matrix.z2u;
}

/// Convert local speed to ECEF speed using LLA position
pub fn speed_local_to_ecef_lla(
    lla_pos: &LlaPosition,
    speed: &LocalSpeed,
    pos_vel: &mut KinematicInfo,
) {
    let convert_matrix = calc_conv_matrix_lla(lla_pos);
    speed_local_to_ecef(&convert_matrix, speed, pos_vel);
}

/// Calculate satellite elevation and azimuth
pub fn sat_el_az_from_lla(
    position_lla: &LlaPosition,
    los_vector: &[f64; 3],
    elevation: &mut f64,
    azimuth: &mut f64,
) {
    let convert_matrix = calc_conv_matrix_lla(position_lla);
    let local_los = [
        los_vector[0] * convert_matrix.x2e + los_vector[1] * convert_matrix.y2e,
        los_vector[0] * convert_matrix.x2n + los_vector[1] * convert_matrix.y2n + los_vector[2] * convert_matrix.z2n,
        los_vector[0] * convert_matrix.x2u + los_vector[1] * convert_matrix.y2u + los_vector[2] * convert_matrix.z2u,
    ];
    
    *azimuth = local_los[0].atan2(local_los[1]);
    if *azimuth < 0.0 {
        *azimuth += PI2;
    }
    *elevation = local_los[2].asin();
}

/// Calculate satellite elevation and azimuth from receiver and satellite positions
pub fn sat_el_az_from_positions(
    receiver: &KinematicInfo,
    satellite: &KinematicInfo,
    elevation: &mut f64,
    azimuth: &mut f64,
) {
    let mut los_vector = [0.0; 3];
    let position = ecef_to_lla(receiver);
    
    let receiver_pos = [receiver.pos_vel()[0], receiver.pos_vel()[1], receiver.pos_vel()[2]];
    let satellite_pos = [satellite.pos_vel()[0], satellite.pos_vel()[1], satellite.pos_vel()[2]];
    geometry_distance_array(&receiver_pos, &satellite_pos, Some(&mut los_vector));
    sat_el_az_from_lla(&position, &los_vector, elevation, azimuth);
}

/// Calculate geometry distance between two positions
pub fn geometry_distance_array(
    receiver: &[f64; 3],
    satellite: &[f64; 3],
    los_vector: Option<&mut [f64; 3]>,
) -> f64 {
    let dx = satellite[0] - receiver[0];
    let dy = satellite[1] - receiver[1];
    let dz = satellite[2] - receiver[2];
    let mut r = (dx * dx + dy * dy + dz * dz).sqrt();
    
    // add earth rotate compensation
    r += (satellite[0] * receiver[1] - satellite[1] * receiver[0]) * (WGS_OMEGDOTE / LIGHT_SPEED);
    
    if let Some(los) = los_vector {
        los[0] = dx / r;
        los[1] = dy / r;
        los[2] = dz / r;
    }
    
    r
}

/// Calculate geometry distance between two kinematic info structures
pub fn geometry_distance(
    receiver: &KinematicInfo,
    satellite: &KinematicInfo,
    los_vector: Option<&mut [f64; 3]>,
) -> f64 {
    let receiver_pos = receiver.pos_vel();
    let satellite_pos = satellite.pos_vel();
    geometry_distance_array(
        &[receiver_pos[0], receiver_pos[1], receiver_pos[2]],
        &[satellite_pos[0], satellite_pos[1], satellite_pos[2]],
        los_vector
    )
}

/// Calculate satellite relative speed
pub fn sat_relative_speed(receiver: &KinematicInfo, satellite: &KinematicInfo) -> f64 {
    let dx = receiver.x - satellite.x;
    let dy = receiver.y - satellite.y;
    let dz = receiver.z - satellite.z;
    let dvx = receiver.vx - satellite.vx;
    let dvy = receiver.vy - satellite.vy;
    let dvz = receiver.vz - satellite.vz;
    let distance = (dx * dx + dy * dy + dz * dz).sqrt();
    
    (dx * dvx + dy * dvy + dz * dvz) / distance
}

/// Calculate GPS ionosphere delay
pub fn gps_iono_delay(
    iono_param: &IonoParam,
    time: f64,
    lat: f64,
    lon: f64,
    elevation: f64,
    azimuth: f64,
) -> f64 {
    let el = elevation / PI;
    let psi = 0.0137 / (el + 0.11) - 0.022;
    let mut lat_rad = lat / PI + psi * azimuth.cos();
    
    if lat_rad > 0.416 {
        lat_rad = 0.416;
    } else if lat_rad < -0.416 {
        lat_rad = -0.416;
    }
    
    let mut lon_rad = lon / PI + psi * azimuth.sin() / (lat_rad * PI).cos();
    lat_rad += 0.064 * ((lon_rad - 1.617) * PI).cos();
    
    let f = 1.0 + 16.0 * (0.53 - el).powi(3);
    let mut per = iono_param.b0 + (iono_param.b1 + (iono_param.b2 + iono_param.b3 * lat_rad) * lat_rad) * lat_rad;
    
    if per < 72000.0 {
        per = 72000.0;
    }
    
    let mut t = (43200.0 * lon_rad) + time;
    while t >= 86400.0 {
        t -= 86400.0;
    }
    while t < 0.0 {
        t += 86400.0;
    }
    
    let x = PI2 * (t - 50400.0) / per;
    let f_scaled = f * LIGHT_SPEED;
    
    if x >= 1.57 || x <= -1.57 {
        f_scaled * 5e-9
    } else {
        let amp = iono_param.a0 + (iono_param.a1 + (iono_param.a2 + iono_param.a3 * lat_rad) * lat_rad) * lat_rad;
        if amp < 0.0 {
            f_scaled * 5e-9
        } else {
            let x_sq = x * x;
            let x1 = 1.0 - x_sq / 2.0 + x_sq * x_sq / 24.0;
            f_scaled * (5e-9 + amp * x1)
        }
    }
}

/// Calculate troposphere delay
pub fn tropo_delay(lat: f64, altitude: f64, elevation: f64) -> f64 {
    const T0: f64 = 273.16 + 15.0; // average temperature at sea level
    
    if altitude < -100.0 || altitude > 1e4 || elevation <= 0.0 {
        return 0.0;
    }
    
    let altitude_adj = if altitude < 0.0 { 0.0 } else { altitude };
    
    let pressure = 1013.25 * (1.0 - 2.2557e-5 * altitude_adj).powf(5.2568);
    let t = T0 - 6.5e-3 * altitude_adj;
    let e = 6.108 * REL_HUMI * ((17.15 * t - 4684.0) / (t - 38.45)).exp();
    
    let z = PI / 2.0 - elevation;
    let trph = 0.0022767 * pressure / (1.0 - 0.00266 * (2.0 * lat).cos() - 0.00028 * altitude_adj / 1e3) / z.cos();
    let trpw = 0.002277 * (1255.0 / t + 0.05) * e / z.cos();
    
    trph + trpw
}

// Helper functions for GLONASS calculations

fn runge_kutta(h: f64, state: &mut [f64; 9]) {
    let mut state1 = [0.0; 9];
    let mut vel_acc1 = [0.0; 6];
    let mut vel_acc2 = [0.0; 6];
    let mut vel_acc3 = [0.0; 6];
    let mut vel_acc4 = [0.0; 6];
    
    state1[6] = state[6];
    state1[7] = state[7];
    state1[8] = state[8];
    
    calc_acceleration(state, &mut vel_acc1);
    predict_state(state, &mut state1, &vel_acc1, 0.5 * h);
    calc_acceleration(&state1, &mut vel_acc2);
    predict_state(state, &mut state1, &vel_acc2, 0.5 * h);
    calc_acceleration(&state1, &mut vel_acc3);
    predict_state(state, &mut state1, &vel_acc3, h);
    calc_acceleration(&state1, &mut vel_acc4);
    
    for i in 0..6 {
        vel_acc1[i] = (vel_acc1[i] + vel_acc4[i] + 2.0 * (vel_acc2[i] + vel_acc3[i])) / 6.0;
    }
    
    let mut new_state = [0.0; 9];
    predict_state(state, &mut new_state, &vel_acc1, h);
    for i in 0..9 {
        state[i] = new_state[i];
    }
}

fn calc_acceleration(state: &[f64; 9], acc: &mut [f64; 6]) {
    acc[0] = state[3];
    acc[1] = state[4];
    acc[2] = state[5];
    
    let r2 = state[0] * state[0] + state[1] * state[1] + state[2] * state[2];
    let r3 = r2 * r2.sqrt();
    let coef20 = PZ90_C20AE2 / r2;
    let scale20 = PZ90_GM * (1.5 * coef20 * (5.0 * state[2] * state[2] / r2 - 1.0) - 1.0) / r3;
    
    acc[3] = scale20 * state[0] + state[6]; // x acceleration
    acc[4] = scale20 * state[1] + state[7]; // y acceleration
    acc[5] = (scale20 - 3.0 * PZ90_GM * coef20 / r3) * state[2] + state[8]; // z acceleration
}

fn predict_state(state: &[f64; 9], state1: &mut [f64; 9], vel_acc: &[f64; 6], step: f64) {
    for i in 0..6 {
        state1[i] = state[i] + vel_acc[i] * step;
    }
}

fn cis_to_cts(
    state: &KinematicInfo,
    delta_t: f64,
    cts_pos: &mut KinematicInfo,
    mut acc: Option<&mut [f64; 3]>,
) {
    // calculate rotate angle between CIS and CTS
    let omega = PZ90_OMEGDOTE * delta_t;
    let cos_value = omega.cos();
    let sin_value = omega.sin();
    
    // calculate position
    cts_pos.x = state.x * cos_value + state.y * sin_value;
    cts_pos.y = state.y * cos_value - state.x * sin_value;
    cts_pos.z = state.z;
    
    // calculate velocity
    cts_pos.vx = state.vx * cos_value + state.vy * sin_value;
    cts_pos.vy = state.vy * cos_value - state.vx * sin_value;
    cts_pos.vz = state.vz;
    
    // calculate acceleration
    if let Some(acc_array) = acc.as_mut() {
        acc_array[0] = state.x * cos_value + state.y * sin_value; // assuming state has acceleration
        acc_array[1] = state.y * cos_value - state.x * sin_value;
        acc_array[2] = state.z;
    }
    
    // additional compensation on velocity/acceleration
    let cos_comp = cos_value * PZ90_OMEGDOTE;
    let sin_comp = sin_value * PZ90_OMEGDOTE;
    cts_pos.vx -= state.x * sin_comp - state.y * cos_comp;
    cts_pos.vy -= state.y * sin_comp + state.x * cos_comp;
    
    if let Some(acc_array) = acc.as_mut() {
        acc_array[0] -= state.vx * sin_comp - state.vy * cos_comp;
        acc_array[1] -= state.vy * sin_comp + state.vx * cos_comp;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_ecef_to_lla_conversion() {
        let ecef = KinematicInfo {
            x: 4000000.0,
            y: 3000000.0,
            z: 5000000.0,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
        };
        
        let lla = ecef_to_lla(&ecef);
        let ecef_back = lla_to_ecef(&lla);
        
        assert!((ecef.x - ecef_back.x).abs() < 1e-6);
        assert!((ecef.y - ecef_back.y).abs() < 1e-6);
        assert!((ecef.z - ecef_back.z).abs() < 1e-6);
    }
    
    #[test]
    fn test_geometry_distance() {
        let receiver = [0.0, 0.0, 0.0];
        let satellite = [1000.0, 0.0, 0.0];
        let mut los_vector = [0.0; 3];
        
        let distance = geometry_distance_array(&receiver, &satellite, Some(&mut los_vector));
        
        assert!((distance - 1000.0).abs() < 1e-6);
        assert!((los_vector[0] - 1.0).abs() < 1e-6);
        assert!(los_vector[1].abs() < 1e-6);
        assert!(los_vector[2].abs() < 1e-6);
    }
}
// Additional functions for trajectory support

