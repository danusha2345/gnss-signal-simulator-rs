//! # Модуль альманахов ГНСС
//!
//! Обработка и чтение альманахов для различных спутниковых навигационных систем.
//! Альманах содержит долгосрочные орбитальные параметры спутников для грубого
//! определения их положения и планирования наблюдений.
//!
//! ## Поддерживаемые форматы:
//! - GPS альманах (стандартный формат)
//! - ГЛОНАСС альманах  
//! - BeiDou альманах
//! - Galileo альманах
//!
//! ## Основные функции:
//! - Чтение альманахов из файлов
//! - Преобразование параметров орбит
//! - Расчет видимости спутников
//! - Валидация данных альманахов

use std::f64::consts::PI;
use std::fs::File;
use std::io::{BufRead, BufReader};

use crate::constants::*;
use crate::coordinate::{glonass_sat_pos_speed_eph, gps_sat_pos_speed_eph};
use crate::gnsstime::{utc_to_galileo_time, utc_to_glonass_time};
use crate::types::*;

/// Определяет тип альманаха по первой строке файла
///
/// Анализирует формат данных для различных GNSS систем:
/// - GPS: строки начинаются с '*' (стандартный формат SEM)
/// - Galileo: строки начинаются с '<' (XML формат)  
/// - ГЛОНАСС: определяется по формату даты XX.XX.XXXX
///
/// # Параметры
/// - `file`: Открытый файл для чтения
///
/// # Возвращает
/// Тип альманаха или AlmanacUnknown при ошибке определения
pub fn check_almanac_type(file: &mut File) -> AlmanacType {
    let mut reader = BufReader::new(file);
    let mut line = String::new();

    if reader.read_line(&mut line).is_err() {
        return AlmanacType::AlmanacUnknown;
    }

    if line.starts_with('*') {
        AlmanacType::AlmanacGps // Формат SEM для GPS
    } else if line.starts_with('<') {
        AlmanacType::AlmanacGalileo // XML формат для Galileo
    } else {
        // Проверяем формат ГЛОНАСС по второму параметру (дате)
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() < 2 {
            return AlmanacType::AlmanacUnknown;
        }

        let second_param = parts[1];
        // ГЛОНАСС дата имеет формат DD.MM.YYYY
        if second_param.len() >= 6
            && second_param.chars().nth(2) == Some('.')
            && second_param.chars().nth(5) == Some('.')
        {
            AlmanacType::AlmanacGlonass
        } else {
            AlmanacType::AlmanacUnknown
        }
    }
}

/// Читает GPS альманах из файла формата SEM
///
/// Парсит файл альманаха GPS в стандартном формате SEM (Satellite Ephemeris Message).
/// Каждая запись спутника начинается со строки, содержащей '*'.
/// Альманах содержит упрощенные орбитальные параметры для долгосрочного планирования.
///
/// # Параметры  
/// - `file`: Файл альманаха для чтения
/// - `almanac`: Массив для хранения альманахов (32 спутника GPS)
///
/// # Возвращает
/// Количество успешно прочитанных альманахов спутников
pub fn read_almanac_gps(mut file: File, almanac: &mut [GpsAlmanac; 32]) -> i32 {
    let mut reader = BufReader::new(&mut file);
    let mut line = String::new();
    let mut alm_count = 0;

    while let Ok(bytes_read) = reader.read_line(&mut line) {
        if bytes_read == 0 {
            break;
        }

        if line.starts_with('*') {
            if let Some(svid) = get_almanac_gps(&mut reader) {
                if svid.svid > 0 && svid.svid <= 32 {
                    alm_count += 1;
                    let idx = (svid.svid - 1) as usize;
                    if (almanac[idx].valid & 1) == 0 || svid.week > almanac[idx].week {
                        almanac[idx] = svid;
                    }
                }
            }
        }
        line.clear();
    }

    alm_count
}

pub fn get_almanac_gps(reader: &mut BufReader<&mut File>) -> Option<GpsAlmanac> {
    let mut alm = GpsAlmanac::default();
    let mut line = String::new();

    // Read SVID
    if reader.read_line(&mut line).is_err() {
        return None;
    }
    if line.len() < 28 {
        return None;
    }
    alm.svid = line[27..].trim().parse().ok()?;
    line.clear();

    // Read Health
    if reader.read_line(&mut line).is_err() {
        return None;
    }
    if line.len() < 28 {
        return None;
    }
    alm.health = line[27..].trim().parse().ok()?;
    line.clear();

    // Read Eccentricity
    if reader.read_line(&mut line).is_err() {
        return None;
    }
    if line.len() < 28 {
        return None;
    }
    alm.ecc = line[27..].trim().parse().ok()?;
    line.clear();

    // Read TOA
    if reader.read_line(&mut line).is_err() {
        return None;
    }
    if line.len() < 28 {
        return None;
    }
    alm.toa = line[27..].trim().parse().ok()?;
    line.clear();

    // Read Inclination
    if reader.read_line(&mut line).is_err() {
        return None;
    }
    if line.len() < 28 {
        return None;
    }
    alm.i0 = line[27..].trim().parse().ok()?;
    line.clear();

    // Read Omega dot
    if reader.read_line(&mut line).is_err() {
        return None;
    }
    if line.len() < 28 {
        return None;
    }
    alm.omega_dot = line[27..].trim().parse().ok()?;
    line.clear();

    // Read sqrt(A)
    if reader.read_line(&mut line).is_err() {
        return None;
    }
    if line.len() < 28 {
        return None;
    }
    alm.sqrtA = line[27..].trim().parse().ok()?;
    line.clear();

    // Read Omega0
    if reader.read_line(&mut line).is_err() {
        return None;
    }
    if line.len() < 28 {
        return None;
    }
    alm.omega0 = line[27..].trim().parse().ok()?;
    line.clear();

    // Read w
    if reader.read_line(&mut line).is_err() {
        return None;
    }
    if line.len() < 28 {
        return None;
    }
    alm.w = line[27..].trim().parse().ok()?;
    line.clear();

    // Read M0
    if reader.read_line(&mut line).is_err() {
        return None;
    }
    if line.len() < 28 {
        return None;
    }
    alm.M0 = line[27..].trim().parse().ok()?;
    line.clear();

    // Read af0
    if reader.read_line(&mut line).is_err() {
        return None;
    }
    if line.len() < 28 {
        return None;
    }
    alm.af0 = line[27..].trim().parse().ok()?;
    line.clear();

    // Read af1
    if reader.read_line(&mut line).is_err() {
        return None;
    }
    if line.len() < 28 {
        return None;
    }
    alm.af1 = line[27..].trim().parse().ok()?;
    line.clear();

    // Read week
    if reader.read_line(&mut line).is_err() {
        return None;
    }
    if line.len() < 28 {
        return None;
    }
    alm.week = line[27..].trim().parse().ok()?;

    alm.health = 0;
    alm.valid = 1;

    if alm.svid == 0 || alm.svid > 32 {
        None
    } else {
        Some(alm)
    }
}

pub fn read_almanac_bds(mut file: File, almanac: &mut [GpsAlmanac; 63]) -> i32 {
    let mut reader = BufReader::new(&mut file);
    let mut alm_count = 0;

    while let Some(alm) = get_almanac_bds(&mut reader) {
        if alm.svid > 0 && alm.svid <= 63 {
            alm_count += 1;
            let idx = (alm.svid - 1) as usize;
            if (almanac[idx].valid & 1) == 0 || alm.week > almanac[idx].week {
                almanac[idx] = alm;
            }
        }
    }

    alm_count
}

pub fn get_almanac_bds(reader: &mut BufReader<&mut File>) -> Option<GpsAlmanac> {
    let mut line = String::new();
    if reader.read_line(&mut line).is_err() || line.trim().is_empty() {
        return None;
    }

    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() < 12 {
        return None;
    }

    let mut alm = GpsAlmanac {
        svid: parts[0].parse().ok()?,
        ecc: parts[1].parse().ok()?,
        toa: parts[2].parse().ok()?,
        i0: parts[3].parse().ok()?,
        omega_dot: parts[4].parse().ok()?,
        sqrtA: parts[5].parse().ok()?,
        omega0: parts[6].parse().ok()?,
        w: parts[7].parse().ok()?,
        M0: parts[8].parse().ok()?,
        af0: parts[9].parse().ok()?,
        af1: parts[10].parse().ok()?,
        week: parts[11].parse().ok()?,
        ..Default::default()
    };

    if alm.svid == 0 || alm.svid > 63 {
        return None;
    }

    alm.health = 0;
    alm.flag = if alm.sqrtA > 6000.0 {
        if alm.i0 > 0.5 {
            2
        } else {
            1
        }
    } else {
        3
    };
    alm.valid = 1;

    Some(alm)
}

pub fn read_almanac_galileo(mut file: File, almanac: &mut [GpsAlmanac; 36]) -> i32 {
    let mut reader = BufReader::new(&mut file);
    let mut line = String::new();
    let mut alm_count = 0;
    let mut utc_time = UtcTime::default();
    let mut time = GnssTime::default();

    // Find WN from header
    while let Ok(bytes_read) = reader.read_line(&mut line) {
        if bytes_read == 0 {
            break;
        }

        if line.contains("</header>") {
            break;
        }

        if let Some(start) = line.find("<issueDate>") {
            let date_str = &line[start + 11..];
            let parts: Vec<&str> = date_str.split('-').collect();
            if parts.len() >= 3 {
                if let (Ok(year), Ok(month), Ok(day)) = (
                    parts[0].parse::<i32>(),
                    parts[1].parse::<i32>(),
                    parts[2].parse::<i32>(),
                ) {
                    utc_time.Year = year;
                    utc_time.Month = month;
                    utc_time.Day = day;
                    utc_time.Hour = 0;
                    utc_time.Minute = 0;
                    utc_time.Second = 0.0;
                    time = utc_to_galileo_time(utc_time);
                }
            }
        }
        line.clear();
    }

    if time.Week < 0 {
        return 0;
    }

    while let Ok(bytes_read) = reader.read_line(&mut line) {
        if bytes_read == 0 {
            break;
        }

        if line.contains("<svAlmanac>") {
            if let Some(alm) = get_almanac_galileo(&mut reader, time.Week) {
                if alm.svid > 0 && alm.svid <= 36 {
                    alm_count += 1;
                    let idx = (alm.svid - 1) as usize;
                    if (almanac[idx].valid & 1) == 0 || alm.week > almanac[idx].week {
                        almanac[idx] = alm;
                    }
                }
            }
        }
        line.clear();
    }

    alm_count
}

pub fn get_almanac_galileo(reader: &mut BufReader<&mut File>, ref_week: i32) -> Option<GpsAlmanac> {
    let mut alm = GpsAlmanac::default();
    let mut svid = 0u8;
    let mut data = 0i32;
    let mut line = String::new();

    while let Ok(bytes_read) = reader.read_line(&mut line) {
        if bytes_read == 0 {
            break;
        }

        if line.contains("</svAlmanac>") {
            break;
        }

        if let Some(start) = line.find("<SVID>") {
            let value_str = &line[start + 6..];
            if let Some(end) = value_str.find('<') {
                svid = value_str[..end].trim().parse().unwrap_or(0);
            }
        } else if let Some(start) = line.find("<aSqRoot>") {
            let value_str = &line[start + 9..];
            if let Some(end) = value_str.find('<') {
                alm.sqrtA = value_str[..end].trim().parse().unwrap_or(0.0);
            }
        } else if let Some(start) = line.find("<ecc>") {
            let value_str = &line[start + 5..];
            if let Some(end) = value_str.find('<') {
                alm.ecc = value_str[..end].trim().parse().unwrap_or(0.0);
            }
        } else if let Some(start) = line.find("<deltai>") {
            let value_str = &line[start + 8..];
            if let Some(end) = value_str.find('<') {
                alm.i0 = value_str[..end].trim().parse().unwrap_or(0.0);
            }
        } else if let Some(start) = line.find("<omega0>") {
            let value_str = &line[start + 8..];
            if let Some(end) = value_str.find('<') {
                alm.omega0 = value_str[..end].trim().parse().unwrap_or(0.0);
            }
        } else if let Some(start) = line.find("<omegaDot>") {
            let value_str = &line[start + 10..];
            if let Some(end) = value_str.find('<') {
                alm.omega_dot = value_str[..end].trim().parse().unwrap_or(0.0);
            }
        } else if let Some(start) = line.find("<w>") {
            let value_str = &line[start + 3..];
            if let Some(end) = value_str.find('<') {
                alm.w = value_str[..end].trim().parse().unwrap_or(0.0);
            }
        } else if let Some(start) = line.find("<m0>") {
            let value_str = &line[start + 4..];
            if let Some(end) = value_str.find('<') {
                alm.M0 = value_str[..end].trim().parse().unwrap_or(0.0);
            }
        } else if let Some(start) = line.find("<af0>") {
            let value_str = &line[start + 5..];
            if let Some(end) = value_str.find('<') {
                alm.af0 = value_str[..end].trim().parse().unwrap_or(0.0);
            }
        } else if let Some(start) = line.find("<af1>") {
            let value_str = &line[start + 5..];
            if let Some(end) = value_str.find('<') {
                alm.af1 = value_str[..end].trim().parse().unwrap_or(0.0);
            }
        } else if let Some(start) = line.find("<iod>") {
            let value_str = &line[start + 5..];
            if let Some(end) = value_str.find('<') {
                data = value_str[..end].trim().parse().unwrap_or(0);
            }
        } else if let Some(start) = line.find("<t0a>") {
            let value_str = &line[start + 5..];
            if let Some(end) = value_str.find('<') {
                alm.toa = value_str[..end].trim().parse().unwrap_or(0);
            }
        } else if let Some(start) = line.find("<wna>") {
            let value_str = &line[start + 5..];
            if let Some(end) = value_str.find('<') {
                data = value_str[..end].trim().parse().unwrap_or(0);
            }
        }

        line.clear();
    }

    if (ref_week & 3) != data {
        return None;
    }

    alm.sqrtA += SQRT_A0;
    alm.i0 = alm.i0 * PI + NORMINAL_I0;
    alm.omega0 *= PI;
    alm.omega_dot *= PI;
    alm.w *= PI;
    alm.M0 *= PI;
    alm.week = ref_week;
    alm.svid = svid;
    alm.flag = data as u8;
    alm.health = 0;
    alm.valid = 1;

    Some(alm)
}

pub fn read_almanac_glonass(mut file: File, almanac: &mut [GlonassAlmanac; 24]) -> i32 {
    let mut reader = BufReader::new(&mut file);
    let mut alm_count = 0;

    while let Some((slot, alm)) = get_almanac_glonass(&mut reader) {
        if slot > 0 && slot <= 24 {
            alm_count += 1;
            let idx = (slot - 1) as usize;
            if (almanac[idx].flag & 1) == 0
                || (alm.leap_year * 1461 + alm.day)
                    > (almanac[idx].leap_year * 1461 + almanac[idx].day)
            {
                almanac[idx] = alm;
            }
        }
    }

    alm_count
}

pub fn get_almanac_glonass(reader: &mut BufReader<&mut File>) -> Option<(i32, GlonassAlmanac)> {
    let mut line = String::new();
    if reader.read_line(&mut line).is_err() || line.trim().is_empty() {
        return None;
    }

    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() < 13 {
        return None;
    }

    let slot: i32 = parts[0].parse().ok()?;
    if slot <= 0 || slot > 24 {
        return None;
    }

    // Parse date (dd.mm.yy format)
    let date_parts: Vec<&str> = parts[1].split('.').collect();
    if date_parts.len() != 3 {
        return None;
    }

    let utc_time = UtcTime {
        Day: date_parts[0].parse().ok()?,
        Month: date_parts[1].parse().ok()?,
        Year: date_parts[2].parse::<i32>().ok()? + 2000,
        Hour: 0,
        Minute: 0,
        Second: 0.0,
    };

    let glonass_time = utc_to_glonass_time(utc_time);

    let mut alm = GlonassAlmanac {
        t: parts[2].parse().ok()?,
        ..Default::default()
    };
    let period: f64 = parts[3].parse().ok()?;
    alm.ecc = parts[4].parse().ok()?;
    let inclination: f64 = parts[5].parse().ok()?;
    alm.lambda = parts[6].parse().ok()?;
    alm.w = parts[7].parse().ok()?;
    alm.clock_error = parts[8].parse().ok()?;
    let freq: i32 = parts[9].parse().ok()?;
    alm.dt = parts[10].parse().ok()?;

    alm.freq = freq as i8;
    alm.flag = 1;
    alm.leap_year = glonass_time.LeapYear as i16;
    alm.day = glonass_time.Day as i16;
    alm.di = (inclination - 63.0) / 180.0;
    alm.lambda /= 180.0;
    alm.w /= 180.0;
    alm.dt = period - 43200.0;

    Some((slot, alm))
}

pub fn norm_angle(mut angle: f64) -> f64 {
    while angle < -PI {
        angle += PI2;
    }
    while angle >= PI {
        angle -= PI2;
    }
    angle
}

pub fn get_almanac_from_ephemeris(eph: &GpsEphemeris, week: i32, toa: i32) -> GpsAlmanac {
    let mut alm = GpsAlmanac {
        valid: eph.valid & 1,
        health: eph.health as u8,
        svid: eph.svid,
        ..Default::default()
    };

    if alm.valid == 0 {
        return alm;
    }

    if eph.sqrtA > 6000.0 && eph.i0 < 0.5 {
        convert_almanac_from_ephemeris_geo(&mut alm, eph, week, toa);
    } else {
        convert_almanac_from_ephemeris(&mut alm, eph, week, toa);
    }

    alm.flag = if eph.sqrtA > 6000.0 {
        if eph.i0 > 0.5 {
            2
        } else {
            1
        }
    } else {
        3
    };

    alm
}

const INCLINATION_FACTOR: f64 = 0.946_515_487_891_825_8;

pub fn get_almanac_from_ephemeris_glonass(
    eph: &GlonassEphemeris,
    day: i32,
    leap_year: i32,
) -> GlonassAlmanac {
    let mut alm = GlonassAlmanac::default();
    let mut pos_vel = KinematicInfo::default();
    let mut t = eph.tb as f64 - eph.z / eph.vz;
    let mut iter = 5;

    // Calculate time of ascending
    while iter > 0 {
        let mut eph_mut = *eph;
        glonass_sat_pos_speed_eph(t, &mut eph_mut, &mut pos_vel, None);
        t -= pos_vel.z / pos_vel.vz;
        if pos_vel.z.abs() <= 1e-3 {
            break;
        }
        iter -= 1;
    }

    // Time and longitude of first ascending node
    alm.t = t;
    alm.lambda = pos_vel.y.atan2(pos_vel.x) / PI;

    // Velocity compensation from ECEF to inertial coordinate
    pos_vel.vx -= pos_vel.y * PZ90_OMEGDOTE;
    pos_vel.vy += pos_vel.x * PZ90_OMEGDOTE;

    // Calculate areal velocity vector
    let h = [
        pos_vel.y * pos_vel.vz - pos_vel.z * pos_vel.vy,
        pos_vel.z * pos_vel.vx - pos_vel.x * pos_vel.vz,
        pos_vel.x * pos_vel.vy - pos_vel.y * pos_vel.vx,
    ];

    // Inclination and longitude of ascending node
    let p = (h[0] * h[0] + h[1] * h[1]).sqrt();
    alm.di = (p / h[2]).atan() / PI - 0.35;

    // Calculate major-axis and eccentricity
    let h2 = h[0] * h[0] + h[1] * h[1] + h[2] * h[2];
    let p = h2 / PZ90_GM;
    let r = (pos_vel.x * pos_vel.x + pos_vel.y * pos_vel.y + pos_vel.z * pos_vel.z).sqrt();
    let v2 = pos_vel.vx * pos_vel.vx + pos_vel.vy * pos_vel.vy + pos_vel.vz * pos_vel.vz;
    let rv = pos_vel.x * pos_vel.vx + pos_vel.y * pos_vel.vy + pos_vel.z * pos_vel.vz;
    let a = 1.0 / (2.0 / r - v2 / PZ90_GM);
    let mut t = PI2 / (PZ90_GM / (a * a * a)).sqrt();
    let c20 = -PZ90_C20 * 1.5 * PZ90_AE2 / (p * p);
    t *= 1.0 + c20 * INCLINATION_FACTOR;
    alm.dt = t - 43200.0;
    alm.ecc = if a > p { (1.0 - p / a).sqrt() } else { 0.0 };

    if alm.t >= t {
        alm.t -= t;
        alm.lambda += (PZ90_OMEGDOTE / PI) * t;
        if alm.lambda > 1.0 {
            alm.lambda -= 2.0;
        }
    }

    // Calculate argument of perigee
    let e = rv.atan2((1.0 - r / a) * (a * PZ90_GM).sqrt());
    let root_ecc = (p / a).sqrt();
    alm.w = -(root_ecc * e.sin()).atan2(e.cos() - alm.ecc) / PI;
    alm.clock_error = eph.tn;

    alm.flag = 1;
    alm.freq = eph.freq;
    alm.leap_year = leap_year as i16;
    alm.day = day as i16;

    alm
}

pub fn convert_almanac_from_ephemeris(
    alm: &mut GpsAlmanac,
    eph: &GpsEphemeris,
    week: i32,
    toa: i32,
) -> bool {
    alm.toa = toa;
    alm.week = week;
    let dt = (week - eph.week) * 604800 + (toa - eph.toe);

    // Non-time-varying parameters
    alm.ecc = eph.ecc;
    alm.sqrtA = eph.sqrtA;
    alm.w = eph.w;
    alm.omega_dot = eph.omega_dot;
    alm.af1 = eph.af1;

    // Parameters adjusted with reference time change
    alm.M0 = norm_angle(eph.M0 + eph.n * dt as f64);
    alm.omega0 = norm_angle(eph.omega0 + eph.omega_dot * dt as f64);
    alm.i0 = eph.i0 + eph.idot * dt as f64;
    alm.af0 = eph.af0 + eph.af1 * dt as f64;

    true
}

pub fn convert_almanac_from_ephemeris_geo(
    alm: &mut GpsAlmanac,
    eph: &GpsEphemeris,
    week: i32,
    toa: i32,
) -> bool {
    let mut pos_vel = KinematicInfo::default();

    // Calculate satellite position and velocity at toe
    let mut eph_mut = *eph;
    gps_sat_pos_speed_eph(
        GnssSystem::BdsSystem,
        eph.toe as f64,
        &mut eph_mut,
        &mut pos_vel,
        None,
    );

    // Velocity compensation from ECEF to inertial coordinate
    pos_vel.vx -= pos_vel.y * CGCS2000_OMEGDOTE;
    pos_vel.vy += pos_vel.x * CGCS2000_OMEGDOTE;

    // Calculate areal velocity vector
    let h = [
        pos_vel.y * pos_vel.vz - pos_vel.z * pos_vel.vy,
        pos_vel.z * pos_vel.vx - pos_vel.x * pos_vel.vz,
        pos_vel.x * pos_vel.vy - pos_vel.y * pos_vel.vx,
    ];

    // Inclination and longitude of ascending node
    let p = (h[0] * h[0] + h[1] * h[1]).sqrt();
    alm.i0 = (p / h[2]).atan();
    alm.omega0 = h[0].atan2(-h[1]) + eph.toe as f64 * CGCS2000_OMEGDOTE;

    // Calculate major-axis and eccentricity
    let h2 = h[0] * h[0] + h[1] * h[1] + h[2] * h[2];
    let p = h2 / (CGCS2000_SQRT_GM * CGCS2000_SQRT_GM);
    let r = (pos_vel.x * pos_vel.x + pos_vel.y * pos_vel.y + pos_vel.z * pos_vel.z).sqrt();
    let v2 = pos_vel.vx * pos_vel.vx + pos_vel.vy * pos_vel.vy + pos_vel.vz * pos_vel.vz;
    let rv = pos_vel.x * pos_vel.vx + pos_vel.y * pos_vel.vy + pos_vel.z * pos_vel.vz;
    let a = 1.0 / (2.0 / r - v2 / (CGCS2000_SQRT_GM * CGCS2000_SQRT_GM));
    alm.sqrtA = a.sqrt();
    alm.ecc = if a > p { (1.0 - p / a).sqrt() } else { 0.0 };

    // Calculate mean anomaly at reference time
    let e_val = rv.atan2((1.0 - r / a) * alm.sqrtA * CGCS2000_SQRT_GM);
    alm.M0 = e_val - alm.ecc * e_val.sin();

    // Calculate true anomaly
    let root_ecc = (p / a).sqrt();
    let u = (pos_vel.z * h2.sqrt()).atan2(pos_vel.y * h[0] - pos_vel.x * h[1]);
    let w = u - (root_ecc * e_val.sin()).atan2(e_val.cos() - alm.ecc);
    alm.w = norm_angle(w);

    // Set parameters
    alm.toa = toa;
    alm.week = week;
    alm.omega_dot = eph.omega_dot;
    alm.af1 = eph.af1;

    let dt = (week - eph.week) * 604800 + (toa - eph.toe);
    alm.M0 = norm_angle(alm.M0 + eph.n * dt as f64);
    alm.omega0 = norm_angle(alm.omega0 + eph.omega_dot * dt as f64);
    alm.af0 = eph.af0 + eph.af1 * dt as f64;

    true
}

pub fn read_almanac(
    mut file: File,
    _system: GnssSystem,
    alm_gps: &mut [GpsAlmanac; 32],
    alm_bds: &mut [GpsAlmanac; 63],
    alm_gal: &mut [GpsAlmanac; 36],
    alm_glo: &mut [GlonassAlmanac; 24],
) -> i32 {
    let alm_type = check_almanac_type(&mut file);
    match alm_type {
        AlmanacType::AlmanacGps => read_almanac_gps(file, alm_gps),
        AlmanacType::AlmanacBds => read_almanac_bds(file, alm_bds),
        AlmanacType::AlmanacGalileo => read_almanac_galileo(file, alm_gal),
        AlmanacType::AlmanacGlonass => read_almanac_glonass(file, alm_glo),
        _ => 0,
    }
}
