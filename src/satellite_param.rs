//! # Модуль параметров спутников
//!
//! Этот модуль содержит функции для расчета различных параметров спутников ГНСС.
//! Основные функции:
//! - Расчет видимости спутников для различных ГНСС систем
//! - Определение геометрических параметров (углы места, азимут)
//! - Вычисление доплеровского сдвига частоты
//! - Расчет времени распространения сигнала
//! - Коррекции на атмосферные задержки
//!
//! Модуль работает с эфемеридами и альманахами для точного определения
//! положения спутников и их параметров относительно приемника.

//----------------------------------------------------------------------
// satellite_param.rs:
//   Implementation of functions to calculate satellite parameters
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::constants::*;
use crate::coordinate::*;
use crate::powercontrol::{ElevationAdjust, SignalPower};
use crate::types::*;

// Constants
const USE_POSITION_PREDICTION: bool = false;

// SatelliteParam is defined in types.rs, so we import it instead of redefining

/// Get visible satellites for GPS/BDS/Galileo systems
pub fn get_visible_satellite(
    position: &KinematicInfo,
    time: &GnssTime,
    output_param: &OutputParam,
    system: GnssSystem,
    eph: &[Option<GpsEphemeris>],
    eph_visible: &mut [Option<GpsEphemeris>],
) -> usize {
    let mut sat_number = 0;

    for (i, eph_opt) in eph.iter().enumerate() {
        if let Some(eph_data) = eph_opt {
            // Check if ephemeris is valid
            if (eph_data.valid & 1) == 0 {
                continue;
            }

            // Check satellite health
            if eph_data.health != 0 {
                continue;
            }

            // Check mask out based on system
            match system {
                GnssSystem::GpsSystem => {
                    if output_param.GpsMaskOut & (1u32 << i) != 0 {
                        continue;
                    }
                }
                GnssSystem::BdsSystem => {
                    if output_param.BdsMaskOut & (1u64 << i) != 0 {
                        continue;
                    }
                }
                GnssSystem::GalileoSystem => {
                    if output_param.GalileoMaskOut & (1u64 << i) != 0 {
                        continue;
                    }
                }
                _ => continue,
            }

            // Calculate satellite position
            let mut sat_position = KinematicInfo::default();
            let satellite_time = time.MilliSeconds as f64 / 1000.0;

            if !gps_sat_pos_speed_eph(system, satellite_time, eph_data, &mut sat_position, None) {
                continue;
            }

            // Calculate elevation and azimuth
            let (elevation, _azimuth) = sat_el_az(position, &sat_position);

            // Check elevation mask
            if elevation < output_param.ElevationMask {
                continue;
            }

            // Add to visible list
            if sat_number < eph_visible.len() {
                eph_visible[sat_number] = Some(*eph_data);
                sat_number += 1;
            }
        }
    }

    sat_number
}

/// Get visible satellites for GLONASS system
pub fn get_glonass_visible_satellite(
    position: &KinematicInfo,
    time: &GlonassTime,
    output_param: &OutputParam,
    eph: &[Option<GlonassEphemeris>],
    eph_visible: &mut [Option<GlonassEphemeris>],
) -> usize {
    let mut sat_number = 0;

    for (i, eph_opt) in eph.iter().enumerate() {
        if let Some(eph_data) = eph_opt {
            // Check if ephemeris is valid
            if eph_data.flag == 0 {
                continue;
            }

            // Check mask out
            if output_param.GlonassMaskOut & (1u32 << i) != 0 {
                continue;
            }

            // Calculate satellite position
            let mut sat_position = KinematicInfo::default();
            let satellite_time = time.MilliSeconds as f64 / 1000.0;

            if !glonass_sat_pos_speed_eph(satellite_time, eph_data, &mut sat_position, None) {
                continue;
            }

            // Calculate elevation and azimuth
            let (elevation, _azimuth) = sat_el_az(position, &sat_position);

            // Check elevation mask
            if elevation < output_param.ElevationMask {
                continue;
            }

            // Add to visible list
            if sat_number < eph_visible.len() {
                eph_visible[sat_number] = Some(*eph_data);
                sat_number += 1;
            }
        }
    }

    sat_number
}

/// Calculate satellite parameters for GLONASS
pub fn get_glonass_satellite_param(
    position_ecef: &KinematicInfo,
    position_lla: &LlaPosition,
    time: &GlonassTime,
    glo_eph: &GlonassEphemeris,
    iono_param: &IonoParam,
    satellite_param: &mut SatelliteParam,
) {
    let mut sat_position = KinematicInfo::default();
    let satellite_time = time.MilliSeconds as f64 / 1000.0;

    satellite_param.system = GnssSystem::GlonassSystem;
    satellite_param.svid = glo_eph.n as i32;
    satellite_param.FreqID = glo_eph.freq as i32;

    // Calculate GLONASS satellite position
    glonass_sat_pos_speed_eph(satellite_time, glo_eph, &mut sat_position, None);

    let mut travel_time = geometry_distance(
        position_ecef,
        &sat_position,
        Some(&mut satellite_param.LosVector),
    ) / LIGHT_SPEED;

    // Correct for satellite motion during signal travel
    sat_position.x -= travel_time * sat_position.vx;
    sat_position.y -= travel_time * sat_position.vy;
    sat_position.z -= travel_time * sat_position.vz;

    travel_time = geometry_distance(
        position_ecef,
        &sat_position,
        Some(&mut satellite_param.LosVector),
    ) / LIGHT_SPEED;
    let corrected_satellite_time = satellite_time - travel_time;

    // Calculate accurate satellite position at transmit time
    glonass_sat_pos_speed_eph(corrected_satellite_time, glo_eph, &mut sat_position, None);

    let distance = geometry_distance(
        position_ecef,
        &sat_position,
        Some(&mut satellite_param.LosVector),
    );
    let (elevation, azimuth) = sat_el_az_from_los(&satellite_param.LosVector);

    // Calculate ionospheric delay
    satellite_param.IonoDelay = gps_iono_delay(
        iono_param,
        corrected_satellite_time,
        position_lla.lat,
        position_lla.lon,
        elevation,
        azimuth,
    );

    // Add tropospheric delay
    let total_distance = distance + tropo_delay(position_lla.lat, position_lla.alt, elevation);

    // Calculate travel time with GLONASS clock correction
    travel_time =
        total_distance / LIGHT_SPEED - glonass_clock_correction(glo_eph, corrected_satellite_time);

    // Set final parameters
    satellite_param.TravelTime = travel_time;
    satellite_param.Elevation = elevation;
    satellite_param.Azimuth = azimuth;

    // For GLONASS, use gamma (relative frequency bias) instead of af1
    satellite_param.RelativeSpeed =
        sat_relative_speed(position_ecef, &sat_position) - LIGHT_SPEED * glo_eph.gamma;
}

/// Calculate satellite parameters
pub fn get_satellite_param(
    position_ecef: &KinematicInfo,
    position_lla: &LlaPosition,
    time: &GnssTime,
    system: GnssSystem,
    eph: &GpsEphemeris,
    iono_param: &IonoParam,
    satellite_param: &mut SatelliteParam,
) {
    let mut sat_position = KinematicInfo::default();
    let mut adjusted_time = *time;

    satellite_param.system = system;

    // Adjust time based on system
    match system {
        GnssSystem::BdsSystem => {
            // Subtract leap second difference for BDS
            adjusted_time.MilliSeconds -= 14000;
        }
        GnssSystem::GlonassSystem => {
            // GLONASS time handling
            let seconds = (time.Week * 604800 + time.MilliSeconds / 1000) as u32;
            let leap_second = get_leap_second(seconds);
            adjusted_time.MilliSeconds -= leap_second * 1000;
        }
        _ => {}
    }

    let mut satellite_time =
        (adjusted_time.MilliSeconds as f64 + adjusted_time.SubMilliSeconds) / 1000.0;

    // Set satellite ID and frequency ID
    satellite_param.svid = eph.svid as i32;
    satellite_param.FreqID = 0; // Default for GPS/BDS/Galileo

    // First estimate of travel time
    if USE_POSITION_PREDICTION {
        get_sat_pos_vel(
            system,
            satellite_time,
            eph,
            satellite_param,
            &mut sat_position,
        );
    } else {
        gps_sat_pos_speed_eph(system, satellite_time, eph, &mut sat_position, None);
    }

    let mut travel_time = geometry_distance(
        position_ecef,
        &sat_position,
        Some(&mut satellite_param.LosVector),
    ) / LIGHT_SPEED;

    // Correct for satellite motion during signal travel
    sat_position.x -= travel_time * sat_position.vx;
    sat_position.y -= travel_time * sat_position.vy;
    sat_position.z -= travel_time * sat_position.vz;

    travel_time = geometry_distance(
        position_ecef,
        &sat_position,
        Some(&mut satellite_param.LosVector),
    ) / LIGHT_SPEED;
    satellite_time -= travel_time;

    // Calculate accurate satellite position at transmit time
    let time_diff = if USE_POSITION_PREDICTION {
        let time_diff = satellite_time - satellite_param.PosTimeTag as f64;
        sat_position.x = satellite_param.PosVel.x
            + (satellite_param.PosVel.vx + satellite_param.Acc[0] * time_diff * 0.5) * time_diff;
        sat_position.y = satellite_param.PosVel.y
            + (satellite_param.PosVel.vy + satellite_param.Acc[1] * time_diff * 0.5) * time_diff;
        sat_position.z = satellite_param.PosVel.z
            + (satellite_param.PosVel.vz + satellite_param.Acc[2] * time_diff * 0.5) * time_diff;
        sat_position.vx = satellite_param.PosVel.vx + satellite_param.Acc[0] * time_diff;
        sat_position.vy = satellite_param.PosVel.vy + satellite_param.Acc[1] * time_diff;
        sat_position.vz = satellite_param.PosVel.vz + satellite_param.Acc[2] * time_diff;
        time_diff
    } else {
        gps_sat_pos_speed_eph(system, satellite_time, eph, &mut sat_position, None);
        0.0
    };

    let distance = geometry_distance(
        position_ecef,
        &sat_position,
        Some(&mut satellite_param.LosVector),
    );
    let (elevation, azimuth) = sat_el_az_from_los(&satellite_param.LosVector);

    // Calculate ionospheric delay
    satellite_param.IonoDelay = gps_iono_delay(
        iono_param,
        satellite_time,
        position_lla.lat,
        position_lla.lon,
        elevation,
        azimuth,
    );

    // Add tropospheric delay
    let total_distance = distance + tropo_delay(position_lla.lat, position_lla.alt, elevation);

    // Calculate travel time with corrections
    travel_time = total_distance / LIGHT_SPEED - gps_clock_correction(eph, satellite_time);

    // Add relativistic correction
    travel_time -= 4.442807633e-10 * eph.ecc * eph.sqrtA * (eph.Ek + time_diff * eph.Ek_dot).sin(); // WGS84 F constant

    // Set group delays based on system
    satellite_param.GroupDelay = [0.0; 8];
    match system {
        GnssSystem::GpsSystem => {
            satellite_param.GroupDelay[SIGNAL_INDEX_L1CA] = eph.tgd;
            satellite_param.GroupDelay[SIGNAL_INDEX_L1C] = eph.tgd_ext[1];
            satellite_param.GroupDelay[SIGNAL_INDEX_L2C] = eph.tgd2;
            satellite_param.GroupDelay[SIGNAL_INDEX_L5] = eph.tgd_ext[3];
        }
        GnssSystem::BdsSystem => {
            satellite_param.GroupDelay[0] = eph.tgd_ext[1]; // B1C
            satellite_param.GroupDelay[1] = eph.tgd; // B1I
            satellite_param.GroupDelay[2] = eph.tgd2; // B2I
            satellite_param.GroupDelay[3] = 0.0; // B3I
            satellite_param.GroupDelay[4] = eph.tgd_ext[3]; // B2A
            satellite_param.GroupDelay[5] = eph.tgd_ext[4]; // B2B
            satellite_param.GroupDelay[6] = (eph.tgd_ext[3] + eph.tgd_ext[4]) / 2.0;
            // B2AB
        }
        GnssSystem::GalileoSystem => {
            satellite_param.GroupDelay[0] = eph.tgd; // E1
            satellite_param.GroupDelay[1] = eph.tgd_ext[2]; // E5A
            satellite_param.GroupDelay[2] = eph.tgd_ext[4]; // E5B
            satellite_param.GroupDelay[3] = (eph.tgd_ext[2] + eph.tgd_ext[4]) / 2.0; // E5
            satellite_param.GroupDelay[4] = eph.tgd_ext[4]; // E6
        }
        _ => {}
    }

    // Set final parameters
    satellite_param.TravelTime = travel_time;
    satellite_param.Elevation = elevation;
    satellite_param.Azimuth = azimuth;
    satellite_param.RelativeSpeed =
        sat_relative_speed(position_ecef, &sat_position) - LIGHT_SPEED * eph.af1;

    // КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ: Сохраняем рассчитанную позицию и скорость спутника
    satellite_param.PosVel = sat_position;
    satellite_param.PosTimeTag = satellite_time as i32;
}

/// Set satellite CN0 based on power list and elevation
pub fn get_satellite_cn0(
    power_list: &[SignalPower],
    default_cn0: f64,
    adjust: ElevationAdjust,
    satellite_param: &mut SatelliteParam,
) {
    let mut cn0 = default_cn0;

    // Find matching power entry
    for power in power_list {
        if (power.svid == satellite_param.svid || power.svid == 0)
            && power.system == satellite_param.system as i32
        {
            if power.cn0 < 0.0 {
                cn0 = default_cn0;
                match adjust {
                    ElevationAdjust::ElevationAdjustNone => {}
                    ElevationAdjust::ElevationAdjustSinSqrtFade => {
                        // ИСПРАВЛЕНИЕ: Используем sin(elevation) вместо sqrt(elevation)
                        // Elevation в радианах: 0° = 0, 90° = π/2
                        // При elevation = 0° → sin(0) = 0 → sqrt(0) = 0 → коррекция = -25 дБ
                        // При elevation = 90° → sin(π/2) = 1 → sqrt(1) = 1 → коррекция = 0 дБ
                        cn0 -= (1.0 - satellite_param.Elevation.sin().sqrt()) * 25.0;
                    }
                }
            } else {
                cn0 = power.cn0;
            }
            satellite_param.CN0 = (cn0 * 100.0 + 0.5) as i32;
            return;
        }
    }

    // No matching power entry found, use default CN0
    satellite_param.CN0 = (cn0 * 100.0 + 0.5) as i32;
}

/// Get ionospheric delay for different signals
pub fn get_iono_delay(iono_delay_l1: f64, system: GnssSystem, signal_index: usize) -> f64 {
    match system {
        GnssSystem::GpsSystem => {
            match signal_index {
                SIGNAL_INDEX_L1CA | SIGNAL_INDEX_L1C => iono_delay_l1,
                SIGNAL_INDEX_L2C => iono_delay_l1 * 1.6469444444444444, // (154/120)^2
                SIGNAL_INDEX_L5 => iono_delay_l1 * 1.7932703213610586,  // (154/115)^2
                _ => iono_delay_l1,
            }
        }
        GnssSystem::BdsSystem => {
            match signal_index {
                SIGNAL_INDEX_B1C => iono_delay_l1,
                SIGNAL_INDEX_B1I => iono_delay_l1 * 1.0184327918525377, // (1540/1526)^2
                SIGNAL_INDEX_B2I | SIGNAL_INDEX_B2B => iono_delay_l1 * 1.7032461936225223, // (154/118)^2
                SIGNAL_INDEX_B3I => iono_delay_l1 * 1.5424037460978148, // (154/124)^2
                SIGNAL_INDEX_B2A => iono_delay_l1 * 1.7932703213610586, // (154/115)^2
                6 => iono_delay_l1 * 1.7473889738252685,                // B2AB (154/116.5)^2
                _ => iono_delay_l1,
            }
        }
        GnssSystem::GalileoSystem => {
            match signal_index {
                SIGNAL_INDEX_E1 => iono_delay_l1,
                SIGNAL_INDEX_E5A => iono_delay_l1 * 1.7932703213610586, // (154/115)^2
                SIGNAL_INDEX_E5B => iono_delay_l1 * 1.7032461936225223, // (154/118)^2
                SIGNAL_INDEX_L5 => iono_delay_l1 * 1.7473889738252685,  // L5 (154/116.5)^2
                SIGNAL_INDEX_E6 => iono_delay_l1 * 1.517824,            // (154/125)^2
                _ => iono_delay_l1,
            }
        }
        GnssSystem::GlonassSystem => {
            match signal_index {
                SIGNAL_INDEX_G1 => iono_delay_l1 * 1.040594059405941, // (1575.42/1602)^2 for k=-7
                SIGNAL_INDEX_G2 => iono_delay_l1 * 1.5980663241899555, // (1575.42/1246)^2 for k=-7
                SIGNAL_INDEX_G3 => iono_delay_l1 * 1.7205547652916243, // (1575.42/1202.025)^2
                _ => iono_delay_l1,
            }
        }
        _ => iono_delay_l1,
    }
}

/// Get wavelength for different signals
pub fn get_wave_length(system: GnssSystem, signal_index: usize, freq_id: i32) -> f64 {
    match system {
        GnssSystem::GpsSystem => match signal_index {
            SIGNAL_INDEX_L1CA | SIGNAL_INDEX_L1C => LIGHT_SPEED / FREQ_GPS_L1,
            SIGNAL_INDEX_L2C => LIGHT_SPEED / FREQ_GPS_L2,
            SIGNAL_INDEX_L5 => LIGHT_SPEED / FREQ_GPS_L5,
            _ => LIGHT_SPEED / FREQ_GPS_L1,
        },
        GnssSystem::BdsSystem => match signal_index {
            SIGNAL_INDEX_B1C => LIGHT_SPEED / FREQ_BDS_B1C,
            SIGNAL_INDEX_B1I => LIGHT_SPEED / FREQ_BDS_B1I,
            SIGNAL_INDEX_B2I => LIGHT_SPEED / FREQ_BDS_B2I,
            SIGNAL_INDEX_B3I => LIGHT_SPEED / FREQ_BDS_B3I,
            SIGNAL_INDEX_B2A => LIGHT_SPEED / FREQ_BDS_B2A,
            SIGNAL_INDEX_B2B => LIGHT_SPEED / FREQ_BDS_B2B,
            _ => LIGHT_SPEED / FREQ_BDS_B1C,
        },
        GnssSystem::GalileoSystem => match signal_index {
            SIGNAL_INDEX_E1 => LIGHT_SPEED / FREQ_GAL_E1,
            SIGNAL_INDEX_E5A => LIGHT_SPEED / FREQ_GAL_E5A,
            SIGNAL_INDEX_E5B => LIGHT_SPEED / FREQ_GAL_E5B,
            SIGNAL_INDEX_L5 => LIGHT_SPEED / FREQ_GAL_E5,
            SIGNAL_INDEX_E6 => LIGHT_SPEED / FREQ_GAL_E6,
            _ => LIGHT_SPEED / FREQ_GAL_E1,
        },
        GnssSystem::GlonassSystem => {
            let freq = match signal_index {
                SIGNAL_INDEX_G1 => FREQ_GLO_G1 + 562500.0 * freq_id as f64,
                SIGNAL_INDEX_G2 => FREQ_GLO_G2 + 437500.0 * freq_id as f64,
                SIGNAL_INDEX_G3 => FREQ_GLO_G3, // G3 uses CDMA, not FDMA
                _ => FREQ_GLO_G1 + 562500.0 * freq_id as f64,
            };
            LIGHT_SPEED / freq
        }
        _ => LIGHT_SPEED / FREQ_GPS_L1,
    }
}

/// Get travel time including group delay and ionospheric delay
pub fn get_travel_time(satellite_param: &SatelliteParam, signal_index: usize) -> f64 {
    let array_index = map_signal_to_array_index(satellite_param.system, signal_index).unwrap_or(0); // Используем индекс 0 если не найден маппинг
    let mut travel_time = satellite_param.TravelTime + satellite_param.GroupDelay[array_index];
    travel_time += get_iono_delay(
        satellite_param.IonoDelay,
        satellite_param.system,
        signal_index,
    ) / LIGHT_SPEED;
    travel_time
}

/// Отображение глобальных индексов сигналов в локальные индексы массивов
fn map_signal_to_array_index(system: GnssSystem, signal_index: usize) -> Option<usize> {
    match system {
        GnssSystem::GpsSystem => {
            match signal_index {
                0..=7 => Some(signal_index), // GPS сигналы 0-7 отображаются напрямую
                _ => None,
            }
        }
        GnssSystem::BdsSystem => {
            match signal_index {
                8 => Some(0),  // SIGNAL_INDEX_B1C
                9 => Some(1),  // SIGNAL_INDEX_B1I
                10 => Some(2), // SIGNAL_INDEX_B2I
                11 => Some(3), // SIGNAL_INDEX_B3I
                12 => Some(4), // SIGNAL_INDEX_B2A
                13 => Some(5), // SIGNAL_INDEX_B2B
                14 => Some(6), // SIGNAL_INDEX_B2AB
                _ => None,
            }
        }
        GnssSystem::GalileoSystem => {
            match signal_index {
                16..=23 => Some(signal_index - 16), // Galileo сигналы 16-23 → 0-7
                _ => None,
            }
        }
        GnssSystem::GlonassSystem => {
            match signal_index {
                24..=31 => Some(signal_index - 24), // GLONASS сигналы 24-31 → 0-7
                _ => None,
            }
        }
        _ => None,
    }
}

/// Get carrier phase measurement
pub fn get_carrier_phase(satellite_param: &SatelliteParam, signal_index: usize) -> f64 {
    let array_index = map_signal_to_array_index(satellite_param.system, signal_index).unwrap_or(0); // Используем индекс 0 если не найден маппинг
    let mut travel_time = satellite_param.TravelTime + satellite_param.GroupDelay[array_index];
    travel_time = travel_time * LIGHT_SPEED
        - get_iono_delay(
            satellite_param.IonoDelay,
            satellite_param.system,
            signal_index,
        );
    travel_time / get_wave_length(satellite_param.system, signal_index, satellite_param.FreqID)
}

/// Get Doppler frequency
pub fn get_doppler(satellite_param: &SatelliteParam, signal_index: usize) -> f64 {
    -satellite_param.RelativeSpeed
        / get_wave_length(satellite_param.system, signal_index, satellite_param.FreqID)
}

/// Get transmit time from receiver time and travel time
pub fn get_transmit_time(receiver_time: &GnssTime, travel_time: f64) -> GnssTime {
    let mut transmit_time = *receiver_time;

    let travel_time_ms = travel_time * 1000.0;
    transmit_time.MilliSeconds -= travel_time_ms as i32;
    let fractional_ms = travel_time_ms - travel_time_ms.floor();
    transmit_time.SubMilliSeconds -= fractional_ms;

    if transmit_time.SubMilliSeconds < 0.0 {
        transmit_time.SubMilliSeconds += 1.0;
        transmit_time.MilliSeconds -= 1;
    }

    if transmit_time.MilliSeconds < 0 {
        transmit_time.MilliSeconds += 604800000; // Add one week in milliseconds
    }

    transmit_time
}

// Helper functions that need to be implemented or imported from coordinate module

/// Calculate satellite elevation and azimuth
fn sat_el_az(receiver_pos: &KinematicInfo, sat_pos: &KinematicInfo) -> (f64, f64) {
    // Convert to LLA for receiver position
    let receiver_lla = ecef_to_lla(receiver_pos);

    // Calculate LOS vector
    let los_x = sat_pos.x - receiver_pos.x;
    let los_y = sat_pos.y - receiver_pos.y;
    let los_z = sat_pos.z - receiver_pos.z;

    // Convert to local ENU coordinates
    let convert_matrix = calc_conv_matrix_lla(&receiver_lla);

    let e = convert_matrix.x2e * los_x + convert_matrix.y2e * los_y;
    let n = convert_matrix.x2n * los_x + convert_matrix.y2n * los_y + convert_matrix.z2n * los_z;
    let u = convert_matrix.x2u * los_x + convert_matrix.y2u * los_y + convert_matrix.z2u * los_z;

    let horizontal_distance = (e * e + n * n).sqrt();
    let elevation = u.atan2(horizontal_distance);
    let azimuth = e.atan2(n);

    (elevation, azimuth)
}

/// Calculate elevation and azimuth from LOS vector
fn sat_el_az_from_los(los_vector: &[f64; 3]) -> (f64, f64) {
    let horizontal_distance =
        (los_vector[0] * los_vector[0] + los_vector[1] * los_vector[1]).sqrt();
    let elevation = los_vector[2].atan2(horizontal_distance);
    let azimuth = los_vector[1].atan2(los_vector[0]);
    (elevation, azimuth)
}

// Calculate relative speed between receiver and satellite
// Use sat_relative_speed from coordinate.rs for consistent sign conventions

// Calculate geometry distance and LOS vector
// Use geometry_distance from coordinate.rs (includes Sagnac compensation)

// Placeholder functions - these should be implemented in coordinate module
/// Рассчитывает позицию и скорость спутника GPS/BeiDou/Galileo на основе эфемерид
/// Реализация алгоритма Кеплера с поправками для разных ГНСС систем
fn gps_sat_pos_speed_eph(
    system: GnssSystem,
    transmit_time: f64,
    eph: &GpsEphemeris,
    pos_vel: &mut KinematicInfo,
    mut acc: Option<&mut [f64; 3]>,
) -> bool {
    // Рассчет временной разности
    let mut delta_t = transmit_time - eph.toe as f64;

    // Защита от переполнения недели
    if delta_t > 302400.0 {
        delta_t -= 604800.0;
    }
    if delta_t < -302400.0 {
        delta_t += 604800.0;
    }

    // Получение Ek из Mk с помощью итеративного алгоритма
    let alpha = eph.delta_n_dot * delta_t;
    let mk = eph.M0 + (eph.n + alpha / 2.0) * delta_t;
    let mut ek = mk;
    let mut ek1 = mk;

    // Итеративное решение уравнения Кеплера: Ek = Mk + e*sin(Ek)
    for _ in 0..10 {
        ek = mk + eph.ecc * ek.sin();
        if (ek - ek1).abs() < 1e-14 {
            break;
        }
        ek1 = ek;
    }

    // Присваиваем Ek1 как 1-e*cos(Ek) для дальнейших вычислений
    ek1 = 1.0 - eph.ecc * ek.cos();

    // Получаем phi(k) с помощью atan2
    let phi = ((eph.root_ecc * ek.sin()).atan2(ek.cos() - eph.ecc)) + eph.w;
    let sin_2phi = (phi + phi).sin();
    let cos_2phi = (phi + phi).cos();

    // Получаем u(k), r(k) и i(k)
    let mut uk = phi;
    let mut rk = (eph.axis + eph.axis_dot * delta_t) * ek1;
    let mut ik = eph.i0 + eph.idot * delta_t;

    // Применяем поправки 2-го порядка к u(k), r(k) и i(k)
    let duk = eph.cuc * cos_2phi + eph.cus * sin_2phi;
    let drk = eph.crc * cos_2phi + eph.crs * sin_2phi;
    let dik = eph.cic * cos_2phi + eph.cis * sin_2phi;

    uk += duk;
    rk += drk;
    ik += dik;

    // Рассчитываем производные r(k) и u(k)
    let ek_dot = (eph.n + alpha) / ek1;
    let phi_dot = ek_dot * eph.root_ecc / ek1;
    let phi_dot_2 = phi_dot * 2.0;

    let rk_dot = eph.axis * eph.ecc * ek.sin() * ek_dot + eph.axis_dot * ek1;
    let drk_dot = (eph.crs * cos_2phi - eph.crc * sin_2phi) * phi_dot_2;
    let duk_dot = (eph.cus * cos_2phi - eph.cuc * sin_2phi) * phi_dot_2;
    let dik_dot = (eph.cis * cos_2phi - eph.cic * sin_2phi) * phi_dot_2;

    let rk_dot = rk_dot + drk_dot;
    let uk_dot = phi_dot + duk_dot;
    let ik_dot = eph.idot + dik_dot;

    // Рассчитываем ускорение если требуется
    if let Some(ref mut acc_array) = acc {
        let ek_dot2 = -ek_dot * ek_dot * eph.ecc * ek.sin() / ek1;
        let phi_dot2 = 2.0 * ek_dot2 * eph.root_ecc / ek1;
        let alpha_acc = 2.0 * phi_dot2 / phi_dot;
        let beta = phi_dot * phi_dot;

        let rk_dot2 = eph.axis * eph.ecc * (ek.sin() * ek_dot2 + ek.cos() * ek_dot * ek_dot)
            + alpha_acc * drk_dot
            - beta * drk;
        let uk_dot2 = phi_dot2 + alpha_acc * duk_dot - beta * duk;
        let ik_dot2 = alpha_acc * dik_dot - beta * dik;

        // Сохраняем для использования при расчете ускорения в ECEF
        // (Эти переменные будут использованы ниже)
        acc_array[0] = rk_dot2;
        acc_array[1] = uk_dot2;
        acc_array[2] = ik_dot2;
    }

    // Рассчитываем Xp и Yp и соответствующие производные
    let sin_uk = uk.sin();
    let cos_uk = uk.cos();
    let xp = rk * cos_uk;
    let yp = rk * sin_uk;
    let xp_dot = rk_dot * cos_uk - yp * uk_dot;
    let yp_dot = rk_dot * sin_uk + xp * uk_dot;

    // Рассчитываем промежуточные переменные для ускорения
    let (xp_dot2, yp_dot2) = if let Some(acc_ref) = &acc {
        let uk_dot2 = acc_ref[1]; // Используем сохраненное значение
        let rk_dot2 = acc_ref[0]; // Используем сохраненное значение
        let xp_dot2 =
            rk_dot2 * cos_uk - 2.0 * uk_dot * rk_dot * sin_uk - uk_dot * uk_dot * xp - uk_dot2 * yp;
        let yp_dot2 =
            rk_dot2 * sin_uk + 2.0 * uk_dot * rk_dot * cos_uk - uk_dot * uk_dot * yp + uk_dot2 * xp;
        (xp_dot2, yp_dot2)
    } else {
        (0.0, 0.0)
    };

    // Получаем финальную позицию и скорость в ECEF координатах
    let omega = eph.omega_t + eph.omega_delta * delta_t;
    let sin_omega = omega.sin();
    let cos_omega = omega.cos();
    let sin_ik = ik.sin();
    let cos_ik = ik.cos();

    pos_vel.z = yp * sin_ik;
    pos_vel.vz = yp_dot * sin_ik;

    pos_vel.x = xp * cos_omega - yp * cos_ik * sin_omega;
    pos_vel.y = xp * sin_omega + yp * cos_ik * cos_omega;

    let phi_dot_ecef = yp_dot * cos_ik - pos_vel.z * ik_dot;
    pos_vel.vx = xp_dot * cos_omega - phi_dot_ecef * sin_omega;
    pos_vel.vy = xp_dot * sin_omega + phi_dot_ecef * cos_omega;

    // Компенсация вращения Земли
    pos_vel.vx -= pos_vel.y * eph.omega_delta;
    pos_vel.vy += pos_vel.x * eph.omega_delta;
    pos_vel.vz += yp * ik_dot * cos_ik;

    // Рассчитываем ускорение если предоставлен валидный указатель массива
    if let Some(ref mut acc_array) = acc {
        let ik_dot2 = acc_array[2]; // Используем сохраненное значение
        let alpha_final = pos_vel.vz * ik_dot + pos_vel.z * ik_dot2 - xp_dot * eph.omega_delta
            + yp_dot * ik_dot * sin_ik
            - yp_dot2 * cos_ik;
        let beta_final =
            xp_dot2 + pos_vel.z * ik_dot * eph.omega_delta - yp_dot * eph.omega_delta * cos_ik;

        acc_array[0] =
            -pos_vel.vy * eph.omega_delta + alpha_final * sin_omega + beta_final * cos_omega;
        acc_array[1] =
            pos_vel.vx * eph.omega_delta - alpha_final * cos_omega + beta_final * sin_omega;
        acc_array[2] = (yp_dot2 - yp * ik_dot * ik_dot) * sin_ik
            + (yp * ik_dot2 + 2.0 * yp_dot * ik_dot) * cos_ik;
    }

    // Специальная обработка для BeiDou GEO спутников (svid <= 5)
    if system == GnssSystem::BdsSystem && eph.svid <= 5 {
        // Поворот на -5 градусов
        let cos_5 = 0.996_194_698_091_745_5; // cos(5°)
        let sin_5 = 0.087_155_742_747_658_18; // sin(5°)

        let yp_rotated = pos_vel.y * cos_5 - pos_vel.z * sin_5;
        pos_vel.z = pos_vel.z * cos_5 + pos_vel.y * sin_5;
        let yp_dot_rotated = pos_vel.vy * cos_5 - pos_vel.vz * sin_5;
        pos_vel.vz = pos_vel.vz * cos_5 + pos_vel.vy * sin_5;

        // Поворот на delta_t * CGCS2000_OMEGDOTE
        let cgcs2000_omegdote = 7.2921150e-5; // Константа для BeiDou
        let omega_rot = cgcs2000_omegdote * delta_t;
        let sin_rot = omega_rot.sin();
        let cos_rot = omega_rot.cos();

        pos_vel.y = yp_rotated * cos_rot - pos_vel.x * sin_rot;
        pos_vel.x = pos_vel.x * cos_rot + yp_rotated * sin_rot;
        pos_vel.vy = yp_dot_rotated * cos_rot - pos_vel.vx * sin_rot;
        pos_vel.vx = pos_vel.vx * cos_rot + yp_dot_rotated * sin_rot;

        // Компенсация вращения Земли для скорости
        pos_vel.vx += pos_vel.y * cgcs2000_omegdote;
        pos_vel.vy -= pos_vel.x * cgcs2000_omegdote;

        // Обработка ускорения для BeiDou GEO
        if let Some(acc_array) = acc {
            let yp_acc = acc_array[1] * cos_5 - acc_array[2] * sin_5;
            acc_array[2] = acc_array[2] * cos_5 + acc_array[1] * sin_5;
            acc_array[1] = yp_acc * cos_rot - acc_array[0] * sin_rot;
            acc_array[0] = acc_array[0] * cos_rot + yp_acc * sin_rot;

            acc_array[0] += 2.0 * pos_vel.vy * cgcs2000_omegdote;
            acc_array[1] -= 2.0 * pos_vel.vx * cgcs2000_omegdote;
        }
    }

    true
}

/// Рассчитывает позицию и скорость спутника ГЛОНАСС на основе эфемерид
/// Использует метод Рунге-Кутта 4-го порядка для точного численного интегрирования
/// с учетом возмущений J2 и вращения Земли
fn glonass_sat_pos_speed_eph(
    transmit_time: f64,
    eph: &GlonassEphemeris,
    pos_vel: &mut KinematicInfo,
    acc: Option<&mut [f64; 3]>,
) -> bool {
    const COARSE_STEP: f64 = 30.0; // шаг интегрирования в секундах

    let mut delta_t = transmit_time - eph.tb as f64;
    if delta_t > 43200.0 {
        delta_t -= 86400.0;
    } else if delta_t < -43200.0 {
        delta_t += 86400.0;
    }

    // Начальные условия из эфемерид ГЛОНАСС (в системе ПЗ-90)
    let mut state = [
        eph.x * 1000.0,
        eph.y * 1000.0,
        eph.z * 1000.0, // позиция в метрах
        eph.vx * 1000.0,
        eph.vy * 1000.0,
        eph.vz * 1000.0, // скорость в м/с
        eph.ax * 1000.0,
        eph.ay * 1000.0,
        eph.az * 1000.0, // ускорение в м/с²
    ];

    // Интегрирование методом Рунге-Кутта с фиксированным шагом
    let step_number = (delta_t / COARSE_STEP) as i32;
    if step_number >= 0 {
        for _ in 0..step_number {
            glonass_runge_kutta(COARSE_STEP, &mut state);
        }
    } else {
        for _ in step_number..0 {
            glonass_runge_kutta(-COARSE_STEP, &mut state);
        }
    }

    // Финальная коррекция для остатка времени
    let remainder_time = delta_t - (step_number as f64) * COARSE_STEP;
    if remainder_time.abs() > 1e-9 {
        glonass_runge_kutta(remainder_time, &mut state);
    }

    // Заполнение результирующих значений (переводим обратно в км и км/с)
    pos_vel.x = state[0] / 1000.0;
    pos_vel.y = state[1] / 1000.0;
    pos_vel.z = state[2] / 1000.0;
    pos_vel.vx = state[3] / 1000.0;
    pos_vel.vy = state[4] / 1000.0;
    pos_vel.vz = state[5] / 1000.0;

    // Возвращаем ускорение если требуется
    if let Some(acceleration) = acc {
        acceleration[0] = state[6] / 1000.0;
        acceleration[1] = state[7] / 1000.0;
        acceleration[2] = state[8] / 1000.0;
    }

    true
}

/// Реализация метода Рунге-Кутта 4-го порядка для интегрирования орбитального движения ГЛОНАСС
/// с учетом возмущений J2 и вращения Земли
fn glonass_runge_kutta(step: f64, state: &mut [f64; 9]) {
    let mut k1 = [0.0; 9];
    let mut k2 = [0.0; 9];
    let mut k3 = [0.0; 9];
    let mut k4 = [0.0; 9];
    let mut temp_state = [0.0; 9];

    // K1 = f(state)
    glonass_orbit_derivatives(state, &mut k1);

    // K2 = f(state + 0.5*step*K1)
    for i in 0..9 {
        temp_state[i] = state[i] + 0.5 * step * k1[i];
    }
    glonass_orbit_derivatives(&temp_state, &mut k2);

    // K3 = f(state + 0.5*step*K2)
    for i in 0..9 {
        temp_state[i] = state[i] + 0.5 * step * k2[i];
    }
    glonass_orbit_derivatives(&temp_state, &mut k3);

    // K4 = f(state + step*K3)
    for i in 0..9 {
        temp_state[i] = state[i] + step * k3[i];
    }
    glonass_orbit_derivatives(&temp_state, &mut k4);

    // Финальное обновление: state = state + (step/6)*(K1 + 2*K2 + 2*K3 + K4)
    for i in 0..9 {
        state[i] += (step / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
}

/// Вычисляет производные орбитального движения для системы дифференциальных уравнений ГЛОНАСС
/// Включает гравитационное поле, возмущения J2, и эффекты вращения Земли
fn glonass_orbit_derivatives(state: &[f64; 9], derivatives: &mut [f64; 9]) {
    use crate::constants::{PZ90_AE, PZ90_GM, PZ90_OMEGDOTE};
    const PZ90_J2: f64 = 1.0826257e-3; // коэффициент J2 (сплющенность Земли)

    let x = state[0];
    let y = state[1];
    let z = state[2];
    let vx = state[3];
    let vy = state[4];
    let vz = state[5];
    let ax = state[6]; // лунно-солнечные возмущения (из эфемерид)
    let ay = state[7];
    let az = state[8];

    // Расстояние от центра Земли
    let r = (x * x + y * y + z * z).sqrt();

    // Гравитационное ускорение (основной член)
    let mu_r3 = -PZ90_GM / (r * r * r);
    let ax_grav = mu_r3 * x;
    let ay_grav = mu_r3 * y;
    let az_grav = mu_r3 * z;

    // Возмущения J2 (сплющенность Земли)
    let r2 = r * r;
    let z2 = z * z;
    let j2_factor = 1.5 * PZ90_J2 * PZ90_GM * PZ90_AE * PZ90_AE / (r2 * r2 * r);

    let ax_j2 = j2_factor * x * (5.0 * z2 / r2 - 1.0);
    let ay_j2 = j2_factor * y * (5.0 * z2 / r2 - 1.0);
    let az_j2 = j2_factor * z * (5.0 * z2 / r2 - 3.0);

    // Центробежная и кориолисова силы от вращения Земли
    let omega2 = PZ90_OMEGDOTE * PZ90_OMEGDOTE;
    let ax_rot = omega2 * x + 2.0 * PZ90_OMEGDOTE * vy;
    let ay_rot = omega2 * y - 2.0 * PZ90_OMEGDOTE * vx;
    let az_rot = 0.0; // ось Z - ось вращения

    // Производные: [dx/dt, dy/dt, dz/dt, dvx/dt, dvy/dt, dvz/dt, dax/dt, day/dt, daz/dt]
    derivatives[0] = vx; // dx/dt = vx
    derivatives[1] = vy; // dy/dt = vy
    derivatives[2] = vz; // dz/dt = vz
    derivatives[3] = ax_grav + ax_j2 + ax_rot + ax; // dvx/dt
    derivatives[4] = ay_grav + ay_j2 + ay_rot + ay; // dvy/dt
    derivatives[5] = az_grav + az_j2 + az_rot + az; // dvz/dt
    derivatives[6] = 0.0; // лунно-солнечные возмущения считаем постоянными
    derivatives[7] = 0.0;
    derivatives[8] = 0.0;
}

/// Рассчитывает коррекцию часов ГЛОНАСС спутника
/// Линейный дрейф: -tn + gamma * (time - tb)
fn glonass_clock_correction(eph: &GlonassEphemeris, transmit_time: f64) -> f64 {
    let mut time_diff = transmit_time - eph.tb as f64;

    if time_diff > 43200.0 {
        time_diff -= 86400.0;
    } else if time_diff < -43200.0 {
        time_diff += 86400.0;
    }

    -eph.tn + eph.gamma * time_diff
}

/// Рассчитывает ионосферную задержку по модели Klobuchar
/// Применяется для GPS системы, подходит для BeiDou и Galileo
fn gps_iono_delay(
    iono_param: &IonoParam,
    time: f64,
    lat: f64,
    lon: f64,
    elevation: f64,
    azimuth: f64,
) -> f64 {
    use std::f64::consts::PI;

    let el = elevation / PI;
    let mut lat_semi = lat / PI;
    let mut lon_semi = lon / PI;

    // Рассчет подспутниковой широты и долготы
    let psi = 0.0137 / (el + 0.11) - 0.022;
    lat_semi += psi * azimuth.cos();

    // Ограничение широты
    lat_semi = lat_semi.clamp(-0.416, 0.416);

    lon_semi += psi * azimuth.sin() / (lat_semi * PI).cos();
    lat_semi += 0.064 * ((lon_semi - 1.617) * PI).cos();

    // Коэффициент отображения
    let f = 1.0 + 16.0 * (0.53 - el).powi(3);

    // Период
    let mut per = iono_param.b0
        + (iono_param.b1 + (iono_param.b2 + iono_param.b3 * lat_semi) * lat_semi) * lat_semi;
    if per < 72000.0 {
        per = 72000.0;
    }

    // Местное время
    let mut t = (43200.0 * lon_semi) + time;
    while t >= 86400.0 {
        t -= 86400.0;
    }
    while t < 0.0 {
        t += 86400.0;
    }

    let x = PI * 2.0 * (t - 50400.0) / per;
    let f_light = f * LIGHT_SPEED;

    if x >= 1.57 || x <= -1.57 {
        // Ночное время
        f_light * 5e-9
    } else {
        // Амплитуда
        let amp = iono_param.a0
            + (iono_param.a1 + (iono_param.a2 + iono_param.a3 * lat_semi) * lat_semi) * lat_semi;
        if amp < 0.0 {
            f_light * 5e-9
        } else {
            let x2 = x * x;
            let x1 = 1.0 - x2 / 2.0 + x2 * x2 / 24.0; // Полином Тейлора для cos(x)
            f_light * (5e-9 + amp * x1)
        }
    }
}

/// Рассчитывает тропосферную задержку по упрощенной модели Saastamoinen
/// Учитывает высоту, широту и угол места
fn tropo_delay(lat: f64, alt: f64, elevation: f64) -> f64 {
    const T0: f64 = 273.16 + 15.0; // средняя температура на уровне моря
    const P0: f64 = 1013.25; // среднее давление на уровне моря (hPa)
    const REL_HUMI: f64 = 0.7; // относительная влажность

    // Коррекция на высоту (экспоненциальное убывание атмосферы)
    let temp = T0 - 0.0065 * alt; // температурный градиент 6.5 K/км
    let pressure = P0 * (1.0 - 0.0065 * alt / T0).powf(5.225);

    // Давление водяного пара (упрощенная формула)
    let e = REL_HUMI * 6.11 * (17.502 * (temp - 273.16) / (temp - 32.19)).exp();

    // Коррекция на широту
    let lat_factor = 1.0 - 0.00266 * (2.0 * lat).cos() - 0.00000028 * alt;

    // Общая задержка в зените (в метрах)
    let zenith_delay = 0.0022768 * pressure / lat_factor
        + (0.0022768 / lat_factor) * (1255.0 / temp + 0.05) * e
        - 1.16e-7;

    // Коррекция на угол места (функция отображения)
    let sin_elev = elevation.sin();
    let mapping_function = 1.0 / (sin_elev + 0.00143 / (sin_elev + 0.0445));

    zenith_delay * mapping_function
}

/// Рассчитывает коррекцию часов GPS спутника
/// Использует полином второго порядка: af0 + af1*dt + af2*dt^2
fn gps_clock_correction(eph: &GpsEphemeris, transmit_time: f64) -> f64 {
    let mut time_diff = transmit_time - eph.toc as f64;

    // Защита от переполнения недели
    if time_diff > 302400.0 {
        time_diff -= 604800.0;
    }
    if time_diff < -302400.0 {
        time_diff += 604800.0;
    }

    let clock_adj = eph.af0 + (eph.af1 + eph.af2 * time_diff) * time_diff;
    clock_adj * (1.0 - eph.af1) // коррекция времени
}

/// Получает количество високосных секунд для заданного GPS времени в секундах с начала эпохи
/// Возвращает количество накопленных високосных секунд
fn get_leap_second(seconds: u32) -> i32 {
    // Таблица времен добавления високосных секунд (в GPS секундах с 6 января 1980)
    const INSERT_TIME: [u32; 18] = [
        46828800,   // 1981-07-01
        78364801,   // 1982-07-01
        109900802,  // 1983-07-01
        173059203,  // 1985-07-01
        252028804,  // 1988-01-01
        315187205,  // 1990-01-01
        346723206,  // 1991-01-01
        393984007,  // 1992-07-01
        425520008,  // 1993-07-01
        457056009,  // 1994-07-01
        504489610,  // 1996-01-01
        551750411,  // 1997-07-01
        599184012,  // 1999-01-01
        820108813,  // 2006-01-01
        914803214,  // 2009-01-01
        1025136015, // 2012-07-01
        1119744016, // 2015-07-01
        1167264017, // 2017-01-01
    ];

    for (i, &insert_time) in INSERT_TIME.iter().enumerate() {
        if seconds <= insert_time {
            return i as i32;
        }
    }

    // Если время больше последней записи в таблице, возвращаем максимальное количество
    INSERT_TIME.len() as i32
}

/// Calculate satellite position and velocity with prediction
/// This is the static function GetSatPosVel from C++
pub fn get_sat_pos_vel(
    system: GnssSystem,
    satellite_time: f64,
    eph: &GpsEphemeris,
    satellite_param: &mut SatelliteParam,
    pos_vel: &mut KinematicInfo,
) {
    let mut time_diff = satellite_time - satellite_param.PosTimeTag as f64;

    // Compensate week round
    if time_diff > 600000.0 {
        time_diff -= 604800.0;
    } else if time_diff < -600000.0 {
        time_diff += 604800.0;
    }

    if satellite_param.PosTimeTag >= 0 && time_diff.abs() <= 0.5 {
        // Do prediction using stored position, velocity and acceleration
        pos_vel.x = satellite_param.PosVel.x
            + (satellite_param.PosVel.vx + satellite_param.Acc[0] * time_diff * 0.5) * time_diff;
        pos_vel.y = satellite_param.PosVel.y
            + (satellite_param.PosVel.vy + satellite_param.Acc[1] * time_diff * 0.5) * time_diff;
        pos_vel.z = satellite_param.PosVel.z
            + (satellite_param.PosVel.vz + satellite_param.Acc[2] * time_diff * 0.5) * time_diff;
        pos_vel.vx = satellite_param.PosVel.vx + satellite_param.Acc[0] * time_diff;
        pos_vel.vy = satellite_param.PosVel.vy + satellite_param.Acc[1] * time_diff;
        pos_vel.vz = satellite_param.PosVel.vz + satellite_param.Acc[2] * time_diff;
    } else {
        // New calculation
        let time_tag = (satellite_time + 0.5) as i32;
        gps_sat_pos_speed_eph(
            system,
            time_tag as f64,
            eph,
            &mut satellite_param.PosVel,
            Some(&mut satellite_param.Acc),
        );
        satellite_param.PosTimeTag = time_tag;
        time_diff = satellite_time - time_tag as f64;

        if time_diff != 0.0 {
            pos_vel.x = satellite_param.PosVel.x
                + (satellite_param.PosVel.vx + satellite_param.Acc[0] * time_diff * 0.5)
                    * time_diff;
            pos_vel.y = satellite_param.PosVel.y
                + (satellite_param.PosVel.vy + satellite_param.Acc[1] * time_diff * 0.5)
                    * time_diff;
            pos_vel.z = satellite_param.PosVel.z
                + (satellite_param.PosVel.vz + satellite_param.Acc[2] * time_diff * 0.5)
                    * time_diff;
            pos_vel.vx = satellite_param.PosVel.vx + satellite_param.Acc[0] * time_diff;
            pos_vel.vy = satellite_param.PosVel.vy + satellite_param.Acc[1] * time_diff;
            pos_vel.vz = satellite_param.PosVel.vz + satellite_param.Acc[2] * time_diff;
        } else {
            *pos_vel = satellite_param.PosVel;
        }
    }
}

/// Enhanced get_satellite_param with position prediction support
#[allow(clippy::too_many_arguments)]
pub fn get_satellite_param_with_prediction(
    position_ecef: &KinematicInfo,
    position_lla: &LlaPosition,
    time: &GnssTime,
    system: GnssSystem,
    eph: &GpsEphemeris,
    iono_param: &IonoParam,
    satellite_param: &mut SatelliteParam,
    use_position_prediction: bool,
) {
    let mut sat_position = KinematicInfo::default();
    let mut adjusted_time = *time;

    satellite_param.system = system;

    // Handle GLONASS separately
    if system == GnssSystem::GlonassSystem {
        // For GLONASS, cast to GLONASS_EPHEMERIS
        // This would need proper GLONASS ephemeris handling
        // For now, use placeholder
        satellite_param.svid = 1; // Would be GloEph->n
        satellite_param.FreqID = 0; // Would be GloEph->freq
                                    // glonass_sat_pos_speed_eph(satellite_time, glo_eph, &sat_position, None);
        return;
    }

    // Adjust time based on system
    match system {
        GnssSystem::BdsSystem => {
            // Subtract leap second difference for BDS
            adjusted_time.MilliSeconds -= 14000;
        }
        GnssSystem::GlonassSystem => {
            // GLONASS time handling
            let seconds = (time.Week * 604800 + time.MilliSeconds / 1000) as u32;
            let leap_second = get_leap_second(seconds);
            adjusted_time.MilliSeconds -= leap_second * 1000;
        }
        _ => {}
    }

    let mut satellite_time =
        (adjusted_time.MilliSeconds as f64 + adjusted_time.SubMilliSeconds) / 1000.0;

    // Set satellite ID and frequency ID
    satellite_param.svid = eph.svid as i32;
    satellite_param.FreqID = 0; // Default for GPS/BDS/Galileo

    // First estimate of travel time
    if use_position_prediction {
        get_sat_pos_vel(
            system,
            satellite_time,
            eph,
            satellite_param,
            &mut sat_position,
        );
    } else {
        gps_sat_pos_speed_eph(system, satellite_time, eph, &mut sat_position, None);
    }

    let mut travel_time = geometry_distance(
        position_ecef,
        &sat_position,
        Some(&mut satellite_param.LosVector),
    ) / LIGHT_SPEED;

    // Correct for satellite motion during signal travel
    sat_position.x -= travel_time * sat_position.vx;
    sat_position.y -= travel_time * sat_position.vy;
    sat_position.z -= travel_time * sat_position.vz;

    travel_time = geometry_distance(
        position_ecef,
        &sat_position,
        Some(&mut satellite_param.LosVector),
    ) / LIGHT_SPEED;
    satellite_time -= travel_time;

    // Calculate accurate satellite position at transmit time
    let time_diff = if use_position_prediction {
        let mut pos_vel_temp = KinematicInfo::default();
        get_sat_pos_vel(
            system,
            satellite_time,
            eph,
            satellite_param,
            &mut pos_vel_temp,
        );
        sat_position = pos_vel_temp;
        satellite_time - satellite_param.PosTimeTag as f64
    } else {
        gps_sat_pos_speed_eph(system, satellite_time, eph, &mut sat_position, None);
        0.0
    };

    let distance = geometry_distance(
        position_ecef,
        &sat_position,
        Some(&mut satellite_param.LosVector),
    );
    let (elevation, azimuth) = sat_el_az_from_los(&satellite_param.LosVector);

    // Calculate ionospheric delay
    satellite_param.IonoDelay = gps_iono_delay(
        iono_param,
        satellite_time,
        position_lla.lat,
        position_lla.lon,
        elevation,
        azimuth,
    );

    // Add tropospheric delay
    let total_distance = distance + tropo_delay(position_lla.lat, position_lla.alt, elevation);

    // Calculate travel time with corrections
    if system == GnssSystem::GlonassSystem {
        // travel_time = total_distance / LIGHT_SPEED - glonass_clock_correction(glo_eph, satellite_time);
        travel_time = total_distance / LIGHT_SPEED; // Placeholder
    } else {
        travel_time = total_distance / LIGHT_SPEED - gps_clock_correction(eph, satellite_time);
        // Add relativistic correction with time difference for prediction
        travel_time -=
            4.442807633e-10 * eph.ecc * eph.sqrtA * (eph.Ek + time_diff * eph.Ek_dot).sin();
    }

    // Set group delays based on system (same as before)
    satellite_param.GroupDelay = [0.0; 8];
    match system {
        GnssSystem::GpsSystem => {
            satellite_param.GroupDelay[SIGNAL_INDEX_L1CA] = eph.tgd;
            satellite_param.GroupDelay[SIGNAL_INDEX_L1C] = eph.tgd_ext[1];
            satellite_param.GroupDelay[SIGNAL_INDEX_L2C] = eph.tgd2;
            satellite_param.GroupDelay[SIGNAL_INDEX_L5] = eph.tgd_ext[3];
        }
        GnssSystem::BdsSystem => {
            satellite_param.GroupDelay[0] = eph.tgd_ext[1]; // B1C
            satellite_param.GroupDelay[1] = eph.tgd; // B1I
            satellite_param.GroupDelay[2] = eph.tgd2; // B2I
            satellite_param.GroupDelay[3] = 0.0; // B3I
            satellite_param.GroupDelay[4] = eph.tgd_ext[3]; // B2A
            satellite_param.GroupDelay[5] = eph.tgd_ext[4]; // B2B
            satellite_param.GroupDelay[6] = (eph.tgd_ext[3] + eph.tgd_ext[4]) / 2.0;
            // B2AB
        }
        GnssSystem::GalileoSystem => {
            satellite_param.GroupDelay[0] = eph.tgd; // E1
            satellite_param.GroupDelay[1] = eph.tgd_ext[2]; // E5A
            satellite_param.GroupDelay[2] = eph.tgd_ext[4]; // E5B
            satellite_param.GroupDelay[3] = (eph.tgd_ext[2] + eph.tgd_ext[4]) / 2.0; // E5
            satellite_param.GroupDelay[4] = eph.tgd_ext[4]; // E6
        }
        _ => {}
    }

    // Set final parameters
    satellite_param.TravelTime = travel_time;
    satellite_param.Elevation = elevation;
    satellite_param.Azimuth = azimuth;

    // Calculate relative speed with proper clock drift correction
    if system == GnssSystem::GlonassSystem {
        // For GLONASS, use gamma (relative frequency bias) instead of af1
        // satellite_param.RelativeSpeed = sat_relative_speed(position_ecef, &sat_position) - LIGHT_SPEED * glo_eph.gamma;
        satellite_param.RelativeSpeed = sat_relative_speed(position_ecef, &sat_position);
    // Placeholder
    } else {
        satellite_param.RelativeSpeed =
            sat_relative_speed(position_ecef, &sat_position) - LIGHT_SPEED * eph.af1;
    }
}
