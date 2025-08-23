//----------------------------------------------------------------------
// satellite_param.rs:
//   Implementation of functions to calculate satellite parameters
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::types::*;
use crate::constants::*;
use crate::coordinate::*;
use crate::ifdatagen::OutputParam;
use std::f64::consts::PI;

// Constants
const USE_POSITION_PREDICTION: bool = false;

// Elevation adjustment types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ElevationAdjust {
    ElevationAdjustNone = 0,
    ElevationAdjustSinSqrtFade = 1,
}

// Signal power structure
#[derive(Debug, Clone, Copy, Default)]
pub struct SignalPower {
    pub system: GnssSystem,
    pub svid: i32,
    pub cn0: f64,
}

// Update SatelliteParam to match C++ structure exactly
#[derive(Debug, Clone, Copy)]
pub struct SatelliteParam {
    pub system: GnssSystem,
    pub svid: i32,
    pub freq_id: i32,
    pub cn0: i32,
    pub pos_time_tag: i32,
    pub pos_vel: KinematicInfo,
    pub acc: [f64; 3],
    pub travel_time: f64,
    pub iono_delay: f64,
    pub group_delay: [f64; 8],
    pub elevation: f64,
    pub azimuth: f64,
    pub relative_speed: f64,
    pub los_vector: [f64; 3],
}

impl Default for SatelliteParam {
    fn default() -> Self {
        SatelliteParam {
            system: GnssSystem::GpsSystem,
            svid: 0,
            freq_id: 0,
            cn0: 0,
            pos_time_tag: -1,
            pos_vel: KinematicInfo::default(),
            acc: [0.0; 3],
            travel_time: 0.0,
            iono_delay: 0.0,
            group_delay: [0.0; 8],
            elevation: 0.0,
            azimuth: 0.0,
            relative_speed: 0.0,
            los_vector: [0.0; 3],
        }
    }
}

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
                    if output_param.gps_mask_out & (1u32 << i) != 0 {
                        continue;
                    }
                },
                GnssSystem::BdsSystem => {
                    if output_param.bds_mask_out & (1u64 << i) != 0 {
                        continue;
                    }
                },
                GnssSystem::GalileoSystem => {
                    if output_param.galileo_mask_out & (1u64 << i) != 0 {
                        continue;
                    }
                },
                _ => continue,
            }
            
            // Calculate satellite position
            let mut sat_position = KinematicInfo::default();
            let satellite_time = time.MilliSeconds as f64 / 1000.0;
            
            if !gps_sat_pos_speed_eph(system, satellite_time, eph_data, &mut sat_position, None) {
                continue;
            }
            
            // Calculate elevation and azimuth
            let (elevation, azimuth) = sat_el_az(position, &sat_position);
            
            // Check elevation mask
            if elevation < output_param.elevation_mask {
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
            if output_param.glonass_mask_out & (1u32 << i) != 0 {
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
            if elevation < output_param.elevation_mask {
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
    satellite_param.freq_id = glo_eph.freq as i32;
    
    // Calculate GLONASS satellite position
    glonass_sat_pos_speed_eph(satellite_time, glo_eph, &mut sat_position, None);
    
    let mut travel_time = geometry_distance(position_ecef, &sat_position, &mut satellite_param.los_vector) / LIGHT_SPEED;
    
    // Correct for satellite motion during signal travel
    sat_position.x -= travel_time * sat_position.vx;
    sat_position.y -= travel_time * sat_position.vy;
    sat_position.z -= travel_time * sat_position.vz;
    
    travel_time = geometry_distance(position_ecef, &sat_position, &mut satellite_param.los_vector) / LIGHT_SPEED;
    let corrected_satellite_time = satellite_time - travel_time;
    
    // Calculate accurate satellite position at transmit time
    glonass_sat_pos_speed_eph(corrected_satellite_time, glo_eph, &mut sat_position, None);
    
    let distance = geometry_distance(position_ecef, &sat_position, &mut satellite_param.los_vector);
    let (elevation, azimuth) = sat_el_az_from_los(&satellite_param.los_vector);
    
    // Calculate ionospheric delay
    satellite_param.iono_delay = gps_iono_delay(iono_param, corrected_satellite_time, position_lla.lat, position_lla.lon, elevation, azimuth);
    
    // Add tropospheric delay
    let total_distance = distance + tropo_delay(position_lla.lat, position_lla.alt, elevation);
    
    // Calculate travel time with GLONASS clock correction
    travel_time = total_distance / LIGHT_SPEED - glonass_clock_correction(glo_eph, corrected_satellite_time);
    
    // Set final parameters
    satellite_param.travel_time = travel_time;
    satellite_param.elevation = elevation;
    satellite_param.azimuth = azimuth;
    
    // For GLONASS, use gamma (relative frequency bias) instead of af1
    satellite_param.relative_speed = sat_relative_speed(position_ecef, &sat_position) - LIGHT_SPEED * glo_eph.gamma;
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
        },
        GnssSystem::GlonassSystem => {
            // GLONASS time handling
            let seconds = (time.Week * 604800 + time.MilliSeconds / 1000) as u32;
            let leap_second = get_leap_second(seconds);
            adjusted_time.MilliSeconds -= leap_second * 1000;
        },
        _ => {}
    }
    
    let mut satellite_time = (adjusted_time.MilliSeconds as f64 + adjusted_time.SubMilliSeconds) / 1000.0;
    
    // Set satellite ID and frequency ID
    satellite_param.svid = eph.svid as i32;
    satellite_param.freq_id = 0; // Default for GPS/BDS/Galileo
    
    // First estimate of travel time
    if USE_POSITION_PREDICTION {
        get_sat_pos_vel(system, satellite_time, eph, satellite_param, &mut sat_position);
    } else {
        gps_sat_pos_speed_eph(system, satellite_time, eph, &mut sat_position, None);
    }
    
    let mut travel_time = geometry_distance(position_ecef, &sat_position, &mut satellite_param.los_vector) / LIGHT_SPEED;
    
    // Correct for satellite motion during signal travel
    sat_position.x -= travel_time * sat_position.vx;
    sat_position.y -= travel_time * sat_position.vy;
    sat_position.z -= travel_time * sat_position.vz;
    
    travel_time = geometry_distance(position_ecef, &sat_position, &mut satellite_param.los_vector) / LIGHT_SPEED;
    satellite_time -= travel_time;
    
    // Calculate accurate satellite position at transmit time
    let time_diff = if USE_POSITION_PREDICTION {
        let time_diff = satellite_time - satellite_param.pos_time_tag as f64;
        sat_position.x = satellite_param.pos_vel.x + (satellite_param.pos_vel.vx + satellite_param.acc[0] * time_diff * 0.5) * time_diff;
        sat_position.y = satellite_param.pos_vel.y + (satellite_param.pos_vel.vy + satellite_param.acc[1] * time_diff * 0.5) * time_diff;
        sat_position.z = satellite_param.pos_vel.z + (satellite_param.pos_vel.vz + satellite_param.acc[2] * time_diff * 0.5) * time_diff;
        sat_position.vx = satellite_param.pos_vel.vx + satellite_param.acc[0] * time_diff;
        sat_position.vy = satellite_param.pos_vel.vy + satellite_param.acc[1] * time_diff;
        sat_position.vz = satellite_param.pos_vel.vz + satellite_param.acc[2] * time_diff;
        time_diff
    } else {
        gps_sat_pos_speed_eph(system, satellite_time, eph, &mut sat_position, None);
        0.0
    };
    
    let distance = geometry_distance(position_ecef, &sat_position, &mut satellite_param.los_vector);
    let (elevation, azimuth) = sat_el_az_from_los(&satellite_param.los_vector);
    
    // Calculate ionospheric delay
    satellite_param.iono_delay = gps_iono_delay(iono_param, satellite_time, position_lla.lat, position_lla.lon, elevation, azimuth);
    
    // Add tropospheric delay
    let total_distance = distance + tropo_delay(position_lla.lat, position_lla.alt, elevation);
    
    // Calculate travel time with corrections
    travel_time = total_distance / LIGHT_SPEED - gps_clock_correction(eph, satellite_time);
    
    // Add relativistic correction
    travel_time -= 4.442807633e-10 * eph.ecc * eph.sqrtA * (eph.Ek + time_diff * eph.Ek_dot).sin(); // WGS84 F constant
    
    // Set group delays based on system
    satellite_param.group_delay = [0.0; 8];
    match system {
        GnssSystem::GpsSystem => {
            satellite_param.group_delay[SIGNAL_INDEX_L1CA] = eph.tgd;
            satellite_param.group_delay[SIGNAL_INDEX_L1C] = eph.tgd_ext[1];
            satellite_param.group_delay[SIGNAL_INDEX_L2C] = eph.tgd2;
            satellite_param.group_delay[SIGNAL_INDEX_L5] = eph.tgd_ext[3];
        },
        GnssSystem::BdsSystem => {
            satellite_param.group_delay[SIGNAL_INDEX_B1C] = eph.tgd_ext[1];
            satellite_param.group_delay[SIGNAL_INDEX_B1I] = eph.tgd;
            satellite_param.group_delay[SIGNAL_INDEX_B2I] = eph.tgd2;
            satellite_param.group_delay[SIGNAL_INDEX_B3I] = 0.0;
            satellite_param.group_delay[SIGNAL_INDEX_B2A] = eph.tgd_ext[3];
            satellite_param.group_delay[SIGNAL_INDEX_B2B] = eph.tgd_ext[4];
            satellite_param.group_delay[SIGNAL_INDEX_B2A + 2] = (eph.tgd_ext[3] + eph.tgd_ext[4]) / 2.0; // B2AB
        },
        GnssSystem::GalileoSystem => {
            satellite_param.group_delay[SIGNAL_INDEX_E1] = eph.tgd;
            satellite_param.group_delay[SIGNAL_INDEX_E5A] = eph.tgd_ext[2];
            satellite_param.group_delay[SIGNAL_INDEX_E5B] = eph.tgd_ext[4];
            satellite_param.group_delay[3] = (eph.tgd_ext[2] + eph.tgd_ext[4]) / 2.0; // E5
            satellite_param.group_delay[SIGNAL_INDEX_E6] = eph.tgd_ext[4];
        },
        _ => {}
    }
    
    // Set final parameters
    satellite_param.travel_time = travel_time;
    satellite_param.elevation = elevation;
    satellite_param.azimuth = azimuth;
    satellite_param.relative_speed = sat_relative_speed(position_ecef, &sat_position) - LIGHT_SPEED * eph.af1;
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
        if (power.svid == satellite_param.svid || power.svid == 0) && power.system == satellite_param.system {
            if power.cn0 < 0.0 {
                cn0 = default_cn0;
                match adjust {
                    ElevationAdjust::ElevationAdjustNone => {},
                    ElevationAdjust::ElevationAdjustSinSqrtFade => {
                        cn0 -= (1.0 - satellite_param.elevation.sqrt()) * 25.0;
                    },
                }
            } else {
                cn0 = power.cn0;
            }
            satellite_param.cn0 = (cn0 * 100.0 + 0.5) as i32;
            break;
        }
    }
}

/// Get ionospheric delay for different signals
pub fn get_iono_delay(iono_delay_l1: f64, system: GnssSystem, signal_index: usize) -> f64 {
    match system {
        GnssSystem::GpsSystem => {
            match signal_index {
                SIGNAL_INDEX_L1CA | SIGNAL_INDEX_L1C => iono_delay_l1,
                SIGNAL_INDEX_L2C => iono_delay_l1 * 1.6469444444444444, // (154/120)^2
                SIGNAL_INDEX_L5 => iono_delay_l1 * 1.7932703213610586, // (154/115)^2
                _ => iono_delay_l1,
            }
        },
        GnssSystem::BdsSystem => {
            match signal_index {
                SIGNAL_INDEX_B1C => iono_delay_l1,
                SIGNAL_INDEX_B1I => iono_delay_l1 * 1.0184327918525377, // (1540/1526)^2
                SIGNAL_INDEX_B2I | SIGNAL_INDEX_B2B => iono_delay_l1 * 1.7032461936225223, // (154/118)^2
                SIGNAL_INDEX_B3I => iono_delay_l1 * 1.5424037460978148, // (154/124)^2
                SIGNAL_INDEX_B2A => iono_delay_l1 * 1.7932703213610586, // (154/115)^2
                6 => iono_delay_l1 * 1.7473889738252685, // B2AB (154/116.5)^2
                _ => iono_delay_l1,
            }
        },
        GnssSystem::GalileoSystem => {
            match signal_index {
                SIGNAL_INDEX_E1 => iono_delay_l1,
                SIGNAL_INDEX_E5A => iono_delay_l1 * 1.7932703213610586, // (154/115)^2
                SIGNAL_INDEX_E5B => iono_delay_l1 * 1.7032461936225223, // (154/118)^2
                SIGNAL_INDEX_E5 => iono_delay_l1 * 1.7473889738252685, // E5 (154/116.5)^2
                SIGNAL_INDEX_E6 => iono_delay_l1 * 1.517824, // (154/125)^2
                _ => iono_delay_l1,
            }
        },
        GnssSystem::GlonassSystem => {
            match signal_index {
                SIGNAL_INDEX_G1 => iono_delay_l1 * 1.040594059405941, // (1575.42/1602)^2 for k=-7
                SIGNAL_INDEX_G2 => iono_delay_l1 * 1.5980663241899555, // (1575.42/1246)^2 for k=-7
                SIGNAL_INDEX_G3 => iono_delay_l1 * 1.7205547652916243, // (1575.42/1202.025)^2
                _ => iono_delay_l1,
            }
        },
        _ => iono_delay_l1,
    }
}

/// Get wavelength for different signals
pub fn get_wave_length(system: GnssSystem, signal_index: usize, freq_id: i32) -> f64 {
    match system {
        GnssSystem::GpsSystem => {
            match signal_index {
                SIGNAL_INDEX_L1CA | SIGNAL_INDEX_L1C => LIGHT_SPEED / FREQ_GPS_L1 as f64,
                SIGNAL_INDEX_L2C => LIGHT_SPEED / FREQ_GPS_L2 as f64,
                SIGNAL_INDEX_L5 => LIGHT_SPEED / FREQ_GPS_L5 as f64,
                _ => LIGHT_SPEED / FREQ_GPS_L1 as f64,
            }
        },
        GnssSystem::BdsSystem => {
            match signal_index {
                SIGNAL_INDEX_B1C => LIGHT_SPEED / FREQ_BDS_B1C as f64,
                SIGNAL_INDEX_B1I => LIGHT_SPEED / FREQ_BDS_B1I as f64,
                SIGNAL_INDEX_B2I => LIGHT_SPEED / FREQ_BDS_B2I as f64,
                SIGNAL_INDEX_B3I => LIGHT_SPEED / FREQ_BDS_B3I as f64,
                SIGNAL_INDEX_B2A => LIGHT_SPEED / FREQ_BDS_B2A as f64,
                SIGNAL_INDEX_B2B => LIGHT_SPEED / FREQ_BDS_B2B as f64,
                _ => LIGHT_SPEED / FREQ_BDS_B1C as f64,
            }
        },
        GnssSystem::GalileoSystem => {
            match signal_index {
                SIGNAL_INDEX_E1 => LIGHT_SPEED / FREQ_GAL_E1 as f64,
                SIGNAL_INDEX_E5A => LIGHT_SPEED / FREQ_GAL_E5A as f64,
                SIGNAL_INDEX_E5B => LIGHT_SPEED / FREQ_GAL_E5B as f64,
                SIGNAL_INDEX_E5 => LIGHT_SPEED / FREQ_GAL_E5 as f64,
                SIGNAL_INDEX_E6 => LIGHT_SPEED / FREQ_GAL_E6 as f64,
                _ => LIGHT_SPEED / FREQ_GAL_E1 as f64,
            }
        },
        GnssSystem::GlonassSystem => {
            let freq = match signal_index {
                SIGNAL_INDEX_G1 => FREQ_GLO_G1 + 562500.0 * freq_id as f64, 
                SIGNAL_INDEX_G2 => FREQ_GLO_G2 + 437500.0 * freq_id as f64,
                SIGNAL_INDEX_G3 => FREQ_GLO_G3, // G3 uses CDMA, not FDMA
                _ => FREQ_GLO_G1 + 562500.0 * freq_id as f64,
            };
            LIGHT_SPEED / freq
        },
        _ => LIGHT_SPEED / FREQ_GPS_L1 as f64,
    }
}

/// Get travel time including group delay and ionospheric delay
pub fn get_travel_time(satellite_param: &SatelliteParam, signal_index: usize) -> f64 {
    let mut travel_time = satellite_param.travel_time + satellite_param.group_delay[signal_index];
    travel_time += get_iono_delay(satellite_param.iono_delay, satellite_param.system, signal_index) / LIGHT_SPEED;
    travel_time
}

/// Get carrier phase measurement
pub fn get_carrier_phase(satellite_param: &SatelliteParam, signal_index: usize) -> f64 {
    let mut travel_time = satellite_param.travel_time + satellite_param.group_delay[signal_index];
    travel_time = travel_time * LIGHT_SPEED - get_iono_delay(satellite_param.iono_delay, satellite_param.system, signal_index);
    travel_time / get_wave_length(satellite_param.system, signal_index, satellite_param.freq_id)
}

/// Get Doppler frequency
pub fn get_doppler(satellite_param: &SatelliteParam, signal_index: usize) -> f64 {
    -satellite_param.relative_speed / get_wave_length(satellite_param.system, signal_index, satellite_param.freq_id)
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
    let convert_matrix = calc_conv_matrix_from_lla(&receiver_lla);
    
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
    let horizontal_distance = (los_vector[0] * los_vector[0] + los_vector[1] * los_vector[1]).sqrt();
    let elevation = los_vector[2].atan2(horizontal_distance);
    let azimuth = los_vector[1].atan2(los_vector[0]);
    (elevation, azimuth)
}

/// Calculate relative speed between receiver and satellite
fn sat_relative_speed(receiver_pos: &KinematicInfo, sat_pos: &KinematicInfo) -> f64 {
    let dx = sat_pos.x - receiver_pos.x;
    let dy = sat_pos.y - receiver_pos.y;
    let dz = sat_pos.z - receiver_pos.z;
    let distance = (dx * dx + dy * dy + dz * dz).sqrt();
    
    let dvx = sat_pos.vx - receiver_pos.vx;
    let dvy = sat_pos.vy - receiver_pos.vy;
    let dvz = sat_pos.vz - receiver_pos.vz;
    
    (dx * dvx + dy * dvy + dz * dvz) / distance
}

/// Calculate geometry distance and LOS vector
fn geometry_distance(pos1: &KinematicInfo, pos2: &KinematicInfo, los_vector: &mut [f64; 3]) -> f64 {
    let dx = pos2.x - pos1.x;
    let dy = pos2.y - pos1.y;
    let dz = pos2.z - pos1.z;
    let distance = (dx * dx + dy * dy + dz * dz).sqrt();
    
    los_vector[0] = dx / distance;
    los_vector[1] = dy / distance;
    los_vector[2] = dz / distance;
    
    distance
}

// Placeholder functions - these should be implemented in coordinate module
fn gps_sat_pos_speed_eph(_system: GnssSystem, _time: f64, _eph: &GpsEphemeris, _pos: &mut KinematicInfo, _acc: Option<&mut [f64; 3]>) -> bool {
    // TODO: Implement satellite position calculation
    true
}

fn glonass_sat_pos_speed_eph(_time: f64, _eph: &GlonassEphemeris, _pos: &mut KinematicInfo, _acc: Option<&mut [f64; 3]>) -> bool {
    // TODO: Implement GLONASS satellite position calculation
    true
}

fn glonass_clock_correction(_eph: &GlonassEphemeris, _time: f64) -> f64 {
    // TODO: Implement GLONASS clock correction
    // return -eph->tn + eph->gamma * (time - eph->tb);
    0.0
}

fn gps_iono_delay(_iono_param: &IonoParam, _time: f64, _lat: f64, _lon: f64, _elevation: f64, _azimuth: f64) -> f64 {
    // TODO: Implement ionospheric delay calculation
    0.0
}

fn tropo_delay(_lat: f64, _alt: f64, _elevation: f64) -> f64 {
    // TODO: Implement tropospheric delay calculation
    0.0
}

fn gps_clock_correction(_eph: &GpsEphemeris, _time: f64) -> f64 {
    // TODO: Implement GPS clock correction
    0.0
}

fn get_leap_second(_seconds: u32) -> i32 {
    // TODO: Implement leap second calculation
    18 // Current GPS-UTC offset
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
    let mut time_diff = satellite_time - satellite_param.pos_time_tag as f64;
    
    // Compensate week round
    if time_diff > 600000.0 {
        time_diff -= 604800.0;
    } else if time_diff < -600000.0 {
        time_diff += 604800.0;
    }
    
    if satellite_param.pos_time_tag >= 0 && time_diff.abs() <= 0.5 {
        // Do prediction using stored position, velocity and acceleration
        pos_vel.x = satellite_param.pos_vel.x + 
                   (satellite_param.pos_vel.vx + satellite_param.acc[0] * time_diff * 0.5) * time_diff;
        pos_vel.y = satellite_param.pos_vel.y + 
                   (satellite_param.pos_vel.vy + satellite_param.acc[1] * time_diff * 0.5) * time_diff;
        pos_vel.z = satellite_param.pos_vel.z + 
                   (satellite_param.pos_vel.vz + satellite_param.acc[2] * time_diff * 0.5) * time_diff;
        pos_vel.vx = satellite_param.pos_vel.vx + satellite_param.acc[0] * time_diff;
        pos_vel.vy = satellite_param.pos_vel.vy + satellite_param.acc[1] * time_diff;
        pos_vel.vz = satellite_param.pos_vel.vz + satellite_param.acc[2] * time_diff;
    } else {
        // New calculation
        let time_tag = (satellite_time + 0.5) as i32;
        gps_sat_pos_speed_eph(system, time_tag as f64, eph, &mut satellite_param.pos_vel, Some(&mut satellite_param.acc));
        satellite_param.pos_time_tag = time_tag;
        time_diff = satellite_time - time_tag as f64;
        
        if time_diff != 0.0 {
            pos_vel.x = satellite_param.pos_vel.x + 
                       (satellite_param.pos_vel.vx + satellite_param.acc[0] * time_diff * 0.5) * time_diff;
            pos_vel.y = satellite_param.pos_vel.y + 
                       (satellite_param.pos_vel.vy + satellite_param.acc[1] * time_diff * 0.5) * time_diff;
            pos_vel.z = satellite_param.pos_vel.z + 
                       (satellite_param.pos_vel.vz + satellite_param.acc[2] * time_diff * 0.5) * time_diff;
            pos_vel.vx = satellite_param.pos_vel.vx + satellite_param.acc[0] * time_diff;
            pos_vel.vy = satellite_param.pos_vel.vy + satellite_param.acc[1] * time_diff;
            pos_vel.vz = satellite_param.pos_vel.vz + satellite_param.acc[2] * time_diff;
        } else {
            *pos_vel = satellite_param.pos_vel;
        }
    }
}

/// Enhanced get_satellite_param with position prediction support
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
        satellite_param.freq_id = 0; // Would be GloEph->freq
        // glonass_sat_pos_speed_eph(satellite_time, glo_eph, &sat_position, None);
        return;
    }
    
    // Adjust time based on system
    match system {
        GnssSystem::BdsSystem => {
            // Subtract leap second difference for BDS
            adjusted_time.MilliSeconds -= 14000;
        },
        GnssSystem::GlonassSystem => {
            // GLONASS time handling
            let seconds = (time.Week * 604800 + time.MilliSeconds / 1000) as u32;
            let leap_second = get_leap_second(seconds);
            adjusted_time.MilliSeconds -= leap_second * 1000;
        },
        _ => {}
    }
    
    let mut satellite_time = (adjusted_time.MilliSeconds as f64 + adjusted_time.SubMilliSeconds) / 1000.0;
    
    // Set satellite ID and frequency ID
    satellite_param.svid = eph.svid as i32;
    satellite_param.freq_id = 0; // Default for GPS/BDS/Galileo
    
    // First estimate of travel time
    if use_position_prediction {
        get_sat_pos_vel(system, satellite_time, eph, satellite_param, &mut sat_position);
    } else {
        gps_sat_pos_speed_eph(system, satellite_time, eph, &mut sat_position, None);
    }
    
    let mut travel_time = geometry_distance(position_ecef, &sat_position, &mut satellite_param.los_vector) / LIGHT_SPEED;
    
    // Correct for satellite motion during signal travel
    sat_position.x -= travel_time * sat_position.vx;
    sat_position.y -= travel_time * sat_position.vy;
    sat_position.z -= travel_time * sat_position.vz;
    
    travel_time = geometry_distance(position_ecef, &sat_position, &mut satellite_param.los_vector) / LIGHT_SPEED;
    satellite_time -= travel_time;
    
    // Calculate accurate satellite position at transmit time
    let time_diff = if use_position_prediction {
        let mut pos_vel_temp = KinematicInfo::default();
        get_sat_pos_vel(system, satellite_time, eph, satellite_param, &mut pos_vel_temp);
        sat_position = pos_vel_temp;
        satellite_time - satellite_param.pos_time_tag as f64
    } else {
        gps_sat_pos_speed_eph(system, satellite_time, eph, &mut sat_position, None);
        0.0
    };
    
    let distance = geometry_distance(position_ecef, &sat_position, &mut satellite_param.los_vector);
    let (elevation, azimuth) = sat_el_az_from_los(&satellite_param.los_vector);
    
    // Calculate ionospheric delay
    satellite_param.iono_delay = gps_iono_delay(iono_param, satellite_time, position_lla.lat, position_lla.lon, elevation, azimuth);
    
    // Add tropospheric delay
    let total_distance = distance + tropo_delay(position_lla.lat, position_lla.alt, elevation);
    
    // Calculate travel time with corrections
    if system == GnssSystem::GlonassSystem {
        // travel_time = total_distance / LIGHT_SPEED - glonass_clock_correction(glo_eph, satellite_time);
        travel_time = total_distance / LIGHT_SPEED; // Placeholder
    } else {
        travel_time = total_distance / LIGHT_SPEED - gps_clock_correction(eph, satellite_time);
        // Add relativistic correction with time difference for prediction
        travel_time -= 4.442807633e-10 * eph.ecc * eph.sqrtA * (eph.Ek + time_diff * eph.Ek_dot).sin();
    }
    
    // Set group delays based on system (same as before)
    satellite_param.group_delay = [0.0; 8];
    match system {
        GnssSystem::GpsSystem => {
            satellite_param.group_delay[SIGNAL_INDEX_L1CA] = eph.tgd;
            satellite_param.group_delay[SIGNAL_INDEX_L1C] = eph.tgd_ext[1];
            satellite_param.group_delay[SIGNAL_INDEX_L2C] = eph.tgd2;
            satellite_param.group_delay[SIGNAL_INDEX_L5] = eph.tgd_ext[3];
        },
        GnssSystem::BdsSystem => {
            satellite_param.group_delay[SIGNAL_INDEX_B1C] = eph.tgd_ext[1];
            satellite_param.group_delay[SIGNAL_INDEX_B1I] = eph.tgd;
            satellite_param.group_delay[SIGNAL_INDEX_B2I] = eph.tgd2;
            satellite_param.group_delay[SIGNAL_INDEX_B3I] = 0.0;
            satellite_param.group_delay[SIGNAL_INDEX_B2A] = eph.tgd_ext[3];
            satellite_param.group_delay[SIGNAL_INDEX_B2B] = eph.tgd_ext[4];
            satellite_param.group_delay[SIGNAL_INDEX_B2A + 2] = (eph.tgd_ext[3] + eph.tgd_ext[4]) / 2.0; // B2AB
        },
        GnssSystem::GalileoSystem => {
            satellite_param.group_delay[SIGNAL_INDEX_E1] = eph.tgd;
            satellite_param.group_delay[SIGNAL_INDEX_E5A] = eph.tgd_ext[2];
            satellite_param.group_delay[SIGNAL_INDEX_E5B] = eph.tgd_ext[4];
            satellite_param.group_delay[3] = (eph.tgd_ext[2] + eph.tgd_ext[4]) / 2.0; // E5
            satellite_param.group_delay[SIGNAL_INDEX_E6] = eph.tgd_ext[4];
        },
        _ => {}
    }
    
    // Set final parameters
    satellite_param.travel_time = travel_time;
    satellite_param.elevation = elevation;
    satellite_param.azimuth = azimuth;
    
    // Calculate relative speed with proper clock drift correction
    if system == GnssSystem::GlonassSystem {
        // For GLONASS, use gamma (relative frequency bias) instead of af1
        // satellite_param.relative_speed = sat_relative_speed(position_ecef, &sat_position) - LIGHT_SPEED * glo_eph.gamma;
        satellite_param.relative_speed = sat_relative_speed(position_ecef, &sat_position); // Placeholder
    } else {
        satellite_param.relative_speed = sat_relative_speed(position_ecef, &sat_position) - LIGHT_SPEED * eph.af1;
    }
}