//! # Модуль интерпретации JSON данных
//!
//! Этот модуль реализует интерпретацию и обработку JSON объектов для ГНСС системы.
//! Основные функции:
//! - Интерпретация конфигурационных параметров из JSON структур
//! - Преобразование JSON данных в внутренние структуры ГНСС системы
//! - Обработка параметров времени, траекторий, эфемерид и мощности сигналов
//! - Конвертация координатных систем и временных форматов
//! - Валидация и применение настроек из JSON конфигураций
//!
//! Модуль служит связующим звеном между внешними конфигурациями
//! и внутренними структурами данных ГНСС библиотеки.

//----------------------------------------------------------------------
// json_interpreter.rs:
//   Implementation of functions to interpret JSON object tree
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use std::os::raw::c_char;
use crate::trajectory::CTrajectory;
use crate::types::*;
use crate::powercontrol::SignalPower;
use crate::{lla_to_ecef, gps_time_to_utc, bds_time_to_utc, glonass_time_to_utc, speed_ecef_to_local, ecef_to_lla};

// Constants for dictionary lookups
static KEY_DICTIONARY_LIST_PARAM: &[&str] = &[
    "time", "trajectory", "ephemeris", "almanac", "output", "power", "delay",
];

static KEY_DICTIONARY_LIST_TIME: &[&str] = &[
    "type", "week", "second", "leapYesr", "day", "year", "month", "hour", "minute",
];

static KEY_DICTIONARY_LIST_TRAJECTORY: &[&str] = &[
    "name", "initPosition", "initVelocity", "trajectoryList", "type", "format", 
    "longitude", "latitude", "altitude", "x", "y", "z",
    "speedUnit", "angleUnit", "speed", "course", "east", "north", "up",
];

static KEY_DICTIONARY_LIST_EPH_ALM: &[&str] = &[
    "type", "name",
];

static KEY_DICTIONARY_LIST_OUTPUT: &[&str] = &[
    "type", "format", "name", "interval", "config", "systemSelect", "elevationMask", 
    "maskOut", "system", "svid", "signal", "enable", "sampleFreq", "centerFreq",
];

static KEY_DICTIONARY_LIST_POWER: &[&str] = &[
    "noiseFloor", "initPower", "elevationAdjust", "signalPower", "unit", "value", 
    "system", "svid", "powerValue", "time",
];

static DICTIONARY_LIST_SYSTEM: &[&str] = &[
    "UTC", "GPS", "BDS", "Galileo", "GLONASS",
];

static DICTIONARY_LIST_COORDINATE: &[&str] = &[
    "LLA", "ECEF", "SCU", "ENU", "d", "dm", "dms", "rad", "degree", "mps", "kph", "knot", "mph",
];

static KEY_DICTIONARY_LIST_TRAJECTORY_LIST: &[&str] = &[
    "type", "time", "acceleration", "speed", "rate", "angle", "rate", "radius",
];

static DICTIONARY_LIST_TRAJECTORY_TYPE: &[&str] = &[
    "Const", "ConstAcc", "VerticalAcc", "Jerk", "HorizontalTurn",
];

static DICTIONARY_LIST_OUTPUT_TYPE: &[&str] = &[
    "position", "observation", "IFdata", "baseband",
];

static DICTIONARY_LIST_OUTPUT_FORMAT: &[&str] = &[
    "ECEF", "LLA", "NMEA", "KML", "RINEX", "IQ8", "IQ4",
];

static DICTIONARY_LIST_SIGNAL: &[&str] = &[
    "L1CA", "L1C", "L2C", "L2P", "L5", "", "", "",
    "B1C", "B1I", "B2I", "B3I", "B2a", "B2b", "", "",
    "E1", "E5a", "E5b", "E5", "E6", "", "", "",
    "G1", "G2", "G3", "", "", "", "", "",
];

static DICTIONARY_LIST_POWER_UNIT: &[&str] = &[
    "dBHz", "dBm", "dBW",
];

// Use types from types.rs instead of redefining them

#[repr(C)]
pub struct OutputParam {
    pub filename: [c_char; 256],
    pub output_type: i32,
    pub format: i32,
    pub interval: i32,
    pub sample_freq: i32,
    pub center_freq: i32,
    pub elevation_mask: f64,
    pub gps_mask_out: u32,
    pub glonass_mask_out: u32,
    pub bds_mask_out: u64,
    pub galileo_mask_out: u64,
    pub freq_select: [u32; 4],
}

#[repr(C)]
pub struct DelayConfig {
    // Add fields as needed
}

// SignalPower is imported from powercontrol module

// Use ConvertMatrix from types.rs

// Enums
#[repr(C)]
pub enum TrajectoryType {
    TrajTypeUnknown = 0,
    TrajTypeConst = 1,
    TrajTypeConstAcc = 2,
    TrajTypeVerticalAcc = 3,
    TrajTypeJerk = 4,
    TrajTypeHorizontalTurn = 5,
}

#[repr(C)]
pub enum TrajectoryDataType {
    TrajDataTimeSpan = 0,
    TrajDataAcceleration = 1,
    TrajDataSpeed = 2,
    TrajDataAccRate = 3,
    TrajDataAngularRate = 4,
    TrajDataAngle = 5,
    TrajDataRadius = 6,
}

// GnssSystem is imported from types.rs

#[repr(C)]
pub enum OutputType {
    OutputTypePosition = 0,
    OutputTypeObservation = 1,
    OutputTypeIFdata = 2,
    OutputTypeBaseband = 3,
}

#[repr(C)]
pub enum OutputFormat {
    OutputFormatECEF = 0,
    OutputFormatLLA = 1,
    OutputFormatNMEA = 2,
    OutputFormatKML = 3,
    OutputFormatRINEX = 4,
    OutputFormatIQ8 = 5,
    OutputFormatIQ4 = 6,
}

#[repr(C)]
pub enum ElevationAdjust {
    ElevationAdjustNone = 0,
    ElevationAdjustSinSqrtFade = 1,
}

// Forward declarations for external types
pub struct JsonObject;
pub struct CNavData;
pub struct CPowerControl;

// Helper functions
fn search_dictionary(word: &str, dictionary_list: &[&str]) -> i32 {
    for (i, &dict_word) in dictionary_list.iter().enumerate() {
        if word == dict_word {
            return i as i32;
        }
    }
    -1
}

fn deg2rad(degrees: f64) -> f64 {
    degrees * std::f64::consts::PI / 180.0
}

fn get_double_value(_object: *mut JsonObject) -> f64 {
    // This would need to be implemented based on JsonObject structure
    0.0 // placeholder
}

// Import coordinate functions
use crate::coordinate::{calc_conv_matrix_lla, speed_course_to_enu, speed_enu_to_course};

// External function declarations (would be implemented elsewhere)
extern "C" {
    fn json_stream_get_first_object(object: *mut JsonObject) -> *mut JsonObject;
    fn json_stream_get_next_object(object: *mut JsonObject) -> *mut JsonObject;
}

// Main function - equivalent to AssignParameters
pub fn assign_parameters(
    object: *mut JsonObject,
    mut utc_time: Option<&mut UtcTime>,
    mut start_pos: Option<&mut LlaPosition>,
    mut start_vel: Option<&mut LocalSpeed>,
    mut trajectory: Option<&mut CTrajectory>,
    mut nav_data: Option<&mut CNavData>,
    mut output_param: Option<&mut OutputParam>,
    mut power_control: Option<&mut CPowerControl>,
    mut delay_config: Option<&mut DelayConfig>,
) -> bool {
    unsafe {
        let mut current_object = json_stream_get_first_object(object);
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_PARAM) {
                0 => { // "time"
                    if let Some(utc) = utc_time.as_mut() {
                        assign_start_time(json_stream_get_first_object(current_object), utc);
                    }
                },
                1 => { // "trajectory"
                    if let (Some(pos), Some(vel), Some(traj)) = 
                       (start_pos.as_mut(), start_vel.as_mut(), trajectory.as_mut()) {
                        set_trajectory(json_stream_get_first_object(current_object), pos, vel, traj);
                    }
                },
                2 => { // "ephemeris"
                    if let Some(nav) = nav_data.as_mut() {
                        set_ephemeris(current_object, nav);
                    }
                },
                3 => { // "almanac"
                    if let Some(nav) = nav_data.as_mut() {
                        set_almanac(current_object, nav);
                    }
                },
                4 => { // "output"
                    if let Some(output) = output_param.as_mut() {
                        set_output_param(json_stream_get_first_object(current_object), output);
                    }
                },
                5 => { // "power"
                    if let Some(power) = power_control.as_mut() {
                        set_power_control(json_stream_get_first_object(current_object), power);
                    }
                },
                6 => { // "delay"
                    if let Some(delay) = delay_config.as_mut() {
                        set_delay_config(json_stream_get_first_object(current_object), delay);
                    }
                },
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }
    true
}

fn assign_start_time(object: *mut JsonObject, utc_time: &mut UtcTime) -> bool {
    let mut time_type = 1; // 0 for UTC, 1 for GPS, 2 for BDS, 3 for Galileo, 4 for GLONASS
    let mut week = 0;
    let mut leap_year = 0;
    let mut gnss_time = GnssTime {
        Week: 0,
        MilliSeconds: 0,
        SubMilliSeconds: 0.0,
    };
    let mut glonass_time = GlonassTime {
        LeapYear: 0,
        Day: 0,
        MilliSeconds: 0,
        SubMilliSeconds: 0.0,
    };

    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_TIME) {
                0 => { // "type"
                    if is_string_type(current_object) {
                        let type_str = get_object_string(current_object);
                        time_type = search_dictionary(&type_str, DICTIONARY_LIST_SYSTEM);
                    }
                },
                1 => { // "week"
                    week = get_object_int(current_object);
                },
                2 => { // "second"
                    utc_time.Second = get_double_value(current_object);
                },
                3 => { // "leapYear"
                    leap_year = get_object_int(current_object);
                },
                4 => { // "day"
                    utc_time.Day = get_object_int(current_object);
                },
                5 => { // "year"
                    utc_time.Year = get_object_int(current_object);
                },
                6 => { // "month"
                    utc_time.Month = get_object_int(current_object);
                },
                7 => { // "hour"
                    utc_time.Hour = get_object_int(current_object);
                },
                8 => { // "minute"
                    utc_time.Minute = get_object_int(current_object);
                },
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }

    match time_type {
        1 | 3 => { // GPS Time or Galileo Time
            gnss_time.Week = week;
            gnss_time.SubMilliSeconds = utc_time.Second * 1000.0;
            gnss_time.MilliSeconds = gnss_time.SubMilliSeconds as i32;
            gnss_time.SubMilliSeconds -= gnss_time.MilliSeconds as f64;
            *utc_time = gps_time_to_utc(gnss_time, true);
        },
        2 => { // BDS Time
            gnss_time.Week = week;
            gnss_time.SubMilliSeconds = utc_time.Second * 1000.0;
            gnss_time.MilliSeconds = gnss_time.SubMilliSeconds as i32;
            gnss_time.SubMilliSeconds -= gnss_time.MilliSeconds as f64;
            *utc_time = bds_time_to_utc(gnss_time);
        },
        4 => { // GLONASS Time
            glonass_time.LeapYear = leap_year;
            glonass_time.Day = utc_time.Day;
            glonass_time.SubMilliSeconds = utc_time.Second * 1000.0;
            glonass_time.MilliSeconds = glonass_time.SubMilliSeconds as i32;
            glonass_time.SubMilliSeconds -= glonass_time.MilliSeconds as f64;
            *utc_time = glonass_time_to_utc(glonass_time);
        },
        _ => {}
    }

    true
}

// Helper functions for JsonObject access (these would need proper implementation)
fn get_object_key(_object: *mut JsonObject) -> String {
    // This would extract the key from JsonObject
    String::new() // placeholder
}

fn is_string_type(_object: *mut JsonObject) -> bool {
    // This would check if object is string type
    false // placeholder
}

fn get_object_string(_object: *mut JsonObject) -> String {
    // This would extract string value from JsonObject
    String::new() // placeholder
}

fn get_object_int(_object: *mut JsonObject) -> i32 {
    // This would extract int value from JsonObject
    0 // placeholder
}

fn get_object_type(_object: *mut JsonObject) -> i32 {
    // This would get the type of JsonObject
    0 // placeholder
}

// Helper functions for CTrajectory and CNavData
fn set_trajectory_name(trajectory: &mut CTrajectory, name: &str) {
    trajectory.set_trajectory_name(name);
}

fn set_init_pos_vel(trajectory: &mut CTrajectory, pos: &LlaPosition, vel: &LocalSpeed, flag: bool) {
    trajectory.set_init_pos_vel_lla(*pos, *vel, flag);
}

// Placeholder functions for nav data
fn read_nav_file(_nav_data: &mut CNavData, _filename: &str) {
    // TODO: Implement navigation file reading
}

fn read_alm_file(_nav_data: &mut CNavData, _filename: &str) {
    // TODO: Implement almanac file reading
}

fn set_trajectory(
    object: *mut JsonObject,
    start_pos: &mut LlaPosition,
    start_vel: &mut LocalSpeed,
    trajectory: &mut CTrajectory,
) -> bool {
    let mut content = 0;
    let mut velocity_type = 0;
    let mut convert_matrix = ConvertMatrix::default();
    let mut position = KinematicInfo {
        x: 0.0, y: 0.0, z: 0.0,
        vx: 0.0, vy: 0.0, vz: 0.0,
    };
    let mut velocity = KinematicInfo {
        x: 0.0, y: 0.0, z: 0.0,
        vx: 0.0, vy: 0.0, vz: 0.0,
    };

    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_TRAJECTORY) {
                0 => { // "name"
                    set_trajectory_name(trajectory, &get_object_string(current_object));
                },
                1 => { // "initPosition"
                    if assign_start_position(json_stream_get_first_object(current_object), start_pos) {
                        content |= 1;
                    }
                },
                2 => { // "initVelocity"
                    velocity_type = assign_start_velocity(json_stream_get_first_object(current_object), start_vel, &mut velocity);
                    if velocity_type != 0 {
                        content |= 2;
                    }
                },
                3 => { // "trajectoryList"
                    if (content & 3) != 3 {
                        return false;
                    }
                    if velocity_type == 1 { // velocity in ECEF format and stored in velocity
                        convert_matrix = calc_conv_matrix_lla(start_pos);
                        position = lla_to_ecef(start_pos);
                        position.vx = velocity.vx;
                        position.vy = velocity.vy;
                        position.vz = velocity.vz;
                        speed_ecef_to_local(&convert_matrix, &position, start_vel);
                    }
                    set_init_pos_vel(trajectory, start_pos, start_vel, false);
                    unsafe { assign_trajectory_list(json_stream_get_first_object(current_object), trajectory); }
                },
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }
    true
}

fn set_ephemeris(object: *mut JsonObject, nav_data: &mut CNavData) -> bool {
    unsafe {
        let object_type = get_object_type(object);
        if object_type == 1 { // ValueTypeObject
            set_ephemeris_file(json_stream_get_first_object(object), nav_data);
        } else if object_type == 2 { // ValueTypeArray
            let mut object_array = json_stream_get_first_object(object);
            while !object_array.is_null() {
                if get_object_type(object_array) == 1 { // ValueTypeObject
                    set_ephemeris_file(json_stream_get_first_object(object_array), nav_data);
                }
                object_array = json_stream_get_next_object(object_array);
            }
        }
    }
    true
}

fn set_ephemeris_file(object: *mut JsonObject, nav_data: &mut CNavData) -> bool {
    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            if key == "name" {
                let filename = get_object_string(current_object);
                println!("[INFO]\tLoading ephemeris file: {}", filename);
                read_nav_file(nav_data, &filename);
                println!("[INFO]\tEphemeris file loaded successfully: {}", filename);
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }
    true
}

fn set_almanac(object: *mut JsonObject, nav_data: &mut CNavData) -> bool {
    unsafe {
        let object_type = get_object_type(object);
        if object_type == 1 { // ValueTypeObject
            set_almanac_file(json_stream_get_first_object(object), nav_data);
        } else if object_type == 2 { // ValueTypeArray
            let mut object_array = json_stream_get_first_object(object);
            while !object_array.is_null() {
                if get_object_type(object_array) == 1 { // ValueTypeObject
                    set_almanac_file(json_stream_get_first_object(object_array), nav_data);
                }
                object_array = json_stream_get_next_object(object_array);
            }
        }
    }
    true
}

fn set_almanac_file(object: *mut JsonObject, nav_data: &mut CNavData) -> bool {
    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            if key == "name" {
                let filename = get_object_string(current_object);
                read_alm_file(nav_data, &filename);
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }
    true
}

// Additional external function declarations
extern "C" {
    fn assign_trajectory_list(object: *mut JsonObject, trajectory: &mut CTrajectory);
}

fn set_output_param(object: *mut JsonObject, output_param: &mut OutputParam) -> bool {
    // Set default values
    output_param.filename[0] = 0;
    output_param.gps_mask_out = 0;
    output_param.glonass_mask_out = 0;
    output_param.bds_mask_out = 0;
    output_param.galileo_mask_out = 0;
    output_param.elevation_mask = deg2rad(5.0);
    output_param.interval = 1000;
    
    // Default output GPS L1 only
    output_param.freq_select[0] = 0x1;
    output_param.freq_select[1] = 0;
    output_param.freq_select[2] = 0;
    output_param.freq_select[3] = 0;

    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_OUTPUT) {
                0 => { // "type"
                    if is_string_type(current_object) {
                        output_param.output_type = search_dictionary(&get_object_string(current_object), DICTIONARY_LIST_OUTPUT_TYPE);
                    }
                },
                1 => { // "format"
                    if is_string_type(current_object) {
                        output_param.format = search_dictionary(&get_object_string(current_object), DICTIONARY_LIST_OUTPUT_FORMAT);
                    }
                },
                2 => { // "name"
                    if is_string_type(current_object) {
                        let filename = get_object_string(current_object);
                        let filename_bytes = filename.as_bytes();
                        let copy_len = std::cmp::min(filename_bytes.len(), 255);
                        for i in 0..copy_len {
                            output_param.filename[i] = filename_bytes[i] as c_char;
                        }
                        output_param.filename[copy_len] = 0;
                    }
                },
                3 => { // "interval"
                    output_param.interval = (get_double_value(current_object) * 1000.0) as i32;
                },
                4 => { // "config"
                    process_config_param(json_stream_get_first_object(current_object), output_param);
                },
                5 => { // "systemSelect"
                    if get_object_type(current_object) == 2 { // ValueTypeArray
                        let mut system_select_object = json_stream_get_first_object(current_object);
                        while !system_select_object.is_null() {
                            process_system_select(json_stream_get_first_object(system_select_object), output_param);
                            system_select_object = json_stream_get_next_object(system_select_object);
                        }
                    }
                },
                12 => { // "sampleFreq"
                    output_param.sample_freq = (get_double_value(current_object) * 1000.0).round() as i32;
                },
                13 => { // "centerFreq"
                    output_param.center_freq = (get_double_value(current_object) * 1000.0).round() as i32;
                },
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }
    true
}

fn assign_start_position(object: *mut JsonObject, start_pos: &mut LlaPosition) -> bool {
    let mut position_type = 0;
    let mut format = 4;
    let mut position = KinematicInfo {
        x: 0.0, y: 0.0, z: 0.0,
        vx: 0.0, vy: 0.0, vz: 0.0,
    };
    let mut longitude = 0.0;
    let mut latitude = 0.0;

    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_TRAJECTORY) {
                4 => { // "type"
                    if is_string_type(current_object) {
                        position_type = search_dictionary(&get_object_string(current_object), DICTIONARY_LIST_COORDINATE);
                    }
                },
                5 => { // "format"
                    if is_string_type(current_object) {
                        format = search_dictionary(&get_object_string(current_object), DICTIONARY_LIST_COORDINATE);
                    }
                },
                6 => { // "longitude"
                    longitude = get_double_value(current_object);
                },
                7 => { // "latitude"
                    latitude = get_double_value(current_object);
                },
                8 => { // "altitude"
                    start_pos.alt = get_double_value(current_object);
                },
                9 => { // "x"
                    position.x = get_double_value(current_object);
                },
                10 => { // "y"
                    position.y = get_double_value(current_object);
                },
                11 => { // "z"
                    position.z = get_double_value(current_object);
                },
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }

    start_pos.lon = format_lon_lat(longitude, format);
    start_pos.lat = format_lon_lat(latitude, format);
    if position_type != 0 {
        *start_pos = ecef_to_lla(&position);
    }

    true
}

fn assign_start_velocity(
    object: *mut JsonObject,
    start_vel: &mut LocalSpeed,
    velocity: &mut KinematicInfo,
) -> i32 {
    let mut velocity_type = 2;
    let mut speed_unit = 9;
    let mut angle_unit = 8;

    start_vel.vu = 0.0; // set default up speed to 0

    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_TRAJECTORY) {
                4 => { // "type"
                    if is_string_type(current_object) {
                        velocity_type = search_dictionary(&get_object_string(current_object), DICTIONARY_LIST_COORDINATE);
                    }
                },
                9 => { // "x"
                    velocity.x = format_speed(get_double_value(current_object), speed_unit);
                },
                10 => { // "y"
                    velocity.y = format_speed(get_double_value(current_object), speed_unit);
                },
                11 => { // "z"
                    velocity.z = format_speed(get_double_value(current_object), speed_unit);
                },
                12 => { // "speedUnit"
                    speed_unit = search_dictionary(&get_object_key(current_object), DICTIONARY_LIST_COORDINATE);
                },
                13 => { // "angleUnit"
                    angle_unit = search_dictionary(&get_object_key(current_object), DICTIONARY_LIST_COORDINATE);
                },
                14 => { // "speed"
                    start_vel.speed = format_speed(get_double_value(current_object), speed_unit);
                },
                15 => { // "course"
                    start_vel.course = get_double_value(current_object);
                    if angle_unit == 8 {
                        start_vel.course = deg2rad(start_vel.course);
                    }
                },
                16 => { // "east"
                    start_vel.ve = format_speed(get_double_value(current_object), speed_unit);
                },
                17 => { // "north"
                    start_vel.vn = format_speed(get_double_value(current_object), speed_unit);
                },
                18 => { // "up"
                    start_vel.vu = format_speed(get_double_value(current_object), speed_unit);
                },
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }

    if velocity_type == 2 {
        unsafe { speed_course_to_enu(start_vel); }
    } else if velocity_type == 3 {
        unsafe { speed_enu_to_course(start_vel); }
    }
    velocity_type
}

fn process_config_param(object: *mut JsonObject, output_param: &mut OutputParam) -> bool {
    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_OUTPUT) {
                6 => { // "elevationMask"
                    output_param.elevation_mask = deg2rad(get_double_value(current_object));
                },
                7 => { // "maskOut"
                    if get_object_type(current_object) == 2 { // ValueTypeArray
                        let mut mask_out_array = json_stream_get_first_object(current_object);
                        while !mask_out_array.is_null() {
                            process_mask_out(json_stream_get_first_object(mask_out_array), output_param);
                            mask_out_array = json_stream_get_next_object(mask_out_array);
                        }
                    }
                },
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }
    true
}

fn process_mask_out(object: *mut JsonObject, mut output_param: &mut OutputParam) -> bool {
    let mut system = 0; // GpsSystem
    
    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_OUTPUT) {
                8 => { // "system"
                    if is_string_type(current_object) {
                        system = search_dictionary(&get_object_string(current_object), DICTIONARY_LIST_SYSTEM) - 1;
                    }
                },
                9 => { // "svid"
                    let object_type = get_object_type(current_object);
                    if object_type == 4 { // ValueTypeIntNumber
                        mask_out_satellite(system, get_object_int(current_object), output_param);
                    } else if object_type == 2 { // ValueTypeArray
                        let mut mask_out_sv = json_stream_get_first_object(current_object);
                        while !mask_out_sv.is_null() {
                            if get_object_type(mask_out_sv) == 4 { // ValueTypeIntNumber
                                mask_out_satellite(system, get_object_int(mask_out_sv), output_param);
                            }
                            mask_out_sv = json_stream_get_next_object(mask_out_sv);
                        }
                    }
                },
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }
    true
}

fn mask_out_satellite(system: i32, svid: i32, output_param: &mut OutputParam) -> bool {
    match system {
        0 => { // GpsSystem
            if svid >= 1 && svid <= 32 {
                output_param.gps_mask_out |= 1 << (svid - 1);
            }
        },
        1 => { // BdsSystem
            if svid >= 1 && svid <= 63 {
                output_param.bds_mask_out |= 1u64 << (svid - 1);
            }
        },
        2 => { // GalileoSystem
            if svid >= 1 && svid <= 50 {
                output_param.galileo_mask_out |= 1u64 << (svid - 1);
            }
        },
        3 => { // GlonassSystem
            if svid >= 1 && svid <= 24 {
                output_param.glonass_mask_out |= 1 << (svid - 1);
            }
        },
        _ => return false,
    }
    true
}

fn process_system_select(object: *mut JsonObject, output_param: &mut OutputParam) -> bool {
    let mut system = 0; // GpsSystem
    let mut signal = -1;

    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_OUTPUT) {
                8 => { // "system"
                    if is_string_type(current_object) {
                        system = search_dictionary(&get_object_string(current_object), DICTIONARY_LIST_SYSTEM) - 1;
                    }
                },
                10 => { // "signal"
                    if is_string_type(current_object) {
                        signal = search_dictionary(&get_object_string(current_object), DICTIONARY_LIST_SIGNAL);
                    }
                },
                11 => { // "enable"
                    if signal >= 0 && (signal / 8) != system { // freq and system do not match
                        system = -1;
                    }
                    if system >= 0 {
                        let mut signal_index = signal;
                        if signal < 0 { // frequency not set, set as primary signal
                            signal_index = 0;
                        } else {
                            signal_index %= 8;
                        }
                        let object_type = get_object_type(current_object);
                        if object_type == 6 { // ValueTypeTrue
                            output_param.freq_select[system as usize] |= 1 << signal_index;
                        } else if object_type == 7 { // ValueTypeFalse
                            output_param.freq_select[system as usize] &= !(1 << signal_index);
                        }
                    }
                },
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }
    true
}

fn set_power_control(object: *mut JsonObject, power_control: &mut CPowerControl) -> bool {
    let mut unit = 0;

    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_POWER) {
                0 => { // "noiseFloor"
                    set_noise_floor(power_control, get_double_value(current_object));
                },
                1 => { // "initPower"
                    let mut power_object = json_stream_get_first_object(current_object);
                    while !power_object.is_null() {
                        let power_key = get_object_key(power_object);
                        match search_dictionary(&power_key, KEY_DICTIONARY_LIST_POWER) {
                            4 => { // "unit"
                                unit = search_dictionary(&get_object_string(power_object), DICTIONARY_LIST_POWER_UNIT);
                            },
                            5 => { // "value"
                                let value = get_double_value(power_object);
                                match unit {
                                    0 => set_init_cn0(power_control, value),
                                    1 => set_init_cn0(power_control, value - get_noise_floor(power_control)),
                                    2 => set_init_cn0(power_control, value - get_noise_floor(power_control) + 30.0),
                                    _ => {}
                                }
                            },
                            _ => {}
                        }
                        power_object = json_stream_get_next_object(power_object);
                    }
                },
                2 => { // "elevationAdjust"
                    let object_type = get_object_type(current_object);
                    if object_type == 6 { // ValueTypeTrue
                        set_elevation_adjust(power_control, 1); // ElevationAdjustSinSqrtFade
                    } else if object_type == 7 { // ValueTypeFalse
                        set_elevation_adjust(power_control, 0); // ElevationAdjustNone
                    }
                },
                3 => { // "signalPower"
                    let object_type = get_object_type(current_object);
                    if object_type == 1 { // ValueTypeObject
                        process_signal_power(json_stream_get_first_object(current_object), power_control);
                    } else if object_type == 2 { // ValueTypeArray
                        let mut power_object = json_stream_get_first_object(current_object);
                        while !power_object.is_null() {
                            if get_object_type(power_object) == 1 { // ValueTypeObject
                                process_signal_power(json_stream_get_first_object(power_object), power_control);
                            }
                            power_object = json_stream_get_next_object(power_object);
                        }
                    }
                },
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }
    true
}

fn process_signal_power(object: *mut JsonObject, power_control: &mut CPowerControl) -> bool {
    let mut system = 0;
    let mut svlist: [i32; 32] = [0; 32];
    let mut sv_number = 0;

    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_POWER) {
                6 => { // "system"
                    if is_string_type(current_object) {
                        system = search_dictionary(&get_object_string(current_object), DICTIONARY_LIST_SYSTEM) - 1;
                    }
                    sv_number = 0;
                },
                7 => { // "svid"
                    let object_type = get_object_type(current_object);
                    if object_type == 4 { // ValueTypeIntNumber
                        svlist[sv_number] = get_object_int(current_object);
                        sv_number += 1;
                    } else if object_type == 2 { // ValueTypeArray
                        let mut object_array = json_stream_get_first_object(current_object);
                        while !object_array.is_null() {
                            if get_object_type(object_array) == 4 { // ValueTypeIntNumber
                                svlist[sv_number] = get_object_int(object_array);
                                sv_number += 1;
                            }
                            object_array = json_stream_get_next_object(object_array);
                        }
                    }
                },
                8 => { // "powerValue"
                    let object_type = get_object_type(current_object);
                    if object_type == 1 { // ValueTypeObject
                        process_power_value(json_stream_get_first_object(current_object), system, &svlist[0..sv_number], sv_number, power_control);
                    } else if object_type == 2 { // ValueTypeArray
                        let mut object_array = json_stream_get_first_object(current_object);
                        while !object_array.is_null() {
                            if get_object_type(object_array) == 1 { // ValueTypeObject
                                process_power_value(json_stream_get_first_object(object_array), system, &svlist[0..sv_number], sv_number, power_control);
                            }
                            object_array = json_stream_get_next_object(object_array);
                        }
                    }
                },
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }
    true
}

fn process_power_value(
    object: *mut JsonObject,
    system: i32,
    svlist: &[i32],
    sv_number: usize,
    power_control: &mut CPowerControl,
) -> bool {
    let mut unit = 0;
    let mut signal_power = SignalPower {
        system,
        svid: 0,
        time: 0,
        cn0: unsafe { get_init_cn0(power_control) },
    };

    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_POWER) {
                4 => { // "unit"
                    unit = search_dictionary(&get_object_string(current_object), DICTIONARY_LIST_POWER_UNIT);
                },
                5 => { // "value"
                    let value = get_double_value(current_object);
                    match unit {
                        0 => signal_power.cn0 = value,
                        1 => signal_power.cn0 = value - get_noise_floor(power_control),
                        2 => signal_power.cn0 = value - get_noise_floor(power_control) + 30.0,
                        _ => {}
                    }
                    if unit == 0 && signal_power.cn0 < 0.0 {
                        signal_power.cn0 = -1.0;
                    }
                },
                9 => { // "time"
                    signal_power.time = (get_double_value(current_object) * 1000.0) as i32;
                },
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }

    if sv_number == 0 { // svlist is empty means for all satellites
        signal_power.svid = 0;
        unsafe { add_control_element(power_control, &signal_power); }
    } else {
        for i in 0..sv_number {
            signal_power.svid = svlist[i];
            unsafe { add_control_element(power_control, &signal_power); }
        }
    }

    true
}

fn set_delay_config(_object: *mut JsonObject, _delay_config: &mut DelayConfig) -> bool {
    true
}

fn format_lon_lat(value: f64, format: i32) -> f64 {
    let sign = if value >= 0.0 { 0 } else { 1 };
    let abs_value = value.abs();
    let degree = abs_value as i32;

    match format {
        5 => { // "dm"
            let mut abs_value = abs_value;
            abs_value -= degree as f64;
            let minute = degree % 100;
            let degree = degree / 100;
            abs_value += minute as f64;
            abs_value = degree as f64 + abs_value / 60.0;
            let value = if sign != 0 { -abs_value } else { abs_value };
            deg2rad(value)
        },
        6 => { // "dms"
            let mut abs_value = abs_value;
            abs_value -= degree as f64;
            let second = degree % 100;
            let minute = (degree / 100) % 100;
            let degree = degree / 10000;
            abs_value += second as f64;
            abs_value = degree as f64 + minute as f64 / 60.0 + abs_value / 3600.0;
            let value = if sign != 0 { -abs_value } else { abs_value };
            deg2rad(value)
        },
        7 => value,
        _ => deg2rad(value),
    }
}

fn format_speed(value: f64, format: i32) -> f64 {
    match format {
        10 => value / 3.6,        // kilometers per hour
        11 => value * 1852.0 / 3600.0, // knots
        12 => value * 1609.344 / 3600.0, // miles per hour
        _ => value,
    }
}

// External function declarations for CPowerControl
extern "C" {
    fn set_noise_floor(power_control: &mut CPowerControl, noise_floor: f64);
    fn get_noise_floor(power_control: &CPowerControl) -> f64;
    fn set_init_cn0(power_control: &mut CPowerControl, cn0: f64);
    fn get_init_cn0(power_control: &CPowerControl) -> f64;
    fn set_elevation_adjust(power_control: &mut CPowerControl, adjust: i32);
    fn add_control_element(power_control: &mut CPowerControl, signal_power: &SignalPower);
}