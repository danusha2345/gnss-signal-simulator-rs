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
use crate::types::{GpsEphemeris, GlonassEphemeris, GpsAlmanac, GlonassAlmanac, UtcParam};
use crate::constants::EARTH_GM;

// Временные алиасы для недостающих типов - в будущем нужно реализовать отдельные структуры
type BdsEphemeris = GpsEphemeris;  // BeiDou использует схожую структуру с GPS
type GalileoEphemeris = GpsEphemeris;  // Galileo также схож с GPS
type BdsAlmanac = GpsAlmanac;
type GalileoAlmanac = GpsAlmanac;

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
// Структура для хранения навигационных данных
pub struct CNavData {
    // Ионосферные параметры GPS
    pub gps_iono_alpha: Option<[f64; 4]>,
    pub gps_iono_beta: Option<[f64; 4]>,
    
    // Эфемериды различных ГНСС систем
    pub gps_ephemeris: Vec<GpsEphemeris>,
    pub glonass_ephemeris: Vec<GlonassEphemeris>, 
    pub beidou_ephemeris: Vec<BdsEphemeris>,
    pub galileo_ephemeris: Vec<GalileoEphemeris>,
    
    // Альманахи
    pub gps_almanac: Vec<GpsAlmanac>,
    pub glonass_almanac: Vec<GlonassAlmanac>,
    pub beidou_almanac: Vec<BdsAlmanac>,
    pub galileo_almanac: Vec<GalileoAlmanac>,
    
    // UTC параметры и корректировки времени
    pub utc_param: Option<UtcParam>,
    pub leap_seconds: Option<i32>,
}

impl Default for CNavData {
    fn default() -> Self {
        Self::new()
    }
}

impl CNavData {
    pub fn new() -> Self {
        CNavData {
            gps_iono_alpha: None,
            gps_iono_beta: None,
            gps_ephemeris: Vec::new(),
            glonass_ephemeris: Vec::new(),
            beidou_ephemeris: Vec::new(),
            galileo_ephemeris: Vec::new(),
            gps_almanac: Vec::new(),
            glonass_almanac: Vec::new(),
            beidou_almanac: Vec::new(),
            galileo_almanac: Vec::new(),
            utc_param: None,
            leap_seconds: None,
        }
    }

    // Методы для ионосферных параметров
    pub fn set_gps_iono_alpha(&mut self, alpha: [f64; 4]) {
        self.gps_iono_alpha = Some(alpha);
        println!("[INFO] GPS iono alpha parameters set: {:?}", alpha);
    }

    pub fn set_gps_iono_beta(&mut self, beta: [f64; 4]) {
        self.gps_iono_beta = Some(beta);
        println!("[INFO] GPS iono beta parameters set: {:?}", beta);
    }

    pub fn get_gps_iono_alpha(&self) -> Option<[f64; 4]> {
        self.gps_iono_alpha
    }

    pub fn get_gps_iono_beta(&self) -> Option<[f64; 4]> {
        self.gps_iono_beta
    }
    
    // Методы для добавления эфемерид
    pub fn add_gps_ephemeris(&mut self, eph: GpsEphemeris) {
        self.gps_ephemeris.push(eph);
        println!("[INFO] Added GPS ephemeris, total: {}", self.gps_ephemeris.len());
    }
    
    pub fn add_glonass_ephemeris(&mut self, eph: GlonassEphemeris) {
        self.glonass_ephemeris.push(eph);
        println!("[INFO] Added GLONASS ephemeris, total: {}", self.glonass_ephemeris.len());
    }
    
    pub fn add_beidou_ephemeris(&mut self, eph: BdsEphemeris) {
        self.beidou_ephemeris.push(eph);
        println!("[INFO] Added BeiDou ephemeris, total: {}", self.beidou_ephemeris.len());
    }
    
    pub fn add_galileo_ephemeris(&mut self, eph: GalileoEphemeris) {
        self.galileo_ephemeris.push(eph);
        println!("[INFO] Added Galileo ephemeris, total: {}", self.galileo_ephemeris.len());
    }
    
    // Методы для добавления альманахов
    pub fn add_gps_almanac(&mut self, alm: GpsAlmanac) {
        self.gps_almanac.push(alm);
        println!("[INFO] Added GPS almanac, total: {}", self.gps_almanac.len());
    }
    
    pub fn add_glonass_almanac(&mut self, alm: GlonassAlmanac) {
        self.glonass_almanac.push(alm);
        println!("[INFO] Added GLONASS almanac, total: {}", self.glonass_almanac.len());
    }
    
    pub fn add_beidou_almanac(&mut self, alm: BdsAlmanac) {
        self.beidou_almanac.push(alm);
        println!("[INFO] Added BeiDou almanac, total: {}", self.beidou_almanac.len());
    }
    
    pub fn add_galileo_almanac(&mut self, alm: GalileoAlmanac) {
        self.galileo_almanac.push(alm);
        println!("[INFO] Added Galileo almanac, total: {}", self.galileo_almanac.len());
    }
    
    // Методы получения данных
    pub fn get_gps_ephemeris(&self) -> &[GpsEphemeris] {
        &self.gps_ephemeris
    }
    
    pub fn get_glonass_ephemeris(&self) -> &[GlonassEphemeris] {
        &self.glonass_ephemeris
    }
    
    pub fn get_beidou_ephemeris(&self) -> &[BdsEphemeris] {
        &self.beidou_ephemeris
    }
    
    pub fn get_galileo_ephemeris(&self) -> &[GalileoEphemeris] {
        &self.galileo_ephemeris
    }
    
    pub fn clear_all(&mut self) {
        self.gps_ephemeris.clear();
        self.glonass_ephemeris.clear();
        self.beidou_ephemeris.clear();
        self.galileo_ephemeris.clear();
        self.gps_almanac.clear();
        self.glonass_almanac.clear();
        self.beidou_almanac.clear();
        self.galileo_almanac.clear();
        println!("[INFO] All navigation data cleared");
    }

    // Методы для тестирования RINEX парсера
    pub fn get_gps_ephemeris_count(&self) -> usize {
        self.gps_ephemeris.len()
    }

    pub fn get_glonass_ephemeris_count(&self) -> usize {
        self.glonass_ephemeris.len()
    }

    pub fn get_beidou_ephemeris_count(&self) -> usize {
        self.beidou_ephemeris.len()
    }

    pub fn get_galileo_ephemeris_count(&self) -> usize {
        self.galileo_ephemeris.len()
    }

    pub fn has_gps_iono(&self) -> bool {
        self.gps_iono_alpha.is_some() && self.gps_iono_beta.is_some()
    }

    pub fn has_bds_iono(&self) -> bool {
        // В текущей структуре нет BDS ионосферных параметров
        false
    }

    pub fn has_gal_iono(&self) -> bool {
        // В текущей структуре нет Galileo ионосферных параметров
        false
    }

    pub fn has_gps_utc(&self) -> bool {
        self.utc_param.is_some()
    }

    pub fn get_first_gps_ephemeris(&self) -> Option<&GpsEphemeris> {
        self.gps_ephemeris.first()
    }
}

pub struct CPowerControl;

impl Default for CPowerControl {
    fn default() -> Self {
        Self::new()
    }
}

impl CPowerControl {
    pub fn new() -> Self {
        CPowerControl {}
    }
}

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

/// Читает ограниченное количество навигационных данных из RINEX файла для валидации
/// max_per_system - максимальное количество эфемерид на каждую систему
pub fn read_nav_file_limited(nav_data: &mut CNavData, filename: &str, max_per_system: usize) {
    use std::fs::File;
    use std::io::{BufRead, BufReader};
    
    let file = match File::open(filename) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("Error: Unable to open navigation file: {} - {}", filename, e);
            return;
        }
    };
    
    let reader = BufReader::new(file);
    let mut lines = reader.lines();
    let mut header_complete = false;
    
    // Счетчики для каждой системы
    let mut gps_count = 0;
    let mut glonass_count = 0; 
    let mut beidou_count = 0;
    let mut galileo_count = 0;
    
    while let Some(Ok(line)) = lines.next() {
        if !header_complete {
            if line.contains("END OF HEADER") {
                header_complete = true;
            } else if line.contains("IONOSPHERIC CORR") {
                if line.starts_with("GPSA") {
                    if let Some(iono_alpha) = parse_rinex3_iono_alpha(&line) {
                        nav_data.set_gps_iono_alpha(iono_alpha);
                    }
                } else if line.starts_with("GPSB") {
                    if let Some(iono_beta) = parse_rinex3_iono_beta(&line) {
                        nav_data.set_gps_iono_beta(iono_beta);
                    }
                }
            }
            continue;
        }
        
        // Проверяем лимиты
        if gps_count >= max_per_system && glonass_count >= max_per_system && 
           beidou_count >= max_per_system && galileo_count >= max_per_system {
            break;
        }
        
        let system_char = line.chars().next().unwrap_or(' ');
        
        match system_char {
            'G' | ' ' => {
                if gps_count < max_per_system {
                    println!("[DEBUG] Parsing GPS ephemeris {}/{}: {}", gps_count+1, max_per_system, &line[..std::cmp::min(20, line.len())]);
                    if let Some(eph) = parse_gps_ephemeris(&line, &mut lines) {
                        nav_data.add_gps_ephemeris(eph);
                        gps_count += 1;
                    }
                }
            },
            'R' => {
                if glonass_count < max_per_system {
                    println!("[DEBUG] Parsing GLONASS ephemeris {}/{}: {}", glonass_count+1, max_per_system, &line[..std::cmp::min(20, line.len())]);
                    if let Some(eph) = parse_glonass_ephemeris_correct(&line, &mut lines) {
                        nav_data.add_glonass_ephemeris(eph);
                        glonass_count += 1;
                    }
                }
            },
            'C' => {
                if beidou_count < max_per_system {
                    println!("[DEBUG] Parsing BeiDou ephemeris {}/{}: {}", beidou_count+1, max_per_system, &line[..std::cmp::min(20, line.len())]);
                    if let Some(mut eph) = parse_gps_ephemeris(&line, &mut lines) {
                        eph.toe = eph.toe - 14;
                        nav_data.add_beidou_ephemeris(eph);
                        beidou_count += 1;
                    }
                }
            },
            'E' => {
                if galileo_count < max_per_system {
                    println!("[DEBUG] Parsing Galileo ephemeris {}/{}: {}", galileo_count+1, max_per_system, &line[..std::cmp::min(20, line.len())]);
                    if let Some(eph) = parse_gps_ephemeris(&line, &mut lines) {
                        nav_data.add_galileo_ephemeris(eph);
                        galileo_count += 1;
                    }
                }
            },
            _ => continue,
        }
    }
    
    println!("[INFO] Limited parsing complete: GPS={}, GLONASS={}, BeiDou={}, Galileo={}", 
             gps_count, glonass_count, beidou_count, galileo_count);
}

/// Читает навигационные данные из RINEX файла
/// Поддерживает RINEX 2.x и 3.x форматы для GPS, ГЛОНАСС, BeiDou, Galileo
pub fn read_nav_file(nav_data: &mut CNavData, filename: &str) {
    use std::fs::File;
    use std::io::{BufRead, BufReader};
    
    let file = match File::open(filename) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("Error: Unable to open navigation file: {} - {}", filename, e);
            eprintln!("Cannot continue without navigation data. Exiting.");
            std::process::exit(1);
        }
    };
    
    let reader = BufReader::new(file);
    let mut lines = reader.lines();
    let mut header_complete = false;
    
    // Простая реализация RINEX парсера
    while let Some(Ok(line)) = lines.next() {
        // Пропустить заголовок до "END OF HEADER"
        if !header_complete {
            if line.contains("END OF HEADER") {
                header_complete = true;
            } else if line.contains("ION ALPHA") {
                // Парсинг ионосферных параметров альфа (RINEX 2.x)
                if let Some(iono_alpha) = parse_iono_alpha(&line) {
                    nav_data.set_gps_iono_alpha(iono_alpha);
                }
            } else if line.contains("ION BETA") {
                // Парсинг ионосферных параметров бета (RINEX 2.x)
                if let Some(iono_beta) = parse_iono_beta(&line) {
                    nav_data.set_gps_iono_beta(iono_beta);
                }
            } else if line.contains("IONOSPHERIC CORR") {
                // Парсинг ионосферных параметров (RINEX 3.x)
                if line.starts_with("GPSA") {
                    if let Some(iono_alpha) = parse_rinex3_iono_alpha(&line) {
                        nav_data.set_gps_iono_alpha(iono_alpha);
                        println!("[INFO] GPS iono alpha parameters parsed: {:?}", iono_alpha);
                    }
                } else if line.starts_with("GPSB") {
                    if let Some(iono_beta) = parse_rinex3_iono_beta(&line) {
                        nav_data.set_gps_iono_beta(iono_beta);
                        println!("[INFO] GPS iono beta parameters parsed: {:?}", iono_beta);
                    }
                }
            }
            continue;
        }
        
        // Определяем тип спутника по первому символу
        if line.len() < 80 {
            continue; // Пропустить короткие строки
        }
        
        let system_char = line.chars().next().unwrap_or(' ');
        match system_char {
            'G' | ' ' => {
                // GPS эфемериды
                println!("[DEBUG] Trying to parse GPS ephemeris: {}", &line[..std::cmp::min(20, line.len())]);
                if let Some(eph) = parse_gps_ephemeris(&line, &mut lines) {
                    nav_data.add_gps_ephemeris(eph);
                } else {
                    println!("[DEBUG] Failed to parse GPS ephemeris");
                }
            },
            'R' => {
                // ГЛОНАСС эфемериды
                println!("[DEBUG] Trying to parse GLONASS ephemeris: {}", &line[..std::cmp::min(20, line.len())]);
                if let Some(eph) = parse_glonass_ephemeris_correct(&line, &mut lines) {
                    nav_data.add_glonass_ephemeris(eph);
                } else {
                    println!("[DEBUG] Failed to parse GLONASS ephemeris");
                }
            },
            'C' => {
                // BeiDou эфемериды (используем GPS формат)
                println!("[DEBUG] Trying to parse BeiDou ephemeris: {}", &line[..std::cmp::min(20, line.len())]);
                if let Some(mut eph) = parse_gps_ephemeris(&line, &mut lines) {
                    // Конвертируем в BeiDou формат
                    eph.toe = eph.toe - 14; // BDT отстает от GPS времени на 14 секунд
                    nav_data.add_beidou_ephemeris(eph);
                } else {
                    println!("[DEBUG] Failed to parse BeiDou ephemeris");
                }
            },
            'E' => {
                // Galileo эфемериды (используем GPS формат)
                println!("[DEBUG] Trying to parse Galileo ephemeris: {}", &line[..std::cmp::min(20, line.len())]);
                if let Some(eph) = parse_gps_ephemeris(&line, &mut lines) {
                    // Конвертируем в Galileo формат (GST синхронизован с GPS)
                    nav_data.add_galileo_ephemeris(eph);
                } else {
                    println!("[DEBUG] Failed to parse Galileo ephemeris");
                }
            },
            _ => {
                // Неизвестный тип, пропускаем
                continue;
            }
        }
    }
    
    println!("[INFO]\tNavigation file loaded successfully: {}", filename);
}

/// Читает альманахи спутников из файла
/// Поддерживает стандартные форматы альманахов GPS, ГЛОНАСС, BeiDou, Galileo
fn read_alm_file(nav_data: &mut CNavData, filename: &str) {
    use std::fs::File;
    use std::io::{BufRead, BufReader};
    
    let file = match File::open(filename) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("Error: Unable to open almanac file: {} - {}", filename, e);
            return;
        }
    };
    
    let reader = BufReader::new(file);
    let mut lines = reader.lines();
    
    // Читаем файл построчно и определяем тип по формату строк
    while let Some(Ok(line)) = lines.next() {
        // GPS YUMA format
        if line.starts_with("*****") {
            if let Some(alm) = parse_gps_almanac(&line, &mut lines) {
                nav_data.add_gps_almanac(alm);
            }
        }
        // GLONASS almanac (simple format) - проверяем что первое слово - число (slot)
        else if line.split_whitespace().count() >= 2 {
            if let Ok(_slot) = line.split_whitespace().next().unwrap_or("").parse::<i16>() {
                if let Some(alm) = parse_glonass_almanac(&line, &mut lines) {
                    nav_data.add_glonass_almanac(alm);
                }
            }
        }
    }
    
    println!("[INFO]\tAlmanac file loaded successfully: {}", filename);
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

fn process_mask_out(object: *mut JsonObject, output_param: &mut OutputParam) -> bool {
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
            if (1..=32).contains(&svid) {
                output_param.gps_mask_out |= 1 << (svid - 1);
            }
        },
        1 => { // BdsSystem
            if (1..=63).contains(&svid) {
                output_param.bds_mask_out |= 1u64 << (svid - 1);
            }
        },
        2 => { // GalileoSystem
            if (1..=50).contains(&svid) {
                output_param.galileo_mask_out |= 1u64 << (svid - 1);
            }
        },
        3 => { // GlonassSystem
            if (1..=24).contains(&svid) {
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

// ============================================================================
// RINEX парсинг - вспомогательные функции и типы
// ============================================================================

/// Тип альманаха для определения формата файла
#[derive(Debug, PartialEq)]
enum AlmanacType {
    Gps,
    Glonass, 
    BdsSystem,
    Galileo,
    Unknown,
}

/// Определяет тип альманаха по содержимому файла
fn detect_almanac_type<I>(lines: &mut I) -> AlmanacType 
where
    I: Iterator<Item = Result<String, std::io::Error>>
{
    // Упрощенная реализация - по умолчанию GPS
    // В реальной версии нужно анализировать заголовок файла
    AlmanacType::Gps
}

/// Парсит ионосферные параметры альфа из RINEX заголовка
fn parse_iono_alpha(line: &str) -> Option<[f64; 4]> {
    // Простой парсинг - в реальной версии нужно точнее разбирать RINEX формат
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() >= 4 {
        let mut alpha = [0.0; 4];
        for (i, part) in parts.iter().take(4).enumerate() {
            if let Ok(value) = part.replace("D", "E").parse::<f64>() {
                alpha[i] = value;
            }
        }
        Some(alpha)
    } else {
        None
    }
}

/// Парсит ионосферные параметры бета из RINEX заголовка  
fn parse_iono_beta(line: &str) -> Option<[f64; 4]> {
    // Аналогично parse_iono_alpha
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() >= 4 {
        let mut beta = [0.0; 4];
        for (i, part) in parts.iter().take(4).enumerate() {
            if let Ok(value) = part.replace("D", "E").parse::<f64>() {
                beta[i] = value;
            }
        }
        Some(beta)
    } else {
        None
    }
}

/// Парсит ионосферные параметры alpha из RINEX 3.x формата (GPSA строка)
fn parse_rinex3_iono_alpha(line: &str) -> Option<[f64; 4]> {
    // Формат: GPSA   1.5832e-08  2.2352e-08 -1.1921e-07 -1.1921e-07       IONOSPHERIC CORR
    let data_part = &line[4..60]; // Пропускаем "GPSA" и берем данные
    let parts: Vec<&str> = data_part.split_whitespace().collect();
    if parts.len() >= 4 {
        let mut alpha = [0.0; 4];
        for (i, part) in parts.iter().take(4).enumerate() {
            if let Ok(value) = part.replace("D", "E").parse::<f64>() {
                alpha[i] = value;
            } else {
                return None;
            }
        }
        Some(alpha)
    } else {
        None
    }
}

/// Парсит ионосферные параметры beta из RINEX 3.x формата (GPSB строка)
fn parse_rinex3_iono_beta(line: &str) -> Option<[f64; 4]> {
    // Формат: GPSB   1.1264e+05  1.4746e+05 -1.3107e+05 -3.9322e+05       IONOSPHERIC CORR
    let data_part = &line[4..60]; // Пропускаем "GPSB" и берем данные
    let parts: Vec<&str> = data_part.split_whitespace().collect();
    if parts.len() >= 4 {
        let mut beta = [0.0; 4];
        for (i, part) in parts.iter().take(4).enumerate() {
            if let Ok(value) = part.replace("D", "E").parse::<f64>() {
                beta[i] = value;
            } else {
                return None;
            }
        }
        Some(beta)
    } else {
        None
    }
}

/// Парсит GPS эфемериды из RINEX формата
fn parse_gps_ephemeris<I>(line: &str, lines: &mut I) -> Option<GpsEphemeris>
where
    I: Iterator<Item = Result<String, std::io::Error>>
{
    let mut eph = GpsEphemeris::default();
    let mut data = [0.0f64; 32]; // Массив для всех данных эфемерид
    
    // Парсим первую строку используя точную логику C++
    let svid = read_contents_time(line, &mut data[0..3])?;
    eph.svid = svid;
    eph.af0 = data[0];
    eph.af1 = data[1]; 
    eph.af2 = data[2];
    
    // Читаем 7 строк данных (каждая содержит 4 значения)
    for i in 0..7 {
        if let Some(Ok(data_line)) = lines.next() {
            if i*4+3 + 4 <= data.len() {
                read_contents_data(&data_line, &mut data[i*4+3..i*4+7]);
            }
        } else {
            println!("[DEBUG] Failed to read data line {}", i+1);
            return None;
        }
    }
    
    // Заполняем структуру эфемерид точно как в C++ (NavDataGpsLnav case)
    eph.toc = 0; // Будет заполнено позже
    eph.sqrtA = data[10];
    eph.ecc = data[8];
    eph.i0 = data[15];
    eph.omega0 = data[13];
    eph.w = data[17];
    eph.M0 = data[6];
    eph.delta_n = data[5];
    eph.omega_dot = data[18];
    eph.idot = data[19];
    eph.crc = data[16];
    eph.crs = data[4];
    eph.cuc = data[7];
    eph.cus = data[9];
    eph.cic = data[12];
    eph.cis = data[14];
    eph.toe = (data[11] + 0.5) as i32;
    
    // Специфичные для GPS LNAV параметры
    if data.len() > 26 {
        eph.iodc = data[26] as u16;
    }
    eph.iode = data[3] as u8;
    if data.len() > 21 {
        eph.week = data[21] as i32;
    }
    if data.len() > 24 {
        eph.health = data[24] as u16;
    }
    if data.len() > 23 {
        eph.ura = get_ura_index(data[23]) as i16;
    }
    if data.len() > 25 {
        eph.tgd = data[25];
    }
    
    // Вычисляем производные значения
    eph.axis = eph.sqrtA * eph.sqrtA;
    eph.n = (EARTH_GM / (eph.axis * eph.axis * eph.axis)).sqrt() + eph.delta_n;
    eph.root_ecc = (1.0 - eph.ecc * eph.ecc).sqrt();
    
    eph.valid = 1;
    eph.flag = 1;
    
    println!("[DEBUG] Successfully parsed GPS ephemeris for SVID {}", eph.svid);
    Some(eph)
}

// GLONASS ephemeris parser
fn parse_glonass_ephemeris<I>(line: &str, lines: &mut I) -> Option<GlonassEphemeris>
where 
    I: Iterator<Item = Result<String, std::io::Error>>
{
    // Parse first line: SVID, UTC time, clock corrections
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() < 8 { return None; }
    
    // Extract SVID
    let svid_str = if parts[0].len() >= 2 { &parts[0][1..] } else { return None; };
    let svid: u16 = svid_str.parse().ok()?;
    
    // Parse UTC time (year, month, day, hour, minute, second)
    let _year: u16 = parts[1].parse().ok()?;
    let _month: u16 = parts[2].parse().ok()?;
    let _day: u16 = parts[3].parse().ok()?;
    let hour: u16 = parts[4].parse().ok()?;
    let minute: u16 = parts[5].parse().ok()?;
    let second: f64 = parts[6].parse().ok()?;
    
    // Parse first line data (clock bias, relative frequency bias, message frame time)
    let mut data = Vec::new();
    for i in 7..parts.len().min(10) {
        data.push(parts[i].replace("D", "E").parse::<f64>().ok()?);
    }
    
    // Read next 3 lines of data (4 values each)
    for _ in 0..3 {
        let next_line = lines.next()?.ok()?;
        let next_parts: Vec<&str> = next_line.split_whitespace().collect();
        if next_parts.len() < 4 { return None; }
        
        for j in 0..4 {
            data.push(next_parts[j].replace("D", "E").parse::<f64>().ok()?);
        }
    }
    
    // Validate we have all 15 data values
    if data.len() < 15 { return None; }
    
    // Convert tk from seconds of day to GLONASS tk format
    let tk_seconds = data[2] as i32;
    let hours_tk = tk_seconds / 3600;
    let minutes_tk = (tk_seconds % 3600) / 60;  
    let half_minutes = (tk_seconds % 60) / 30;
    let tk = ((hours_tk << 7) | (minutes_tk << 1) | half_minutes) as u16;
    
    // Create GLONASS ephemeris structure
    let mut eph = GlonassEphemeris {
        flag: 1,
        valid: 1,
        slot: svid as u8,
        freq: data[10] as i8,
        tk,
        P: 0xc0,  // P=11, ln=0, P4=0, P3=0, P2=0, P1=00 (will update P2 after tb)
        M: 1,     // assume GLONASS-M satellite
        Ft: 0,    // no data
        n: svid as u8,  // satellite number
        Bn: data[6] as u8,
        En: data[14] as u8,
        tb: 0,    // will be calculated below
        day: 1,   // placeholder - would need proper GLONASS time conversion
        gamma: data[1],
        tn: -data[0],
        dtn: 0.0, // no data in RINEX
        x: data[3] * 1e3,
        y: data[7] * 1e3,
        z: data[11] * 1e3,
        vx: data[4] * 1e3,
        vy: data[8] * 1e3,
        vz: data[12] * 1e3,
        ax: data[5] * 1e3,
        ay: data[9] * 1e3,
        az: data[13] * 1e3,
        tc: 0.0,  // derived variable
        PosVelT: KinematicInfo::default(),  // derived variable
    };
    
    // Calculate tb (reference time in seconds, aligned to 15-minute intervals)
    let millis = (hour as i32 * 3600 + minute as i32 * 60 + second as i32) * 1000;
    eph.tb = ((millis + 450000) / 900000 * 900) as u32;
    
    // Update P2 bit based on tb (P2 = 1 if tb interval number is odd)
    if (eph.tb / 900) & 1 != 0 {
        eph.P |= 0x04; // Set P2 bit
    }
    
    Some(eph)
}

// GPS almanac parser (simplified YUMA format)
fn parse_gps_almanac<I>(line: &str, lines: &mut I) -> Option<GpsAlmanac>
where 
    I: Iterator<Item = Result<String, std::io::Error>>
{
    // YUMA format starts with "*****"
    if !line.starts_with("*****") {
        return None;
    }
    
    let mut alm = GpsAlmanac::default();
    
    // Read 13 lines of data
    let mut data_lines = Vec::new();
    for _ in 0..13 {
        if let Some(Ok(data_line)) = lines.next() {
            data_lines.push(data_line);
        } else {
            return None;
        }
    }
    
    if data_lines.len() < 13 {
        return None;
    }
    
    // Parse each field (data starts at column 27)
    alm.svid = data_lines[0][27..].trim().parse().ok()?;
    alm.health = data_lines[1][27..].trim().parse().ok()?;
    alm.ecc = data_lines[2][27..].trim().parse().ok()?;
    alm.toa = data_lines[3][27..].trim().parse().ok()?;
    alm.i0 = data_lines[4][27..].trim().parse().ok()?;
    alm.omega_dot = data_lines[5][27..].trim().parse().ok()?;
    alm.sqrtA = data_lines[6][27..].trim().parse().ok()?;
    alm.omega0 = data_lines[7][27..].trim().parse().ok()?;
    alm.w = data_lines[8][27..].trim().parse().ok()?;
    alm.M0 = data_lines[9][27..].trim().parse().ok()?;
    alm.af0 = data_lines[10][27..].trim().parse().ok()?;
    alm.af1 = data_lines[11][27..].trim().parse().ok()?;
    alm.week = data_lines[12][27..].trim().parse().ok()?;
    
    alm.valid = 1;
    
    Some(alm)
}

// GLONASS almanac parser (simplified format)
fn parse_glonass_almanac<I>(line: &str, lines: &mut I) -> Option<GlonassAlmanac>
where 
    I: Iterator<Item = Result<String, std::io::Error>>
{
    // Simple format parser - looking for slot number and data
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() < 2 {
        return None;
    }
    
    let slot: i16 = parts[0].parse().ok()?;
    
    // Read multiple lines to get all almanac data
    let mut all_data: Vec<String> = Vec::new();
    for part in &parts[1..] {
        all_data.push(part.to_string());
    }
    
    // Read additional lines if needed
    for _ in 0..3 {
        if let Some(Ok(data_line)) = lines.next() {
            let line_parts: Vec<String> = data_line.split_whitespace().map(|s| s.to_string()).collect();
            for part in line_parts {
                all_data.push(part);
            }
        }
    }
    
    if all_data.len() < 8 {
        return None;
    }
    
    let mut alm = GlonassAlmanac::default();
    alm.flag = 1;
    
    // Parse GLONASS almanac fields (simplified)
    if let Ok(freq) = all_data[0].parse::<i8>() {
        alm.freq = freq;
    }
    if let Ok(day) = all_data[1].parse::<i16>() {
        alm.day = day;
    }
    if let Ok(t) = all_data[2].parse::<f64>() {
        alm.t = t;
    }
    if let Ok(lambda) = all_data[3].parse::<f64>() {
        alm.lambda = lambda;
    }
    if let Ok(di) = all_data[4].parse::<f64>() {
        alm.di = di;
    }
    if let Ok(ecc) = all_data[5].parse::<f64>() {
        alm.ecc = ecc;
    }
    if let Ok(w) = all_data[6].parse::<f64>() {
        alm.w = w;
    }
    if let Ok(dt) = all_data[7].parse::<f64>() {
        alm.dt = dt;
    }
    
    alm.leap_year = 0; // Would need proper calculation
    
    Some(alm)
}

/// Парсит BeiDou эфемериды из RINEX формата
fn parse_beidou_ephemeris<I>(line: &str, lines: &mut I) -> Option<GpsEphemeris>
where
    I: Iterator<Item = Result<String, std::io::Error>>
{
    // BeiDou uses similar format to GPS but with system-specific adjustments
    let mut eph = parse_gps_ephemeris(line, lines)?;
    
    // Convert to BeiDou-specific parameters
    // BeiDou uses BDT (BeiDou Time) which differs from GPS time
    eph.toe -= 14; // BDT is 14 seconds behind GPS time
    eph.top -= 14;
    
    // BeiDou week number uses different epoch (January 1, 2006)
    eph.week = eph.week.saturating_sub(1356); // Convert GPS week to BDS week
    
    // BeiDou uses CGCS2000 coordinate system (very similar to WGS84)
    // No significant adjustments needed for most applications
    
    Some(eph)
}

/// Парсит Galileo эфемериды из RINEX формата
fn parse_galileo_ephemeris<I>(line: &str, lines: &mut I) -> Option<GpsEphemeris>
where
    I: Iterator<Item = Result<String, std::io::Error>>
{
    // Galileo uses GPS-compatible ephemeris format with system-specific adjustments
    let mut eph = parse_gps_ephemeris(line, lines)?;
    
    // Convert to Galileo-specific parameters
    // Galileo System Time (GST) epoch: August 22, 1999 00:00:00 UTC
    // GST is synchronized with TAI (continuous time scale)
    
    // Galileo week number starts from GST epoch
    // GPS week 1024 corresponds to GST week 0 (August 22, 1999)
    if eph.week >= 1024 {
        eph.week -= 1024; // Convert GPS week to GST week
    }
    
    // Galileo uses essentially the same coordinate system as GPS (WGS84)
    // Signal-in-space accuracy (SISA) mapping may differ from GPS URA
    
    // Galileo-specific health flags and data validity flags
    // would need to be handled differently in full implementation
    
    Some(eph)
}


/// Парсит BeiDou альманах
fn parse_beidou_almanac<I>(lines: &mut I) -> Option<Vec<GpsAlmanac>>
where
    I: Iterator<Item = Result<String, std::io::Error>>
{
    let mut almanacs = Vec::new();
    
    while let Some(Ok(line)) = lines.next() {
        if line.starts_with("*****") {
            // BeiDou YUMA format similar to GPS
            if let Some(mut alm) = parse_gps_almanac(&line, lines) {
                // Convert to BeiDou format
                alm.week = alm.week.saturating_sub(1356); // Convert to BDS week
                
                if alm.svid <= 63 { // BeiDou supports up to 63 satellites
                    almanacs.push(alm);
                }
            }
        }
    }
    
    if !almanacs.is_empty() {
        Some(almanacs)
    } else {
        None
    }
}

/// Парсит Galileo альманах
fn parse_galileo_almanac<I>(lines: &mut I) -> Option<Vec<GpsAlmanac>>
where
    I: Iterator<Item = Result<String, std::io::Error>>
{
    let mut almanacs = Vec::new();
    
    while let Some(Ok(line)) = lines.next() {
        if line.starts_with("*****") {
            // Galileo YUMA format similar to GPS
            if let Some(mut alm) = parse_gps_almanac(&line, lines) {
                // Convert to Galileo format
                if alm.week >= 1024 {
                    alm.week -= 1024; // Convert to GST week
                }
                
                if alm.svid <= 36 { // Galileo currently has up to 36 satellites
                    almanacs.push(alm);
                }
            }
        }
    }
    
    if !almanacs.is_empty() {
        Some(almanacs)
    } else {
        None
    }
}

/// Парсит первую строку RINEX эфемериды (время + 3 параметра) - ТОЧНО как в C++
fn read_contents_time(line: &str, data: &mut [f64]) -> Option<u8> {
    if line.len() < 23 {
        return None;
    }
    
    // Извлекаем SVID из позиций 1-2 (как в C++)
    let svid = if line.len() > 2 && !line.chars().nth(1).unwrap_or(' ').is_whitespace() {
        line[1..3].trim().parse::<u8>().unwrap_or(0)
    } else {
        0
    };
    
    // Конвертируем D в E для экспоненциального формата
    let line = line.replace("D", "E");
    
    // Читаем af0, af1, af2 с ТОЧНЫХ позиций как в C++ (позиции 23-42, 42-61, 61-80)
    if line.len() > 42 {
        data[0] = line[23..42].trim().parse().unwrap_or(0.0);
    }
    if line.len() > 61 {
        data[1] = line[42..61].trim().parse().unwrap_or(0.0);
    }
    if line.len() > 80 {
        data[2] = line[61..80].trim().parse().unwrap_or(0.0);
    } else if line.len() > 61 {
        data[2] = line[61..].trim().parse().unwrap_or(0.0);
    }
    
    Some(svid)
}

/// Парсит строку данных RINEX (4 параметра с ТОЧНЫХ позиций) - как в C++
fn read_contents_data(line: &str, data: &mut [f64]) {
    // Конвертируем D в E для экспоненциального формата  
    let line = line.replace("D", "E");
    
    // Читаем 4 значения с ТОЧНЫХ позиций как в C++ (позиции 4-23, 23-42, 42-61, 61-80)
    if line.len() > 23 {
        data[0] = line[4..23].trim().parse().unwrap_or(0.0);
    }
    if line.len() > 42 {
        data[1] = line[23..42].trim().parse().unwrap_or(0.0);
    }
    if line.len() > 61 {
        data[2] = line[42..61].trim().parse().unwrap_or(0.0);
    }
    if line.len() > 80 {
        data[3] = line[61..80].trim().parse().unwrap_or(0.0);
    } else if line.len() > 61 {
        data[3] = line[61..].trim().parse().unwrap_or(0.0);
    }
}

/// Преобразует URA в индекс (из C++ версии)
fn get_ura_index(ura: f64) -> i32 {
    if ura <= 2.4 { (ura / 0.3 + 0.5) as i32 }
    else if ura <= 6.0 { ((ura - 2.4) / 0.6 + 8.5) as i32 }
    else if ura <= 12.0 { ((ura - 6.0) / 1.2 + 14.5) as i32 }
    else if ura <= 24.0 { ((ura - 12.0) / 2.4 + 19.5) as i32 }
    else if ura <= 48.0 { ((ura - 24.0) / 4.8 + 24.5) as i32 }
    else if ura <= 96.0 { ((ura - 48.0) / 9.6 + 29.5) as i32 }
    else if ura <= 192.0 { ((ura - 96.0) / 19.2 + 34.5) as i32 }
    else { 15 }
}

/// Правильный парсер GLONASS эфемерид по образцу C++ DecodeEphOrbit
fn parse_glonass_ephemeris_correct<I>(line: &str, lines: &mut I) -> Option<GlonassEphemeris>
where
    I: Iterator<Item = Result<String, std::io::Error>>
{
    let mut data = [0.0f64; 19];
    
    // Парсим первую строку (время + 3 параметра)
    let svid = read_contents_time(line, &mut data[0..3])?;
    
    // Читаем 3 строки данных (как в C++ версии)
    for i in 0..3 {
        if let Some(Ok(data_line)) = lines.next() {
            read_contents_data(&data_line, &mut data[i*4+3..i*4+7]);
        } else {
            println!("[DEBUG] Failed to read GLONASS data line {}", i+1);
            return None;
        }
    }
    
    // Создаем GLONASS эфемериду по образцу C++
    let mut eph = GlonassEphemeris {
        flag: 1,
        valid: 1,
        slot: svid as u8,
        n: svid as u8,
        freq: data[10] as i8,  // Частотный номер
        tk: 0,  // Будет вычислен ниже
        P: 0xc0,  // P=11, ln=0, P4=0, P3=0, P2=0, P1=00
        M: 1,     // GLONASS-M
        Ft: 0,    // Нет данных
        Bn: data[6] as u8,
        En: data[14] as u8, 
        tb: 0,    // Будет вычислен ниже
        day: 1,   // Заглушка
        gamma: data[1],
        tn: -data[0],  // Clock bias (знак меняется)
        dtn: 0.0,      // Нет в RINEX
        // Позиция (км -> м)
        x: data[3] * 1e3,
        y: data[7] * 1e3,
        z: data[11] * 1e3,
        // Скорость (км/с -> м/с) 
        vx: data[4] * 1e3,
        vy: data[8] * 1e3,
        vz: data[12] * 1e3,
        // Ускорение (км/с² -> м/с²)
        ax: data[5] * 1e3,
        ay: data[9] * 1e3,
        az: data[13] * 1e3,
        tc: 0.0,
        PosVelT: KinematicInfo::default(),
    };
    
    // Вычисляем tk из data[2] (секунды дня)
    let tk_seconds = data[2] as i32;
    let hours = tk_seconds / 3600;
    let minutes = (tk_seconds % 3600) / 60;
    let half_minutes = (tk_seconds % 60) / 30;
    eph.tk = ((hours << 7) | (minutes << 1) | half_minutes) as u16;
    
    // Вычисляем tb (опорное время в 15-минутных интервалах)  
    eph.tb = ((tk_seconds + 450) / 900 * 900) as u32;
    
    // Устанавливаем P2 бит в зависимости от tb
    if (eph.tb / 900) & 1 != 0 {
        eph.P |= 0x04; // Устанавливаем P2 бит
    }
    
    println!("[DEBUG] Successfully parsed GLONASS ephemeris for SVID {}", eph.n);
    Some(eph)
}