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

use crate::constants::CGCS2000_OMEGDOTE;
use crate::constants::EARTH_GM;
use crate::powercontrol::SignalPower;
use crate::trajectory::CTrajectory;
use crate::types::*;
use crate::types::{
    BeiDouEphemeris, GlonassAlmanac, GlonassEphemeris, GpsAlmanac, GpsEphemeris, UtcParam,
};
use crate::utc_to_gps_time;
use crate::{
    bds_time_to_utc, ecef_to_lla, glonass_time_to_utc, gps_time_to_utc, lla_to_ecef,
    speed_ecef_to_local, WGS_OMEGDOTE,
};
use crate::{CPowerControl, JsonObject};
use std::os::raw::c_char;

// Временные алиасы для недостающих типов - в будущем нужно реализовать отдельные структуры
type GalileoEphemeris = GpsEphemeris; // Galileo также схож с GPS
type BdsAlmanac = GpsAlmanac;
type GalileoAlmanac = GpsAlmanac;

// Constants for dictionary lookups
static KEY_DICTIONARY_LIST_PARAM: &[&str] = &[
    "time",
    "trajectory",
    "ephemeris",
    "almanac",
    "output",
    "power",
    "delay",
];

static KEY_DICTIONARY_LIST_TIME: &[&str] = &[
    "type", "week", "second", "leapYesr", "day", "year", "month", "hour", "minute",
];

static KEY_DICTIONARY_LIST_TRAJECTORY: &[&str] = &[
    "name",
    "initPosition",
    "initVelocity",
    "trajectoryList",
    "type",
    "format",
    "longitude",
    "latitude",
    "altitude",
    "x",
    "y",
    "z",
    "speedUnit",
    "angleUnit",
    "speed",
    "course",
    "east",
    "north",
    "up",
];

static KEY_DICTIONARY_LIST_OUTPUT: &[&str] = &[
    "type",
    "format",
    "name",
    "interval",
    "config",
    "systemSelect",
    "elevationMask",
    "maskOut",
    "system",
    "svid",
    "signal",
    "enable",
    "sampleFreq",
    "centerFreq",
];

static KEY_DICTIONARY_LIST_POWER: &[&str] = &[
    "noiseFloor",
    "initPower",
    "elevationAdjust",
    "signalPower",
    "unit",
    "value",
    "system",
    "svid",
    "powerValue",
    "time",
];

static DICTIONARY_LIST_SYSTEM: &[&str] = &["UTC", "GPS", "BDS", "Galileo", "GLONASS"];

static DICTIONARY_LIST_COORDINATE: &[&str] = &[
    "LLA", "ECEF", "SCU", "ENU", "d", "dm", "dms", "rad", "degree", "mps", "kph", "knot", "mph",
];

static DICTIONARY_LIST_OUTPUT_TYPE: &[&str] = &["position", "observation", "IFdata", "baseband"];

static DICTIONARY_LIST_OUTPUT_FORMAT: &[&str] =
    &["ECEF", "LLA", "NMEA", "KML", "RINEX", "IQ8", "IQ4"];

static DICTIONARY_LIST_SIGNAL: &[&str] = &[
    "L1CA", "L1C", "L2C", "L2P", "L5", "", "", "", "B1C", "B1I", "B2I", "B3I", "B2a", "B2b", "",
    "", "E1", "E5a", "E5b", "E5", "E6", "", "", "", "G1", "G2", "G3", "", "", "", "", "",
];

static DICTIONARY_LIST_POWER_UNIT: &[&str] = &["dBHz", "dBm", "dBW"];

// Use types from types.rs instead of redefining them

#[repr(C)]
pub struct COutputParam {
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
pub struct CDelayConfig {
    // Add fields as needed
}

// SignalPower is imported from powercontrol module

// Use ConvertMatrix from types.rs

// Enums
// TrajectoryType imported from trajectory.rs

// TrajectoryDataType imported from trajectory.rs

// GnssSystem is imported from types.rs

// OutputType imported from types.rs

// OutputFormat imported from types.rs

// ElevationAdjust imported from powercontrol.rs

// JsonObject imported from json_parser.rs
// Структура для хранения навигационных данных
pub struct CNavData {
    // Ионосферные параметры GPS
    pub gps_iono_alpha: Option<[f64; 4]>,
    pub gps_iono_beta: Option<[f64; 4]>,

    // Эфемериды различных ГНСС систем
    pub gps_ephemeris: Vec<GpsEphemeris>,
    pub glonass_ephemeris: Vec<GlonassEphemeris>,
    pub beidou_ephemeris: Vec<BeiDouEphemeris>,
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
        println!(
            "[INFO] Added GPS ephemeris, total: {}",
            self.gps_ephemeris.len()
        );
    }

    pub fn add_glonass_ephemeris(&mut self, eph: GlonassEphemeris) {
        self.glonass_ephemeris.push(eph);
        println!(
            "[INFO] Added GLONASS ephemeris, total: {}",
            self.glonass_ephemeris.len()
        );
    }

    pub fn add_beidou_ephemeris(&mut self, eph: BeiDouEphemeris) {
        self.beidou_ephemeris.push(eph);
        println!(
            "[INFO] Added BeiDou ephemeris, total: {}",
            self.beidou_ephemeris.len()
        );
    }

    pub fn add_galileo_ephemeris(&mut self, eph: GalileoEphemeris) {
        self.galileo_ephemeris.push(eph);
        println!(
            "[INFO] Added Galileo ephemeris, total: {}",
            self.galileo_ephemeris.len()
        );
    }

    // Методы для добавления альманахов
    pub fn add_gps_almanac(&mut self, alm: GpsAlmanac) {
        self.gps_almanac.push(alm);
        println!(
            "[INFO] Added GPS almanac, total: {}",
            self.gps_almanac.len()
        );
    }

    pub fn add_glonass_almanac(&mut self, alm: GlonassAlmanac) {
        self.glonass_almanac.push(alm);
        println!(
            "[INFO] Added GLONASS almanac, total: {}",
            self.glonass_almanac.len()
        );
    }

    pub fn add_beidou_almanac(&mut self, alm: BdsAlmanac) {
        self.beidou_almanac.push(alm);
        println!(
            "[INFO] Added BeiDou almanac, total: {}",
            self.beidou_almanac.len()
        );
    }

    pub fn add_galileo_almanac(&mut self, alm: GalileoAlmanac) {
        self.galileo_almanac.push(alm);
        println!(
            "[INFO] Added Galileo almanac, total: {}",
            self.galileo_almanac.len()
        );
    }

    // Методы получения данных
    pub fn get_gps_ephemeris(&self) -> &[GpsEphemeris] {
        &self.gps_ephemeris
    }

    pub fn get_glonass_ephemeris(&self) -> &[GlonassEphemeris] {
        &self.glonass_ephemeris
    }

    pub fn get_beidou_ephemeris(&self) -> &[BeiDouEphemeris] {
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

// CPowerControl imported from powercontrol.rs

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
use crate::dprintln;

// External function declarations (would be implemented elsewhere)
#[allow(improper_ctypes)]
extern "C" {
    fn json_stream_get_first_object(object: *mut JsonObject) -> *mut JsonObject;
    fn json_stream_get_next_object(object: *mut JsonObject) -> *mut JsonObject;
}

// Main function - equivalent to AssignParameters
pub unsafe fn assign_parameters(
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
                0 => {
                    // "time"
                    if let Some(utc) = utc_time.as_mut() {
                        assign_start_time(json_stream_get_first_object(current_object), utc);
                    }
                }
                1 => {
                    // "trajectory"
                    if let (Some(pos), Some(vel), Some(traj)) =
                        (start_pos.as_mut(), start_vel.as_mut(), trajectory.as_mut())
                    {
                        set_trajectory(
                            json_stream_get_first_object(current_object),
                            pos,
                            vel,
                            traj,
                        );
                    }
                }
                2 => {
                    // "ephemeris"
                    if let Some(nav) = nav_data.as_mut() {
                        set_ephemeris(current_object, nav);
                    }
                }
                3 => {
                    // "almanac"
                    if let Some(nav) = nav_data.as_mut() {
                        set_almanac(current_object, nav);
                    }
                }
                4 => {
                    // "output"
                    if let Some(output) = output_param.as_mut() {
                        set_output_param(json_stream_get_first_object(current_object), output);
                    }
                }
                5 => {
                    // "power"
                    if let Some(power) = power_control.as_mut() {
                        set_power_control(json_stream_get_first_object(current_object), power);
                    }
                }
                6 => {
                    // "delay"
                    if let Some(delay) = delay_config.as_mut() {
                        set_delay_config(json_stream_get_first_object(current_object), delay);
                    }
                }
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
                0 => {
                    // "type"
                    if is_string_type(current_object) {
                        let type_str = get_object_string(current_object);
                        time_type = search_dictionary(&type_str, DICTIONARY_LIST_SYSTEM);
                    }
                }
                1 => {
                    // "week"
                    week = get_object_int(current_object);
                }
                2 => {
                    // "second"
                    utc_time.Second = get_double_value(current_object);
                }
                3 => {
                    // "leapYear"
                    leap_year = get_object_int(current_object);
                }
                4 => {
                    // "day"
                    utc_time.Day = get_object_int(current_object);
                }
                5 => {
                    // "year"
                    utc_time.Year = get_object_int(current_object);
                }
                6 => {
                    // "month"
                    utc_time.Month = get_object_int(current_object);
                }
                7 => {
                    // "hour"
                    utc_time.Hour = get_object_int(current_object);
                }
                8 => {
                    // "minute"
                    utc_time.Minute = get_object_int(current_object);
                }
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }

    match time_type {
        1 | 3 => {
            // GPS Time or Galileo Time
            gnss_time.Week = week;
            gnss_time.SubMilliSeconds = utc_time.Second * 1000.0;
            gnss_time.MilliSeconds = gnss_time.SubMilliSeconds as i32;
            gnss_time.SubMilliSeconds -= gnss_time.MilliSeconds as f64;
            *utc_time = gps_time_to_utc(gnss_time, true);
        }
        2 => {
            // BDS Time
            gnss_time.Week = week;
            gnss_time.SubMilliSeconds = utc_time.Second * 1000.0;
            gnss_time.MilliSeconds = gnss_time.SubMilliSeconds as i32;
            gnss_time.SubMilliSeconds -= gnss_time.MilliSeconds as f64;
            *utc_time = bds_time_to_utc(gnss_time);
        }
        4 => {
            // GLONASS Time
            glonass_time.LeapYear = leap_year;
            glonass_time.Day = utc_time.Day;
            glonass_time.SubMilliSeconds = utc_time.Second * 1000.0;
            glonass_time.MilliSeconds = glonass_time.SubMilliSeconds as i32;
            glonass_time.SubMilliSeconds -= glonass_time.MilliSeconds as f64;
            *utc_time = glonass_time_to_utc(glonass_time);
        }
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
            eprintln!(
                "Error: Unable to open navigation file: {} - {}",
                filename, e
            );
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

        // Удаляем раннее завершение - пусть парсер дочитает файл до конца
        // чтобы найти BeiDou и Galileo данные в конце RINEX файла
        // Старый код прерывал парсинг слишком рано!

        let system_char = line.chars().next().unwrap_or(' ');

        match system_char {
            'G' | ' ' => {
                if gps_count < max_per_system {
                    // GPS ephemeris parsing - НЕ добавляем напрямую, только группируем по эпохам
                    if let Some(_eph) = parse_gps_ephemeris(&line, &mut lines) {
                        // Прямое добавление в nav_data УДАЛЕНО - будет добавлено в критической секции
                        gps_count += 1;
                    }
                }
            }
            'R' => {
                if glonass_count < max_per_system {
                    // GLONASS ephemeris parsing...
                    if let Some(eph) = parse_glonass_ephemeris_correct(&line, &mut lines) {
                        nav_data.add_glonass_ephemeris(eph);
                        glonass_count += 1;
                    }
                }
            }
            'C' => {
                if beidou_count < max_per_system {
                    // BeiDou ephemeris parsing - НЕ добавляем напрямую, только группируем по эпохам
                    if let Some(_eph) = parse_beidou_ephemeris(&line, &mut lines) {
                        // Прямое добавление в nav_data УДАЛЕНО - будет добавлено в критической секции
                        beidou_count += 1;
                    }
                }
            }
            'E' => {
                if galileo_count < max_per_system {
                    // Galileo ephemeris parsing...
                    if let Some(eph) = parse_galileo_ephemeris(&line, &mut lines) {
                        nav_data.add_galileo_ephemeris(eph);
                        galileo_count += 1;
                    }
                }
            }
            _ => continue,
        }
    }

    println!(
        "[INFO] Limited parsing complete: GPS={}, GLONASS={}, BeiDou={}, Galileo={}",
        gps_count, glonass_count, beidou_count, galileo_count
    );
}

/// КРИТИЧЕСКАЯ ФУНКЦИЯ ПАРСИНГА RINEX: Читает навигационные данные из RINEX файла с фильтрацией
///
/// ЭТА ФУНКЦИЯ ЯВЛЯЕТСЯ КЛЮЧЕВОЙ ДЛЯ ЗАГРУЗКИ ЭФЕМЕРИД!
///
/// Поддерживает RINEX 2.x и 3.x форматы для GPS, ГЛОНАСС, BeiDou, Galileo
///
/// # Параметры
/// * `nav_data` - Структура для хранения загруженных эфемерид
/// * `filename` - Путь к RINEX файлу (из JSON конфигурации)  
/// * `target_time` - UTC время из пресета для фильтрации эфемерид по времени
/// * `enabled_systems` - Массив включенных GNSS систем из JSON конфигурации
///
/// # Критический процесс:
/// 1. Открывает RINEX файл и парсит заголовок
/// 2. Читает строки эфемерид для каждой системы (GPS: G##, GAL: E##, BDS: C##)
/// 3. Фильтрует эфемериды по времени (выбирает ближайшие к target_time)
/// 4. Сохраняет отобранные эфемериды в nav_data структуру
pub fn read_nav_file_filtered(
    nav_data: &mut CNavData,
    filename: &str,
    target_time: Option<UtcTime>,
    enabled_systems: &[&str],
) {
    use std::fs::File;
    use std::io::{BufRead, BufReader};

    let file = match File::open(filename) {
        Ok(f) => f,
        Err(e) => {
            eprintln!(
                "Error: Unable to open navigation file: {} - {}",
                filename, e
            );
            eprintln!("Cannot continue without navigation data. Exiting.");
            std::process::exit(1);
        }
    };

    let reader = BufReader::new(file);
    let mut lines = reader.lines();
    let mut header_complete = false;

    // Создаём фильтр систем для быстрой проверки
    let gps_enabled = enabled_systems.contains(&"GPS");
    let glonass_enabled = enabled_systems.contains(&"GLONASS");
    let beidou_enabled = enabled_systems.contains(&"BeiDou");
    let galileo_enabled = enabled_systems.contains(&"Galileo");

    println!(
        "[INFO] RINEX filtering: GPS={}, GLONASS={}, BeiDou={}, Galileo={}",
        gps_enabled, glonass_enabled, beidou_enabled, galileo_enabled
    );

    if !gps_enabled && !glonass_enabled && !beidou_enabled && !galileo_enabled {
        println!("[WARN] No GNSS systems enabled - skipping RINEX parsing");
        return;
    }

    // Временные рамки для фильтрации (±2 часа от целевого времени)
    // Широкое окно для первичной фильтрации эфемерид GNSS
    let time_window_hours = 2.0;

    // Счетчики для статистики фильтрации
    let mut gps_parsed = 0;
    let mut gps_accepted = 0;
    let mut beidou_parsed = 0;
    let mut beidou_accepted = 0;
    let mut galileo_parsed = 0;
    let mut galileo_accepted = 0;
    let mut _other_skipped = 0;
    let mut total_lines_processed = 0;

    // Кэш эфемерид по эпохам для GPS и BeiDou
    let mut gps_ephemeris_by_epoch: std::collections::HashMap<
        i32,
        std::collections::HashMap<u8, GpsEphemeris>,
    > = std::collections::HashMap::new();
    let mut gps_available_epochs: std::collections::HashSet<i32> = std::collections::HashSet::new();
    let mut beidou_ephemeris_by_epoch: std::collections::HashMap<
        i32,
        std::collections::HashMap<u8, BeiDouEphemeris>,
    > = std::collections::HashMap::new();
    let mut beidou_available_epochs: std::collections::HashSet<i32> =
        std::collections::HashSet::new();
    let mut galileo_ephemeris_by_epoch: std::collections::HashMap<
        i32,
        std::collections::HashMap<u8, GpsEphemeris>,
    > = std::collections::HashMap::new();
    let mut galileo_available_epochs: std::collections::HashSet<i32> =
        std::collections::HashSet::new();

    // Оптимизированный RINEX парсер с фильтрацией
    loop {
        match lines.next() {
            Some(Ok(line)) => {
                total_lines_processed += 1;
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
                                println!(
                                    "[INFO] GPS iono alpha parameters parsed: {:?}",
                                    iono_alpha
                                );
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

                // Обрабатываем только заголовочные строки записей (как в C версии)
                // Строки данных эфемерид начинаются с пробелов и читаются внутри parse_gps_ephemeris
                match system_char {
                    'G' if gps_enabled => {
                        // *** КРИТИЧЕСКИЙ БЛОК: GPS ЭФЕМЕРИДЫ ПАРСИНГ ***
                        // GPS эфемериды (только если GPS включена в enabled_systems)
                        // Формат строки: "G## YYYY MM DD HH MM SS ..." где ## - SVID спутника
                        if let Some(eph) = parse_gps_ephemeris(&line, &mut lines) {
                            gps_parsed += 1;
                            // Parsed GPS ephemeris

                            // *** ВАЖНО: GPS логика группировки эфемерид по эпохам времени (toe) ***
                            // Проблема может быть здесь! Нужна временная фильтрация для отбора правильных эфемерид
                            if let Some(target) = target_time {
                                // ПРОВЕРКА: Проверяем, попадает ли эфемерида в временное окно вокруг target_time
                                if is_ephemeris_within_time_window(&eph, &target, time_window_hours)
                                {
                                    // DEBUG: GPS фильтр отключен для уменьшения вывода
                                    // println!("[GPS-FILTER] GPS{:02} passed time filter: toe={}", eph.svid, eph.toe);
                                    // Добавляем эпоху (toe) в список доступных времен
                                    gps_available_epochs.insert(eph.toe);

                                    // *** КРИТИЧНО: Группируем эфемериды по эпоху времени (toe) ***
                                    // Каждая эпоха содержит HashMap<svid, ephemeris> для всех спутников этого времени
                                    let epoch_map =
                                        gps_ephemeris_by_epoch.entry(eph.toe).or_default();
                                    epoch_map.insert(eph.svid, eph);

                                    // Accepted GPS ephemeris
                                } else {
                                    // Rejected GPS ephemeris (outside time window)
                                }
                            } else {
                                // Без временной фильтрации - просто берем первую эпоху для каждого SVID
                                gps_available_epochs.insert(eph.toe);
                                let epoch_map = gps_ephemeris_by_epoch.entry(eph.toe).or_default();
                                epoch_map.entry(eph.svid).or_insert(eph);
                                // Accepted GPS ephemeris (no time filtering)
                            }

                            // УДАЛЕНО: раннее завершение парсера мешает мультисистемной генерации
                            // BeiDou и Galileo данные находятся в конце RINEX файла после ~93000 строк
                            // Парсинг должен продолжаться до конца файла для всех систем
                        }
                    }
                    'R' if glonass_enabled => {
                        // ГЛОНАСС эфемериды (только если GLONASS включена)
                        if let Some(eph) = parse_glonass_ephemeris_correct(&line, &mut lines) {
                            nav_data.add_glonass_ephemeris(eph);
                        }
                    }
                    'C' if beidou_enabled => {
                        // *** КРИТИЧЕСКИЙ БЛОК: BEIDOU ЭФЕМЕРИДЫ ПАРСИНГ ***
                        // BeiDou эфемериды (только если BeiDou включена в enabled_systems)
                        // Формат строки: "C## YYYY MM DD HH MM SS ..." где ## - SVID спутника (01-63)
                        // ВАЖНО: BeiDou данные в RINEX файле начинаются поздно (~строка 93528)
                        // Found BeiDou ephemeris line

                        if let Some(eph) = parse_beidou_ephemeris(&line, &mut lines) {
                            beidou_parsed += 1;

                            // *** ВАЖНО: BeiDou логика группировки эфемерид по эпохам времени (toe) ***
                            // Аналогично GPS, но BeiDou использует свое время системы (BDT)
                            if let Some(target) = target_time {
                                // ПРОВЕРКА: Проверяем, попадает ли BeiDou эфемерида в временное окно вокруг target_time
                                if is_beidou_ephemeris_within_time_window(
                                    &eph,
                                    &target,
                                    time_window_hours,
                                ) {
                                    // DEBUG: BDS фильтр отключен для уменьшения вывода
                                    // println!("[BDS-FILTER] BDS{:02} passed time filter: toe={}", eph.svid, eph.toe);
                                    // Добавляем эпоху (toe) в список доступных времен BeiDou
                                    beidou_available_epochs.insert(eph.toe);

                                    // *** КРИТИЧНО: Группируем BeiDou эфемериды по эпохе времени (toe) ***
                                    // Каждая эпоха содержит HashMap<svid, ephemeris> для всех BeiDou спутников этого времени
                                    let epoch_map =
                                        beidou_ephemeris_by_epoch.entry(eph.toe).or_default();
                                    epoch_map.insert(eph.svid, eph);

                                    beidou_accepted += 1;
                                    // Accepted BeiDou ephemeris
                                }
                            } else {
                                // Без временной фильтрации - берем все доступные эпохи BeiDou
                                beidou_available_epochs.insert(eph.toe);
                                let epoch_map =
                                    beidou_ephemeris_by_epoch.entry(eph.toe).or_default();
                                epoch_map.entry(eph.svid).or_insert(eph);
                                // Accepted BeiDou ephemeris (no time filtering)
                            }
                        }
                    }
                    'E' if galileo_enabled => {
                        // *** КРИТИЧЕСКИЙ БЛОК: GALILEO ЭФЕМЕРИДЫ ПАРСИНГ ***
                        // Используем специализированный парсер Galileo (6 строк данных)
                        if let Some(eph) = parse_galileo_ephemeris(&line, &mut lines) {
                            // Структура данных совместима (используется GpsEphemeris как контейнер)

                            galileo_parsed += 1;
                            // Parsed Galileo ephemeris

                            // *** ВАЖНО: Galileo логика группировки эфемерид по эпохам времени (toe) ***
                            // Теперь используется та же временная фильтрация как для GPS и BeiDou
                            if let Some(target) = target_time {
                                // ПРОВЕРКА: Проверяем, попадает ли эфемерида в временное окно вокруг target_time
                                if is_ephemeris_within_time_window(&eph, &target, time_window_hours)
                                {
                                    // Отладка специальных SVID отключена для уменьшения вывода
                                    // if [14, 15, 29, 30, 34].contains(&eph.svid) {
                                    //     println!("[GAL-SPECIAL] Found target SVID {} with toe={}", eph.svid, eph.toe);
                                    // }
                                    // println!("[GAL-FILTER] GAL{:02} passed time filter: toe={}", eph.svid, eph.toe);
                                    // Добавляем эпоху (toe) в список доступных времен
                                    galileo_available_epochs.insert(eph.toe);

                                    // *** КРИТИЧНО: Группируем эфемериды по эпоху времени (toe) ***
                                    galileo_ephemeris_by_epoch
                                        .entry(eph.toe)
                                        .or_default()
                                        .insert(eph.svid, eph);
                                    galileo_accepted += 1;
                                } else {
                                    // if [14, 15, 29, 30, 34].contains(&eph.svid) {
                                    //     println!("[GAL-SPECIAL] Target SVID {} REJECTED by time filter: toe={}", eph.svid, eph.toe);
                                    // }
                                }
                            } else {
                                // Без временной фильтрации - принимаем все эфемериды Galileo
                                nav_data.add_galileo_ephemeris(eph);
                                galileo_accepted += 1;
                            }

                            // ВАЖНОЕ ЗАМЕЧАНИЕ: Galileo эфемериды добавляются напрямую в nav_data
                            // в отличие от GPS и BeiDou которые группируются по эпохам (toe)
                            // Это разное поведение может вызывать проблемы с выбором времени эфемерид
                        }
                    }
                    'G' | ' ' if !gps_enabled => {
                        // GPS отключена в конфиге - пропускаем эфемериду
                        _other_skipped += 1;
                        // ВРЕМЕННО: не вызываем skip_ephemeris_lines для диагностики
                        continue;
                    }
                    'R' if !glonass_enabled => {
                        // GLONASS отключена в конфиге - пропускаем эфемериду
                        _other_skipped += 1;
                        continue;
                    }
                    'C' if !beidou_enabled => {
                        // BeiDou отключена в конфиге - пропускаем эфемериду
                        _other_skipped += 1;
                        continue;
                    }
                    'E' if !galileo_enabled => {
                        // Galileo отключена в конфиге - пропускаем эфемериду
                        _other_skipped += 1;
                        continue;
                    }
                    'S' => {
                        // SBAS спутники - пропускаем запись целиком (7 строк данных)
                        // DEBUG: SBAS пропуск отключен для уменьшения вывода
                        // println!("[DEBUG] Skipping SBAS satellite at line {}", total_lines_processed);
                        skip_ephemeris_lines(&mut lines);
                        _other_skipped += 1;
                        continue;
                    }
                    ' ' => {
                        // Строки данных эфемерид начинаются с пробела - должны читаться внутри parse_*_ephemeris
                        // В основном цикле их нужно игнорировать (как в C версии)
                        continue;
                    }
                    _ => {
                        // Неизвестный тип, пропускаем
                        continue;
                    }
                }
            }
            Some(Err(e)) => {
                dprintln!(
                    "[WARN] Error reading line {}: {}",
                    total_lines_processed + 1,
                    e
                );
                continue; // Продолжаем несмотря на ошибку чтения одной строки
            }
            None => {
                dprintln!(
                    "[DEBUG] End of file reached at line {}",
                    total_lines_processed
                );
                break; // Конец файла
            }
        }
    }

    // *** КРИТИЧЕСКАЯ СЕКЦИЯ: ВЫБОР ЕДИНОЙ ЭПОХИ ВРЕМЕНИ ДЛЯ ВСЕХ СПУТНИКОВ ***
    // Эта логика была добавлена для исправления проблемы разных эпох эфемерид
    if let Some(target) = target_time {
        // ШАГ 1: Объединяем все доступные эпохи GPS и BeiDou
        // ВАЖНО: Galileo НЕ участвует в выборе единой эпохи (потенциальная проблема!)
        let mut all_available_epochs: std::collections::HashSet<i32> =
            std::collections::HashSet::new();
        all_available_epochs.extend(&gps_available_epochs);
        all_available_epochs.extend(&beidou_available_epochs);
        all_available_epochs.extend(&galileo_available_epochs);

        dprintln!(
            "[EPOCH-SELECT] Available GPS epochs: {:?}",
            gps_available_epochs
        );
        dprintln!(
            "[EPOCH-SELECT] Available BeiDou epochs: {:?}",
            beidou_available_epochs
        );
        dprintln!(
            "[EPOCH-SELECT] Available Galileo epochs: {:?}",
            galileo_available_epochs
        );
        dprintln!(
            "[EPOCH-SELECT] Combined epochs for selection: {:?}",
            all_available_epochs
        );

        if !all_available_epochs.is_empty() {
            // ШАГ 2: Находим эпоху ближайшую к target_time из пресета
            // Преобразуем UTC время пресета в GPS время для сравнения с toe
            let target_gps = utc_to_gps_time(target, false);
            let target_seconds =
                (target_gps.Week as f64) * 604800.0 + (target_gps.MilliSeconds as f64) / 1000.0;

            let mut _best_epoch = 0i32;
            let mut best_epoch_diff = f64::INFINITY;

            // ШАГ 3: Ищем эпоху с минимальной разницей по времени (ближайшую)
            for &epoch in &all_available_epochs {
                // ВАЖНО: toe уже в секундах GPS недели, добавляем текущую неделю
                let epoch_seconds = (target_gps.Week as f64) * 604800.0 + (epoch as f64);
                let time_diff = (epoch_seconds - target_seconds).abs();

                dprintln!(
                    "[EPOCH-SELECT] Checking epoch {} ({}s): target={}s, diff={:.1}h",
                    epoch,
                    epoch_seconds,
                    target_seconds,
                    time_diff / 3600.0
                );

                if time_diff < best_epoch_diff {
                    best_epoch_diff = time_diff;
                    _best_epoch = epoch;
                    dprintln!(
                        "[EPOCH-SELECT] → New closest epoch {} (diff={:.1}h)",
                        epoch,
                        time_diff / 3600.0
                    );
                }
            }

            // Selected unified epoch for both systems

            // ШАГ 4: Находим лучшую GPS эпоху отдельно от общей
            let mut best_gps_epoch = 0i32;
            let mut best_gps_diff = f64::INFINITY;

            for &epoch in &gps_available_epochs {
                let epoch_seconds = (target_gps.Week as f64) * 604800.0 + (epoch as f64);
                let time_diff = (epoch_seconds - target_seconds).abs();

                if time_diff < best_gps_diff {
                    best_gps_diff = time_diff;
                    best_gps_epoch = epoch;
                }
            }

            if !gps_available_epochs.is_empty() {
                if let Some(epoch_ephemeris) = gps_ephemeris_by_epoch.get(&best_gps_epoch) {
                    // GPS-specific epoch selected
                    for eph in epoch_ephemeris.values() {
                        nav_data.add_gps_ephemeris(*eph);
                        gps_accepted += 1;
                    }
                }
            }

            // ШАГ 5: Находим лучшую BeiDou эпоху отдельно от общей
            let mut best_beidou_epoch = 0i32;
            let mut best_beidou_diff = f64::INFINITY;

            dprintln!(
                "[BEIDOU-DEBUG] Available BeiDou epochs: {:?}",
                beidou_available_epochs
            );
            for &epoch in &beidou_available_epochs {
                let epoch_seconds = (target_gps.Week as f64) * 604800.0 + (epoch as f64);
                let time_diff = (epoch_seconds - target_seconds).abs();
                let epoch_time_of_day = epoch % 86400;
                let hours = epoch_time_of_day / 3600;
                let minutes = (epoch_time_of_day % 3600) / 60;
                dprintln!(
                    "[BEIDOU-DEBUG] Epoch toe={} ({:02}:{:02}:00) diff={:.1}min",
                    epoch,
                    hours,
                    minutes,
                    time_diff / 60.0
                );

                if time_diff < best_beidou_diff {
                    best_beidou_diff = time_diff;
                    best_beidou_epoch = epoch;
                }
            }

            // Добавляем все BeiDou эфемериды с выбранной эпохи в nav_data
            if !beidou_available_epochs.is_empty() {
                if let Some(epoch_ephemeris) = beidou_ephemeris_by_epoch.get(&best_beidou_epoch) {
                    dprintln!(
                        "[EPOCH-SELECT] Selected BeiDou epoch: toe={} (diff={:.1}h)",
                        best_beidou_epoch,
                        best_beidou_diff / 3600.0
                    );
                    dprintln!(
                        "[BEIDOU-DEBUG] BeiDou satellites in selected epoch: {} satellites",
                        epoch_ephemeris.len()
                    );
                    dprintln!(
                        "[BEIDOU-DEBUG] SVIDs: {:?}",
                        epoch_ephemeris.keys().collect::<Vec<_>>()
                    );
                    // Check toe values in this epoch
                    let mut toe_values = std::collections::HashMap::new();
                    for eph in epoch_ephemeris.values() {
                        *toe_values.entry(eph.toe).or_insert(0) += 1;
                    }
                    dprintln!(
                        "[BEIDOU-DEBUG] TOE values in this 'epoch': {:?}",
                        toe_values
                    );

                    // Adding BeiDou satellites from selected epoch
                    for eph in epoch_ephemeris.values() {
                        nav_data.add_beidou_ephemeris(*eph);
                        beidou_accepted += 1;
                        // Added BeiDou ephemeris
                    }
                }
            }

            // ШАГ 6: Находим лучшую Galileo эпоху отдельно от общей
            let mut best_galileo_epoch = 0i32;
            let mut best_galileo_diff = f64::INFINITY;

            dprintln!(
                "[GALILEO-DEBUG] Available Galileo epochs: {:?}",
                galileo_available_epochs
            );
            for &epoch in &galileo_available_epochs {
                let epoch_seconds = (target_gps.Week as f64) * 604800.0 + (epoch as f64);
                let time_diff = (epoch_seconds - target_seconds).abs();
                let epoch_time_of_day = epoch % 86400;
                let hours = epoch_time_of_day / 3600;
                let minutes = (epoch_time_of_day % 3600) / 60;
                dprintln!(
                    "[GALILEO-DEBUG] Epoch toe={} ({:02}:{:02}:00) diff={:.1}min",
                    epoch,
                    hours,
                    minutes,
                    time_diff / 60.0
                );

                if time_diff < best_galileo_diff {
                    best_galileo_diff = time_diff;
                    best_galileo_epoch = epoch;
                }
            }

            // Добавляем все Galileo эфемериды с выбранной эпохи в nav_data
            if !galileo_available_epochs.is_empty() {
                if let Some(epoch_ephemeris) = galileo_ephemeris_by_epoch.get(&best_galileo_epoch) {
                    println!(
                        "[EPOCH-SELECT] Selected Galileo epoch: toe={} (diff={:.1}h)",
                        best_galileo_epoch,
                        best_galileo_diff / 3600.0
                    );
                    // Adding Galileo satellites from selected epoch
                    for (svid, eph) in epoch_ephemeris {
                        nav_data.add_galileo_ephemeris(*eph);
                        galileo_accepted += 1;
                        if [14, 15, 29, 30, 34].contains(svid) {
                            println!(
                                "[EPOCH-DEBUG] Added target SVID {} from toe={}",
                                svid, eph.toe
                            );
                        }
                        // Added Galileo ephemeris
                    }
                } else {
                    // No Galileo ephemerides found for selected epoch
                }
            }
        } else {
            println!(
                "[ERROR] ❌ No epochs available for selection - GPS or BeiDou parsing failed!"
            );
        }
    } else {
        // Без временной фильтрации - берем первую доступную эпоху
        let mut selected_epoch: Option<i32> = None;

        // Приоритет GPS если доступна
        if let Some((&first_epoch, first_epoch_ephemeris)) = gps_ephemeris_by_epoch.iter().next() {
            selected_epoch = Some(first_epoch);
            println!(
                "[INFO] No time filtering - using first GPS epoch: toe={}",
                first_epoch
            );
            for eph in first_epoch_ephemeris.values() {
                nav_data.add_gps_ephemeris(*eph);
                gps_accepted += 1;
            }
        }

        // Добавляем BeiDou с той же эпохи если есть, иначе с первой доступной
        if let Some(epoch) = selected_epoch {
            if let Some(epoch_ephemeris) = beidou_ephemeris_by_epoch.get(&epoch) {
                for eph in epoch_ephemeris.values() {
                    nav_data.add_beidou_ephemeris(*eph);
                    beidou_accepted += 1;
                }
            }
        } else if let Some((&first_epoch, first_epoch_ephemeris)) =
            beidou_ephemeris_by_epoch.iter().next()
        {
            selected_epoch = Some(first_epoch);
            println!(
                "[INFO] No time filtering - using first BeiDou epoch: toe={}",
                first_epoch
            );
            for eph in first_epoch_ephemeris.values() {
                nav_data.add_beidou_ephemeris(*eph);
                beidou_accepted += 1;
            }
        }

        // Добавляем Galileo с той же эпохи если есть, иначе с первой доступной
        if let Some(epoch) = selected_epoch {
            if let Some(epoch_ephemeris) = galileo_ephemeris_by_epoch.get(&epoch) {
                for eph in epoch_ephemeris.values() {
                    nav_data.add_galileo_ephemeris(*eph);
                    galileo_accepted += 1;
                }
            }
        } else if let Some((&first_epoch, first_epoch_ephemeris)) =
            galileo_ephemeris_by_epoch.iter().next()
        {
            println!(
                "[INFO] No time filtering - using first Galileo epoch: toe={}",
                first_epoch
            );
            for eph in first_epoch_ephemeris.values() {
                nav_data.add_galileo_ephemeris(*eph);
                galileo_accepted += 1;
            }
        }
    }

    // ========== ИТОГОВЫЙ ОТЧЁТ О ОТОБРАННЫХ СПУТНИКАХ ==========
    if crate::logutil::is_verbose() {
        println!("\n📊 ИТОГОВЫЙ ОТЧЁТ ПО ЭФЕМЕРИДАМ СПУТНИКОВ");
        println!("═══════════════════════════════════════════════════════");
    }

    if gps_enabled && gps_accepted > 0 && crate::logutil::is_verbose() {
        println!("📡 GPS система: {} спутников отобрано", gps_accepted);
        for eph in &nav_data.gps_ephemeris {
            println!(
                "   └─ GPS{:02}: toe={:>6} сек, valid={}, health={}",
                eph.svid, eph.toe, eph.valid, eph.health
            );
        }
    } else if gps_enabled && crate::logutil::is_verbose() {
        println!("⚠️  GPS система: НЕТ ОТОБРАННЫХ СПУТНИКОВ");
    }

    if beidou_enabled && beidou_accepted > 0 && crate::logutil::is_verbose() {
        println!("📡 BeiDou система: {} спутников отобрано", beidou_accepted);
        for eph in &nav_data.beidou_ephemeris {
            println!(
                "   └─ BDS{:02}: toe={:>6} сек, valid={}, health={}",
                eph.svid, eph.toe, eph.valid, eph.health
            );
        }
    } else if beidou_enabled && crate::logutil::is_verbose() {
        println!("⚠️  BeiDou система: НЕТ ОТОБРАННЫХ СПУТНИКОВ");
    }

    if galileo_enabled && galileo_accepted > 0 && crate::logutil::is_verbose() {
        println!(
            "📡 Galileo система: {} спутников отобрано",
            galileo_accepted
        );
        for eph in &nav_data.galileo_ephemeris {
            println!(
                "   └─ GAL{:02}: toe={:>6} сек, valid={}, health={}",
                eph.svid, eph.toe, eph.valid, eph.health
            );
        }
    } else if galileo_enabled && crate::logutil::is_verbose() {
        println!("⚠️  Galileo система: НЕТ ОТОБРАННЫХ СПУТНИКОВ");
    }

    if crate::logutil::is_verbose() {
        println!("═══════════════════════════════════════════════════════");
        println!(
            "📈 Статистика парсинга: GPS {}/{}, BeiDou {}/{}, Galileo {}/{}",
            gps_accepted,
            gps_parsed,
            beidou_accepted,
            beidou_parsed,
            galileo_accepted,
            galileo_parsed
        );
        println!("🗂️  Источник: {}", filename);
        println!("═══════════════════════════════════════════════════════\n");
    }
}

/// Оригинальная функция для обратной совместимости
pub fn read_nav_file(nav_data: &mut CNavData, filename: &str) {
    // Загружаем все системы без фильтрации (старое поведение)
    read_nav_file_filtered(
        nav_data,
        filename,
        None,
        &["GPS", "GLONASS", "BeiDou", "Galileo"],
    );
}

/// *** КРИТИЧЕСКАЯ ФУНКЦИЯ ВРЕМЕННОЙ ФИЛЬТРАЦИИ ЭФЕМЕРИД ***
///
/// Проверяет попадает ли эфемерида в временное окно вокруг целевого времени из пресета.
/// ЭТА ФУНКЦИЯ ОПРЕДЕЛЯЕТ, КАКИЕ ЭФЕМЕРИДЫ БУДУТ ИСПОЛЬЗОВАНЫ ДЛЯ РАСЧЕТА ПОЗИЦИЙ СПУТНИКОВ!
///
/// # Параметры
/// * `eph` - GPS эфемерида (используется также для Galileo и BeiDou через структурную совместимость)
/// * `target_time` - UTC время из JSON пресета
/// * `window_hours` - Размер временного окна в часах (по умолчанию 3.0h)
///
/// # Критическая логика:
/// 1. Конвертирует UTC время пресета в GPS время
/// 2. Сравнивает с toe (time of ephemeris) из RINEX файла
/// 3. Возвращает true если разница меньше window_hours
///
/// # Возможные проблемы:
/// - GPS эфемериды обновляются каждые 2 часа, окно 3h может быть мало
/// - BeiDou использует BDT время, может потребоваться коррекция
/// - Galileo использует GST время, синхронизирован с GPS
fn is_beidou_ephemeris_within_time_window(
    eph: &BeiDouEphemeris,
    target_time: &UtcTime,
    window_hours: f64,
) -> bool {
    // BeiDou временная фильтрация с коррекцией BDT → GPS времени
    let target_gps = utc_to_gps_time(*target_time, false);
    let target_seconds =
        (target_gps.Week as f64) * 604800.0 + (target_gps.MilliSeconds as f64) / 1000.0;

    // ВАЖНО: BeiDou время (BDT) началось 1 января 2006, GPS - 6 января 1980
    // Разница: 26 лет ≈ 1356 недель. Добавляем коррекцию к BeiDou неделе.
    const BDT_TO_GPS_WEEK_OFFSET: i32 = 1356;
    let corrected_bds_week = eph.week + BDT_TO_GPS_WEEK_OFFSET;
    let eph_seconds = (corrected_bds_week as f64) * 604800.0 + (eph.toe as f64);

    let time_diff_hours = (eph_seconds - target_seconds).abs() / 3600.0;
    // BeiDou time filtering completed successfully
    time_diff_hours <= window_hours
}

fn is_ephemeris_within_time_window(
    eph: &GpsEphemeris,
    target_time: &UtcTime,
    window_hours: f64,
) -> bool {
    // Skip debug output for cleaner logs

    // ШАГ 1: Конвертируем UTC время из пресета в GPS время для сравнения
    // ВАЖНО: utc_to_gps_time учитывает leap seconds и корректирует время
    let target_gps = utc_to_gps_time(*target_time, false);
    let target_seconds =
        (target_gps.Week as f64) * 604800.0 + (target_gps.MilliSeconds as f64) / 1000.0;

    // ШАГ 2: Вычисляем время эфемериды в абсолютных секундах GPS
    // toe уже в секундах недели, добавляем номер недели
    let eph_seconds = (eph.week as f64) * 604800.0 + (eph.toe as f64);

    // Time conversion completed

    // ШАГ 3: Вычисляем разность времен и проверяем попадание в окно
    let time_diff_hours = (eph_seconds - target_seconds).abs() / 3600.0;
    // Time filtering completed successfully
    // Time window filtering result (no debug output)
    time_diff_hours <= window_hours
}

/// Galileo time-window check: adjusts week number from GST to GPS epoch
#[allow(dead_code)]
fn is_galileo_ephemeris_within_time_window(
    eph: &GpsEphemeris,
    target_time: &UtcTime,
    window_hours: f64,
) -> bool {
    // Target time in GPS seconds
    let target_gps = utc_to_gps_time(*target_time, false);
    let target_seconds =
        (target_gps.Week as f64) * 604800.0 + (target_gps.MilliSeconds as f64) / 1000.0;

    // Galileo week is relative to GST epoch; align to GPS by adding offset
    const GAL_TO_GPS_WEEK_OFFSET: i32 = 1024;
    let corrected_week = eph.week + GAL_TO_GPS_WEEK_OFFSET;
    let eph_seconds = (corrected_week as f64) * 604800.0 + (eph.toe as f64);

    let time_diff_hours = (eph_seconds - target_seconds).abs() / 3600.0;
    time_diff_hours <= window_hours
}

/// Пропускает строки эфемериды (7 строк для GPS/BeiDou/Galileo, 3 для GLONASS)
fn skip_ephemeris_lines<T: Iterator<Item = Result<String, std::io::Error>>>(lines: &mut T) {
    // Для универсальности пропускаем максимально 7 строк
    for _ in 0..7 {
        if lines.next().is_none() {
            break;
        }
    }
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
    // Initialize-on-use inside the velocity_type == 1 branch below
    let mut velocity = KinematicInfo {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
    };

    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_TRAJECTORY) {
                0 => {
                    // "name"
                    set_trajectory_name(trajectory, &get_object_string(current_object));
                }
                1 => {
                    // "initPosition"
                    if assign_start_position(
                        json_stream_get_first_object(current_object),
                        start_pos,
                    ) {
                        content |= 1;
                    }
                }
                2 => {
                    // "initVelocity"
                    velocity_type = assign_start_velocity(
                        json_stream_get_first_object(current_object),
                        start_vel,
                        &mut velocity,
                    );
                    if velocity_type != 0 {
                        content |= 2;
                    }
                }
                3 => {
                    // "trajectoryList"
                    if (content & 3) != 3 {
                        return false;
                    }
                    if velocity_type == 1 {
                        // velocity in ECEF format and stored in velocity
                        let convert_matrix = calc_conv_matrix_lla(start_pos);
                        let mut position = lla_to_ecef(start_pos);
                        position.vx = velocity.vx;
                        position.vy = velocity.vy;
                        position.vz = velocity.vz;
                        speed_ecef_to_local(&convert_matrix, &position, start_vel);
                    }
                    set_init_pos_vel(trajectory, start_pos, start_vel, false);
                    assign_trajectory_list(
                        json_stream_get_first_object(current_object),
                        trajectory,
                    );
                }
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
        if object_type == 1 {
            // ValueTypeObject
            set_ephemeris_file(json_stream_get_first_object(object), nav_data);
        } else if object_type == 2 {
            // ValueTypeArray
            let mut object_array = json_stream_get_first_object(object);
            while !object_array.is_null() {
                if get_object_type(object_array) == 1 {
                    // ValueTypeObject
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
        if object_type == 1 {
            // ValueTypeObject
            set_almanac_file(json_stream_get_first_object(object), nav_data);
        } else if object_type == 2 {
            // ValueTypeArray
            let mut object_array = json_stream_get_first_object(object);
            while !object_array.is_null() {
                if get_object_type(object_array) == 1 {
                    // ValueTypeObject
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
#[allow(improper_ctypes)]
extern "C" {
    fn assign_trajectory_list(object: *mut JsonObject, trajectory: &mut CTrajectory);
}

fn set_output_param(object: *mut JsonObject, output_param: &mut OutputParam) -> bool {
    // Set default values
    output_param.filename[0] = 0;
    output_param.GpsMaskOut = 0;
    output_param.GlonassMaskOut = 0;
    output_param.BdsMaskOut = 0;
    output_param.GalileoMaskOut = 0;
    output_param.ElevationMask = deg2rad(5.0);
    output_param.Interval = 1000;

    // Default output GPS L1 only
    output_param
        .CompactConfig
        .enable_system_parsing(crate::types::PARSE_GPS);
    output_param
        .CompactConfig
        .enable_signal(crate::types::GEN_L1CA);

    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_OUTPUT) {
                0 => {
                    // "type"
                    if is_string_type(current_object) {
                        let output_type_index = search_dictionary(
                            &get_object_string(current_object),
                            DICTIONARY_LIST_OUTPUT_TYPE,
                        );
                        output_param.Type = match output_type_index {
                            0 => OutputType::OutputTypePosition,
                            1 => OutputType::OutputTypeObservation,
                            2 => OutputType::OutputTypeIFdata,
                            3 => OutputType::OutputTypeBaseband,
                            _ => OutputType::default(),
                        };
                    }
                }
                1 => {
                    // "format"
                    if is_string_type(current_object) {
                        let format_index = search_dictionary(
                            &get_object_string(current_object),
                            DICTIONARY_LIST_OUTPUT_FORMAT,
                        );
                        output_param.Format = match format_index {
                            0 => OutputFormat::OutputFormatEcef,
                            1 => OutputFormat::OutputFormatLla,
                            2 => OutputFormat::OutputFormatNmea,
                            3 => OutputFormat::OutputFormatKml,
                            4 => OutputFormat::OutputFormatRinex,
                            5 => OutputFormat::OutputFormatIQ8,
                            6 => OutputFormat::OutputFormatIQ4,
                            _ => OutputFormat::default(),
                        };
                    }
                }
                2 => {
                    // "name"
                    if is_string_type(current_object) {
                        let filename = get_object_string(current_object);
                        let filename_bytes = filename.as_bytes();
                        let copy_len = std::cmp::min(filename_bytes.len(), 255);
                        for i in 0..copy_len {
                            output_param.filename[i] = filename_bytes[i];
                        }
                        output_param.filename[copy_len] = 0;
                    }
                }
                3 => {
                    // "interval"
                    output_param.Interval = (get_double_value(current_object) * 1000.0) as i32;
                }
                4 => {
                    // "config"
                    process_config_param(
                        json_stream_get_first_object(current_object),
                        output_param,
                    );
                }
                5 => {
                    // "systemSelect"
                    if get_object_type(current_object) == 2 {
                        // ValueTypeArray
                        let mut system_select_object = json_stream_get_first_object(current_object);
                        while !system_select_object.is_null() {
                            process_system_select(
                                json_stream_get_first_object(system_select_object),
                                output_param,
                            );
                            system_select_object =
                                json_stream_get_next_object(system_select_object);
                        }
                    }
                }
                12 => {
                    // "sampleFreq" (MHz -> Hz)
                    output_param.SampleFreq =
                        (get_double_value(current_object) * 1_000_000.0).round() as i32;
                }
                13 => {
                    // "centerFreq"
                    output_param.CenterFreq =
                        (get_double_value(current_object) * 1_000_000.0).round() as i32;
                    // МГц -> Гц
                }
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
        x: 0.0,
        y: 0.0,
        z: 0.0,
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
    };
    let mut longitude = 0.0;
    let mut latitude = 0.0;

    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_TRAJECTORY) {
                4 => {
                    // "type"
                    if is_string_type(current_object) {
                        position_type = search_dictionary(
                            &get_object_string(current_object),
                            DICTIONARY_LIST_COORDINATE,
                        );
                    }
                }
                5 => {
                    // "format"
                    if is_string_type(current_object) {
                        format = search_dictionary(
                            &get_object_string(current_object),
                            DICTIONARY_LIST_COORDINATE,
                        );
                    }
                }
                6 => {
                    // "longitude"
                    longitude = get_double_value(current_object);
                }
                7 => {
                    // "latitude"
                    latitude = get_double_value(current_object);
                }
                8 => {
                    // "altitude"
                    start_pos.alt = get_double_value(current_object);
                }
                9 => {
                    // "x"
                    position.x = get_double_value(current_object);
                }
                10 => {
                    // "y"
                    position.y = get_double_value(current_object);
                }
                11 => {
                    // "z"
                    position.z = get_double_value(current_object);
                }
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
                4 => {
                    // "type"
                    if is_string_type(current_object) {
                        velocity_type = search_dictionary(
                            &get_object_string(current_object),
                            DICTIONARY_LIST_COORDINATE,
                        );
                    }
                }
                9 => {
                    // "x"
                    velocity.x = format_speed(get_double_value(current_object), speed_unit);
                }
                10 => {
                    // "y"
                    velocity.y = format_speed(get_double_value(current_object), speed_unit);
                }
                11 => {
                    // "z"
                    velocity.z = format_speed(get_double_value(current_object), speed_unit);
                }
                12 => {
                    // "speedUnit"
                    speed_unit = search_dictionary(
                        &get_object_key(current_object),
                        DICTIONARY_LIST_COORDINATE,
                    );
                }
                13 => {
                    // "angleUnit"
                    angle_unit = search_dictionary(
                        &get_object_key(current_object),
                        DICTIONARY_LIST_COORDINATE,
                    );
                }
                14 => {
                    // "speed"
                    start_vel.speed = format_speed(get_double_value(current_object), speed_unit);
                }
                15 => {
                    // "course"
                    start_vel.course = get_double_value(current_object);
                    if angle_unit == 8 {
                        start_vel.course = deg2rad(start_vel.course);
                    }
                }
                16 => {
                    // "east"
                    start_vel.ve = format_speed(get_double_value(current_object), speed_unit);
                }
                17 => {
                    // "north"
                    start_vel.vn = format_speed(get_double_value(current_object), speed_unit);
                }
                18 => {
                    // "up"
                    start_vel.vu = format_speed(get_double_value(current_object), speed_unit);
                }
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }

    if velocity_type == 2 {
        speed_course_to_enu(start_vel);
    } else if velocity_type == 3 {
        speed_enu_to_course(start_vel);
    }
    velocity_type
}

fn process_config_param(object: *mut JsonObject, output_param: &mut OutputParam) -> bool {
    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_OUTPUT) {
                6 => {
                    // "elevationMask"
                    output_param.ElevationMask = deg2rad(get_double_value(current_object));
                }
                7 => {
                    // "maskOut"
                    if get_object_type(current_object) == 2 {
                        // ValueTypeArray
                        let mut mask_out_array = json_stream_get_first_object(current_object);
                        while !mask_out_array.is_null() {
                            process_mask_out(
                                json_stream_get_first_object(mask_out_array),
                                output_param,
                            );
                            mask_out_array = json_stream_get_next_object(mask_out_array);
                        }
                    }
                }
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
                8 => {
                    // "system"
                    if is_string_type(current_object) {
                        system = search_dictionary(
                            &get_object_string(current_object),
                            DICTIONARY_LIST_SYSTEM,
                        ) - 1;
                    }
                }
                9 => {
                    // "svid"
                    let object_type = get_object_type(current_object);
                    if object_type == 4 {
                        // ValueTypeIntNumber
                        mask_out_satellite(system, get_object_int(current_object), output_param);
                    } else if object_type == 2 {
                        // ValueTypeArray
                        let mut mask_out_sv = json_stream_get_first_object(current_object);
                        while !mask_out_sv.is_null() {
                            if get_object_type(mask_out_sv) == 4 {
                                // ValueTypeIntNumber
                                mask_out_satellite(
                                    system,
                                    get_object_int(mask_out_sv),
                                    output_param,
                                );
                            }
                            mask_out_sv = json_stream_get_next_object(mask_out_sv);
                        }
                    }
                }
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }
    true
}

fn mask_out_satellite(system: i32, svid: i32, output_param: &mut OutputParam) -> bool {
    match system {
        0 => {
            // GpsSystem
            if (1..=32).contains(&svid) {
                output_param.GpsMaskOut |= 1 << (svid - 1);
            }
        }
        1 => {
            // BdsSystem
            if (1..=63).contains(&svid) {
                output_param.BdsMaskOut |= 1u64 << (svid - 1);
            }
        }
        2 => {
            // GalileoSystem
            if (1..=50).contains(&svid) {
                output_param.GalileoMaskOut |= 1u64 << (svid - 1);
            }
        }
        3 => {
            // GlonassSystem
            if (1..=24).contains(&svid) {
                output_param.GlonassMaskOut |= 1 << (svid - 1);
            }
        }
        _ => return false,
    }
    true
}

fn process_system_select(object: *mut JsonObject, _output_param: &mut OutputParam) -> bool {
    let mut system = 0; // GpsSystem
    let mut signal = -1;

    unsafe {
        let mut current_object = object;
        while !current_object.is_null() {
            let key = get_object_key(current_object);
            match search_dictionary(&key, KEY_DICTIONARY_LIST_OUTPUT) {
                8 => {
                    // "system"
                    if is_string_type(current_object) {
                        system = search_dictionary(
                            &get_object_string(current_object),
                            DICTIONARY_LIST_SYSTEM,
                        ) - 1;
                    }
                }
                10 => {
                    // "signal"
                    if is_string_type(current_object) {
                        signal = search_dictionary(
                            &get_object_string(current_object),
                            DICTIONARY_LIST_SIGNAL,
                        );
                    }
                }
                11 => {
                    // "enable"
                    if signal >= 0 && (signal / 8) != system {
                        // freq and system do not match
                        system = -1;
                    }
                    if system >= 0 {
                        let mut _signal_index = signal;
                        if signal < 0 {
                            // frequency not set, set as primary signal
                            _signal_index = 0;
                        } else {
                            _signal_index %= 8;
                        }
                        let object_type = get_object_type(current_object);
                        if object_type == 6 { // ValueTypeTrue
                             // Сигнал включен - обрабатывается новой системой CompactConfig
                        } else if object_type == 7 { // ValueTypeFalse
                             // Сигнал выключен - обрабатывается новой системой CompactConfig
                        }
                    }
                }
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
                0 => {
                    // "noiseFloor"
                    set_noise_floor(power_control, get_double_value(current_object));
                }
                1 => {
                    // "initPower"
                    let mut power_object = json_stream_get_first_object(current_object);
                    while !power_object.is_null() {
                        let power_key = get_object_key(power_object);
                        match search_dictionary(&power_key, KEY_DICTIONARY_LIST_POWER) {
                            4 => {
                                // "unit"
                                unit = search_dictionary(
                                    &get_object_string(power_object),
                                    DICTIONARY_LIST_POWER_UNIT,
                                );
                            }
                            5 => {
                                // "value"
                                let value = get_double_value(power_object);
                                match unit {
                                    0 => set_init_cn0(power_control, value),
                                    1 => set_init_cn0(
                                        power_control,
                                        value - get_noise_floor(power_control),
                                    ),
                                    2 => set_init_cn0(
                                        power_control,
                                        value - get_noise_floor(power_control) + 30.0,
                                    ),
                                    _ => {}
                                }
                            }
                            _ => {}
                        }
                        power_object = json_stream_get_next_object(power_object);
                    }
                }
                2 => {
                    // "elevationAdjust"
                    let object_type = get_object_type(current_object);
                    if object_type == 6 {
                        // ValueTypeTrue
                        set_elevation_adjust(power_control, 1); // ElevationAdjustSinSqrtFade
                    } else if object_type == 7 {
                        // ValueTypeFalse
                        set_elevation_adjust(power_control, 0); // ElevationAdjustNone
                    }
                }
                3 => {
                    // "signalPower"
                    let object_type = get_object_type(current_object);
                    if object_type == 1 {
                        // ValueTypeObject
                        process_signal_power(
                            json_stream_get_first_object(current_object),
                            power_control,
                        );
                    } else if object_type == 2 {
                        // ValueTypeArray
                        let mut power_object = json_stream_get_first_object(current_object);
                        while !power_object.is_null() {
                            if get_object_type(power_object) == 1 {
                                // ValueTypeObject
                                process_signal_power(
                                    json_stream_get_first_object(power_object),
                                    power_control,
                                );
                            }
                            power_object = json_stream_get_next_object(power_object);
                        }
                    }
                }
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
                6 => {
                    // "system"
                    if is_string_type(current_object) {
                        system = search_dictionary(
                            &get_object_string(current_object),
                            DICTIONARY_LIST_SYSTEM,
                        ) - 1;
                    }
                    sv_number = 0;
                }
                7 => {
                    // "svid"
                    let object_type = get_object_type(current_object);
                    if object_type == 4 {
                        // ValueTypeIntNumber
                        svlist[sv_number] = get_object_int(current_object);
                        sv_number += 1;
                    } else if object_type == 2 {
                        // ValueTypeArray
                        let mut object_array = json_stream_get_first_object(current_object);
                        while !object_array.is_null() {
                            if get_object_type(object_array) == 4 {
                                // ValueTypeIntNumber
                                svlist[sv_number] = get_object_int(object_array);
                                sv_number += 1;
                            }
                            object_array = json_stream_get_next_object(object_array);
                        }
                    }
                }
                8 => {
                    // "powerValue"
                    let object_type = get_object_type(current_object);
                    if object_type == 1 {
                        // ValueTypeObject
                        process_power_value(
                            json_stream_get_first_object(current_object),
                            system,
                            &svlist[0..sv_number],
                            sv_number,
                            power_control,
                        );
                    } else if object_type == 2 {
                        // ValueTypeArray
                        let mut object_array = json_stream_get_first_object(current_object);
                        while !object_array.is_null() {
                            if get_object_type(object_array) == 1 {
                                // ValueTypeObject
                                process_power_value(
                                    json_stream_get_first_object(object_array),
                                    system,
                                    &svlist[0..sv_number],
                                    sv_number,
                                    power_control,
                                );
                            }
                            object_array = json_stream_get_next_object(object_array);
                        }
                    }
                }
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
                4 => {
                    // "unit"
                    unit = search_dictionary(
                        &get_object_string(current_object),
                        DICTIONARY_LIST_POWER_UNIT,
                    );
                }
                5 => {
                    // "value"
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
                }
                9 => {
                    // "time"
                    signal_power.time = (get_double_value(current_object) * 1000.0) as i32;
                }
                _ => {}
            }
            current_object = json_stream_get_next_object(current_object);
        }
    }

    if sv_number == 0 {
        // svlist is empty means for all satellites
        signal_power.svid = 0;
        unsafe {
            add_control_element(power_control, &signal_power);
        }
    } else {
        for i in 0..sv_number {
            signal_power.svid = svlist[i];
            unsafe {
                add_control_element(power_control, &signal_power);
            }
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
        5 => {
            // "dm"
            let mut abs_value = abs_value;
            abs_value -= degree as f64;
            let minute = degree % 100;
            let degree = degree / 100;
            abs_value += minute as f64;
            abs_value = degree as f64 + abs_value / 60.0;
            let value = if sign != 0 { -abs_value } else { abs_value };
            deg2rad(value)
        }
        6 => {
            // "dms"
            let mut abs_value = abs_value;
            abs_value -= degree as f64;
            let second = degree % 100;
            let minute = (degree / 100) % 100;
            let degree = degree / 10000;
            abs_value += second as f64;
            abs_value = degree as f64 + minute as f64 / 60.0 + abs_value / 3600.0;
            let value = if sign != 0 { -abs_value } else { abs_value };
            deg2rad(value)
        }
        7 => value,
        _ => deg2rad(value),
    }
}

fn format_speed(value: f64, format: i32) -> f64 {
    match format {
        10 => value / 3.6,               // kilometers per hour
        11 => value * 1852.0 / 3600.0,   // knots
        12 => value * 1609.344 / 3600.0, // miles per hour
        _ => value,
    }
}

// External function declarations for CPowerControl
#[allow(improper_ctypes)]
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

/// Парсит GPS эфемериды из RINEX 3.04 формата
///
/// Парсинг выполняется согласно спецификации RINEX 3.04 для GPS LNAV сообщений.
/// Функция читает 8 строк данных:
/// - Строка 1: SVID, время эпохи (toc), часовые корректировки af0, af1, af2
/// - Строки 2-8: Орбитальные параметры (28 значений)
///
/// # Параметры
/// - `line`: Первая строка с заголовком эфемерид
/// - `lines`: Итератор для чтения последующих строк
///
/// # Возвращает
/// - `Some(GpsEphemeris)` при успешном парсинге
/// - `None` при ошибке или неполных данных
fn parse_gps_ephemeris<I>(line: &str, lines: &mut I) -> Option<GpsEphemeris>
where
    I: Iterator<Item = Result<String, std::io::Error>>,
{
    let mut eph = GpsEphemeris::default();
    let mut data = [0.0f64; 32]; // Массив для 32 параметров RINEX эфемерид

    // Парсим первую строку: SVID + af0/af1/af2 (и извлечём эпоху для toc)
    let svid = read_contents_time(line, &mut data[0..3])?;
    eph.svid = svid;
    eph.af0 = data[0];
    eph.af1 = data[1];
    eph.af2 = data[2];

    // Вычисляем toc из календарной эпохи заголовка
    if let Some(epoch_utc) = parse_epoch_from_header_line(line) {
        // Преобразуем календарное время в GPS-время и берём секунды недели
        let gps_time = utc_to_gps_time(epoch_utc, false);
        eph.toc = (gps_time.MilliSeconds / 1000) as i32;
    }

    // Читаем 7 строк данных (каждая содержит 4 значения)
    for i in 0..7 {
        if let Some(Ok(data_line)) = lines.next() {
            if i * 4 + 3 + 4 <= data.len() {
                read_contents_data(&data_line, &mut data[i * 4 + 3..i * 4 + 7]);
            }
        } else {
            println!("[WARN] Failed to read ephemeris data line {} for SVID {} - incomplete record, skipping", i+1, eph.svid);
            return None; // Все же возвращаем None для неполных записей, но с лучшим логированием
        }
    }

    // Заполняем структуру эфемерид точно как в C++ (NavDataGpsLnav case)
    // toe остаётся как в данных RINEX (секунды недели)
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

    // Специфичные параметры для GPS и Galileo
    // Определяем систему по значениям: у Galileo data[26] обычно близко к 0, у GPS - большие числа
    if data.len() > 26 && data[26] > 10.0 {
        // GPS: используем стандартный IODC из data[26]
        eph.iodc = data[26] as u16;
        // Убираем отладку GPS чтобы не засорять вывод
    } else {
        // GALILEO: используем IODnav (data[3]) или SVID если IODnav равен 0
        eph.iodc = if data[3] == 0.0 {
            eph.svid as u16
        } else {
            data[3] as u16
        };
        // DEBUG: Galileo IODC фикс отключен для уменьшения вывода
        // println!("[GAL-IODC-FIX] SV{:02}: data[3]={:.0} (IODnav), data[26]={:.1} → iodc={}", eph.svid, data[3], if data.len() > 26 { data[26] } else { -1.0 }, eph.iodc);
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

    // КРИТИЧЕСКИЙ ФИКС: Как в C версии - учитываем WGS_OMEGDOTE при инициализации!
    // C версия: Eph->omega_t = Eph->omega0 - WGS_OMEGDOTE * Eph->toe;
    // C версия: Eph->omega_delta = Eph->omega_dot - WGS_OMEGDOTE;
    eph.omega_t = eph.omega0 - WGS_OMEGDOTE * (eph.toe as f64);
    eph.omega_delta = eph.omega_dot - WGS_OMEGDOTE;

    eph.valid = 1;
    eph.flag = 1;

    Some(eph)
}

// GLONASS ephemeris parser
#[allow(dead_code)]
fn parse_glonass_ephemeris<I>(line: &str, lines: &mut I) -> Option<GlonassEphemeris>
where
    I: Iterator<Item = Result<String, std::io::Error>>,
{
    // Parse first line: SVID, UTC time, clock corrections
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() < 8 {
        return None;
    }

    // Extract SVID
    let svid_str = if parts[0].len() >= 2 {
        &parts[0][1..]
    } else {
        return None;
    };
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
        if next_parts.len() < 4 {
            return None;
        }

        for j in 0..4 {
            data.push(next_parts[j].replace("D", "E").parse::<f64>().ok()?);
        }
    }

    // Validate we have all 15 data values
    if data.len() < 15 {
        return None;
    }

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
        P: 0xc0,       // P=11, ln=0, P4=0, P3=0, P2=0, P1=00 (will update P2 after tb)
        M: 1,          // assume GLONASS-M satellite
        Ft: 0,         // no data
        n: svid as u8, // satellite number
        Bn: data[6] as u8,
        En: data[14] as u8,
        tb: 0,  // will be calculated below
        day: 1, // placeholder - would need proper GLONASS time conversion
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
        tc: 0.0,                           // derived variable
        PosVelT: KinematicInfo::default(), // derived variable
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
    I: Iterator<Item = Result<String, std::io::Error>>,
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
    I: Iterator<Item = Result<String, std::io::Error>>,
{
    // Simple format parser - looking for slot number and data
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() < 2 {
        return None;
    }

    let _slot: i16 = parts[0].parse().ok()?;

    // Read multiple lines to get all almanac data
    let mut all_data: Vec<String> = Vec::new();
    for part in &parts[1..] {
        all_data.push(part.to_string());
    }

    // Read additional lines if needed
    for _ in 0..3 {
        if let Some(Ok(data_line)) = lines.next() {
            let line_parts: Vec<String> = data_line
                .split_whitespace()
                .map(|s| s.to_string())
                .collect();
            for part in line_parts {
                all_data.push(part);
            }
        }
    }

    if all_data.len() < 8 {
        return None;
    }

    let mut alm = GlonassAlmanac {
        flag: 1,
        ..Default::default()
    };

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

/// Парсит BeiDou эфемериды из RINEX 3.04 формата
///
/// Специализированный парсер для системы BeiDou (Compass), учитывающий специфику
/// китайской GNSS системы. Использует базовый GPS парсер с последующим преобразованием
/// в BeiDou-специфическую структуру.
///
/// Особенности BeiDou RINEX:
/// - AODE (Age of Data Ephemeris) соответствует IODE
/// - AODC (Age of Data Clock) - младшие 8 бит IODC  
/// - TGD1/TGD2 - групповые задержки для разных частот
/// - Система координат CGCS2000 (аналог WGS84)
///
/// # Параметры
/// - `line`: Первая строка с заголовком BeiDou эфемерид
/// - `lines`: Итератор для чтения данных эфемерид
///
/// # Возвращает
/// - `Some(BeiDouEphemeris)` при успешном парсинге
/// - `None` при ошибке парсинга или неполных данных
fn parse_beidou_ephemeris<I>(line: &str, lines: &mut I) -> Option<BeiDouEphemeris>
where
    I: Iterator<Item = Result<String, std::io::Error>>,
{
    // Парсим базовую структуру GPS для совместимости с RINEX форматом
    let gps_eph = parse_gps_ephemeris(line, lines)?;

    // Преобразуем GpsEphemeris в BeiDouEphemeris с добавлением BeiDou-специфических полей
    let mut bds_eph = BeiDouEphemeris {
        // Основные поля копируем из GPS структуры
        ura: gps_eph.ura,
        iodc: gps_eph.iodc,
        iode: gps_eph.iode,
        svid: gps_eph.svid,
        source: gps_eph.source,
        valid: gps_eph.valid,
        flag: gps_eph.flag,
        health: gps_eph.health,
        aode: gps_eph.iode,                // В RINEX 3.04 AODE = IODE для BeiDou
        aodc: (gps_eph.iodc & 0xFF) as u8, // В RINEX 3.04 AODC = младшие биты IODC

        // Временные параметры
        toe: gps_eph.toe,
        toc: gps_eph.toc,
        top: gps_eph.top,
        week: gps_eph.week,
        weekh: (gps_eph.week >> 8) as i32, // Старшие биты недели

        // Орбитальные параметры Кеплера (идентичны GPS)
        M0: gps_eph.M0,
        delta_n: gps_eph.delta_n,
        delta_n_dot: gps_eph.delta_n_dot,
        ecc: gps_eph.ecc,
        sqrtA: gps_eph.sqrtA,
        axis_dot: gps_eph.axis_dot,
        omega0: gps_eph.omega0,
        i0: gps_eph.i0,
        w: gps_eph.w,
        omega_dot: gps_eph.omega_dot,
        idot: gps_eph.idot,

        // Гармонические коррекции орбиты (идентичны GPS)
        cuc: gps_eph.cuc,
        cus: gps_eph.cus,
        crc: gps_eph.crc,
        crs: gps_eph.crs,
        cic: gps_eph.cic,
        cis: gps_eph.cis,

        // Параметры часов
        af0: gps_eph.af0,
        af1: gps_eph.af1,
        af2: gps_eph.af2,

        // BeiDou имеет два TGD параметра вместо одного GPS
        tgd1: gps_eph.tgd,  // TGD1 для B1I сигнала
        tgd2: gps_eph.tgd2, // TGD2 для B2I сигнала

        // Дополнительные TGD для новых сигналов (из расширенного массива)
        tgd_b1cp: gps_eph.tgd_ext.first().copied().unwrap_or(0.0),
        tgd_b2ap: gps_eph.tgd_ext.get(1).copied().unwrap_or(0.0),
        tgd_b2bp: gps_eph.tgd_ext.get(2).copied().unwrap_or(0.0),

        // BeiDou специфические параметры (определяем из SVID)
        sat_type: determine_beidou_satellite_type(gps_eph.svid),
        urai: (gps_eph.ura.clamp(0, 15)) as u8, // URA Index для BeiDou (0-15)
        integrity_flag: 0, // По умолчанию = 0 (будет определено из навигационного сообщения)

        // Производные переменные копируем из GPS
        axis: gps_eph.axis,
        n: gps_eph.n,
        root_ecc: gps_eph.root_ecc,
        // BeiDou использует CGCS2000_OMEGDOTE вместо WGS_OMEGDOTE
        omega_t: gps_eph.omega0 - CGCS2000_OMEGDOTE * (gps_eph.toe as f64),
        omega_delta: gps_eph.omega_dot - CGCS2000_OMEGDOTE,
        Ek: gps_eph.Ek,
        Ek_dot: gps_eph.Ek_dot,
    };

    // Устанавливаем источник данных для BeiDou (по умолчанию D1/D2)
    bds_eph.source = crate::types::BDS_SOURCE_D1D2;

    // RINEX данные уже приведены к единому времени GPS
    // НЕ нужно делать дополнительные конвертации времени BDT -> GPS
    // bds_eph.toe и bds_eph.week остаются как есть

    Some(bds_eph)
}

/// Определяет тип спутника BeiDou по SVID
fn determine_beidou_satellite_type(svid: u8) -> u8 {
    use crate::types::*;

    match svid {
        1..=5 => BDS_SAT_GEO,   // C01-C05: Geostationary satellites
        6..=17 => BDS_SAT_IGSO, // C06-C17: Inclined Geosynchronous Orbit
        18..=63 => BDS_SAT_MEO, // C18-C63: Medium Earth Orbit satellites
        _ => BDS_SAT_MEO,       // По умолчанию MEO для неизвестных SVID
    }
}

/// Парсит Galileo эфемериды из RINEX 3.04 формата
///
/// Специализированный парсер для европейской системы Galileo.
/// Отличается от GPS парсера количеством строк данных (6 вместо 7)
/// и специфическими параметрами навигационного сообщения I/NAV.
///
/// Особенности Galileo RINEX:
/// - 6 строк данных (24 параметра) вместо 7 строк GPS (28 параметров)
/// - BGD (Background Group Delay) вместо TGD
/// - SISA (Signal-In-Space Accuracy) вместо URA
/// - IODnav (Issue Of Data Navigation) для контроля целостности
/// - Система времени GST (Galileo System Time)
///
/// # Параметры
/// - `line`: Первая строка с заголовком Galileo эфемерид
/// - `lines`: Итератор для чтения данных эфемерид
///
/// # Возвращает
/// - `Some(GalileoEphemeris)` при успешном парсинге
/// - `None` при ошибке парсинга или неполных данных
fn parse_galileo_ephemeris<I>(line: &str, lines: &mut I) -> Option<GalileoEphemeris>
where
    I: Iterator<Item = Result<String, std::io::Error>>,
{
    let mut eph = GpsEphemeris::default();
    let mut data = [0.0f64; 28]; // Массив для Galileo данных (6 строк × 4 параметра + 4 начальных)

    // Парсим первую строку используя точную логику C++
    let svid = read_contents_time(line, &mut data[0..3])?;
    eph.svid = svid;
    eph.af0 = data[0];
    eph.af1 = data[1];
    eph.af2 = data[2];

    // toc: используем календарную эпоху заголовка, конвертируя в GPS-время
    if let Some(epoch_utc) = parse_epoch_from_header_line(line) {
        let gps_time = utc_to_gps_time(epoch_utc, false);
        eph.toc = (gps_time.MilliSeconds / 1000) as i32;
    }

    // Читаем 7 строк данных для Galileo (как и GPS в RINEX 3.x навигации)
    for i in 0..7 {
        if let Some(Ok(data_line)) = lines.next() {
            if i * 4 + 3 + 4 <= data.len() {
                read_contents_data(&data_line, &mut data[i * 4 + 3..i * 4 + 7]);
            }
        } else {
            println!("[WARN] Failed to read Galileo ephemeris data line {} for SVID {} - incomplete record, skipping", i+1, eph.svid);
            return None;
        }
    }

    // Заполняем структуру эфемерид для Galileo (аналогично GPS но с учетом разных индексов)
    eph.sqrtA = data[10]; // √a
    eph.ecc = data[8]; // e
    eph.i0 = data[15]; // i0
    eph.omega0 = data[13]; // Ω0
    eph.w = data[17]; // ω
    eph.M0 = data[6]; // M0
    eph.delta_n = data[5]; // Δn
    eph.omega_dot = data[18]; // ΩDOT
    eph.idot = data[19]; // IDOT
    eph.crc = data[16]; // Crc
    eph.crs = data[4]; // Crs
    eph.cuc = data[7]; // Cuc
    eph.cus = data[9]; // Cus
    eph.cic = data[12]; // Cic
    eph.cis = data[14]; // Cis
    eph.toe = (data[11] + 0.5) as i32; // toe

    // Galileo-специфические параметры
    eph.iode = data[3] as u8; // IODnav
                              // GALILEO ФИКС: Если IODnav равен 0, используем SVID как уникальный идентификатор
    eph.iodc = if data[3] == 0.0 {
        eph.svid as u16
    } else {
        data[3] as u16
    };
    println!(
        "[GAL-PARSE] SV{:02}: data[3]={:.0} (IODnav), iodc={}, toe={}",
        eph.svid, data[3], eph.iodc, eph.toe
    );
    eph.week = data[21] as i32; // Week (GST/GPS-aligned in RINEX 3.x)
                                // Galileo SISA is reported; map to URA index similarly to GPS as an approximation
    eph.ura = get_ura_index(data[23]) as i16; // SISA (data[23]) → URA index
                                              // Health field for Galileo is data[24] in our C reference; use the same index here
    eph.health = data[24] as u16; // Health

    // BGD параметры Galileo (в отличие от TGD в GPS)
    // data[24] = BGD_E5a_E1
    // data[25] = BGD_E5b_E1
    eph.tgd = data[24]; // Используем BGD_E5a_E1 как основной TGD

    // Вычисляем производные значения
    eph.axis = eph.sqrtA * eph.sqrtA;
    eph.n = (EARTH_GM / (eph.axis * eph.axis * eph.axis)).sqrt() + eph.delta_n;
    eph.root_ecc = (1.0 - eph.ecc * eph.ecc).sqrt();

    // КРИТИЧЕСКИЙ ФИКС: Как в C версии - учитываем WGS_OMEGDOTE при инициализации для Galileo!
    eph.omega_t = eph.omega0 - WGS_OMEGDOTE * (eph.toe as f64);
    eph.omega_delta = eph.omega_dot - WGS_OMEGDOTE;

    eph.valid = 1;
    eph.flag = 1;

    // RINEX данные уже приведены к единому времени GPS
    // GST (Galileo System Time) синхронизирована с GPS временем в RINEX
    // НЕ нужно делать дополнительные конвертации времени

    Some(eph)
}

// Парсит BeiDou альманах

/// Парсит первую строку RINEX эфемериды (время + 3 параметра) - ТОЧНО как в C++
/// *** КРИТИЧЕСКАЯ ФУНКЦИЯ ПАРСИНГА ЗАГОЛОВОЧНОЙ СТРОКИ RINEX ЭФЕМЕРИДЫ ***
///
/// Парсит первую строку записи эфемериды содержащую SVID и clock параметры (af0, af1, af2).
/// ВАЖНО: Использует ТОЧНЫЕ позиции символов согласно спецификации RINEX!
///
/// # Формат заголовочной строки RINEX:
/// - Позиции 1-2: SVID спутника (G01, E05, C07 и т.д.)  
/// - Позиции 23-42: af0 (19 символов, включая пробелы) - clock bias
/// - Позиции 43-62: af1 (19 символов) - clock drift  
/// - Позиции 63-80: af2 (19 символов) - clock drift rate
///
/// # Возможные проблемы:
/// - Если длина строки < 80 символов, последние поля могут быть пустыми
/// - 'D' в экспоненциальном формате должно заменяться на 'E' для Rust парсера
fn read_contents_time(line: &str, data: &mut [f64]) -> Option<u8> {
    if line.len() < 23 {
        println!(
            "[RINEX-PARSE] ❌ ERROR: Header line too short: {} chars (need >23)",
            line.len()
        );
        return None;
    }

    // ШАГ 1: Извлекаем SVID спутника из позиций 1-2
    // Формат: G01, E05, C07 - первый символ определяет систему, 01-99 номер спутника
    let svid = if line.len() > 2 && !line.chars().nth(1).unwrap_or(' ').is_whitespace() {
        let svid_str = &line[1..3];
        let parsed = svid_str.trim().parse::<u8>().unwrap_or(0);
        // println!("[RINEX-PARSE] 📡 Parsed SVID: '{}' → {}", svid_str, parsed);
        parsed
    } else {
        println!("[RINEX-PARSE] ⚠️  WARNING: Unable to parse SVID from positions 1-2");
        0
    };

    // ШАГ 2: Конвертируем научную нотацию 'D' → 'E' (RINEX использует FORTRAN формат)
    let line = line.replace("D", "E");

    // ШАГ 3: Парсим clock параметры с правильных позиций (каждое поле 19 символов)
    // af0: позиции 24-42 (19 символов) - clock bias в секундах
    if line.len() > 42 {
        let af0_str = &line[23..42];
        data[0] = af0_str.trim().parse().unwrap_or(0.0);
        // println!("[RINEX-PARSE] 🕐 af0 (24-42): '{}' → {:.6e}", af0_str.trim(), data[0]);
    }

    // af1: позиции 43-61 (19 символов) - clock drift в с/с
    if line.len() > 61 {
        let af1_str = &line[42..61];
        data[1] = af1_str.trim().parse().unwrap_or(0.0);
        // println!("[RINEX-PARSE] 🕐 af1 (43-61): '{}' → {:.6e}", af1_str.trim(), data[1]);
    }

    // af2: позиции 62-80 (19 символов) - clock drift rate в с/с²
    if line.len() > 80 {
        let af2_str = &line[61..80];
        data[2] = af2_str.trim().parse().unwrap_or(0.0);
        // println!("[RINEX-PARSE] 🕐 af2 (62-80): '{}' → {:.6e}", af2_str.trim(), data[2]);
    } else if line.len() > 61 {
        // Последнее поле может быть короче если строка обрезана
        let af2_str = &line[61..];
        data[2] = af2_str.trim().parse().unwrap_or(0.0);
        // println!("[RINEX-PARSE] 🕐 af2 (62-end): '{}' → {:.6e}", af2_str.trim(), data[2]);
    }

    Some(svid)
}

/// Парсит календарную эпоху из заголовочной строки RINEX (YYYY MM DD hh mm ss.s)
/// Возвращает UtcTime (используем его как общий календарный контейнер)
fn parse_epoch_from_header_line(line: &str) -> Option<UtcTime> {
    // Формат: <Sys><SVID> YYYY MM DD hh mm ss.sssss ....
    // Используем разбиение по пробелам для устойчивости к отступам
    let mut parts = line.split_whitespace();
    let _sys_svid = parts.next()?; // G07, E05, C13, ...
    let year = parts.next()?.parse::<i32>().ok()?;
    let month = parts.next()?.parse::<i32>().ok()?;
    let day = parts.next()?.parse::<i32>().ok()?;
    let hour = parts.next()?.parse::<i32>().ok()?;
    let minute = parts.next()?.parse::<i32>().ok()?;
    let second = parts.next()?.replace('D', "E").parse::<f64>().ok()?;
    Some(UtcTime {
        Year: year,
        Month: month,
        Day: day,
        Hour: hour,
        Minute: minute,
        Second: second,
    })
}

/// *** КРИТИЧЕСКАЯ ФУНКЦИЯ ПАРСИНГА СТРОК ДАННЫХ RINEX ЭФЕМЕРИДЫ ***
///
/// Парсит строки данных эфемериды (строки 2-8) содержащие орбитальные параметры.
/// КЛЮЧЕВАЯ ОСОБЕННОСТЬ: Каждое поле имеет строго 19 символов включая пробелы!
///
/// # Формат строки данных RINEX (80 символов):
/// - Позиции 5-23:   Параметр 1 (19 символов, с отступом 4 пробела)
/// - Позиции 24-42:  Параметр 2 (19 символов)  
/// - Позиции 43-61:  Параметр 3 (19 символов)
/// - Позиции 62-80:  Параметр 4 (19 символов)
///
/// # Критические детали:
/// - FORTRAN экспоненциальный формат 'D' должен конвертироваться в 'E'
/// - Каждое поле может содержать пробелы для выравнивания
/// - Строка может быть короче 80 символов (последние поля пустые)
fn read_contents_data(line: &str, data: &mut [f64]) {
    // ШАГ 1: Конвертируем FORTRAN экспоненциальный формат 'D' → 'E' для Rust парсера
    let line = line.replace("D", "E");

    // ШАГ 2: Парсим 4 орбитальных параметра с ПРАВИЛЬНЫХ позиций (каждое поле 19 символов)

    // Параметр 1: позиции 5-23 (19 символов после отступа 4 пробела)
    if line.len() > 23 {
        let param1_str = &line[4..23];
        data[0] = param1_str.trim().parse().unwrap_or(0.0);
        // println!("[RINEX-DATA] 🔢 Param1 (5-23): '{}' → {:.6e}", param1_str.trim(), data[0]);
    } else {
        println!(
            "[RINEX-DATA] ⚠️  Line too short for param1 (need >23 chars, got {})",
            line.len()
        );
    }

    // Параметр 2: позиции 24-42 (19 символов)
    if line.len() > 42 {
        let param2_str = &line[23..42];
        data[1] = param2_str.trim().parse().unwrap_or(0.0);
        // println!("[RINEX-DATA] 🔢 Param2 (24-42): '{}' → {:.6e}", param2_str.trim(), data[1]);
    }

    // Параметр 3: позиции 43-61 (19 символов)
    if line.len() > 61 {
        let param3_str = &line[42..61];
        data[2] = param3_str.trim().parse().unwrap_or(0.0);
        // println!("[RINEX-DATA] 🔢 Param3 (43-61): '{}' → {:.6e}", param3_str.trim(), data[2]);
    }

    // Параметр 4: позиции 62-80 (19 символов) - может быть обрезан в конце строки
    if line.len() > 80 {
        let param4_str = &line[61..80];
        data[3] = param4_str.trim().parse().unwrap_or(0.0);
        // println!("[RINEX-DATA] 🔢 Param4 (62-80): '{}' → {:.6e}", param4_str.trim(), data[3]);
    } else if line.len() > 61 {
        // Обрабатываем укороченную строку (параметр 4 до конца строки)
        let param4_str = &line[61..];
        data[3] = param4_str.trim().parse().unwrap_or(0.0);
        // println!("[RINEX-DATA] 🔢 Param4 (62-end): '{}' → {:.6e}", param4_str.trim(), data[3]);
    }
}

/// Преобразует URA в индекс (из C++ версии)
fn get_ura_index(ura: f64) -> i32 {
    if ura <= 2.4 {
        (ura / 0.3 + 0.5) as i32
    } else if ura <= 6.0 {
        ((ura - 2.4) / 0.6 + 8.5) as i32
    } else if ura <= 12.0 {
        ((ura - 6.0) / 1.2 + 14.5) as i32
    } else if ura <= 24.0 {
        ((ura - 12.0) / 2.4 + 19.5) as i32
    } else if ura <= 48.0 {
        ((ura - 24.0) / 4.8 + 24.5) as i32
    } else if ura <= 96.0 {
        ((ura - 48.0) / 9.6 + 29.5) as i32
    } else if ura <= 192.0 {
        ((ura - 96.0) / 19.2 + 34.5) as i32
    } else {
        15
    }
}

/// Правильный парсер GLONASS эфемерид по образцу C++ DecodeEphOrbit
fn parse_glonass_ephemeris_correct<I>(line: &str, lines: &mut I) -> Option<GlonassEphemeris>
where
    I: Iterator<Item = Result<String, std::io::Error>>,
{
    let mut data = [0.0f64; 19];

    // Парсим первую строку (время + 3 параметра)
    let svid = read_contents_time(line, &mut data[0..3])?;

    // Дополнительно извлекаем календарную эпоху из заголовка
    // и переводим её в GLONASS-время для корректного поля day
    let epoch_utc = parse_epoch_from_header_line(line)?;
    let gtime = crate::gnsstime::utc_to_glonass_time_corrected(epoch_utc);
    let seconds_of_day =
        (epoch_utc.Hour * 3600 + epoch_utc.Minute * 60) as i32 + epoch_utc.Second.floor() as i32;

    // Читаем 3 строки данных (как в C++ версии)
    for i in 0..3 {
        if let Some(Ok(data_line)) = lines.next() {
            read_contents_data(&data_line, &mut data[i * 4 + 3..i * 4 + 7]);
        } else {
            println!("[DEBUG] Failed to read GLONASS data line {}", i + 1);
            return None;
        }
    }

    // Создаем GLONASS эфемериду по образцу C++
    let mut eph = GlonassEphemeris {
        flag: 1,
        valid: 1,
        slot: svid as u8,
        n: svid as u8,
        freq: data[10] as i8, // Частотный номер
        tk: 0,                // Будет вычислен ниже
        P: 0xc0,              // P=11, ln=0, P4=0, P3=0, P2=0, P1=00
        M: 1,                 // GLONASS-M
        Ft: 0,                // Нет данных
        Bn: data[6] as u8,
        En: data[14] as u8,
        tb: 0,                 // Будет вычислен ниже
        day: gtime.Day as u16, // Совместимо с нашим GlonassTime::Day
        gamma: data[1],
        tn: -data[0], // Clock bias (знак меняется)
        dtn: 0.0,     // Нет в RINEX
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

    // Вычисляем tk из заголовка (секунды дня) — надёжнее, чем полагаться на формат tk в некоторых RINEX
    let tk_seconds = seconds_of_day;
    let hours = tk_seconds / 3600;
    let minutes = (tk_seconds % 3600) / 60;
    let half_minutes = (tk_seconds % 60) / 30;
    eph.tk = ((hours << 7) | (minutes << 1) | half_minutes) as u16;

    // Вычисляем tb (опорное время в 15-минутных интервалах) по ближайшему 15-минутному интервалу
    eph.tb = ((tk_seconds + 450) / 900 * 900) as u32;

    // Устанавливаем P2 бит в зависимости от tb
    if (eph.tb / 900) & 1 != 0 {
        eph.P |= 0x04; // Устанавливаем P2 бит
    }

    println!(
        "[DEBUG] Successfully parsed GLONASS ephemeris for SVID {}",
        eph.n
    );
    Some(eph)
}
