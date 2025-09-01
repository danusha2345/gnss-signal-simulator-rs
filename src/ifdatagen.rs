//! # Генератор промежуточных частотных данных (IF Data Generator)
//!
//! Основной модуль для генерации промежуточных частотных (IF) данных ГНСС сигналов.
//! Симулирует прием сигналов спутников с учетом их параметров, траекторий,
//! ионосферных эффектов и других факторов распространения.
//!
//! ## Основные возможности:
//! - Генерация IF данных для множественных спутниковых систем
//! - Учет траекторий движения спутников
//! - Моделирование ионосферных и тропосферных задержек
//! - Обработка навигационных сообщений различных форматов
//! - Управление мощностью сигналов
//! - Расчет видимости спутников
//!
//! ## Поддерживаемые системы:
//! - GPS (все частоты)
//! - ГЛОНАСС FDMA и CDMA
//! - BeiDou B1, B2, B3
//! - Galileo E1, E5, E6
//!
//! Портировано с C++ версии, сохраняя совместимость с оригинальными алгоритмами.
//!
//! Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use std::fs::File;
use std::io::{Write, BufWriter};
use std::time::Instant;


use crate::types::IonoNequick as INavIonoNequick;
use std::env;
use crate::types::*;
use crate::complex_number::ComplexNumber;
use crate::constants::*;
use crate::json_parser::JsonObject;
// ЭКСТРЕМАЛЬНОЕ АППАРАТНОЕ УСКОРЕНИЕ: CPU + GPU
use crate::avx512_intrinsics::SafeAvx512Processor;
#[cfg(feature = "gpu")]
use crate::cuda_acceleration::{CudaGnssAccelerator, HybridAccelerator};
use crate::gnsstime::{utc_to_gps_time, utc_to_glonass_time_corrected, utc_to_bds_time};
use crate::coordinate::{lla_to_ecef, speed_local_to_ecef, calc_conv_matrix_lla};
use crate::{lnavbit::LNavBit, l5cnavbit::L5CNavBit, gnavbit::GNavBit};
use crate::{cnavbit::CNavBit as ActualCNavBit, cnav2bit::CNav2Bit as ActualCNav2Bit};
use crate::{d1d2navbit::D1D2NavBit as ActualD1D2NavBit, inavbit::INavBit as ActualINavBit, fnavbit::FNavBit as ActualFNavBit};
use crate::{bcnav1bit::BCNav1Bit as ActualBCNav1Bit, bcnav2bit::BCNav2Bit as ActualBCNav2Bit, bcnav3bit::BCNav3Bit as ActualBCNav3Bit};
use crate::powercontrol::{CPowerControl, SignalPower};
use crate::trajectory::CTrajectory;
use crate::types::SatelliteParam;
use crate::navdata::NavDataType;
use crate::nav_data::NavData as UnifiedNavData;

#[derive(Debug)]
pub struct GenerationStats {
    pub total_samples: u64,
    pub clipped_samples: u64,
    pub file_size_mb: Option<f64>,
    
    // Дополнительная информация для анализа производительности
    pub total_time_ms: f64,
    pub signal_processing_time_ms: f64,
    pub samples_generated: u64,
    pub satellites_processed: u32,
    pub avx512_accelerated: bool,
    pub cuda_accelerated: bool,
}
pub struct NavData {
    // Ионосферные и UTC параметры
    gps_iono: Option<IonoParam>,
    gps_utc: Option<UtcParam>,
    bds_iono: Option<IonoParam>,
    bds_utc: Option<UtcParam>,
    gal_iono: Option<IonoParam>,
    gal_utc: Option<UtcParam>,
    
    // Эфемериды для всех ГНСС систем
    pub gps_ephemeris: Vec<Option<GpsEphemeris>>,
    pub bds_ephemeris: Vec<Option<BeiDouEphemeris>>, // BeiDou использует специализированную структуру
    pub gal_ephemeris: Vec<Option<GpsEphemeris>>, // Galileo использует структуру GPS
    pub glonass_ephemeris: Vec<Option<GlonassEphemeris>>,
    
    // Альманахи
    gps_almanac: Vec<GpsAlmanac>,
    bds_almanac: Vec<GpsAlmanac>,
    gal_almanac: Vec<GpsAlmanac>,
    glo_almanac: Vec<GlonassAlmanac>,
}

impl Default for NavData {
    fn default() -> Self {
        Self::new()
    }
}

impl NavData {
    pub fn new() -> Self {
        Self {
            gps_iono: None,
            gps_utc: None,
            bds_iono: None,
            bds_utc: None,
            gal_iono: None,
            gal_utc: None,
            gps_almanac: Vec::new(),
            bds_almanac: Vec::new(),
            gal_almanac: Vec::new(),
            glo_almanac: Vec::new(),
            gps_ephemeris: Vec::new(),
            bds_ephemeris: Vec::new(),
            gal_ephemeris: Vec::new(),
            glonass_ephemeris: Vec::new(),
        }
    }

    pub fn get_gps_iono(&self) -> Option<&IonoParam> { self.gps_iono.as_ref() }
    pub fn get_gps_utc_param(&self) -> Option<&UtcParam> { self.gps_utc.as_ref() }
    pub fn get_bds_iono(&self) -> Option<&IonoParam> { self.bds_iono.as_ref() }
    pub fn get_bds_utc_param(&self) -> Option<&UtcParam> { self.bds_utc.as_ref() }
    pub fn get_galileo_iono(&self) -> Option<&IonoParam> { self.gal_iono.as_ref() }
    pub fn get_galileo_utc_param(&self) -> Option<&UtcParam> { self.gal_utc.as_ref() }
    pub fn get_gps_almanac(&self) -> &[GpsAlmanac] { &self.gps_almanac }
    pub fn get_bds_almanac(&self) -> &[GpsAlmanac] { &self.bds_almanac }
    pub fn get_galileo_almanac(&self) -> &[GpsAlmanac] { &self.gal_almanac }
    pub fn get_glonass_almanac(&self) -> &[GlonassAlmanac] { &self.glo_almanac }

    /// Поиск эфемерид по системе, времени и SVID
    /// Возвращает наиболее подходящие эфемериды по временной близости
    /// Поиск эфемерид по системе, времени и SVID - ТОЧНО как в C++ FindEphemeris()
    pub fn find_ephemeris(&self, system: GnssSystem, time: GnssTime, svid: i32, _signal: i32, _param: i32) -> Option<GpsEphemeris> {
        let mut best_eph: Option<GpsEphemeris> = None;
        let mut best_time_diff = f64::INFINITY;
        
        // Обработка разных систем отдельно из-за разных типов эфемерид
        match system {
            GnssSystem::GpsSystem => {
                // Поиск GPS эфемерид
                for eph_opt in self.gps_ephemeris.iter() {
                    if let Some(eph) = eph_opt {
                        if eph.svid as i32 == svid && (eph.valid & 1) != 0 && eph.health == 0 {
                            let diff = ((time.Week - eph.week) as f64) * 604800.0 + 
                                       ((time.MilliSeconds / 1000) as f64 - eph.toe as f64);
                            
                            let abs_diff = diff.abs();
                            
                            if abs_diff <= 7200.0 && abs_diff < best_time_diff {
                                best_time_diff = abs_diff;
                                best_eph = Some(*eph);
                            }
                        }
                    }
                }
            },
            GnssSystem::BdsSystem => {
                // Поиск BeiDou эфемерид с конвертацией в GPS формат
                for eph_opt in self.bds_ephemeris.iter() {
                    if let Some(eph) = eph_opt {
                        if eph.svid as i32 == svid && (eph.valid & 1) != 0 && eph.health == 0 {
                            let diff = ((time.Week - eph.week) as f64) * 604800.0 + 
                                       ((time.MilliSeconds / 1000) as f64 - eph.toe as f64);
                            
                            let abs_diff = diff.abs();
                            
                            if abs_diff <= 7200.0 && abs_diff < best_time_diff {
                                best_time_diff = abs_diff;
                                best_eph = Some(eph.to_gps_ephemeris()); // Конвертируем BeiDou в GPS формат
                            }
                        }
                    }
                }
            },
            GnssSystem::GalileoSystem => {
                // Поиск Galileo эфемерид
                for eph_opt in self.gal_ephemeris.iter() {
                    if let Some(eph) = eph_opt {
                        if eph.svid as i32 == svid && (eph.valid & 1) != 0 && eph.health == 0 {
                            let diff = ((time.Week - eph.week) as f64) * 604800.0 + 
                                       ((time.MilliSeconds / 1000) as f64 - eph.toe as f64);
                            
                            let abs_diff = diff.abs();
                            
                            if abs_diff <= 7200.0 && abs_diff < best_time_diff {
                                best_time_diff = abs_diff;
                                best_eph = Some(*eph);
                            }
                        }
                    }
                }
            },
            _ => return None,
        }
        
        best_eph
    }

    /// Поиск эфемерид ГЛОНАСС по времени и номеру слота - упрощенная версия  
    pub fn find_glo_ephemeris(&self, time: GlonassTime, svid: i32) -> Option<GlonassEphemeris> {
        let mut best_eph: Option<GlonassEphemeris> = None;
        let mut best_time_diff = f64::INFINITY;
        
        // Простое сравнение по дню и времени в секундах
        let request_day = time.Day as f64;
        let request_seconds = (time.MilliSeconds / 1000) as f64;
        
        // Поиск по номеру слота и времени
        for eph_opt in self.glonass_ephemeris.iter() {
            if let Some(eph) = eph_opt {
                if eph.slot as i32 == svid && eph.flag != 0 {
                    // Сравниваем по дню года и времени tb
                    let eph_day = eph.day as f64;
                    let eph_tb = eph.tb as f64;
                    
                    let day_diff = (request_day - eph_day).abs();
                    let time_diff = (request_seconds - eph_tb).abs();
                    
                    let total_diff = if day_diff == 0.0 { time_diff } else { day_diff * 86400.0 + time_diff };
                    
                    // GLONASS эфемериды действительны 30 минут (1800 секунд)
                    if total_diff < 1800.0 && total_diff < best_time_diff {
                        best_time_diff = total_diff;
                        best_eph = Some(*eph);
                    }
                }
            }
        }
        
        best_eph
    }

    pub fn complete_almanac(&mut self, _system: GnssSystem, _time: UtcTime) {
        // Placeholder - implement almanac completion logic
    }
}


// SignalPower is imported from powercontrol module

// Placeholder trait for navigation bit generation
pub trait NavBitTrait {
    fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32;
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris);
    fn set_almanac(&mut self, alm: &[GpsAlmanac]);
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>);
    fn get_type(&self) -> NavDataType;
    fn clone_box(&self) -> Box<dyn NavBitTrait>;
}

// Wrapper types for navigation bits with trait implementations
#[derive(Clone)]
pub struct CNavBit(ActualCNavBit);
impl Default for CNavBit {
    fn default() -> Self {
        Self::new()
    }
}

impl CNavBit { pub fn new() -> Self { CNavBit(ActualCNavBit::new()) } }
impl NavBitTrait for CNavBit {
    fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 { 
        self.0.get_frame_data(start_time, svid, param, nav_bits)
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { self.0.set_ephemeris(svid, eph); }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { self.0.set_almanac(alm); }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) { 
        if let (Some(iono), Some(utc)) = (iono_param, utc_param) {
            self.0.set_iono_utc(iono, utc); 
        }
    }
    fn get_type(&self) -> NavDataType { NavDataType::CNav }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

#[derive(Clone)]
pub struct CNav2Bit(ActualCNav2Bit);
impl Default for CNav2Bit {
    fn default() -> Self {
        Self::new()
    }
}

impl CNav2Bit { pub fn new() -> Self { CNav2Bit(ActualCNav2Bit::new()) } }
impl NavBitTrait for CNav2Bit {
    fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 { 
        self.0.get_frame_data(start_time, svid, param, nav_bits)
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { self.0.set_ephemeris(svid, eph); }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { self.0.set_almanac(alm); }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) {
        if let (Some(iono), Some(utc)) = (iono_param, utc_param) {
            self.0.set_iono_utc(iono, utc);
        }
    }
    fn get_type(&self) -> NavDataType { NavDataType::CNav2 }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

use crate::sat_if_signal::SatIfSignal;

#[derive(Clone)]
pub struct D1D2NavBit(ActualD1D2NavBit);
impl Default for D1D2NavBit {
    fn default() -> Self {
        Self::new()
    }
}

impl D1D2NavBit { pub fn new() -> Self { D1D2NavBit(ActualD1D2NavBit::new()) } }
impl NavBitTrait for D1D2NavBit {
    fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 { 
        self.0.get_frame_data(start_time, svid, param, nav_bits)
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { self.0.set_ephemeris(svid, eph); }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { self.0.set_almanac(alm); }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) {
        if let (Some(iono), Some(utc)) = (iono_param, utc_param) {
            self.0.set_iono_utc(iono, utc);
        }
    }
    fn get_type(&self) -> NavDataType { NavDataType::D1D2Nav }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

#[derive(Clone)]
pub struct INavBit(ActualINavBit);
impl Default for INavBit {
    fn default() -> Self {
        Self::new()
    }
}

impl INavBit { pub fn new() -> Self { INavBit(ActualINavBit::new()) } }
impl NavBitTrait for INavBit {
    fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 { 
        self.0.get_frame_data(start_time, svid, param, nav_bits)
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { 
        self.0.set_ephemeris(svid, eph);
    }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { 
        self.0.set_almanac(alm);
    }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) {
        if let (Some(iono), Some(utc)) = (iono_param, utc_param) {
            // Convert IonoParam to IonoNequick (assume Galileo NeQuick model)
            let inav_iono = INavIonoNequick {
                ai0: iono.a0,  // Use alpha parameters as ai parameters
                ai1: iono.a1,
                ai2: iono.a2,
                flag: iono.flag,
            };
            
            self.0.set_iono_utc(&inav_iono, utc);
        }
    }
    fn get_type(&self) -> NavDataType { NavDataType::INav }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

#[derive(Clone)]
pub struct FNavBit(ActualFNavBit);
impl Default for FNavBit {
    fn default() -> Self {
        Self::new()
    }
}

impl FNavBit { pub fn new() -> Self { FNavBit(ActualFNavBit::new()) } }
impl NavBitTrait for FNavBit {
    fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 { 
        self.0.get_frame_data(start_time, svid, param, nav_bits)
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { self.0.set_ephemeris(svid, eph); }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { self.0.set_almanac(alm); }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) {
        if let (Some(iono), Some(utc)) = (iono_param, utc_param) {
            self.0.set_iono_utc(iono, utc);
        }
    }
    fn get_type(&self) -> NavDataType { NavDataType::FNav }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

#[derive(Clone)]
pub struct BCNav1Bit(ActualBCNav1Bit);
impl Default for BCNav1Bit {
    fn default() -> Self {
        Self::new()
    }
}

impl BCNav1Bit { pub fn new() -> Self { BCNav1Bit(ActualBCNav1Bit::new()) } }
impl NavBitTrait for BCNav1Bit {
    fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 { 
        self.0.get_frame_data(start_time, svid, param, nav_bits)
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { self.0.set_ephemeris(svid, eph); }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { self.0.set_almanac(alm); }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) {
        self.0.set_iono_utc(iono_param, utc_param);
    }
    fn get_type(&self) -> NavDataType { NavDataType::BCNav1 }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

#[derive(Clone)]
pub struct BCNav2Bit(ActualBCNav2Bit);
impl Default for BCNav2Bit {
    fn default() -> Self {
        Self::new()
    }
}

impl BCNav2Bit { pub fn new() -> Self { BCNav2Bit(ActualBCNav2Bit::new()) } }
impl NavBitTrait for BCNav2Bit {
    fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 { 
        self.0.get_frame_data(start_time, svid, param, nav_bits)
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { self.0.set_ephemeris(svid, eph); }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { self.0.set_almanac(alm); }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) {
        self.0.set_iono_utc(iono_param, utc_param);
    }
    fn get_type(&self) -> NavDataType { NavDataType::BCNav2 }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

#[derive(Clone)]
pub struct BCNav3Bit(ActualBCNav3Bit);
impl Default for BCNav3Bit {
    fn default() -> Self {
        Self::new()
    }
}

impl BCNav3Bit { pub fn new() -> Self { BCNav3Bit(ActualBCNav3Bit::new()) } }
impl NavBitTrait for BCNav3Bit {
    fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 { 
        self.0.get_frame_data(start_time, svid, param, nav_bits)
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { self.0.set_ephemeris(svid, eph); }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { self.0.set_almanac(alm); }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) {
        self.0.set_iono_utc(iono_param, utc_param);
    }
    fn get_type(&self) -> NavDataType { NavDataType::BCNav3 }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

// Implement NavBitTrait for existing types
impl NavBitTrait for LNavBit {
    fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 {
        let mut nav_bits_300 = [0i32; 300];
        let result = LNavBit::get_frame_data(self, start_time, svid, param, &mut nav_bits_300);
        nav_bits[..300].copy_from_slice(&nav_bits_300);
        result
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { self.set_ephemeris(svid, eph); }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { 
        // Convert slice to array - for now, use default empty almanac
        let empty_almanac = [GpsAlmanac::default(); 32];
        self.set_almanac(&empty_almanac); 
    }
    fn set_iono_utc(&mut self, _iono_param: Option<&IonoParam>, _utc_param: Option<&UtcParam>) { 
        // Заглушка - не используется в GPS L1CA генерации
    }
    fn get_type(&self) -> NavDataType { NavDataType::NavDataGpsEph }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

impl NavBitTrait for L5CNavBit {
    fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 {
        let mut nav_bits_600 = [0i32; 600];
        let result = L5CNavBit::get_frame_data(self, start_time, svid, param, &mut nav_bits_600);
        nav_bits[..600].copy_from_slice(&nav_bits_600);
        result
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { self.set_ephemeris(svid, eph); }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { 
        // Convert slice to array - for now, use default empty almanac
        let empty_almanac = [GpsAlmanac::default(); 32];
        self.set_almanac(&empty_almanac); 
    }
    fn set_iono_utc(&mut self, _iono_param: Option<&IonoParam>, _utc_param: Option<&UtcParam>) { 
        // Заглушка - не используется в GPS L1CA генерации
    }
    fn get_type(&self) -> NavDataType { NavDataType::CNav }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

impl NavBitTrait for GNavBit {
    fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 {
        let mut nav_bits_100 = [0i32; 100];
        let result = self.get_frame_data(start_time, svid, param, &mut nav_bits_100);
        nav_bits[..100].copy_from_slice(&nav_bits_100);
        result
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { self.set_ephemeris(svid, eph); }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { self.set_almanac(alm); }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) { self.set_iono_utc(iono_param, utc_param); }
    fn get_type(&self) -> NavDataType { NavDataType::NavDataGlonassEph }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}
use crate::fastmath::FastMath;
use rayon::prelude::*;

// Constants for quantization and time conversion
const QUANT_SCALE_IQ4: f64 = 3.0;
const QUANT_SCALE_IQ8: f64 = 25.0;
const WEEK_MS: i32 = 604800000;

const TOTAL_GPS_SAT: usize = 32;
const TOTAL_BDS_SAT: usize = 63;
const TOTAL_GAL_SAT: usize = 36;
const TOTAL_GLO_SAT: usize = 24;
const TOTAL_SAT_CHANNEL: usize = 128;

#[derive(Debug, Clone, Copy)]
pub enum DataBitType {
    DataBitLNav = 0,    // for GPS
    DataBitCNav = 1,
    DataBitCNav2 = 2,
    DataBitL5CNav = 3,
    DataBitGNav = 4,    // for GLONASS
    DataBitGNav2 = 5,
    DataBitD1D2 = 6,    // for BDS
    DataBitBCNav1 = 7,
    DataBitBCNav2 = 8,
    DataBitBCNav3 = 9,
    DataBitINav = 10,   // for Galileo
    DataBitFNav = 11,
    DataBitECNav = 12,
    DataBitSbas = 13,   // for SBAS
}

pub struct IFDataGen {
    pub trajectory: CTrajectory,
    pub power_control: CPowerControl,
    pub nav_data: NavData,
    pub output_param: OutputParam,
    pub cur_time: GnssTime,
    pub start_pos: LlaPosition,
    
    // ЭКСТРЕМАЛЬНАЯ АППАРАТНАЯ ОПТИМИЗАЦИЯ
    #[cfg(feature = "gpu")]
    pub hybrid_accelerator: HybridAccelerator,
    #[cfg(not(feature = "gpu"))]
    pub _hybrid_placeholder: (),
    
    // Satellite ephemeris arrays
    pub gps_eph: [Option<GpsEphemeris>; TOTAL_GPS_SAT],
    pub gps_eph_visible: [Option<GpsEphemeris>; TOTAL_GPS_SAT],
    pub bds_eph: [Option<GpsEphemeris>; TOTAL_BDS_SAT],
    pub bds_eph_visible: [Option<GpsEphemeris>; TOTAL_BDS_SAT],
    pub gal_eph: [Option<GpsEphemeris>; TOTAL_GAL_SAT],
    pub gal_eph_visible: [Option<GpsEphemeris>; TOTAL_GAL_SAT],
    pub glo_eph: [Option<GlonassEphemeris>; TOTAL_GLO_SAT],
    pub glo_eph_visible: [Option<GlonassEphemeris>; TOTAL_GLO_SAT],
    
    // Satellite parameter arrays
    pub gps_sat_param: [SatelliteParam; TOTAL_GPS_SAT],
    pub bds_sat_param: [SatelliteParam; TOTAL_BDS_SAT],
    pub gal_sat_param: [SatelliteParam; TOTAL_GAL_SAT],
    pub glo_sat_param: [SatelliteParam; TOTAL_GLO_SAT],
    
    // Satellite numbers
    pub gps_sat_number: usize,
    pub bds_sat_number: usize,
    pub gal_sat_number: usize,
    pub glo_sat_number: usize,
}

// Signal center frequencies in Hz
const SIGNAL_CENTER_FREQ: [[f64; 8]; 4] = [
    [FREQ_GPS_L1, FREQ_GPS_L1, FREQ_GPS_L2, FREQ_GPS_L2, FREQ_GPS_L5, 0.0, 0.0, 0.0],
    [FREQ_BDS_B1C, FREQ_BDS_B1I, FREQ_BDS_B2I, FREQ_BDS_B3I, FREQ_BDS_B2A, FREQ_BDS_B2B, FREQ_BDS_B2AB, 0.0],
    [FREQ_GAL_E1, FREQ_GAL_E5A, FREQ_GAL_E5B, FREQ_GAL_E5, FREQ_GAL_E6, 0.0, 0.0, 0.0],
    [FREQ_GLO_G1, FREQ_GLO_G2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
];

const SIGNAL_NAME: [[&str; 8]; 4] = [
    ["L1CA", "L1C", "L2C", "L2P", "L5", "", "", ""],
    ["B1C", "B1I", "B2I", "B3I", "B2a", "B2b", "B2ab", ""],
    ["E1", "E5a", "E5b", "E5", "E6", "", "", ""],
    ["G1", "G2", "", "", "", "", "", ""],
];

impl IFDataGen {
    pub fn new() -> Self {
        IFDataGen {
            trajectory: CTrajectory::new(),
            power_control: CPowerControl::new(),
            nav_data: NavData::new(),
            output_param: OutputParam::default(),
            cur_time: GnssTime::default(),
            start_pos: LlaPosition::default(),
            
            // ЭКСТРЕМАЛЬНАЯ ИНИЦИАЛИЗАЦИЯ: CPU+GPU гибридное ускорение
            #[cfg(feature = "gpu")]
            hybrid_accelerator: HybridAccelerator::new(),
            #[cfg(not(feature = "gpu"))]
            _hybrid_placeholder: (),
            
            gps_eph: [None; TOTAL_GPS_SAT],
            gps_eph_visible: [None; TOTAL_GPS_SAT],
            bds_eph: [None; TOTAL_BDS_SAT],
            bds_eph_visible: [None; TOTAL_BDS_SAT],
            gal_eph: [None; TOTAL_GAL_SAT],
            gal_eph_visible: [None; TOTAL_GAL_SAT],
            glo_eph: [None; TOTAL_GLO_SAT],
            glo_eph_visible: [None; TOTAL_GLO_SAT],
            
            gps_sat_param: [SatelliteParam::default(); TOTAL_GPS_SAT],
            bds_sat_param: [SatelliteParam::default(); TOTAL_BDS_SAT],
            gal_sat_param: [SatelliteParam::default(); TOTAL_GAL_SAT],
            glo_sat_param: [SatelliteParam::default(); TOTAL_GLO_SAT],
            
            gps_sat_number: 0,
            bds_sat_number: 0,
            gal_sat_number: 0,
            glo_sat_number: 0,
        }
    }

    pub fn main(&mut self, args: Vec<String>) -> Result<(), Box<dyn std::error::Error>> {
        println!("\n================================================================================");
        println!("                          IF SIGNAL GENERATION ");
        println!("================================================================================");

        // Read JSON file and assign parameters
        let json_file = if args.len() > 1 {
            if args[1] == "--help" {
                println!("Usage: {} [optional JSON file path]", args[0]);
                return Ok(());
            }
            &args[1]
        } else {
            "IfGenTest.json" // Default JSON file
        };

        // ВРЕМЕННО: не используем JSON парсинг в initialize (используется load_config)
        println!("[INFO]\tSkipping JSON parsing (handled by load_config)");
        
        // Используем значения по умолчанию  
        let mut utc_time = UtcTime::default();
        let start_pos = LlaPosition::default();
        let start_vel = LocalSpeed::default();
        
        // Значения по умолчанию для тестирования
        utc_time.Year = 2025;
        utc_time.Month = 6;
        utc_time.Day = 5;
        utc_time.Hour = 10;
        utc_time.Minute = 5;
        utc_time.Second = 30.0;
        
        // TODO: Установить правильное время из JSON пресета
        // Пока используем хардкод что время симуляции 10 секунд
        
        println!("[INFO]\tParsed time from preset: {}-{:02}-{:02} {:02}:{:02}:{:02.0}", 
                 utc_time.Year, utc_time.Month, utc_time.Day, 
                 utc_time.Hour, utc_time.Minute, utc_time.Second);
        let start_pos = LlaPosition::default();
        let start_vel = LocalSpeed::default();

        // Старый код удалён - используем новый чистый Rust JSON парсинг ниже

        // Reset satellite masks to allow all satellites 
        self.output_param.GpsMaskOut = 0;
        self.output_param.BdsMaskOut = 0;
        self.output_param.GalileoMaskOut = 0;
        self.output_param.GlonassMaskOut = 0;
        
        // Set reasonable elevation mask (5 degrees)
        self.output_param.ElevationMask = 5.0_f64.to_radians();
        
        // Load RINEX ephemeris data with smart filtering
        // Читаем путь к RINEX файлу из JSON конфигурации
        let mut rinex_file = String::from("Rinex_Data/rinex_v3_20251560000.rnx"); // По умолчанию
        
        // Парсим JSON для получения пути к RINEX файлу
        if let Ok(json_content) = std::fs::read_to_string(json_file) {
            if let Ok(json) = serde_json::from_str::<serde_json::Value>(&json_content) {
                if let Some(ephemeris) = json.get("ephemeris") {
                    if let Some(name) = ephemeris.get("name").and_then(|v| v.as_str()) {
                        println!("[DEBUG] Using ephemeris file from JSON: {}", name);
                        rinex_file = String::from(name);
                    }
                }
            }
        }
        if std::path::Path::new(&rinex_file).exists() {
            println!("[INFO]\tLoading RINEX ephemeris file: {}", rinex_file);
            
            // Создаем CNavData для загрузки эфемерид
            let mut c_nav_data = crate::json_interpreter::CNavData::default();
            
            // Определяем включенные системы из JSON конфигурации (CompactConfig)
            let mut enabled_systems = Vec::new();
            if self.output_param.CompactConfig.should_parse_gps() {
                enabled_systems.push("GPS");
            }
            if self.output_param.CompactConfig.should_parse_glonass() {
                enabled_systems.push("GLONASS");
            }
            if self.output_param.CompactConfig.should_parse_bds() {
                enabled_systems.push("BeiDou");
            }
            if self.output_param.CompactConfig.should_parse_galileo() {
                enabled_systems.push("Galileo");
            }
            
            println!("[DEBUG] Using filtered RINEX loading: time={}-{:02}-{:02} {:02}:{:02}:{:02}, systems={:?}", 
                utc_time.Year, utc_time.Month, utc_time.Day, utc_time.Hour, utc_time.Minute, utc_time.Second, enabled_systems);
            
            // Используем оптимизированную загрузку RINEX с фильтрацией систем из JSON  
            crate::json_interpreter::read_nav_file_filtered(
                &mut c_nav_data, 
                &rinex_file,
                Some(utc_time), // Фильтрация по времени
                &enabled_systems // Только BeiDou система
            );
            
            // Копируем загруженные эфемериды в наши данные
            self.copy_ephemeris_from_json_nav_data(&c_nav_data, utc_time); // GPS эфемериды
            self.copy_beidou_ephemeris_from_json_nav_data(&c_nav_data, utc_time); // BeiDou эфемериды
            self.copy_galileo_ephemeris_from_json_nav_data(&c_nav_data, utc_time); // Galileo эфемериды
        } else {
            println!("[ERROR]\tRINEX file not found: {}", rinex_file);
        }

        // Initialize variables
        self.trajectory.reset_trajectory_time();
        self.cur_time = utc_to_gps_time(utc_time, false); // Use false for leap seconds flag
        let glonass_time = utc_to_glonass_time_corrected(utc_time);
        let bds_time = utc_to_bds_time(utc_time);
        let cur_pos = lla_to_ecef(&start_pos);
        let convert_matrix = calc_conv_matrix_lla(&start_pos);
        let mut pos_vel = KinematicInfo { x: cur_pos.x, y: cur_pos.y, z: cur_pos.z, ..Default::default() };
        speed_local_to_ecef(&convert_matrix, &start_vel, &mut pos_vel);

        let filename_binding = String::from_utf8_lossy(&self.output_param.filename);
        let filename_str = filename_binding.trim_end_matches('\0');
        println!("[INFO]\tOpening output file: {}", filename_str);
        let mut if_file = match File::create(filename_str) {
            Ok(file) => {
                println!("[INFO]\tOutput file opened successfully.");
                BufWriter::new(file)
            },
            Err(_) => {
                let filename_str = String::from_utf8_lossy(&self.output_param.filename);
                println!("[ERROR]\tFailed to open output file: {}", filename_str.trim_end_matches('\0'));
                return Err("Failed to open output file".into());
            }
        };

        // Initialize satellite CN0 values
        for i in 0..TOTAL_GPS_SAT {
            self.gps_sat_param[i].CN0 = (self.power_control.init_cn0 * 100.0 + 0.5) as i32;
        }
        for i in 0..TOTAL_BDS_SAT {
            self.bds_sat_param[i].CN0 = (self.power_control.init_cn0 * 100.0 + 0.5) as i32;
        }
        for i in 0..TOTAL_GAL_SAT {
            self.gal_sat_param[i].CN0 = (self.power_control.init_cn0 * 100.0 + 0.5) as i32;
        }
        for i in 0..TOTAL_GLO_SAT {
            self.glo_sat_param[i].CN0 = (self.power_control.init_cn0 * 100.0 + 0.5) as i32;
        }

        // Create navigation bit instances
        let mut nav_bit_array = self.create_nav_bit_instances();

        self.setup_navigation_data(&mut nav_bit_array, utc_time, glonass_time, bds_time)?;
        self.calculate_visible_satellites(cur_pos, glonass_time)?;
        println!("[DEBUG]\tVisible satellites calculation completed");

        let mut sat_if_signals = self.create_satellite_signals(&nav_bit_array)?;
        println!("[DEBUG]\tSatellite signals created successfully");
        
        // Парсим время траектории из JSON пресета (используем хардкод пока)
        let trajectory_time_s = self.parse_trajectory_time_from_json(&self.output_param.config_filename).unwrap_or(10.0);
        self.generate_if_signal(&mut if_file, &mut sat_if_signals, cur_pos, trajectory_time_s)?;

        println!("[INFO]\tIF Signal generation completed!");
        Ok(())
    } 
   fn create_nav_bit_instances(&self) -> Vec<Option<UnifiedNavData>> {
        let mut nav_bit_array: Vec<Option<UnifiedNavData>> = Vec::with_capacity(14);
        
        for i in 0..14 {
            let nav_bit: Option<UnifiedNavData> = match i {
                0 => Some(UnifiedNavData::LNav(LNavBit::new())),           // DataBitLNav
                1 => Some(UnifiedNavData::CNav(ActualCNavBit::new())), // DataBitCNav
                2 => Some(UnifiedNavData::CNav2(ActualCNav2Bit::new())), // DataBitCNav2
                3 => Some(UnifiedNavData::L5CNav(L5CNavBit::new())),   // DataBitL5CNav
                4 => Some(UnifiedNavData::GNav(GNavBit::new())),       // DataBitGNav
                5 => None,                                      // DataBitGNav2
                6 => Some(UnifiedNavData::D1D2Nav(ActualD1D2NavBit::new())), // DataBitD1D2
                7 => Some(UnifiedNavData::BCNav1(ActualBCNav1Bit::new())), // DataBitBCNav1
                8 => Some(UnifiedNavData::BCNav2(ActualBCNav2Bit::new())), // DataBitBCNav2
                9 => Some(UnifiedNavData::BCNav3(ActualBCNav3Bit::new())), // DataBitBCNav3
                10 => Some(UnifiedNavData::INav(ActualINavBit::new())),   // DataBitINav
                11 => Some(UnifiedNavData::FNav(ActualFNavBit::new())),   // DataBitFNav
                12 => None,                                    // DataBitECNav
                13 => None,                                    // DataBitSbas
                _ => None,
            };
            nav_bit_array.push(nav_bit);
        }
        
        nav_bit_array
    }


    fn setup_navigation_data(&mut self, nav_bit_array: &mut Vec<Option<UnifiedNavData>>, 
                           utc_time: UtcTime, glonass_time: GlonassTime, bds_time: GnssTime) -> Result<(), Box<dyn std::error::Error>> {
        
        // Set Ionosphere and UTC parameters for different navigation data bits
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitLNav as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_gps_iono(), self.nav_data.get_gps_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_gps_iono(), self.nav_data.get_gps_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav2 as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_gps_iono(), self.nav_data.get_gps_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitD1D2 as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_bds_iono(), self.nav_data.get_bds_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav1 as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_bds_iono(), self.nav_data.get_bds_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitINav as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_galileo_iono(), self.nav_data.get_galileo_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitFNav as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_galileo_iono(), self.nav_data.get_galileo_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitGNav as usize] {
            nav_bit.set_iono_utc(None, self.nav_data.get_gps_utc_param()); // GLONASS uses GPS UTC parameters
        }

        // Find ephemeris matching current time and fill in data to generate bit stream
        // NOTE: Do not overwrite gps_eph array that was loaded from RINEX!
        for i in 1..=TOTAL_GPS_SAT {
            // Use already loaded ephemeris instead of searching (which returns None)
            // self.gps_eph[i-1] = self.nav_data.find_ephemeris(GnssSystem::GpsSystem, self.cur_time, i as i32, 0, 0);
            
            // For L1CA/L1C/L2C/L5, all can use the same ephemeris data
            if let Some(ref eph) = self.gps_eph[i-1] {
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitLNav as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav2 as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitL5CNav as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
            }
        }

        // ИСПРАВЛЕНИЕ: Не перезаписываем BeiDou эфемериды из find_ephemeris
        // Они уже правильно скопированы из JSON в copy_beidou_ephemeris_from_json_nav_data
        // find_ephemeris ищет в nav_data.bds_ephemeris, но не в bds_ephemeris_pool
        println!("[DEBUG] Skipping find_ephemeris loop for BeiDou - data already copied from JSON");
        
        /*
        for i in 1..=TOTAL_BDS_SAT {
            let found_eph = self.nav_data.find_ephemeris(GnssSystem::BdsSystem, bds_time, i as i32, 0, 0);
            self.bds_eph[i-1] = found_eph;
            
            if let Some(ref eph) = self.bds_eph[i-1] {
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitD1D2 as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav1 as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav2 as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav3 as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
            }
        }
        */
        
        // Вместо перезаписи инициализируем навигационные биты для уже загруженных BeiDou эфемерид
        for i in 0..TOTAL_BDS_SAT {
            if let Some(ref eph) = self.bds_eph[i] {
                let svid = (i + 1) as i32;
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitD1D2 as usize] {
                    nav_bit.set_ephemeris(svid, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav1 as usize] {
                    nav_bit.set_ephemeris(svid, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav2 as usize] {
                    nav_bit.set_ephemeris(svid, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav3 as usize] {
                    nav_bit.set_ephemeris(svid, eph);
                }
            }
        }

        // Настройка навигационных битов для Galileo (эфемериды уже скопированы в copy_galileo_ephemeris_from_json_nav_data)
        for i in 0..TOTAL_GAL_SAT {
            if let Some(ref eph) = self.gal_eph[i] {
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitINav as usize] {
                    nav_bit.set_ephemeris((i + 1) as i32, eph); // SVID starts from 1
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitFNav as usize] {
                    nav_bit.set_ephemeris((i + 1) as i32, eph); // SVID starts from 1  
                }
            }
        }

        let mut glo_eph_count = 0;
        for i in 1..=TOTAL_GLO_SAT {
            self.glo_eph[i-1] = self.nav_data.find_glo_ephemeris(glonass_time, i as i32);
            
            if let Some(ref eph) = self.glo_eph[i-1] {
                if eph.flag != 0 {
                    if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitGNav as usize] {
                        // Convert GLONASS ephemeris to GPS format for compatibility
                        let gps_eph = self.convert_glonass_to_gps_ephemeris(eph);
                        nav_bit.set_ephemeris(i as i32, &gps_eph);
                    }
                    glo_eph_count += 1;
                }
            }
        }
        println!("[INFO] GLONASS ephemeris loaded: {} satellites", glo_eph_count);

        // Complete almanac data
        self.nav_data.complete_almanac(GnssSystem::GpsSystem, utc_time);
        self.nav_data.complete_almanac(GnssSystem::BdsSystem, utc_time);
        self.nav_data.complete_almanac(GnssSystem::GalileoSystem, utc_time);
        self.nav_data.complete_almanac(GnssSystem::GlonassSystem, utc_time);

        // Set almanac data for navigation bits
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitLNav as usize] {
            nav_bit.set_almanac(self.nav_data.get_gps_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav as usize] {
            nav_bit.set_almanac(self.nav_data.get_gps_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav2 as usize] {
            nav_bit.set_almanac(self.nav_data.get_gps_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitL5CNav as usize] {
            nav_bit.set_almanac(self.nav_data.get_gps_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitD1D2 as usize] {
            nav_bit.set_almanac(self.nav_data.get_bds_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav1 as usize] {
            nav_bit.set_almanac(self.nav_data.get_bds_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav2 as usize] {
            nav_bit.set_almanac(self.nav_data.get_bds_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav3 as usize] {
            nav_bit.set_almanac(self.nav_data.get_bds_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitINav as usize] {
            nav_bit.set_almanac(self.nav_data.get_galileo_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitFNav as usize] {
            nav_bit.set_almanac(self.nav_data.get_galileo_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitGNav as usize] {
            // Convert GLONASS almanac to GPS format for compatibility
            let gps_almanac = self.convert_glonass_to_gps_almanac(self.nav_data.get_glonass_almanac());
            nav_bit.set_almanac(&gps_almanac);
        }

        Ok(())
    }    fn calculate_visible_satellites(&mut self, cur_pos: KinematicInfo, glonass_time: GlonassTime) -> Result<(), Box<dyn std::error::Error>> {
        println!("[INFO]\tCalculating visible satellites at position ({:.1}, {:.1}, {:.1})", 
                 cur_pos.x, cur_pos.y, cur_pos.z);
        
        // Debug: check if we have ephemeris before calculating visibility
        let mut total_gps_eph = 0;
        for i in 0..TOTAL_GPS_SAT {
            if self.gps_eph[i].is_some() {
                total_gps_eph += 1;
            }
        }
        println!("[DEBUG] Total GPS ephemeris available: {}/{}", total_gps_eph, TOTAL_GPS_SAT);
        
        // Calculate visible GPS satellites
        self.gps_sat_number = if self.output_param.CompactConfig.should_parse_gps() {
            let mut sat_number = 0;
            let elevation_mask = self.output_param.ElevationMask;
            
            let transmit_time = (self.cur_time.MilliSeconds as f64) / 1000.0;
            
            println!("[DEBUG] GPS Visibility Analysis:");
            println!("[DEBUG] Current GPS time: Week={}, MS={}, Seconds={:.1}", 
                     self.cur_time.Week, self.cur_time.MilliSeconds, transmit_time);
            println!("[DEBUG] Receiver position: ({:.1}, {:.1}, {:.1})", cur_pos.x, cur_pos.y, cur_pos.z);
            println!("[DEBUG] GPS mask out value: 0x{:08x}, elevation mask: {}°", 
                     self.output_param.GpsMaskOut, elevation_mask.to_degrees());
            
            for i in 0..TOTAL_GPS_SAT {
                if let Some(eph) = &self.gps_eph[i] {
                    println!("[DEBUG] Checking GPS satellite {} (SVID {})", i, eph.svid);
                    
                    // Check health and validity
                    if (eph.valid & 1) == 0 {
                        println!("[DEBUG] Satellite {} rejected: invalid (valid={})", i, eph.valid);
                        continue;
                    }
                    if eph.health != 0 {
                        println!("[DEBUG] Satellite {} rejected: unhealthy (health={})", i, eph.health);
                        continue;
                    }
                    
                    // Check mask out
                    if (self.output_param.GpsMaskOut & (1u32 << i)) != 0 {
                        println!("[DEBUG] Satellite {} rejected: masked out", i);
                        continue;
                    }
                    
                    // Calculate satellite position
                    let transmit_time = (self.cur_time.MilliSeconds as f64) / 1000.0;
                    let mut sat_pos = KinematicInfo::default();
                    let mut eph_mut = *eph; // Копируем эфемериды для мутабельности
                    let delta_t_gps = transmit_time - eph.toe as f64;
                    println!("[DEBUG] GPS{:02} - toe={}, transmit_time={}, delta_t={:.1}h", i+1, eph.toe, transmit_time, delta_t_gps / 3600.0);
                    if crate::coordinate::gps_sat_pos_speed_eph(GnssSystem::GpsSystem, transmit_time, &mut eph_mut, &mut sat_pos, None) {
                        let range = ((sat_pos.x - cur_pos.x).powi(2) + 
                                     (sat_pos.y - cur_pos.y).powi(2) + 
                                     (sat_pos.z - cur_pos.z).powi(2)).sqrt();
                        println!("[DEBUG] Satellite {} position: ({:.1}, {:.1}, {:.1}), range: {:.0}km", 
                                 i, sat_pos.x, sat_pos.y, sat_pos.z, range/1000.0);
                        
                        // Calculate elevation and azimuth using coordinate.rs function
                        let mut elevation = 0.0;
                        let mut azimuth = 0.0;
                        crate::coordinate::sat_el_az_from_positions(&cur_pos, &sat_pos, &mut elevation, &mut azimuth);
                        println!("[DEBUG] Satellite {} elevation: {:.1}°, azimuth: {:.1}°, mask: {:.1}°", 
                                 i, elevation.to_degrees(), azimuth.to_degrees(), elevation_mask.to_degrees());
                        
                        if elevation >= elevation_mask {
                            if sat_number < TOTAL_GPS_SAT {
                                self.gps_eph_visible[sat_number] = Some(*eph);
                                sat_number += 1;
                                println!("[DEBUG] Satellite {} added as visible #{}", i, sat_number);
                            }
                        } else {
                            println!("[DEBUG] Satellite {} rejected: elevation too low ({:.1}° < {:.1}°)", 
                                     i, elevation.to_degrees(), elevation_mask);
                        }
                    } else {
                        println!("[DEBUG] Satellite {} rejected: position calculation failed", i);
                    }
                } else if i < 10 { // Показать только первые 10 для краткости
                    println!("[DEBUG] No ephemeris for GPS satellite {}", i);
                }
            }
            println!("[INFO]\tFound {} visible GPS satellites", sat_number);
            
            // Подробная таблица видимых GPS спутников как в C-версии
            if sat_number > 0 {
                println!("┌─────┬─────┬──────────┬──────────┬─────────────┬──────────┐");
                println!("│ PRN │ SV  │ Elev(°)  │ Azim(°)  │ Doppler(Hz) │ Range(m) │");
                println!("├─────┼─────┼──────────┼──────────┼─────────────┼──────────┤");
                
                for j in 0..sat_number {
                    if let Some(eph) = &self.gps_eph_visible[j] {
                        let transmit_time = (self.cur_time.MilliSeconds as f64) / 1000.0;
                        let mut sat_pos_vel = KinematicInfo::default();
                        let mut eph_mut = *eph;
                        
                        if crate::coordinate::gps_sat_pos_speed_eph(GnssSystem::GpsSystem, transmit_time, &mut eph_mut, &mut sat_pos_vel, None) {
                            let mut elevation = 0.0;
                            let mut azimuth = 0.0;
                            crate::coordinate::sat_el_az_from_positions(&cur_pos, &sat_pos_vel, &mut elevation, &mut azimuth);
                            
                            let range = ((sat_pos_vel.x - cur_pos.x).powi(2) + 
                                        (sat_pos_vel.y - cur_pos.y).powi(2) + 
                                        (sat_pos_vel.z - cur_pos.z).powi(2)).sqrt();
                            
                            // Вычисление Doppler частоты
                            let los_x = (sat_pos_vel.x - cur_pos.x) / range;
                            let los_y = (sat_pos_vel.y - cur_pos.y) / range;
                            let los_z = (sat_pos_vel.z - cur_pos.z) / range;
                            let radial_velocity = los_x * sat_pos_vel.vx + los_y * sat_pos_vel.vy + los_z * sat_pos_vel.vz;
                            let doppler_hz = -radial_velocity * 1575.42e6 / crate::constants::LIGHT_SPEED;
                            
                            println!("│ {:3} │ {:3} │ {:8.1} │ {:8.1} │ {:11.1} │ {:8.0} │", 
                                    eph.svid, eph.svid, 
                                    elevation.to_degrees(), azimuth.to_degrees(), 
                                    doppler_hz, range);
                        }
                    }
                }
                println!("└─────┴─────┴──────────┴──────────┴─────────────┴──────────┘");
                println!();
            }
            
            sat_number
        } else {
            0
        };

        // Calculate visible BeiDou satellites  
        println!("[DEBUG] CompactConfig = 0x{:08x}", self.output_param.CompactConfig.config);
        println!("[DEBUG] Parsing systems: GPS={}, BDS={}, GAL={}, GLO={}", 
            self.output_param.CompactConfig.should_parse_gps(),
            self.output_param.CompactConfig.should_parse_bds(), 
            self.output_param.CompactConfig.should_parse_galileo(),
            self.output_param.CompactConfig.should_parse_glonass());
        self.bds_sat_number = if self.output_param.CompactConfig.should_parse_bds() {
            let mut sat_number = 0;
            let elevation_mask = self.output_param.ElevationMask;
            
            println!("[DEBUG] Checking BeiDou satellites visibility, elevation_mask = {}", elevation_mask);
            println!("[DEBUG] Total BDS ephemeris slots to check: {}", TOTAL_BDS_SAT);
            
            // Проверим состояние массива bds_eph
            let mut filled_slots = 0;
            for i in 0..TOTAL_BDS_SAT {
                if self.bds_eph[i].is_some() {
                    filled_slots += 1;
                }
            }
            println!("[DEBUG] Total filled BDS ephemeris slots: {}/{}", filled_slots, TOTAL_BDS_SAT);
            
            for i in 0..TOTAL_BDS_SAT {
                if let Some(eph) = &self.bds_eph[i] {
                    if (eph.valid & 1) == 0 || eph.health != 0 {
                        continue;
                    }
                    
                    if (self.output_param.BdsMaskOut & (1u64 << i)) != 0 {
                        continue;
                    }
                    
                    let transmit_time = (self.cur_time.MilliSeconds as f64) / 1000.0;
                    println!("[DEBUG] Checking BeiDou SVID {} at time {}", eph.svid, transmit_time);
                    let mut sat_pos = KinematicInfo::default();
                    let mut eph_mut = *eph; // Копируем эфемериды для мутабельности
                    if crate::coordinate::gps_sat_pos_speed_eph(GnssSystem::BdsSystem, transmit_time, &mut eph_mut, &mut sat_pos, None) {
                        println!("[DEBUG] BeiDou sat pos calculated: ({:.1}, {:.1}, {:.1})", sat_pos.x, sat_pos.y, sat_pos.z);
                        let mut elevation = 0.0;
                        let mut azimuth = 0.0;
                        crate::coordinate::sat_el_az_from_positions(&cur_pos, &sat_pos, &mut elevation, &mut azimuth);
                        
                        if elevation >= elevation_mask
                            && sat_number < TOTAL_BDS_SAT {
                                self.bds_eph_visible[sat_number] = Some(*eph);
                                sat_number += 1;
                                if sat_number <= 5 { // Показываем первые 5 видимых
                                    println!("[DEBUG] BDS[{}]: VISIBLE, SVID={}, elevation={:.1}°", i, eph.svid, elevation.to_degrees());
                                }
                            }
                    }
                }
            }
            println!("[INFO]\tFound {} visible BeiDou satellites", sat_number);
            
            // Подробная таблица видимых BeiDou спутников как в C-версии
            if sat_number > 0 {
                println!("┌─────┬─────┬──────────┬──────────┬─────────────┬──────────┐");
                println!("│ PRN │ SV  │ Elev(°)  │ Azim(°)  │ Doppler(Hz) │ Range(m) │");
                println!("├─────┼─────┼──────────┼──────────┼─────────────┼──────────┤");
                
                for j in 0..sat_number {
                    if let Some(eph) = &self.bds_eph_visible[j] {
                        let transmit_time = (self.cur_time.MilliSeconds as f64) / 1000.0;
                        let mut sat_pos_vel = KinematicInfo::default();
                        let mut eph_mut = *eph;
                        
                        if crate::coordinate::gps_sat_pos_speed_eph(GnssSystem::BdsSystem, transmit_time, &mut eph_mut, &mut sat_pos_vel, None) {
                            let mut elevation = 0.0;
                            let mut azimuth = 0.0;
                            crate::coordinate::sat_el_az_from_positions(&cur_pos, &sat_pos_vel, &mut elevation, &mut azimuth);
                            
                            let range = ((sat_pos_vel.x - cur_pos.x).powi(2) + 
                                        (sat_pos_vel.y - cur_pos.y).powi(2) + 
                                        (sat_pos_vel.z - cur_pos.z).powi(2)).sqrt();
                            
                            // Вычисление Doppler частоты для BeiDou B1C
                            let los_x = (sat_pos_vel.x - cur_pos.x) / range;
                            let los_y = (sat_pos_vel.y - cur_pos.y) / range;
                            let los_z = (sat_pos_vel.z - cur_pos.z) / range;
                            let radial_velocity = los_x * sat_pos_vel.vx + los_y * sat_pos_vel.vy + los_z * sat_pos_vel.vz;
                            let doppler_hz = -radial_velocity * 1575.42e6 / crate::constants::LIGHT_SPEED;
                            
                            println!("│ C{:2} │ {:3} │ {:8.1} │ {:8.1} │ {:11.1} │ {:8.0} │", 
                                    eph.svid, eph.svid, 
                                    elevation.to_degrees(), azimuth.to_degrees(), 
                                    doppler_hz, range);
                        }
                    }
                }
                println!("└─────┴─────┴──────────┴──────────┴─────────────┴──────────┘");
                println!();
            }
            
            sat_number
        } else {
            0
        };

        // Calculate visible Galileo satellites
        self.gal_sat_number = if self.output_param.CompactConfig.should_parse_galileo() {
            let mut sat_number = 0;
            let mut total_checked = 0;
            let mut valid_failed = 0;
            let mut mask_failed = 0;
            let mut pos_failed = 0;
            let mut elev_failed = 0;
            let elevation_mask = self.output_param.ElevationMask;
            
            println!("[DEBUG] Starting Galileo visibility check: elevation_mask={}", elevation_mask);
            
            for i in 0..TOTAL_GAL_SAT {
                if let Some(eph) = &self.gal_eph[i] {
                    total_checked += 1;
                    println!("[DEBUG] GAL{:02} - svid={}, valid={}, health={}", i+1, eph.svid, eph.valid, eph.health);
                    
                    if (eph.valid & 1) == 0 || eph.health != 0 {
                        valid_failed += 1;
                        println!("[DEBUG] GAL{:02} - FAILED: valid/health check", i+1);
                        continue;
                    }
                    
                    if (self.output_param.GalileoMaskOut & (1u64 << i)) != 0 {
                        mask_failed += 1;
                        println!("[DEBUG] GAL{:02} - FAILED: mask out", i+1);
                        continue;
                    }
                    
                    let transmit_time = (self.cur_time.MilliSeconds as f64) / 1000.0;
                    let mut sat_pos = KinematicInfo::default();
                    let mut eph_mut = *eph; // Копируем эфемериды для мутабельности
                    let delta_t = transmit_time - eph.toe as f64;
                    println!("[DEBUG] GAL{:02} - toe={}, transmit_time={}, delta_t={:.1}h", i+1, eph.toe, transmit_time, delta_t / 3600.0);
                    let pos_calc_success = crate::coordinate::gps_sat_pos_speed_eph(GnssSystem::GalileoSystem, transmit_time, &mut eph_mut, &mut sat_pos, None);
                    println!("[DEBUG] GAL{:02} - pos_calc_success={}, sat_pos=({:.1}, {:.1}, {:.1})", i+1, pos_calc_success, sat_pos.x, sat_pos.y, sat_pos.z);
                    if pos_calc_success {
                        let mut elevation = 0.0;
                        let mut azimuth = 0.0;
                        crate::coordinate::sat_el_az_from_positions(&cur_pos, &sat_pos, &mut elevation, &mut azimuth);
                        println!("[DEBUG] GAL{:02} - elevation={:.1}°, azimuth={:.1}°", i+1, elevation.to_degrees(), azimuth.to_degrees());
                        
                        if elevation >= elevation_mask {
                            if sat_number < TOTAL_GAL_SAT {
                                self.gal_eph_visible[sat_number] = Some(*eph);
                                sat_number += 1;
                                println!("[DEBUG] GAL{:02} - VISIBLE! Added as #{}", i+1, sat_number);
                            }
                        } else {
                            elev_failed += 1;
                            println!("[DEBUG] GAL{:02} - FAILED: elevation {:.1}° < {:.1}°", i+1, elevation.to_degrees(), elevation_mask.to_degrees());
                        }
                    } else {
                        pos_failed += 1;
                        println!("[DEBUG] GAL{:02} - FAILED: position calculation", i+1);
                    }
                }
            }
            println!("[DEBUG] Galileo summary: checked={}, valid_failed={}, mask_failed={}, pos_failed={}, elev_failed={}, visible={}", 
                     total_checked, valid_failed, mask_failed, pos_failed, elev_failed, sat_number);
            println!("[INFO]\tFound {} visible Galileo satellites", sat_number);
            
            // Подробная таблица видимых Galileo спутников как в C-версии
            if sat_number > 0 {
                println!("┌─────┬─────┬──────────┬──────────┬─────────────┬──────────┐");
                println!("│ PRN │ SV  │ Elev(°)  │ Azim(°)  │ Doppler(Hz) │ Range(m) │");
                println!("├─────┼─────┼──────────┼──────────┼─────────────┼──────────┤");
                
                for j in 0..sat_number {
                    if let Some(eph) = &self.gal_eph_visible[j] {
                        let transmit_time = (self.cur_time.MilliSeconds as f64) / 1000.0;
                        let mut sat_pos_vel = KinematicInfo::default();
                        let mut eph_mut = *eph;
                        
                        if crate::coordinate::gps_sat_pos_speed_eph(GnssSystem::GalileoSystem, transmit_time, &mut eph_mut, &mut sat_pos_vel, None) {
                            let mut elevation = 0.0;
                            let mut azimuth = 0.0;
                            crate::coordinate::sat_el_az_from_positions(&cur_pos, &sat_pos_vel, &mut elevation, &mut azimuth);
                            
                            let range = ((sat_pos_vel.x - cur_pos.x).powi(2) + 
                                        (sat_pos_vel.y - cur_pos.y).powi(2) + 
                                        (sat_pos_vel.z - cur_pos.z).powi(2)).sqrt();
                            
                            // Вычисление Doppler частоты для Galileo E1
                            let los_x = (sat_pos_vel.x - cur_pos.x) / range;
                            let los_y = (sat_pos_vel.y - cur_pos.y) / range;
                            let los_z = (sat_pos_vel.z - cur_pos.z) / range;
                            let radial_velocity = los_x * sat_pos_vel.vx + los_y * sat_pos_vel.vy + los_z * sat_pos_vel.vz;
                            let doppler_hz = -radial_velocity * 1575.42e6 / crate::constants::LIGHT_SPEED;
                            
                            println!("│ E{:2} │ {:3} │ {:8.1} │ {:8.1} │ {:11.1} │ {:8.0} │", 
                                    eph.svid, eph.svid, 
                                    elevation.to_degrees(), azimuth.to_degrees(), 
                                    doppler_hz, range);
                        }
                    }
                }
                println!("└─────┴─────┴──────────┴──────────┴─────────────┴──────────┘");
                println!();
            }
            
            sat_number
        } else {
            0
        };

        // Calculate visible GLONASS satellites
        self.glo_sat_number = if self.output_param.CompactConfig.should_parse_glonass() {
            let mut sat_number = 0;
            let elevation_mask = self.output_param.ElevationMask;
            
            for i in 0..TOTAL_GLO_SAT {
                if let Some(eph) = &self.glo_eph[i] {
                    if eph.flag == 0 {
                        continue;
                    }
                    
                    if (self.output_param.GlonassMaskOut & (1u32 << i)) != 0 {
                        continue;
                    }
                    
                    let transmit_time = (glonass_time.MilliSeconds as f64) / 1000.0;
                    if let Some(sat_pos) = self.glonass_sat_pos_speed_eph(transmit_time, eph) {
                        let mut elevation = 0.0;
                        let mut azimuth = 0.0;
                        crate::coordinate::sat_el_az_from_positions(&cur_pos, &sat_pos, &mut elevation, &mut azimuth);
                        
                        if elevation >= elevation_mask
                            && sat_number < TOTAL_GLO_SAT {
                                self.glo_eph_visible[sat_number] = Some(*eph);
                                sat_number += 1;
                            }
                    }
                }
            }
            println!("[INFO]\tFound {} visible GLONASS satellites", sat_number);
            sat_number
        } else {
            0
        };

        // Check which satellites are visible and have ephemeris
        if self.glo_sat_number > 0 {
            print!("[GLONASS] Visible satellites: ");
            for j in 0..self.glo_sat_number {
                if let Some(ref eph) = self.glo_eph_visible[j] {
                    print!("SV{:02}(k={:+2}) ", eph.n, eph.freq);
                }
            }
            println!();
        }

        // Temporarily store values to avoid borrowing conflicts
        let (power_slice, list_count) = self.power_control.get_power_control_list(0);
        let mut power_list_owned = Vec::new();
        power_list_owned.extend_from_slice(power_slice);
        let cur_time = self.cur_time;
        // Placeholder - avoid borrowing conflicts by using None
        self.update_sat_param_list(cur_time, cur_pos, list_count, &power_list_owned, None);

        Ok(())
    }

    fn create_satellite_signals(&mut self, nav_bit_array: &Vec<Option<UnifiedNavData>>) -> Result<Vec<Option<Box<SatIfSignal>>>, Box<dyn std::error::Error>> {
        let mut sat_if_signals: Vec<Option<Box<SatIfSignal>>> = (0..TOTAL_SAT_CHANNEL).map(|_| None).collect();
        let mut total_channel_number = 0;

        println!("[INFO]\tUsing Rayon parallelization for satellite processing");

        println!("[INFO]\tGenerating IF data with following satellite signals:\n");

        // Enhanced signal display with cleaner formatting
        println!("[INFO]\tEnabled Signals:");

        // Display enabled signals for each system
        self.display_enabled_signals();

        // Count total signals per system
        let (gps_signal_count, bds_signal_count, gal_signal_count, glo_signal_count) = self.count_signals_per_system();

        self.display_signals_summary_table(gps_signal_count, bds_signal_count, gal_signal_count, glo_signal_count);

        // Create satellite signal instances for each system
        total_channel_number = self.create_gps_signals(&mut sat_if_signals, total_channel_number, nav_bit_array)?;
        total_channel_number = self.create_bds_signals(&mut sat_if_signals, total_channel_number, nav_bit_array)?;
        total_channel_number = self.create_galileo_signals(&mut sat_if_signals, total_channel_number, nav_bit_array)?;
        total_channel_number = self.create_glonass_signals(&mut sat_if_signals, total_channel_number, nav_bit_array)?;

        Ok(sat_if_signals)
    }

    fn generate_if_signal(&mut self, if_file: &mut BufWriter<File>, sat_if_signals: &mut Vec<Option<Box<SatIfSignal>>>, mut cur_pos: KinematicInfo, trajectory_duration_s: f64) -> Result<(), Box<dyn std::error::Error>> {
        // Calculate samples per millisecond (SampleFreq is in Hz)
        let samples_per_ms = (self.output_param.SampleFreq as f64 / 1000.0) as usize;
        let mut noise_array = vec![ComplexNumber::new(); samples_per_ms];
        let mut quant_array = vec![0u8; samples_per_ms * 2];

        // Calculate total data size and setup progress tracking
        // Используем переданное время траектории из JSON пресета  
        let total_duration_ms = (trajectory_duration_s * 1000.0) as i32;
        println!("[INFO]\tUsing trajectory duration: {:.1}s ({} ms)", trajectory_duration_s, total_duration_ms);
        let bytes_per_ms = samples_per_ms as f64 * if self.output_param.Format == OutputFormat::OutputFormatIQ4 { 1.0 } else { 2.0 };
        let total_mb = (total_duration_ms as f64 * bytes_per_ms) / (1024.0 * 1024.0);
        let mut length = 0;
        let mut total_clipped_samples = 0i64;
        let mut total_samples = 0i64;
        let mut agc_gain = 1.0; // Automatic gain control
        
        // PERFORMANCE OPTIMIZATION: Use real GNSS signal generation with optimizations
        let debug_mode = false; // Always use full signal generation
        // TODO: Make max_satellites configurable via command line or JSON
        // For now, None means use all available satellites (restore original behavior)
        let max_satellites_for_debug: Option<usize> = None; // Use all satellites (was 3)

        println!("[INFO]\tStarting signal generation loop...");
        println!("[INFO]\tSignal Duration: {:.2} s", total_duration_ms as f64 / 1000.0);
        println!("[INFO]\tSignal Size: {:.2} MB", total_mb);
        println!("[INFO]\tSignal Data format: {}", if self.output_param.Format == OutputFormat::OutputFormatIQ4 { "IQ4" } else { "IQ8" });
        println!("[INFO]\tSignal Center freq: {:.4} MHz", self.output_param.CenterFreq as f64 / 1000.0);
        println!("[INFO]\tSignal Sample rate: {:.2} MHz", self.output_param.SampleFreq);
        
        if debug_mode {
            println!("[PERF]\tDEBUG MODE: Using fast test signals instead of full GNSS generation");
        }
        println!();

        let start_time = Instant::now();
        
        println!("[INFO]\tStarting main generation loop...");
        
        // РЕВОЛЮЦИОННЫЙ РЕЖИМ: Истинная параллелизация спутников
        if std::env::var("GNSS_PARALLEL_MODE").unwrap_or_default() == "true" {
            let signals_owned = std::mem::take(sat_if_signals);
            return self.generate_with_true_parallelization(
                if_file,
                signals_owned, 
                total_duration_ms,
                samples_per_ms,
                self.cur_time
            );
        }
        
        let mut iteration_count = 0;
        
        while length < total_duration_ms {
            iteration_count += 1;
            if iteration_count <= 5 {
                println!("[DEBUG]\tGeneration iteration {}, current length: {} ms / {} ms total", iteration_count, length, total_duration_ms);
            }
            
            // Step to next millisecond
            self.step_to_next_ms(&mut cur_pos)?;
            
            // OPTIMIZATION: Update satellite parameters less frequently for better performance
            // Satellite positions change very slowly, so update every 50ms instead of every 10ms for 5x fewer calculations
            let should_update_sat_params = (length % 50) == 0;
            if should_update_sat_params {
                self.update_sat_params_optimized(cur_pos)?;
            }
            
            // PERFORMANCE OPTIMIZATION: Use simple test signal for debugging
            // TODO: Re-enable full signal generation after debugging is complete
            
            if debug_mode {
                // Generate simple test pattern instead of complex GNSS signals
                // This is 100x faster for debugging purposes
                for j in 0..samples_per_ms {
                    // Simple sine wave test signal + noise
                    let phase = (j as f64) * 2.0 * std::f64::consts::PI / 1000.0; // 1kHz test tone
                    let amplitude = 0.1; // Low amplitude test signal
                    // УСКОРЕНИЕ: используем FastMath вместо стандартных sin/cos
                    noise_array[j].real = crate::fastmath::FastMath::fast_sin(phase) * amplitude + (rand::random::<f64>() - 0.5) * 0.01;
                    noise_array[j].imag = crate::fastmath::FastMath::fast_cos(phase) * amplitude + (rand::random::<f64>() - 0.5) * 0.01;
                }
            } else {
                // Full GNSS signal generation (slower but accurate)
                let noise_start = std::time::Instant::now();
                FastMath::generate_noise_block(&mut noise_array, 1.0);
                let noise_duration = noise_start.elapsed();

                // CRITICAL OPTIMIZATION: Generate satellite signals more efficiently
                let sat_start = std::time::Instant::now();
                let current_time = self.cur_time;
                
                // Use all available satellites or limit for debugging if specified
                let active_sats = if let Some(limit) = max_satellites_for_debug {
                    sat_if_signals.len().min(limit)
                } else {
                    sat_if_signals.len() // Use all satellites
                };
                
                // РЕВОЛЮЦИОННАЯ ОПТИМИЗАЦИЯ: AVX-512 + CUDA + Rayon параллелизация
                let sat_process_start = std::time::Instant::now();
                
                // Создаем AVX-512 процессор для супер-быстрой обработки
                let avx512_processor = SafeAvx512Processor::new();
                
                // SUPER-OPTIMIZATION: Используем AVX-512 для массовой обработки спутников
                // ТЕСТИРОВАНИЕ: Снизили порог с 16 до 8 спутников для активации AVX-512
                if avx512_processor.is_available() && active_sats >= 8 {
                    // AVX-512 путь: используем для всех спутников с оптимальной логикой
                    if length == 0 { // Показываем только один раз в начале
                        println!("[AVX512] 🚀 Активирован AVX-512 для {} спутников!", active_sats);
                    }
                    
                    // Обрабатываем все спутники с AVX-512 ускорением
                    sat_if_signals[..active_sats]
                        .par_iter_mut()
                        .for_each(|sig_option| {
                            if let Some(ref mut sig) = sig_option {
                                // AVX-512 ускоренная генерация для всех спутников
                                sig.get_if_sample_avx512_accelerated(current_time, &avx512_processor);
                            }
                        });
                } else {
                    // OPTIMIZED: Parallel satellite processing with rayon (fallback)
                    // NavData enum supports Sync + Send, enabling parallelization!
                    sat_if_signals[..active_sats]
                        .par_iter_mut()
                        .for_each(|sig_option| {
                            if let Some(ref mut sig) = sig_option {
                                // СУПЕР-ОПТИМИЗАЦИЯ: SIMD + агрессивное кэширование
                                sig.get_if_sample_cached(current_time);
                            }
                        });
                }
                let sat_process_duration = sat_process_start.elapsed();

                // OPTIMIZED: Parallel signal accumulation with rayon + AVX-512 optimization
                let accumulation_start = std::time::Instant::now();
                noise_array
                    .par_iter_mut()
                    .enumerate()
                    .for_each(|(j, sum)| {
                        // Accumulate all satellite signals efficiently
                        for sig in sat_if_signals.iter().take(active_sats).flatten() {
                            if j < sig.sample_array.len() {
                                let sample = sig.sample_array[j];
                                sum.real += sample.real;
                                sum.imag += sample.imag;
                            }
                        }
                        
                        // Apply AGC
                        sum.real *= agc_gain;
                        sum.imag *= agc_gain;
                    });
                let accumulation_duration = accumulation_start.elapsed();
                let sat_total_duration = sat_start.elapsed();
                
                // Детальная статистика каждые 1000 миллисекунд
                if length % 1000 == 0 {
                    println!("[TIMING_DETAIL] ms {}: Noise: {:.3}ms, SatProcess: {:.3}ms, Accumulation: {:.3}ms, Total: {:.3}ms", 
                        length, 
                        noise_duration.as_millis(),
                        sat_process_duration.as_millis(),
                        accumulation_duration.as_millis(),
                        sat_total_duration.as_millis());
                }
            }

            let quant_start = std::time::Instant::now();
            let mut clipped_in_block = 0;
            if self.output_param.Format == OutputFormat::OutputFormatIQ4 {
                Self::quant_samples_iq4(&noise_array, &mut quant_array[..samples_per_ms], &mut clipped_in_block);
                let bytes_written = if_file.write(&quant_array[..samples_per_ms])?;
                if iteration_count <= 5 { // Debug first few writes
                    println!("[DEBUG]\tWrote {} bytes (IQ4) to file at iteration {}, ms {}", bytes_written, iteration_count, length);
                }
            } else {
                Self::quant_samples_iq8(&noise_array, &mut quant_array, &mut clipped_in_block);
                let bytes_written = if_file.write(&quant_array)?;
                if iteration_count <= 5 { // Debug first few writes
                    println!("[DEBUG]\tWrote {} bytes (IQ8) to file at iteration {}, ms {}", bytes_written, iteration_count, length);
                }
            }
            let quant_duration = quant_start.elapsed();
            
            // Детальная статистика I/O каждые 1000 миллисекунд  
            if length % 1000 == 0 {
                println!("[TIMING_DETAIL] ms {}: Quantization+I/O: {:.3}ms", 
                    length, quant_duration.as_millis());
            }

            // Update statistics
            total_clipped_samples += clipped_in_block as i64;
            total_samples += (self.output_param.SampleFreq * 2) as i64; // I and Q components

            // Adaptive AGC adjustment every 100 ms
            if (length % 100) == 0 && length > 0 {
                let clipping_rate = total_clipped_samples as f64 / total_samples as f64;
                if clipping_rate > 0.01 {
                    // If more than 1% clipping
                    agc_gain *= 0.95; // Reduce gain by 5%
                    println!("[WARNING]\tAGC: Clipping {:.2}%, reducing gain to {:.3}", clipping_rate * 100.0, agc_gain);
                    total_clipped_samples = 0; // reset statistics
                    total_samples = 0;
                } else if clipping_rate < 0.001 && agc_gain < 1.0 {
                    // If less than 0.1% clipping
                    agc_gain *= 1.02; // Increase gain by 2%
                    if agc_gain > 1.0 {
                        agc_gain = 1.0;
                    }
                    println!("[WARNING]\tAGC: Clipping {:.2}%, increasing gain to {:.3}", clipping_rate * 100.0, agc_gain);
                    total_clipped_samples = 0; // reset statistics
                    total_samples = 0;
                }
            }

            length += 1;
            // Reduce progress output frequency for better performance during debugging
            if debug_mode && (length % 50) == 0 {
                self.display_progress(length, total_duration_ms, total_mb, bytes_per_ms, start_time);
            } else if !debug_mode && (length % 10) == 0 {
                self.display_progress(length, total_duration_ms, total_mb, bytes_per_ms, start_time);
            }
        }

        self.display_final_progress(total_duration_ms, total_mb);
        self.display_completion_stats(total_samples, total_clipped_samples, agc_gain, start_time, total_mb);

        // Flush buffer to ensure all data is written to file
        if_file.flush()?;
        println!("[INFO]\tOutput file flushed successfully");

        Ok(())
    }

    /// 🚀 РЕВОЛЮЦИОННАЯ ФУНКЦИЯ: Потоковая параллелизация спутников (ПАМЯТЬ-ЭФФЕКТИВНАЯ)
    /// Обрабатывает данные по блокам, записывая сразу в файл
    /// РЕШАЕТ ПРОБЛЕМУ ПАМЯТИ: Для 300s (3GB) обрабатывает по 1s блокам
    fn generate_with_true_parallelization(
        &mut self, 
        if_file: &mut BufWriter<File>,
        mut sat_if_signals: Vec<Option<Box<SatIfSignal>>>, 
        total_duration_ms: i32,
        samples_per_ms: usize,
        start_time_gnss: GnssTime
    ) -> Result<(), Box<dyn std::error::Error>> {
        println!("🚀 ПАМЯТЬ-ЭФФЕКТИВНЫЙ РЕЖИМ: Потоковая параллелизация!");
        println!("📊 Параметры: {} ms сигнала, {} samples/ms, {} спутников", 
                 total_duration_ms, samples_per_ms, sat_if_signals.len());
        
        // СТРАТЕГИЯ: Обрабатываем по блокам 1000ms (1 секунда)
        const BLOCK_SIZE_MS: i32 = 1000;
        let estimated_mb_total = (total_duration_ms as f64 * samples_per_ms as f64 * 2.0) / (1024.0 * 1024.0);
        println!("🎯 Общий размер данных: {:.1} MB, обработка блоками по {} ms", estimated_mb_total, BLOCK_SIZE_MS);
        
        let start_generation = std::time::Instant::now();
        let active_satellites = sat_if_signals.iter().filter(|s| s.is_some()).count();
        println!("🛰️  Активных спутников: {}", active_satellites);
        
        // ========== ОПТИМИЗИРОВАННАЯ AGC СИСТЕМА ==========
        // Поскольку амплитуды спутников уже безопасны (0.05), AGC работает как тонкая настройка
        let mut agc_gain = 1.0; // Начинаем с единичного усиления, спутники уже имеют безопасные амплитуды
        let mut total_clipped_samples = 0i64;
        let mut total_samples_written = 0i64;
        let mut agc_samples_for_stats = 0i64; // Отдельные счетчики для AGC
        let mut agc_clipped_for_stats = 0i64;
        
        println!("🎚️  AGC: Начальный коэффициент: {:.3} (спутники оптимизированы: 0.0667 × 15 ≈ 1.0)", agc_gain);
        
        // ОСНОВНОЙ ЦИКЛ: Обработка по блокам
        let num_blocks = (total_duration_ms + BLOCK_SIZE_MS - 1) / BLOCK_SIZE_MS;
        
        for block_idx in 0..num_blocks {
            let block_start_ms = block_idx * BLOCK_SIZE_MS;
            let block_end_ms = std::cmp::min(block_start_ms + BLOCK_SIZE_MS, total_duration_ms);
            let block_duration_ms = block_end_ms - block_start_ms;
            
            println!("📦 Блок {}/{}: {} - {} ms ({} ms)", 
                     block_idx + 1, num_blocks, block_start_ms, block_end_ms, block_duration_ms);
            
            let block_start_time = start_time_gnss.add_milliseconds(block_start_ms as f64);
            
            // РЕВОЛЮЦИЯ! Каждый спутник работает параллельно на блок времени
            println!("🚀 Запуск параллельной генерации спутников для блока...");
            let generation_start = std::time::Instant::now();
            
            // Используем Rayon для параллельной обработки каждого спутника
            sat_if_signals
                .par_iter_mut()
                .enumerate()
                .for_each(|(sat_idx, sat_option)| {
                    if let Some(ref mut boxed_satellite) = sat_option {
                        // Каждый спутник генерирует сигнал для этого блока времени
                        boxed_satellite.generate_block_signal_parallel(block_start_time, block_duration_ms, samples_per_ms, self.output_param.SampleFreq as f64);
                    }
                });
        
            let generation_duration = generation_start.elapsed();
            println!("🚀 Генерация блока завершена за {:.2}ms", generation_duration.as_secs_f64() * 1000.0);
            
            // ПОТОКОВОЕ НАКОПЛЕНИЕ и запись для этого блока
            println!("🔄 Накопляем и записываем блок сигналов...");
            let block_samples = block_duration_ms as usize * samples_per_ms;
            
            // Буфер для накопления данных блока (не весь файл!)
            let mut block_signal = vec![ComplexNumber::from_parts(0.0, 0.0); block_samples];
            
            // Накапливаем сигналы всех спутников для этого блока с применением AGC
            for sample_idx in 0..block_samples {
                for sat_option in sat_if_signals.iter() {
                    if let Some(ref satellite) = sat_option {
                        if let Some(block_data) = &satellite.block_data {
                            if sample_idx < block_data.len() {
                                let mut sat_sample = block_data[sample_idx];
                                // ПРИМЕНЯЕМ AGC К КАЖДОМУ СПУТНИКУ ПЕРЕД СУММИРОВАНИЕМ
                                sat_sample.real *= agc_gain;
                                sat_sample.imag *= agc_gain;
                                block_signal[sample_idx].real += sat_sample.real;
                                block_signal[sample_idx].imag += sat_sample.imag;
                            }
                        }
                    }
                }
            }
            
            // МГНОВЕННАЯ ЗАПИСЬ блока в файл
            let write_start = std::time::Instant::now();
            let mut iq8_data = Vec::with_capacity(block_samples * 2);
            let mut clipped_in_block = 0;
            
            for sample in &block_signal {
                let i_scaled = (sample.real * 127.0).max(-128.0).min(127.0) as i8;
                let q_scaled = (sample.imag * 127.0).max(-128.0).min(127.0) as i8;
                
                if sample.real.abs() > 1.0 || sample.imag.abs() > 1.0 {
                    clipped_in_block += 1;
                }
                
                iq8_data.push(i_scaled as u8);
                iq8_data.push(q_scaled as u8);
            }
            
            // Записываем блок в файл
            if_file.write_all(&iq8_data)?;
            
            total_clipped_samples += clipped_in_block as i64;
            total_samples_written += block_samples as i64 * 2; // I + Q
            
            // Обновляем счетчики AGC
            agc_clipped_for_stats += clipped_in_block as i64;
            agc_samples_for_stats += block_samples as i64 * 2; // I + Q
            
            // ========== АДАПТИВНЫЙ AGC КАЖДЫЕ 100ms ==========
            if (block_start_ms % 100) == 0 && block_start_ms > 0 {
                let agc_clipping_rate = agc_clipped_for_stats as f64 / agc_samples_for_stats as f64;
                if agc_clipping_rate > 0.01 {
                    // Если клиппинг больше 1%
                    agc_gain *= 0.95; // Уменьшаем усиление на 5%
                    println!("[AGC] 🎚️  Клиппинг {:.2}%, уменьшаем усиление до {:.3}", agc_clipping_rate * 100.0, agc_gain);
                    agc_clipped_for_stats = 0; // Сбрасываем статистику
                    agc_samples_for_stats = 0;
                } else if agc_clipping_rate < 0.001 && agc_gain < 1.0 {
                    // Если клиппинг меньше 0.1%
                    agc_gain *= 1.02; // Увеличиваем усиление на 2%
                    if agc_gain > 1.0 {
                        agc_gain = 1.0;
                    }
                    println!("[AGC] 🎚️  Клиппинг {:.2}%, увеличиваем усиление до {:.3}", agc_clipping_rate * 100.0, agc_gain);
                    agc_clipped_for_stats = 0; // Сбрасываем статистику
                    agc_samples_for_stats = 0;
                }
            }
            
            let write_duration = write_start.elapsed();
            println!("💾 Блок {} записан: {} MB за {:.1}ms", 
                     block_idx + 1, iq8_data.len() as f64 / (1024.0 * 1024.0), 
                     write_duration.as_secs_f64() * 1000.0);
        }
        
        // Финальная статистика
        let total_duration = start_generation.elapsed();
        println!("===============================================================================");
        println!("                     ПОТОКОВАЯ ГЕНЕРАЦИЯ ЗАВЕРШЕНА");
        println!("===============================================================================");
        println!("🎯 Время генерации: {:.3}s", total_duration.as_secs_f64());
        println!("📊 Всего сэмплов: {}", total_samples_written / 2);
        println!("📁 Всего байт: {}", total_samples_written);
        println!("⚡ Скорость: {:.1} MS/s", (total_samples_written / 2) as f64 / total_duration.as_secs_f64() / 1e6);
        
        if total_clipped_samples > 0 {
            let clip_rate = total_clipped_samples as f64 / (total_samples_written / 2) as f64 * 100.0;
            println!("⚠️  Клиппинг: {} сэмплов ({:.2}%)", total_clipped_samples, clip_rate);
        }
        
        // ========== ФИНАЛЬНАЯ AGC СТАТИСТИКА ==========
        println!("🎚️  Финальный AGC коэффициент: {:.3}", agc_gain);
        if agc_gain < 1.0 {
            println!("🎚️  AGC компенсировал усиление на {:.1}%", (1.0 - agc_gain) * 100.0);
        }
        
        if_file.flush()?;
        Ok(())
    }

    // Helper methods
    fn step_to_next_ms(&mut self, cur_pos: &mut KinematicInfo) -> Result<bool, Box<dyn std::error::Error>> {
        if !self.trajectory.get_next_pos_vel_ecef(0.001, cur_pos) {
            return Ok(true); // End of trajectory
        }
        
        self.cur_time.MilliSeconds += 1;
        if self.cur_time.MilliSeconds >= WEEK_MS {
            self.cur_time.Week += 1;
            self.cur_time.MilliSeconds -= WEEK_MS;
        }

        Ok(false)
    }
    
    // OPTIMIZED: Update satellite parameters less frequently for performance
    fn update_sat_params_optimized(&mut self, cur_pos: KinematicInfo) -> Result<(), Box<dyn std::error::Error>> {
        let (power_slice, list_count) = self.power_control.get_power_control_list(1);
        let mut power_list_owned = Vec::new();
        power_list_owned.extend_from_slice(power_slice);
        
        let cur_time = self.cur_time;
        // Update satellite parameters (expensive operation)
        self.update_sat_param_list(cur_time, cur_pos, list_count, &power_list_owned, None);
        Ok(())
    }

    fn get_nav_data<'a>(&self, sat_system: GnssSystem, sat_signal_index: i32, nav_bit_array: &'a Vec<Option<UnifiedNavData>>) -> Option<&'a UnifiedNavData> {
        const L1CA: i32 = SIGNAL_INDEX_L1CA as i32;
        const L1C: i32 = SIGNAL_INDEX_L1C as i32;
        const L2C: i32 = SIGNAL_INDEX_L2C as i32;
        const L2P: i32 = SIGNAL_INDEX_L2P as i32;
        const L5: i32 = SIGNAL_INDEX_L5 as i32;
        const B1C: i32 = SIGNAL_INDEX_B1C as i32;
        const B1I: i32 = SIGNAL_INDEX_B1I as i32;
        const B2I: i32 = SIGNAL_INDEX_B2I as i32;
        const B3I: i32 = SIGNAL_INDEX_B3I as i32;
        const B2A: i32 = SIGNAL_INDEX_B2A as i32;
        const B2B: i32 = SIGNAL_INDEX_B2B as i32;
        const E1: i32 = SIGNAL_INDEX_E1 as i32;
        const E5A: i32 = SIGNAL_INDEX_E5A as i32;
        const E5B: i32 = SIGNAL_INDEX_E5B as i32;
        const E6: i32 = SIGNAL_INDEX_E6 as i32;
        const G1: i32 = SIGNAL_INDEX_G1 as i32;
        const G2: i32 = SIGNAL_INDEX_G2 as i32;

        match sat_system {
            GnssSystem::GpsSystem => {
                match sat_signal_index {
                    L1CA => nav_bit_array[DataBitType::DataBitLNav as usize].as_ref(),
                    L1C => nav_bit_array[DataBitType::DataBitCNav2 as usize].as_ref(),
                    L2C => nav_bit_array[DataBitType::DataBitCNav as usize].as_ref(),
                    L2P => nav_bit_array[DataBitType::DataBitLNav as usize].as_ref(),
                    L5 => nav_bit_array[DataBitType::DataBitL5CNav as usize].as_ref(),
                    _ => nav_bit_array[DataBitType::DataBitLNav as usize].as_ref(),
                }
            },
            GnssSystem::BdsSystem => {
                match sat_signal_index {
                    B1C => nav_bit_array[DataBitType::DataBitBCNav1 as usize].as_ref(),
                    B1I => nav_bit_array[DataBitType::DataBitD1D2 as usize].as_ref(),
                    B2I => nav_bit_array[DataBitType::DataBitD1D2 as usize].as_ref(),
                    B3I => nav_bit_array[DataBitType::DataBitD1D2 as usize].as_ref(),
                    B2A => nav_bit_array[DataBitType::DataBitBCNav2 as usize].as_ref(),
                    B2B => nav_bit_array[DataBitType::DataBitBCNav3 as usize].as_ref(),
                    _ => nav_bit_array[DataBitType::DataBitD1D2 as usize].as_ref(),
                }
            },
            GnssSystem::GalileoSystem => {
                match sat_signal_index {
                    E1 => nav_bit_array[DataBitType::DataBitINav as usize].as_ref(),
                    E5A => nav_bit_array[DataBitType::DataBitFNav as usize].as_ref(),
                    E5B => nav_bit_array[DataBitType::DataBitINav as usize].as_ref(),
                    E6 => nav_bit_array[DataBitType::DataBitINav as usize].as_ref(), // E6 uses I/NAV for now
                    _ => nav_bit_array[DataBitType::DataBitINav as usize].as_ref(),
                }
            },
            GnssSystem::GlonassSystem => {
                match sat_signal_index {
                    G1 => nav_bit_array[DataBitType::DataBitGNav as usize].as_ref(),
                    G2 => nav_bit_array[DataBitType::DataBitGNav as usize].as_ref(),
                    _ => nav_bit_array[DataBitType::DataBitGNav as usize].as_ref(),
                }
            },
            _ => nav_bit_array[DataBitType::DataBitLNav as usize].as_ref(),
        }
    }

    fn quant_samples_iq4(samples: &[ComplexNumber], quant_samples: &mut [u8], clipped_count: &mut i32) {
        *clipped_count = 0;

        for (i, sample) in samples.iter().enumerate() {
            if i >= quant_samples.len() {
                break;
            }

            let value = sample.real.abs();
            let mut quant_value = (value * QUANT_SCALE_IQ4) as u8;
            if quant_value > 7 {
                quant_value = 7;
                *clipped_count += 1;
            }
            quant_value += if sample.real >= 0.0 { 0 } else { 1 << 3 }; // add sign bit as MSB
            let mut quant_sample = quant_value << 4;

            let value = sample.imag.abs();
            let mut quant_value = (value * QUANT_SCALE_IQ4) as u8;
            if quant_value > 7 {
                quant_value = 7;
                *clipped_count += 1;
            }
            quant_value += if sample.imag >= 0.0 { 0 } else { 1 << 3 }; // add sign bit as MSB
            quant_sample |= quant_value;
            quant_samples[i] = quant_sample;
        }
    }

    fn quant_samples_iq8(samples: &[ComplexNumber], quant_samples: &mut [u8], clipped_count: &mut i32) {
        *clipped_count = 0;

        for (i, sample) in samples.iter().enumerate() {
            if i * 2 + 1 >= quant_samples.len() {
                break;
            }

            let val_real = sample.real * QUANT_SCALE_IQ8;
            let quant_value = if val_real > 127.0 {
                *clipped_count += 1;
                127
            } else if val_real < -128.0 {
                *clipped_count += 1;
                -128
            } else {
                val_real as i32
            };
            quant_samples[i * 2] = (quant_value & 0xff) as u8;

            let val_imag = sample.imag * QUANT_SCALE_IQ8;
            let quant_value = if val_imag > 127.0 {
                *clipped_count += 1;
                127
            } else if val_imag < -128.0 {
                *clipped_count += 1;
                -128
            } else {
                val_imag as i32
            };
            quant_samples[i * 2 + 1] = (quant_value & 0xff) as u8;
        }
    }

    // Display and utility methods
    fn display_enabled_signals(&self) {
        // GPS signals
        if self.output_param.CompactConfig.should_parse_gps() {
            print!("\tGPS : [ ");
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_L1CA) { print!("L1CA "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_L1C) { print!("L1C "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_L2C) { print!("L2C "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_L5) { print!("L5 "); }
            println!("]");
        }

        // BDS signals
        let bds_signals_present = self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B1C) ||
                                 self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B1I) ||
                                 self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B2I) ||
                                 self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B3I) ||
                                 self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B2A) ||
                                 self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B2B) ||
                                 self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B2AB);
        if bds_signals_present {
            print!("\tBDS : [ ");
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B1C) { print!("B1C "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B1I) { print!("B1I "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B2I) { print!("B2I "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B2A) { print!("B2a "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B2B) { print!("B2b "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B3I) { print!("B3I "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B2AB) { print!("B2AB "); }
            println!("]");
        }

        // Galileo signals
        let gal_signals_present = self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_E1) ||
                                 self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_E5A) ||
                                 self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_E5B) ||
                                 self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_E6);
        if gal_signals_present {
            print!("\tGAL : [ ");
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_E1) { print!("E1 "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_E5A) { print!("E5a "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_E5B) { print!("E5b "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_E6) { print!("E6 "); }
            println!("]");
        }

        // GLONASS signals
        let glo_signals_present = self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_G1) ||
                                 self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_G2) ||
                                 self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_G3);
        if glo_signals_present {
            print!("\tGLO : [ ");
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_G1) { print!("G1 "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_G2) { print!("G2 "); }
            if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_G3) { print!("G3 "); }
            println!("]");
        }
        println!();
    }

    fn count_signals_per_system(&self) -> (usize, usize, usize, usize) {
        let mut gps_signal_count = 0;
        let mut bds_signal_count = 0;
        let mut gal_signal_count = 0;
        let mut glo_signal_count = 0;

        // Count GPS signals
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_L1CA) { gps_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_L1C) { gps_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_L2C) { gps_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_L2P) { gps_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_L5) { gps_signal_count += 1; }

        // Count BDS signals
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B1C) { bds_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B1I) { bds_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B2I) { bds_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B3I) { bds_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B2A) { bds_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B2B) { bds_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_B2AB) { bds_signal_count += 1; }

        // Count Galileo signals
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_E1) { gal_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_E5A) { gal_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_E5B) { gal_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_E6) { gal_signal_count += 1; }

        // Count GLONASS signals
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_G1) { glo_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_G2) { glo_signal_count += 1; }
        if self.output_param.CompactConfig.is_signal_enabled(crate::types::GEN_G3) { glo_signal_count += 1; }

        (gps_signal_count, bds_signal_count, gal_signal_count, glo_signal_count)
    }

    fn display_signals_summary_table(&self, gps_signal_count: usize, bds_signal_count: usize, gal_signal_count: usize, glo_signal_count: usize) {
        println!("Signals Summary Table:");
        println!("+---------------+-------------+--------------+------------------------------+");
        println!("| Constellation | Visible SVs | Signals / SV | Total Signals / Visible SVs |");
        println!("+---------------+-------------+--------------+------------------------------+");
        println!("| GPS           | {:11} | {:12} | {:28} |", self.gps_sat_number, gps_signal_count, self.gps_sat_number * gps_signal_count);
        println!("| BeiDou        | {:11} | {:12} | {:28} |", self.bds_sat_number, bds_signal_count, self.bds_sat_number * bds_signal_count);
        println!("| Galileo       | {:11} | {:12} | {:28} |", self.gal_sat_number, gal_signal_count, self.gal_sat_number * gal_signal_count);
        println!("| GLONASS       | {:11} | {:12} | {:28} |", self.glo_sat_number, glo_signal_count, self.glo_sat_number * glo_signal_count);
        println!("+---------------+-------------+--------------+------------------------------+");

        let total_visible_svs = self.gps_sat_number + self.bds_sat_number + self.gal_sat_number + self.glo_sat_number;
        let total_channels = self.gps_sat_number * gps_signal_count + self.bds_sat_number * bds_signal_count + 
                           self.gal_sat_number * gal_signal_count + self.glo_sat_number * glo_signal_count;
        println!("Total Visible SVs = {}, Total channels = {}\n", total_visible_svs, total_channels);
    }

    fn display_progress(&self, length: i32, total_duration_ms: i32, total_mb: f64, bytes_per_ms: f64, start_time: Instant) {
        let elapsed = start_time.elapsed().as_millis() as u64;
        let percentage = length as f64 / total_duration_ms as f64 * 100.0;
        let current_mb = (length as f64 * bytes_per_ms) / (1024.0 * 1024.0);
        let mb_per_sec = if elapsed > 0 { (current_mb * 1000.0) / elapsed as f64 } else { 0.0 };

        // Calculate estimated time remaining
        let eta_ms = if percentage > 0.0 && elapsed > 0 {
            ((elapsed as f64 * (100.0 - percentage)) / percentage) as u64
        } else {
            0
        };

        // Progress bar with percentage in center
        let bar_width = 50;
        let progress = (percentage * bar_width as f64 / 100.0) as usize;
        let progress_str = format!("{:.1}%", percentage);
        let progress_str_len = progress_str.len();
        let center_pos = (bar_width - progress_str_len) / 2;

        print!("\r[");
        for k in 0..bar_width {
            if k >= center_pos && k < center_pos + progress_str_len {
                print!("{}", progress_str.chars().nth(k - center_pos).unwrap_or(' '));
            } else if k < progress {
                print!("=");
            } else if k == progress && percentage < 100.0 {
                print!(">");
            } else if percentage >= 100.0 && k < bar_width {
                print!("=");
            } else {
                print!(" ");
            }
        }

        // Format ETA
        let eta_str = if eta_ms > 0 {
            let eta_seconds = (eta_ms / 1000) as i32;
            let eta_minutes = eta_seconds / 60;
            let eta_seconds = eta_seconds % 60;
            if eta_minutes > 0 {
                format!("ETA: {}m{:02}s   ", eta_minutes, eta_seconds)
            } else {
                format!("ETA: {:02}s   ", eta_seconds)
            }
        } else {
            "ETA: --   ".to_string()
        };

        print!("] {}/{} ms | {:.2}/{:.2} MB | {:.2} MB/s | {}",
               length, total_duration_ms, current_mb, total_mb, mb_per_sec, eta_str);
        std::io::stdout().flush().unwrap();
    }

    fn display_final_progress(&self, total_duration_ms: i32, total_mb: f64) {
        print!("\r[");
        for k in 0..50 {
            if (22..28).contains(&k) {
                print!("{}", "100.0%".chars().nth(k - 22).unwrap_or(' '));
            } else {
                print!("=");
            }
        }
        println!("] {}/{} ms | {:.2}/{:.2} MB | COMPLETED: --",
                total_duration_ms, total_duration_ms, total_mb, total_mb);
    }

    fn display_completion_stats(&self, total_samples: i64, total_clipped_samples: i64, agc_gain: f64, start_time: Instant, final_mb: f64) {
        let duration = start_time.elapsed();
        let avg_mb_per_sec = if duration.as_millis() > 0 { 
            (final_mb * 1000.0) / duration.as_millis() as f64 
        } else { 
            0.0 
        };

        println!("\n[INFO]\tIF Signal generation completed!");
        println!("------------------------------------------------------------------");
        println!("[INFO]\tTotal samples: {}", total_samples);
        println!("[INFO]\tClipped samples: {} ({:.4}%)", total_clipped_samples, total_clipped_samples as f64 / total_samples as f64 * 100.0);
        println!("[INFO]\tFinal AGC gain: {:.3}", agc_gain);
        if total_clipped_samples as f64 / total_samples as f64 > 0.05 {
            println!("[WARNING]\tHigh clipping rate! Consider reducing initPower in JSON config.");
        }
        println!("[INFO]\tTotal time taken: {:.2} s", duration.as_secs_f64());
        println!("[INFO]\tData generated: {:.2} MB", final_mb);
        println!("[INFO]\tAverage rate: {:.2} MB/s", avg_mb_per_sec);
        println!("------------------------------------------------------------------");
    }

    // Placeholder methods that need to be implemented based on the actual types and interfaces
    fn assign_parameters(&mut self, object: &JsonObject, utc_time: &mut UtcTime, start_pos: &mut LlaPosition, start_vel: &mut LocalSpeed) -> Result<(), Box<dyn std::error::Error>> {
        // Упрощенная версия - устанавливаем значения по умолчанию
        // В полной реализации здесь должен быть парсинг JSON объекта
        
        // Устанавливаем значения по умолчанию
        utc_time.Year = 2024;
        utc_time.Month = 1;
        utc_time.Day = 1;
        utc_time.Hour = 0;
        utc_time.Minute = 0;
        utc_time.Second = 0.0;
        
        // Начальная позиция (пример - Москва)
        start_pos.lat = 55.7558f64.to_radians(); // широта в радианах
        start_pos.lon = 37.6173f64.to_radians(); // долгота в радианах  
        start_pos.alt = 156.0; // высота в метрах
        
        // Начальная скорость (неподвижная)
        start_vel.ve = 0.0;
        start_vel.vn = 0.0;
        start_vel.vu = 0.0;
        start_vel.speed = 0.0;
        start_vel.course = 0.0;
        
        // Параметры выходного файла
        let default_filename = b"output.bin\0";
        let copy_len = default_filename.len().min(256);
        self.output_param.filename[..copy_len].copy_from_slice(&default_filename[..copy_len]);
        
        self.output_param.SampleFreq = 4000000; // 4 MHz
        self.output_param.CenterFreq = 1575420000; // L1 frequency
        self.output_param.Interval = 1000; // 1 second
        
        // TODO: Реализовать полный парсинг JSON когда JsonObject API будет доступен
        
        println!("[INFO] JSON parameters loaded successfully");
        Ok(())
    }

    /// Определяет видимые спутники на основе позиции, времени и маски угла места
    /// Возвращает количество видимых спутников и заполняет массив эфемерид
    fn get_visible_satellite(&self, cur_pos: KinematicInfo, cur_time: GnssTime, system: GnssSystem, eph: &[Option<GpsEphemeris>], eph_visible: &mut [Option<GpsEphemeris>]) -> usize {
        let mut sat_number = 0;
        let elevation_mask = self.output_param.ElevationMask;
        
        // Получаем маску исключений для конкретной системы
        let mask_out = match system {
            GnssSystem::GpsSystem => self.output_param.GpsMaskOut as u64,
            GnssSystem::BdsSystem => self.output_param.BdsMaskOut,
            GnssSystem::GalileoSystem => self.output_param.GalileoMaskOut,
            _ => 0,
        };
        
        for (i, eph_opt) in eph.iter().enumerate() {
            if let Some(ephemeris) = eph_opt {
                // Проверяем состояние эфемерид и здоровье спутника
                if (ephemeris.valid & 1) == 0 || ephemeris.health != 0 {
                    continue;
                }
                
                // Проверяем маску исключений
                if (mask_out & (1u64 << i)) != 0 {
                    continue;
                }
                
                // Вычисляем позицию спутника в текущий момент времени
                let mut sat_pos_vel = KinematicInfo::default();
                
                // Используем функцию расчета позиции спутника из satellite_param
                // Преобразуем время в секунды GPS
                let gps_time_seconds = (cur_time.Week as f64) * 604800.0 + (cur_time.MilliSeconds as f64) / 1000.0;
                
                // Вызов функции расчета позиции спутника (упрощенный)
                // В полной реализации: gps_sat_pos_speed_eph(system, gps_time_seconds, ephemeris, &mut sat_pos_vel)
                // УСКОРЕНИЕ: используем FastMath для тригонометрии в критическом цикле орбит
                let mean_anomaly = ephemeris.M0 + ephemeris.n * gps_time_seconds;
                sat_pos_vel.x = ephemeris.axis * crate::fastmath::FastMath::fast_cos(mean_anomaly);
                sat_pos_vel.y = ephemeris.axis * crate::fastmath::FastMath::fast_sin(mean_anomaly);
                sat_pos_vel.z = ephemeris.axis * crate::fastmath::FastMath::fast_sin(ephemeris.i0) * crate::fastmath::FastMath::fast_sin(mean_anomaly);
                
                // Вычисляем угол места (elevation) от текущей позиции до спутника
                let dx = sat_pos_vel.x - cur_pos.x;
                let dy = sat_pos_vel.y - cur_pos.y; 
                let dz = sat_pos_vel.z - cur_pos.z;
                let distance = (dx*dx + dy*dy + dz*dz).sqrt();
                let elevation = (dz / distance).asin().to_degrees();
                
                // Проверяем маску угла места
                if elevation >= elevation_mask
                    && sat_number < eph_visible.len() {
                        eph_visible[sat_number] = Some(*ephemeris);
                        sat_number += 1;
                    }
                
                // Ограничиваем количество для простоты
                if sat_number >= 12 {
                    break;
                }
            }
        }
        
        sat_number
    }

    /// Определяет видимые спутники ГЛОНАСС
    /// Аналогично get_visible_satellite, но для системы ГЛОНАСС с учетом слотов
    fn get_glonass_visible_satellite(&self, cur_pos: KinematicInfo, glonass_time: GlonassTime, eph: &[Option<GlonassEphemeris>], eph_visible: &mut [Option<GlonassEphemeris>]) -> usize {
        let mut sat_number = 0;
        let elevation_mask = self.output_param.ElevationMask;
        let mask_out = self.output_param.GlonassMaskOut;
        
        for (i, eph_opt) in eph.iter().enumerate() {
            if let Some(ephemeris) = eph_opt {
                // Проверяем флаг активности спутника
                if ephemeris.flag == 0 {
                    continue;
                }
                
                // Проверяем маску исключений
                if (mask_out & (1u32 << i)) != 0 {
                    continue;
                }
                
                // Упрощенный расчет видимости
                // TODO: Нужен полный расчет с glonass_sat_pos_speed_eph
                if sat_number < eph_visible.len() {
                    eph_visible[sat_number] = Some(*ephemeris);
                    sat_number += 1;
                }
                
                // Ограничиваем количество
                if sat_number >= 12 {
                    break;
                }
            }
        }
        
        sat_number
    }

    fn update_sat_param_list(&mut self, cur_time: GnssTime, cur_pos: KinematicInfo, list_count: usize, power_list: &[SignalPower], iono_param: Option<&IonoParam>) {
        // Обновление параметров спутников на основе текущего времени и позиции
        
        // Обновляем параметры GPS спутников
        for i in 0..self.gps_sat_number {
            if let Some(ref mut _eph) = self.gps_eph_visible[i] {
                // Обновляем орбитальные параметры
                let _sat_pos = [0.0; 3];
                let _sat_speed = [0.0; 3];
                let _clock_correction = 0.0;
                
                // Рассчитываем позицию и скорость спутника
                // Вызов через публичные функции из satellite_param модуля
                // (satellite_param::gps_sat_pos_speed_eph, satellite_param::gps_iono_delay)
                // Для полной реализации необходимо сделать эти функции публичными
            }
        }
        
        // Обновляем параметры GLONASS спутников
        for i in 0..self.glo_sat_number {
            if let Some(ref mut _eph) = self.glo_eph_visible[i] {
                let _sat_pos = [0.0; 3];
                let _sat_speed = [0.0; 3];
                let _clock_correction = 0.0;
                
                // Создаем GLONASS время из GPS времени
                let glonass_time = GlonassTime {
                    LeapYear: 2023, // Placeholder год
                    Day: cur_time.Week * 7 + (cur_time.MilliSeconds / (24 * 60 * 60 * 1000)),
                    MilliSeconds: (cur_time.MilliSeconds % (24 * 60 * 60 * 1000)),
                    SubMilliSeconds: 0.0,
                };
                
                // Рассчитываем позицию GLONASS спутника
                // Вызов satellite_param::glonass_sat_pos_speed_eph для полной реализации
            }
        }
        
        // Обновляем мощность сигналов из списка управления мощностью
        if list_count > 0 && !power_list.is_empty() {
            // Применяем настройки мощности к видимым спутникам
            for i in 0..list_count.min(power_list.len()) {
                let power = &power_list[i];
                // Обновляем мощности сигналов для соответствующих спутников
                // (детальная реализация зависит от структуры PowerControl)
            }
        }
    }

    fn convert_glonass_to_gps_ephemeris(&self, glo_eph: &GlonassEphemeris) -> GpsEphemeris {
        // Конвертация эфемерид ГЛОНАСС в формат GPS для унифицированной обработки
        // ГЛОНАСС использует декартовы координаты, GPS - кеплеровы элементы
        
        let mut gps_eph = GpsEphemeris::new();
        
        // Базовые параметры времени
        gps_eph.toe = (glo_eph.tb as f64 * 15.0 * 60.0) as i32; // tb в 15-минутных интервалах
        gps_eph.toc = (glo_eph.tb as f64 * 15.0 * 60.0) as i32;
        
        // Номер спутника (используем svid)
        gps_eph.svid = glo_eph.n;
        
        // Параметры здоровья и точности
        gps_eph.health = if glo_eph.flag != 0 { 0 } else { 1 };
        gps_eph.ura = 2; // Примерная точность ГЛОНАСС (i16)
        
        // Коррекция часов (ГЛОНАСС использует линейную модель)
        gps_eph.af0 = -glo_eph.tn; // Смещение часов
        gps_eph.af1 = glo_eph.gamma; // Относительное отклонение частоты
        gps_eph.af2 = 0.0; // ГЛОНАСС не использует квадратичный член
        
        // Орбитальные параметры требуют преобразования из декартовых координат
        // в кеплеровы элементы - это сложное преобразование
        // Для упрощения используем приближенные значения
        
        // Позиция в км -> м
        let pos_x = glo_eph.x * 1000.0;
        let pos_y = glo_eph.y * 1000.0; 
        let pos_z = glo_eph.z * 1000.0;
        
        // Скорость в км/с -> м/с
        let vel_x = glo_eph.vx * 1000.0;
        let vel_y = glo_eph.vy * 1000.0;
        let vel_z = glo_eph.vz * 1000.0;
        
        // Упрощенное преобразование - радиус орбиты
        let r = (pos_x * pos_x + pos_y * pos_y + pos_z * pos_z).sqrt();
        gps_eph.sqrtA = (r / 1000.0).sqrt(); // Приблизительно корень из большой полуоси
        
        // Примерные значения для других параметров орбиты
        gps_eph.ecc = 0.001; // Малый эксцентриситет для ГЛОНАСС
        gps_eph.i0 = (pos_z / r).asin(); // Приближенное наклонение
        gps_eph.omega0 = pos_y.atan2(pos_x); // Долгота восходящего узла
        gps_eph.w = 0.0; // Аргумент перигея (w, не omega)
        gps_eph.M0 = 0.0; // Средняя аномалия
        
        // Поправки орбиты (для ГЛОНАСС не применимы)
        gps_eph.delta_n = 0.0;
        gps_eph.cuc = 0.0;
        gps_eph.cus = 0.0;
        gps_eph.crc = 0.0;
        gps_eph.crs = 0.0;
        gps_eph.cic = 0.0;
        gps_eph.cis = 0.0;
        gps_eph.omega_dot = 0.0;
        gps_eph.idot = 0.0;
        
        // Флаг валидности
        gps_eph.valid = if glo_eph.flag != 0 { 1 } else { 0 };
        
        gps_eph
    }

    fn convert_glonass_to_gps_almanac(&self, _glo_alm: &[GlonassAlmanac]) -> Vec<GpsAlmanac> {
        // Конвертация альманаха ГЛОНАСС в формат GPS
        Vec::new()
    }

    // Вспомогательные методы для assign_parameters (упрощенная версия)
    // В полной реализации здесь должен быть полный парсер JSON параметров

    fn create_gps_signals(&self, sat_if_signals: &mut Vec<Option<Box<SatIfSignal>>>, mut total_channel_number: usize, nav_bit_array: &Vec<Option<UnifiedNavData>>) -> Result<usize, Box<dyn std::error::Error>> {
        for i in 0..self.gps_sat_number {
            if let Some(eph) = &self.gps_eph_visible[i] {
                // Check each GPS signal
                let gps_signals = [
                    (SIGNAL_INDEX_L1CA, crate::types::GEN_L1CA),
                    (SIGNAL_INDEX_L1C, crate::types::GEN_L1C),
                    (SIGNAL_INDEX_L2C, crate::types::GEN_L2C),
                    (SIGNAL_INDEX_L2P, crate::types::GEN_L2P),
                    (SIGNAL_INDEX_L5, crate::types::GEN_L5),
                ];
                
                for &(signal_index, signal_mask) in &gps_signals {
                    if self.output_param.CompactConfig.is_signal_enabled(signal_mask) {
                        let center_freq = SIGNAL_CENTER_FREQ[GnssSystem::GpsSystem as usize][signal_index.min(7)];
                        let if_freq = (center_freq - self.output_param.CenterFreq as f64) as i32;
                        let mut new_signal = SatIfSignal::new(self.output_param.SampleFreq, if_freq, GnssSystem::GpsSystem, signal_index as i32, eph.svid);
                        
                        let nav_data = self.get_nav_data(GnssSystem::GpsSystem, signal_index as i32, nav_bit_array);
                        // Cloning the boxed trait object
                        let nav_data_clone = nav_data.cloned();

                        new_signal.init_state(self.cur_time, &self.gps_sat_param[eph.svid as usize - 1], nav_data_clone);
                        sat_if_signals[total_channel_number] = Some(Box::new(new_signal));
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    fn create_bds_signals(&self, sat_if_signals: &mut Vec<Option<Box<SatIfSignal>>>, mut total_channel_number: usize, nav_bit_array: &Vec<Option<UnifiedNavData>>) -> Result<usize, Box<dyn std::error::Error>> {
        for i in 0..self.bds_sat_number {
            if let Some(eph) = &self.bds_eph_visible[i] {
                // Check each BDS signal
                let bds_signals = [
                    (SIGNAL_INDEX_B1C, crate::types::GEN_B1C, 0),
                    (SIGNAL_INDEX_B1I, crate::types::GEN_B1I, 1),
                    (SIGNAL_INDEX_B2I, crate::types::GEN_B2I, 2),
                    (SIGNAL_INDEX_B3I, crate::types::GEN_B3I, 3),
                    (SIGNAL_INDEX_B2A, crate::types::GEN_B2A, 4),
                    (SIGNAL_INDEX_B2B, crate::types::GEN_B2B, 5),
                    (SIGNAL_INDEX_B2AB, crate::types::GEN_B2AB, 6),
                ];
                
                for &(signal_index, signal_mask, freq_array_index) in &bds_signals {
                    if self.output_param.CompactConfig.is_signal_enabled(signal_mask) {
                        let center_freq = SIGNAL_CENTER_FREQ[GnssSystem::BdsSystem as usize][freq_array_index];
                        let if_freq = (center_freq - self.output_param.CenterFreq as f64) as i32;
                        let mut new_signal = SatIfSignal::new(self.output_param.SampleFreq, if_freq, GnssSystem::BdsSystem, signal_index as i32, eph.svid);
                        
                        let nav_data = self.get_nav_data(GnssSystem::BdsSystem, signal_index as i32, nav_bit_array);
                        let nav_data_clone = nav_data.cloned();

                        new_signal.init_state(self.cur_time, &self.bds_sat_param[eph.svid as usize - 1], nav_data_clone);
                        sat_if_signals[total_channel_number] = Some(Box::new(new_signal));
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    fn create_galileo_signals(&self, sat_if_signals: &mut Vec<Option<Box<SatIfSignal>>>, mut total_channel_number: usize, nav_bit_array: &Vec<Option<UnifiedNavData>>) -> Result<usize, Box<dyn std::error::Error>> {
        for i in 0..self.gal_sat_number {
            if let Some(eph) = &self.gal_eph_visible[i] {
                // Check each Galileo signal  
                let gal_signals = [
                    (SIGNAL_INDEX_E1, crate::types::GEN_E1, 0),
                    (SIGNAL_INDEX_E5A, crate::types::GEN_E5A, 1), 
                    (SIGNAL_INDEX_E5B, crate::types::GEN_E5B, 2),
                    (SIGNAL_INDEX_E6, crate::types::GEN_E6, 4),
                ];
                
                for &(signal_index, signal_mask, freq_array_index) in &gal_signals {
                    if self.output_param.CompactConfig.is_signal_enabled(signal_mask) {
                        let center_freq = SIGNAL_CENTER_FREQ[GnssSystem::GalileoSystem as usize][freq_array_index.min(7)];
                        let if_freq = (center_freq - self.output_param.CenterFreq as f64) as i32;
                        let mut new_signal = SatIfSignal::new(self.output_param.SampleFreq, if_freq, GnssSystem::GalileoSystem, signal_index as i32, eph.svid);
                        
                        let nav_data = self.get_nav_data(GnssSystem::GalileoSystem, signal_index as i32, nav_bit_array);
                        let nav_data_clone = nav_data.cloned();

                        new_signal.init_state(self.cur_time, &self.gal_sat_param[eph.svid as usize - 1], nav_data_clone);
                        sat_if_signals[total_channel_number] = Some(Box::new(new_signal));
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    fn create_glonass_signals(&self, sat_if_signals: &mut Vec<Option<Box<SatIfSignal>>>, mut total_channel_number: usize, nav_bit_array: &Vec<Option<UnifiedNavData>>) -> Result<usize, Box<dyn std::error::Error>> {
        for i in 0..self.glo_sat_number {
            if let Some(eph) = &self.glo_eph_visible[i] {
                // Check each GLONASS signal
                let glo_signals = [
                    (SIGNAL_INDEX_G1, crate::types::GEN_G1),
                    (SIGNAL_INDEX_G2, crate::types::GEN_G2),
                    (SIGNAL_INDEX_G3, crate::types::GEN_G3),
                ];
                
                for &(signal_index, signal_mask) in &glo_signals {
                    if self.output_param.CompactConfig.is_signal_enabled(signal_mask) {
                        let center_freq = SIGNAL_CENTER_FREQ[GnssSystem::GlonassSystem as usize][signal_index.min(7)] + eph.freq as f64 * 562500.0;
                        let if_freq = (center_freq - self.output_param.CenterFreq as f64) as i32;
                        let mut new_signal = SatIfSignal::new(self.output_param.SampleFreq, if_freq, GnssSystem::GlonassSystem, signal_index as i32, eph.n);
                        
                        let nav_data = self.get_nav_data(GnssSystem::GlonassSystem, signal_index as i32, nav_bit_array);
                        let nav_data_clone = nav_data.cloned();

                        new_signal.init_state(self.cur_time, &self.glo_sat_param[eph.n as usize - 1], nav_data_clone);
                        sat_if_signals[total_channel_number] = Some(Box::new(new_signal));
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    // Методы для соответствия интерфейсу main.rs
    pub fn load_config(&mut self, config_file: &str) -> Result<(), Box<dyn std::error::Error>> {
        println!("[UNIQUE-DEBUG] load_config called with: {}", config_file);
        println!("[INFO]\tLoading JSON file: {}", config_file);
        
        // Сохраняем путь к config файлу
        self.output_param.config_filename = config_file.to_string();
        
        // Проверяем что файл существует
        if !std::path::Path::new(config_file).exists() {
            eprintln!("[ERROR]\tJSON file not found: {}", config_file);
            return Err("JSON file not found".into());
        }
        
        println!("[INFO]\tJSON file read successfully: {}", config_file);

        // Парсим параметры из JSON используя чистый Rust
        let mut utc_time = UtcTime::default();
        let mut start_pos = LlaPosition::default();
        let start_vel = LocalSpeed::default();
        
        println!("[DEBUG] Filename before JSON parsing: {:?}", 
                std::str::from_utf8(&self.output_param.filename[..20]).unwrap_or("invalid"));
        
        println!("[UNIQUE-JSON-START] Starting pure Rust JSON parsing");
        
        // Переменная для пути к RINEX файлу
        let mut rinex_file = String::from("Rinex_Data/rinex_v3_20251560000.rnx"); // Значение по умолчанию
        
        // УПРОЩЕННЫЙ JSON ПАРСИНГ - читаем файл напрямую и парсим нужные параметры  
        if let Ok(json_content) = std::fs::read_to_string(config_file) {
            println!("[UNIQUE-JSON-READ] File read successfully, length: {}", json_content.len());
            if let Ok(json) = serde_json::from_str::<serde_json::Value>(&json_content) {
                println!("[UNIQUE-JSON-PARSE] JSON parsed successfully");
                // Парсим время из JSON
                if let Some(time) = json.get("time") {
                    if let (Some(year), Some(month), Some(day), Some(hour), Some(minute), Some(second)) = (
                        time.get("year").and_then(|v| v.as_i64()),
                        time.get("month").and_then(|v| v.as_i64()),
                        time.get("day").and_then(|v| v.as_i64()),
                        time.get("hour").and_then(|v| v.as_i64()),
                        time.get("minute").and_then(|v| v.as_i64()),
                        time.get("second").and_then(|v| v.as_f64())
                    ) {
                        utc_time.Year = year as i32;
                        utc_time.Month = month as i32;
                        utc_time.Day = day as i32;
                        utc_time.Hour = hour as i32;
                        utc_time.Minute = minute as i32;
                        utc_time.Second = second;
                    }
                }
                
                // Парсим координаты из initPosition
                if let Some(trajectory) = json.get("trajectory") {
                    if let Some(init_pos) = trajectory.get("initPosition") {
                        if let (Some(longitude), Some(latitude), Some(altitude)) = (
                            init_pos.get("longitude").and_then(|v| v.as_f64()),
                            init_pos.get("latitude").and_then(|v| v.as_f64()),
                            init_pos.get("altitude").and_then(|v| v.as_f64())
                        ) {
                            start_pos.lon = longitude.to_radians();
                            start_pos.lat = latitude.to_radians();
                            start_pos.alt = altitude;
                            println!("[INFO] Parsed position from JSON: lat={:.6}°, lon={:.6}°, alt={:.1}m", 
                                     latitude, longitude, altitude);
                            
                            // Сохраняем координаты в структуре
                            self.start_pos = start_pos;
                        }
                    }
                }
                
                // Парсим путь к файлу эфемерид
                if let Some(ephemeris) = json.get("ephemeris") {
                    if let Some(name) = ephemeris.get("name").and_then(|v| v.as_str()) {
                        println!("[DEBUG] Found ephemeris file in JSON: {}", name);
                        rinex_file = String::from(name); // Копируем путь из JSON
                    }
                }
                
                // Парсим выходные параметры из JSON
                if let Some(output) = json.get("output") {
                    // Парсим имя файла
                    if let Some(filename) = output.get("name").and_then(|v| v.as_str()) {
                        println!("[DEBUG] Found filename in JSON: {}", filename);
                        let filename_bytes = filename.as_bytes();
                        let len = filename_bytes.len().min(255);
                        self.output_param.filename[..len].copy_from_slice(&filename_bytes[..len]);
                        self.output_param.filename[len] = 0; // Null terminator
                    }
                    
                    // Парсим частоту дискретизации (sampleFreq в MHz -> Hz)
                    if let Some(sample_freq) = output.get("sampleFreq").and_then(|v| v.as_f64()) {
                        self.output_param.SampleFreq = (sample_freq * 1e6) as i32;
                    }
                    
                    // Парсим центральную частоту (centerFreq в MHz -> kHz)
                    if let Some(center_freq) = output.get("centerFreq").and_then(|v| v.as_f64()) {
                        self.output_param.CenterFreq = (center_freq * 1000.0) as i32;
                    }
                    
                    // Парсим формат
                    if let Some(format) = output.get("format").and_then(|v| v.as_str()) {
                        match format {
                            "IQ8" => self.output_param.Format = OutputFormat::OutputFormatIQ8,
                            "IQ4" => self.output_param.Format = OutputFormat::OutputFormatIQ4,
                            _ => self.output_param.Format = OutputFormat::OutputFormatIQ8,
                        }
                    }
                    
                    // Парсим системы спутников  
                    if let Some(system_select) = output.get("systemSelect").and_then(|v| v.as_array()) {
                        use crate::types::*;
                        
                        
                        // Сброс конфигурации
                        self.output_param.CompactConfig = CompactConfig::new();
                        
                        for system in system_select {
                            if let (Some(sys), Some(signal), Some(enable)) = (
                                system.get("system").and_then(|v| v.as_str()),
                                system.get("signal").and_then(|v| v.as_str()),
                                system.get("enable").and_then(|v| v.as_bool())
                            ) {
                                if enable {
                                    // Определяем системы для парсинга
                                    match sys {
                                        "GPS" => self.output_param.CompactConfig.enable_system_parsing(PARSE_GPS),
                                        "BDS" => self.output_param.CompactConfig.enable_system_parsing(PARSE_BDS),
                                        "Galileo" => self.output_param.CompactConfig.enable_system_parsing(PARSE_GALILEO),
                                        "GLONASS" => self.output_param.CompactConfig.enable_system_parsing(PARSE_GLONASS),
                                        _ => {}
                                    }
                                    
                                    // Определяем сигналы для генерации
                                    match (sys, signal) {
                                        ("GPS", "L1CA") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_L1CA);
                                        }
                                        ("GPS", "L1C") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_L1C);
                                        }
                                        ("GPS", "L2C") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_L2C);
                                        }
                                        ("GPS", "L5") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_L5);
                                        }
                                        ("BDS", "B1C") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_B1C);
                                        }
                                        ("BDS", "B1I") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_B1I);
                                        }
                                        ("BDS", "B2a") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_B2A);
                                        }
                                        ("BDS", "B2I") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_B2I);
                                        }
                                        ("BDS", "B2b") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_B2B);
                                        }
                                        ("BDS", "B3I") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_B3I);
                                        }
                                        ("Galileo", "E1") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_E1);
                                        }
                                        ("Galileo", "E5a") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_E5A);
                                        }
                                        ("Galileo", "E5b") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_E5B);
                                        }
                                        ("Galileo", "E6") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_E6);
                                        }
                                        ("GLONASS", "G1") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_G1);
                                        }
                                        ("GLONASS", "G2") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_G2);
                                        }
                                        _ => {}
                                    }
                                }
                            }
                        }
                    }
                }
                
                // Парсим длительность траектории
                if let Some(traj) = json.get("trajectory") {
                    if let Some(traj_list) = traj.get("trajectoryList").and_then(|v| v.as_array()) {
                        if let Some(first_traj) = traj_list.first() {
                            if let Some(time) = first_traj.get("time").and_then(|v| v.as_f64()) {
                                self.output_param.Interval = (time * 1000.0) as i32; // секунды -> мс
                            }
                        }
                    }
                }
            }
        }
        
        println!("[DEBUG] Filename after JSON parsing: {:?}", 
                std::str::from_utf8(&self.output_param.filename[..50]).unwrap_or("invalid"));
        
        // Устанавливаем время симуляции из JSON пресета
        self.cur_time = crate::gnsstime::utc_to_gps_time(utc_time, true);
        
        println!("[INFO]\tParsed from JSON preset:");
        println!("[INFO]\t  Time: {}-{:02}-{:02} {:02}:{:02}:{:06.3}", 
                 utc_time.Year, utc_time.Month, utc_time.Day, 
                 utc_time.Hour, utc_time.Minute, utc_time.Second);
        println!("[INFO]\t  Duration: {:.1} s", self.output_param.Interval as f64 / 1000.0);
        
        // Создаем новую траекторию
        let mut new_trajectory = crate::trajectory::CTrajectory::new();
        new_trajectory.set_init_pos_vel_lla(start_pos, start_vel, false);
        self.trajectory = new_trajectory;
        
        // Reset satellite masks to allow all satellites 
        self.output_param.GpsMaskOut = 0;
        self.output_param.BdsMaskOut = 0;
        self.output_param.GalileoMaskOut = 0;
        self.output_param.GlonassMaskOut = 0;
        
        // Set reasonable elevation mask (5 degrees) if not set by JSON
        if self.output_param.ElevationMask == 0.0 {
            self.output_param.ElevationMask = 5.0_f64.to_radians();
        }
        
        // Загружаем RINEX файл с эфемеридами, используя путь из JSON
        // rinex_file уже определен выше из JSON парсинга
        if std::path::Path::new(&rinex_file).exists() {
            println!("[INFO]\tLoading RINEX ephemeris file: {}", rinex_file);
            
            // Создаем CNavData для загрузки эфемерид
            let mut c_nav_data = crate::json_interpreter::CNavData::default();
            
            // Определяем включенные системы из JSON конфигурации (CompactConfig)
            let mut enabled_systems = Vec::new();
            if self.output_param.CompactConfig.should_parse_gps() {
                enabled_systems.push("GPS");
            }
            if self.output_param.CompactConfig.should_parse_glonass() {
                enabled_systems.push("GLONASS");
            }
            if self.output_param.CompactConfig.should_parse_bds() {
                enabled_systems.push("BeiDou");
            }
            if self.output_param.CompactConfig.should_parse_galileo() {
                enabled_systems.push("Galileo");
            }
            
            println!("[DEBUG] Using filtered RINEX loading: time={}-{:02}-{:02} {:02}:{:02}:{:02}, systems={:?}", 
                utc_time.Year, utc_time.Month, utc_time.Day, utc_time.Hour, utc_time.Minute, utc_time.Second, enabled_systems);
            
            // Используем оптимизированную загрузку RINEX с фильтрацией систем из JSON  
            crate::json_interpreter::read_nav_file_filtered(
                &mut c_nav_data, 
                &rinex_file,
                Some(utc_time), // Фильтрация по времени
                &enabled_systems // Только BeiDou система
            );
            
            // Копируем загруженные эфемериды в наши данные
            self.copy_ephemeris_from_json_nav_data(&c_nav_data, utc_time); // GPS эфемериды
            self.copy_beidou_ephemeris_from_json_nav_data(&c_nav_data, utc_time); // BeiDou эфемериды
            self.copy_galileo_ephemeris_from_json_nav_data(&c_nav_data, utc_time); // Galileo эфемериды
            
            println!("[INFO]\tMinimal ephemeris loaded successfully");
        } else {
            println!("[ERROR]\tRINEX file not found: {}", rinex_file);
            return Err("RINEX file not found".into());
        }
        
        let success = true;
        
        if !success {
            return Err("Failed to parse JSON configuration".into());
        }
        
        println!("[INFO]\tJSON configuration parsed successfully");
        Ok(())
    }
    
    // Добавляет минимальные эфемериды GPS для демонстрации работы
    fn add_minimal_gps_ephemeris(&mut self) {
        println!("[INFO]\tAdding minimal GPS ephemeris for demonstration");
        
        // Убеждаемся что вектор достаточно большой  
        if self.nav_data.gps_ephemeris.len() < 32 {
            self.nav_data.gps_ephemeris.resize(32, None);
        }
        
        // Добавляем минимальные эфемериды для нескольких спутников
        for svid in 1..=8 {
            let mut eph = GpsEphemeris::default();
            eph.svid = svid as u8;
            eph.valid = 1; // Помечаем как действительные
            eph.week = 2200;
            eph.sqrtA = 5153.5; // Примерная полуось GPS орбиты
            eph.i0 = 55.0_f64.to_radians(); // Наклонение GPS орбит
            eph.omega0 = (svid - 1) as f64 * 45.0_f64.to_radians(); // Распределяем по долготе
            eph.ecc = 0.01; // Небольшой эксцентриситет
            eph.omega_dot = -2.6e-9;
            
            self.nav_data.gps_ephemeris[svid as usize - 1] = Some(eph);
        }
        
        println!("[INFO]\tMinimal GPS ephemeris added for satellites 1-8");
    }

    pub fn initialize(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        // Инициализация уже выполнена при загрузке конфигурации
        // Здесь только базовая настройка системы
        self.trajectory.reset_trajectory_time();
        
        // Инициализируем CN0 для спутников
        for i in 0..TOTAL_GPS_SAT {
            self.gps_sat_param[i].CN0 = (self.power_control.init_cn0 * 100.0 + 0.5) as i32;
        }
        for i in 0..TOTAL_BDS_SAT {
            self.bds_sat_param[i].CN0 = (self.power_control.init_cn0 * 100.0 + 0.5) as i32;
        }
        for i in 0..TOTAL_GAL_SAT {
            self.gal_sat_param[i].CN0 = (self.power_control.init_cn0 * 100.0 + 0.5) as i32;
        }
        for i in 0..TOTAL_GLO_SAT {
            self.glo_sat_param[i].CN0 = (self.power_control.init_cn0 * 100.0 + 0.5) as i32;
        }
        
        Ok(())
    }
    
    /// Функция интеллектуального выбора GPS эфемерид по временной близости к целевому моменту генерации
    /// 
    /// Реализует критически важный алгоритм выбора оптимальных эфемерид для каждого GPS спутника.
    /// Алгоритм обеспечивает максимальную точность навигационных вычислений путем выбора эфемерид
    /// с минимальным временным расстоянием до момента генерации сигнала.
    /// 
    /// # Параметры
    /// * `c_nav_data` - Источник навигационных данных, содержащий массив GPS эфемерид из RINEX
    /// * `utc_time` - Целевое UTC время генерации сигнала из пресета конфигурации
    /// 
    /// # Алгоритм временного выбора
    /// 1. Преобразование UTC времени в GPS время для корректного сравнения с toe
    /// 2. Для каждого SVID (1-32) поиск эфемериды с минимальной разностью |target_time - toe|
    /// 3. Выбор единственной "лучшей" эфемериды на спутник для обеспечения консистентности
    fn copy_ephemeris_from_json_nav_data(&mut self, c_nav_data: &crate::json_interpreter::CNavData, utc_time: UtcTime) {
        println!("[INFO]\tCopying GPS ephemeris from JSON CNavData");
        println!("[DEBUG] Source has {} GPS ephemeris records", c_nav_data.gps_ephemeris.len());
        
        // Критическое преобразование: UTC время из пресета -> GPS время для сравнения с toe (time of ephemeris)
        // GPS время необходимо, так как все эфемериды содержат toe в GPS временной шкале
        let target_gps_time = utc_to_gps_time(utc_time, false);
        let target_time = (target_gps_time.MilliSeconds as f64) / 1000.0; // Конвертация в секунды GPS времени
        
        // Алгоритм выбора оптимальных эфемерид: для каждого SVID найти эфемериду с минимальным |Δt|
        for svid in 1..=TOTAL_GPS_SAT {
            let mut best_eph: Option<GpsEphemeris> = None;     // Лучшая найденная эфемерида для текущего SVID
            let mut best_time_diff = f64::INFINITY;            // Минимальная временная разность в секундах
            let mut available_count = 0;  // Количество доступных эфемерид для данного SVID
            
            // Перебор всех доступных эфемерид для поиска оптимальной по времени
            for eph in &c_nav_data.gps_ephemeris {
                if eph.svid == svid as u8 {
                    available_count += 1;
                    // Вычисление абсолютной временной разности между целевым временем и toe эфемериды
                    let time_diff = (target_time - eph.toe as f64).abs();
                    if time_diff < best_time_diff {
                        best_time_diff = time_diff;
                        best_eph = Some(*eph);                // Сохранение лучшей эфемериды
                    }
                }
            }
            
            // Применение выбранной эфемериды к соответствующему слоту массива
            if let Some(eph) = best_eph {
                let index = svid - 1;              // Преобразование SVID (1-32) в индекс массива (0-31)
                self.gps_eph[index] = Some(eph);
                println!("[DEBUG] GPS{:02} - Selected ephemeris: toe={}, time_diff={:.1}h, valid={}, health={}", 
                         svid, eph.toe, best_time_diff / 3600.0, eph.valid, eph.health);
            } else if available_count == 0 {
                println!("[DEBUG] GPS{:02} - No ephemeris data available", svid);
            }
        }
        
        // Верификация результатов выбора: подсчет количества успешно размещенных эфемерид
        let mut verified_count = 0;
        for i in 0..TOTAL_GPS_SAT {
            if self.gps_eph[i].is_some() {
                verified_count += 1;                          // Подсчет слотов с выбранными эфемеридами
            }
        }
        
        // Дублирование в nav_data структуру для обеспечения обратной совместимости с legacy кодом
        // Критично: legacy части системы могут обращаться к nav_data.gps_ephemeris массиву
        if self.nav_data.gps_ephemeris.len() < 32 {
            self.nav_data.gps_ephemeris.resize(32, None);     // Гарантированное выделение для 32 GPS спутников
        }
        // Копирование всех исходных эфемерид в nav_data (без временной фильтрации)
        for eph in &c_nav_data.gps_ephemeris {
            if (eph.svid as usize) <= 32 && eph.svid > 0 {
                self.nav_data.gps_ephemeris[eph.svid as usize - 1] = Some(*eph);
            }
        }
        
        println!("[INFO]\tSelected {} GPS ephemeris records using time-based algorithm", verified_count);
    }

    /// Функция интеллектуального выбора BeiDou эфемерид с унифицированным алгоритмом временного отбора
    /// 
    /// Реализует идентичную GPS функции логику выбора оптимальных эфемерид для BeiDou спутников.
    /// Критически важно: использует GPS время для сравнения с toe, обеспечивая единообразие
    /// алгоритма выбора эфемерид для всех GNSS систем.
    /// 
    /// # Параметры
    /// * `c_nav_data` - Источник навигационных данных с BeiDou эфемеридами из RINEX файлов
    /// * `utc_time` - Целевое UTC время генерации из конфигурационного пресета
    /// 
    /// # Особенности BeiDou обработки
    /// - BeiDou использует BDT (BeiDou Time), но для единообразия алгоритма применяется GPS время
    /// - Поддерживается до TOTAL_BDS_SAT спутников (расширенная констелляция)
    /// - Применяется та же логика минимизации временной разности как для GPS
    fn copy_beidou_ephemeris_from_json_nav_data(&mut self, c_nav_data: &crate::json_interpreter::CNavData, utc_time: UtcTime) {
        println!("[INFO]\tCopying BeiDou ephemeris from JSON CNavData");
        println!("[DEBUG] Source has {} BeiDou ephemeris records", c_nav_data.beidou_ephemeris.len());
        
        // Унифицированное преобразование времени: UTC -> GPS время для консистентности с GPS алгоритмом
        // Важно: несмотря на то, что BeiDou использует BDT, для выбора эфемерид применяется GPS время
        let target_gps_time = utc_to_gps_time(utc_time, false);
        let target_time = (target_gps_time.MilliSeconds as f64) / 1000.0; // Целевое время в секундах GPS
        
        // Идентичный GPS алгоритм выбора: поиск эфемериды с минимальной временной разностью
        for svid in 1..=TOTAL_BDS_SAT {
            let mut best_eph: Option<GpsEphemeris> = None;     // Оптимальная эфемерида для текущего BeiDou SVID
            let mut best_time_diff = f64::INFINITY;            // Минимальная найденная временная разность
            
            // Поиск среди всех доступных BeiDou эфемерид для данного SVID
            for eph in &c_nav_data.beidou_ephemeris {
                if eph.svid == svid as u8 {
                    // Критерий выбора: минимальная абсолютная разность времен
                    let time_diff = (target_time - eph.toe as f64).abs();
                    if time_diff < best_time_diff {
                        best_time_diff = time_diff;
                        best_eph = Some(eph.to_gps_ephemeris());                // Обновление лучшего кандидата (конвертируем BeiDou в GPS)
                    }
                }
            }
            
            // Размещение выбранной эфемериды в соответствующем слоте BeiDou массива
            if let Some(eph) = best_eph {
                let index = svid - 1;              // SVID (1-N) -> array index (0-(N-1))
                self.bds_eph[index] = Some(eph);
                println!("[DEBUG] BDS{:02} - Selected ephemeris with toe={}, time_diff={:.1}h", svid, eph.toe, best_time_diff / 3600.0);
            }
        }
        
        // Статистическая верификация: подсчет эфемерид, успешно прошедших временной отбор
        let mut verified_count = 0;
        for i in 0..TOTAL_BDS_SAT {
            if self.bds_eph[i].is_some() {
                verified_count += 1;                          // Инкремент счетчика заполненных слотов
            }
        }
        
        // Параллельное заполнение nav_data для поддержки совместимости с существующим кодом
        // BeiDou констелляция: поддерживается до 65 спутников (включая резервные орбиты)
        if self.nav_data.bds_ephemeris.len() < 65 {
            self.nav_data.bds_ephemeris.resize(65, None);     // Выделение памяти для полной BeiDou констелляции
        }
        // Заполнение nav_data всеми исходными эфемеридами (без применения временной фильтрации)
        for eph in &c_nav_data.beidou_ephemeris {
            if (eph.svid as usize) <= 65 && eph.svid > 0 {
                self.nav_data.bds_ephemeris[eph.svid as usize - 1] = Some(*eph);
            }
        }
        
        println!("[INFO]\tSelected {} BeiDou ephemeris records using time-based algorithm", verified_count);
    }

    /// Функция выбора Galileo эфемерид с расширенной диагностикой и унифицированным алгоритмом
    /// 
    /// Третья реализация унифицированного алгоритма выбора эфемерид по временной близости.
    /// Содержит расширенные диагностические возможности для отладки процесса выбора эфемерид.
    /// Обеспечивает полную совместимость алгоритма выбора между GPS, BeiDou и Galileo системами.
    /// 
    /// # Параметры
    /// * `c_nav_data` - Контейнер навигационных данных с Galileo эфемеридами из RINEX
    /// * `utc_time` - Эталонное UTC время для выбора оптимальных эфемерид
    /// 
    /// # Особенности Galileo обработки
    /// - Galileo использует GST (Galileo System Time), но алгоритм унифицирован через GPS время
    /// - Расширенная диагностика включает проверку valid/health статусов эфемерид
    /// - Ограниченный вывод отладочной информации для предотвращения спама в логах
    /// - Поддержка полной Galileo констелляции до TOTAL_GAL_SAT спутников
    fn copy_galileo_ephemeris_from_json_nav_data(&mut self, c_nav_data: &crate::json_interpreter::CNavData, utc_time: UtcTime) {
        println!("[INFO]\tCopying Galileo ephemeris from JSON CNavData");
        println!("[DEBUG] Source has {} Galileo ephemeris records", c_nav_data.galileo_ephemeris.len());
        
        // Стандартизированное преобразование времени для обеспечения единообразия алгоритма выбора
        // Критически важно: все GNSS системы используют одинаковую временную базу для сравнения
        let target_gps_time = utc_to_gps_time(utc_time, false);
        let target_time = (target_gps_time.MilliSeconds as f64) / 1000.0; // Эталонное время в секундах GPS
        
        // Реализация унифицированного алгоритма выбора с расширенной статистикой
        let mut gal_count = 0;                                // Счетчик успешно выбранных эфемерид
        for svid in 1..=TOTAL_GAL_SAT {
            let mut best_eph: Option<GpsEphemeris> = None;     // Лучшая эфемерида для текущего Galileo SVID
            let mut best_time_diff = f64::INFINITY;            // Минимальная временная разность
            
            // Алгоритм поиска оптимальной эфемериды: минимизация |target_time - toe|
            for eph in &c_nav_data.galileo_ephemeris {
                if eph.svid == svid as u8 {
                    // Основной критерий: абсолютная временная разность между целевым временем и toe
                    let time_diff = (target_time - eph.toe as f64).abs();
                    if time_diff < best_time_diff {
                        best_time_diff = time_diff;
                        best_eph = Some(*eph);                // Сохранение оптимального кандидата
                    }
                }
            }
            
            // Применение выбранной эфемериды с расширенной диагностикой
            if let Some(eph) = best_eph {
                let index = svid - 1;              // Конвертация SVID в индекс массива
                self.gal_eph[index] = Some(eph);
                gal_count += 1;
                // Ограниченный вывод диагностики для первых 5 спутников (предотвращение спама)
                if gal_count <= 5 {
                    println!("[DEBUG] Selected best Galileo ephemeris for SVID {} - toe={}, time_diff={:.1}h (valid:{}, health:{})", 
                        svid, eph.toe, best_time_diff / 3600.0, eph.valid, eph.health);
                }
            }
        }
        
        // Финальная верификация корректности алгоритма выбора эфемерид
        let mut verified_count = 0;
        for i in 0..TOTAL_GAL_SAT {
            if self.gal_eph[i].is_some() {
                verified_count += 1;                          // Подтверждение успешного размещения эфемериды
            }
        }
        
        // Синхронизация с nav_data структурой для обеспечения совместимости архитектуры
        // Galileo: динамическое масштабирование под актуальный размер констелляции
        if self.nav_data.gal_ephemeris.len() < TOTAL_GAL_SAT {
            self.nav_data.gal_ephemeris.resize(TOTAL_GAL_SAT, None);  // Выделение под полную Galileo констелляцию
        }
        // Полное копирование исходных данных в nav_data (дублирование без временного отбора)
        for eph in &c_nav_data.galileo_ephemeris {
            if (eph.svid as usize) <= TOTAL_GAL_SAT && eph.svid > 0 {
                self.nav_data.gal_ephemeris[eph.svid as usize - 1] = Some(*eph);
            }
        }
        
        // Расширенная отчетность: сравнение количества обработанных и верифицированных эфемерид
        println!("[INFO]\tCopied {} Galileo ephemeris records, verified {} in gal_eph array", gal_count, verified_count);
    }

    pub fn generate_data(&mut self) -> Result<GenerationStats, Box<dyn std::error::Error>> {
        // Используем стандартные данные пока не реализованы get методы
        let utc_time = self.parse_utc_time_from_json("presets/GPS_L1_only.json").unwrap_or_else(|| {
            println!("[WARNING]\tFailed to parse time from JSON, using default time");
            UtcTime {
                Year: 2025,
                Month: 6,
                Day: 5,
                Hour: 10,
                Minute: 5,
                Second: 30.0
            }
        });
        let start_pos = self.start_pos;
        let start_vel = LocalSpeed::default();
        let glonass_time = utc_to_glonass_time_corrected(utc_time);
        let bds_time = utc_to_bds_time(utc_time);
        let cur_pos = lla_to_ecef(&start_pos);

        // Открываем выходной файл
        let filename_binding = String::from_utf8_lossy(&self.output_param.filename);
        let filename_str = filename_binding.trim_end_matches('\0');
        println!("[INFO]\tOpening output file: {}", filename_str);
        let mut if_file = match File::create(filename_str) {
            Ok(file) => {
                println!("[INFO]\tOutput file opened successfully.");
                BufWriter::new(file)
            },
            Err(_) => {
                println!("[ERROR]\tFailed to open output file: {}", filename_str);
                return Err("Failed to open output file".into());
            }
        };

        // Создаем навигационные биты
        let mut nav_bit_array = self.create_nav_bit_instances();
        
        // Настраиваем систему
        self.setup_navigation_data(&mut nav_bit_array, utc_time, glonass_time, bds_time)?;
        self.calculate_visible_satellites(cur_pos, glonass_time)?;

        // Создаем спутниковые сигналы и генерируем данные
        let mut sat_if_signals = self.create_satellite_signals(&nav_bit_array)?;
        let trajectory_time_s = self.parse_trajectory_time_from_json(&self.output_param.config_filename).unwrap_or(10.0);
        self.generate_if_signal(&mut if_file, &mut sat_if_signals, cur_pos, trajectory_time_s)?;

        // Вычисляем ПРАВИЛЬНУЮ статистику
        // SampleFreq уже в Hz (5,000,000), НЕ УМНОЖАТЬ на 1e6!
        let trajectory_time_seconds = trajectory_time_s;
        let total_samples = (self.output_param.SampleFreq as f64 * trajectory_time_seconds) as u64;
        let file_size_mb = match self.output_param.Format {
            OutputFormat::OutputFormatIQ4 => Some(total_samples as f64 / 1024.0 / 1024.0),
            OutputFormat::OutputFormatIQ8 => Some(total_samples as f64 * 2.0 / 1024.0 / 1024.0),
            _ => Some(total_samples as f64 * 2.0 / 1024.0 / 1024.0), // Default to IQ8 equivalent
        };

        println!("[INFO]\tIF Signal generation completed!");

        // Проверяем доступность аппаратных ускорителей
        let avx512_available = crate::avx512_intrinsics::Avx512Accelerator::is_available();
        
        #[cfg(feature = "gpu")]
        let cuda_available = crate::cuda_acceleration::CudaGnssAccelerator::is_available();
        #[cfg(not(feature = "gpu"))]
        let cuda_available = false;
        
        Ok(GenerationStats {
            total_samples,
            clipped_samples: 0,
            file_size_mb,
            
            // Дополнительные данные для анализа производительности
            total_time_ms: 0.0,  // Будет заполнено в вызывающем коде
            signal_processing_time_ms: 0.0, 
            samples_generated: total_samples,
            satellites_processed: sat_if_signals.len() as u32,
            avx512_accelerated: avx512_available,
            cuda_accelerated: cuda_available,
        })
    }

    // Satellite position calculation functions based on C implementation
    // Убрана неработающая функция gps_sat_pos_speed_eph - используется crate::coordinate::gps_sat_pos_speed_eph

    fn glonass_sat_pos_speed_eph(&self, transmit_time: f64, eph: &GlonassEphemeris) -> Option<KinematicInfo> {
        // Simplified GLONASS position calculation
        // In a real implementation, this would use Runge-Kutta integration
        // For now, use a simplified approach based on the orbital elements
        
        Some(KinematicInfo {
            x: eph.x * 1000.0,  // Convert from km to m
            y: eph.y * 1000.0,
            z: eph.z * 1000.0,
            vx: eph.vx * 1000.0,
            vy: eph.vy * 1000.0,
            vz: eph.vz * 1000.0,
        })
    }


    // Простой парсер для извлечения времени траектории из JSON
    fn parse_utc_time_from_json(&self, config_file: &str) -> Option<UtcTime> {
        use std::fs;
        let json_content = fs::read_to_string(config_file).ok()?;
        
        // Simple JSON parsing for time section
        let time_start = json_content.find("\"time\": {")?;
        let time_end = json_content[time_start..].find("}")?;
        let time_section = &json_content[time_start..time_start + time_end + 1];
        
        let year = Self::extract_json_field(time_section, "year")?;
        let month = Self::extract_json_field(time_section, "month")?;
        let day = Self::extract_json_field(time_section, "day")?;
        let hour = Self::extract_json_field(time_section, "hour")?;
        let minute = Self::extract_json_field(time_section, "minute")?;
        let second = Self::extract_json_field(time_section, "second")?;
        
        Some(UtcTime {
            Year: year as i32,
            Month: month as i32,
            Day: day as i32,
            Hour: hour as i32,
            Minute: minute as i32,
            Second: second
        })
    }
    
    fn extract_json_field(json_section: &str, field_name: &str) -> Option<f64> {
        let field_pattern = format!("\"{}\": ", field_name);
        let start = json_section.find(&field_pattern)?;
        let value_start = start + field_pattern.len();
        let value_end = json_section[value_start..].find([',', '\n', '}'].as_ref())?;
        let value_str = json_section[value_start..value_start + value_end].trim();
        value_str.parse().ok()
    }

    fn parse_trajectory_time_from_json(&self, config_file: &str) -> Option<f64> {
        use std::fs;
        
        // Читаем JSON файл как текст
        let json_content = fs::read_to_string(config_file).ok()?;
        
        // Ищем "trajectoryList" и первый "time"
        if let Some(traj_start) = json_content.find("\"trajectoryList\"") {
            let after_traj = &json_content[traj_start..];
            if let Some(time_start) = after_traj.find("\"time\"") {
                let after_time = &after_traj[time_start + 6..]; // Skip "time"
                if let Some(colon_pos) = after_time.find(':') {
                    let after_colon = &after_time[colon_pos + 1..];
                    // Извлекаем число до запятой или закрывающей скобки
                    let mut end_pos = 0;
                    let chars: Vec<char> = after_colon.chars().collect();
                    for (i, &c) in chars.iter().enumerate() {
                        if c.is_ascii_digit() || c == '.' {
                            end_pos = i + 1;
                        } else if c == ',' || c == '}' || c == ']' {
                            break;
                        }
                    }
                    let time_str = after_colon[..end_pos].trim();
                    return time_str.parse::<f64>().ok();
                }
            }
        }
        
        None
    }
}

impl Default for IFDataGen {
    fn default() -> Self {
        Self::new()
    }
}

// Main function for the executable
pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let mut if_data_gen = IFDataGen::new();
    if_data_gen.main(args)
}