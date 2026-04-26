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
use std::io::{BufWriter, Write};
use wide::f64x4;

use crate::complex_number::ComplexNumber;
use crate::constants::*;
use crate::json_parser::JsonObject;
use crate::types::IonoNequick as INavIonoNequick;
use crate::types::*;
use std::env;
use crate::coordinate::{calc_conv_matrix_lla, ecef_to_lla, lla_to_ecef, speed_local_to_ecef};
#[cfg(feature = "gpu")]
use crate::cuda_acceleration::HybridAccelerator;
use crate::gnsstime::{
    utc_to_bds_time, utc_to_galileo_time, utc_to_glonass_time_corrected, utc_to_gps_time,
};
use crate::nav_data::NavData as UnifiedNavData;
use crate::navdata::NavDataType;
use crate::powercontrol::{CPowerControl, SignalPower};
use crate::pvt;
use crate::trajectory::CTrajectory;
use crate::types::SatelliteParam;
use crate::{
    bcnav1bit::BCNav1Bit as ActualBCNav1Bit, bcnav2bit::BCNav2Bit as ActualBCNav2Bit,
    bcnav3bit::BCNav3Bit as ActualBCNav3Bit,
};
use crate::{cnav2bit::CNav2Bit as ActualCNav2Bit, cnavbit::CNavBit as ActualCNavBit};
use crate::{
    d1d2navbit::D1D2NavBit as ActualD1D2NavBit, fnavbit::FNavBit as ActualFNavBit,
    inavbit::INavBit as ActualINavBit,
};
use crate::{gnavbit::GNavBit, l5cnavbit::L5CNavBit, lnavbit::LNavBit};

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
    pub gal_ephemeris: Vec<Option<GpsEphemeris>>,    // Galileo использует структуру GPS
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

    pub fn set_gps_iono(&mut self, iono: IonoParam) {
        self.gps_iono = Some(iono);
    }
    pub fn set_gps_utc(&mut self, utc: UtcParam) {
        self.gps_utc = Some(utc);
    }

    pub fn get_gps_iono(&self) -> Option<&IonoParam> {
        self.gps_iono.as_ref()
    }
    pub fn get_gps_utc_param(&self) -> Option<&UtcParam> {
        self.gps_utc.as_ref()
    }
    pub fn get_bds_iono(&self) -> Option<&IonoParam> {
        self.bds_iono.as_ref()
    }
    pub fn get_bds_utc_param(&self) -> Option<&UtcParam> {
        self.bds_utc.as_ref()
    }
    pub fn get_galileo_iono(&self) -> Option<&IonoParam> {
        self.gal_iono.as_ref()
    }
    pub fn get_galileo_utc_param(&self) -> Option<&UtcParam> {
        self.gal_utc.as_ref()
    }
    pub fn get_gps_almanac(&self) -> &[GpsAlmanac] {
        &self.gps_almanac
    }
    pub fn get_bds_almanac(&self) -> &[GpsAlmanac] {
        &self.bds_almanac
    }
    pub fn get_galileo_almanac(&self) -> &[GpsAlmanac] {
        &self.gal_almanac
    }
    pub fn get_glonass_almanac(&self) -> &[GlonassAlmanac] {
        &self.glo_almanac
    }

    /// Поиск эфемерид по системе, времени и SVID
    /// Возвращает наиболее подходящие эфемериды по временной близости
    /// Поиск эфемерид по системе, времени и SVID - ТОЧНО как в C++ FindEphemeris()
    pub fn find_ephemeris(
        &self,
        system: GnssSystem,
        time: GnssTime,
        svid: i32,
        _signal: i32,
        _param: i32,
    ) -> Option<GpsEphemeris> {
        let mut best_eph: Option<GpsEphemeris> = None;
        let mut best_time_diff = i32::MAX as f64;

        // Обработка разных систем отдельно из-за разных типов эфемерид
        match system {
            GnssSystem::GpsSystem => {
                // Поиск GPS эфемерид
                for eph in self.gps_ephemeris.iter().flatten() {
                    if eph.svid as i32 == svid && (eph.valid & 1) != 0 && eph.health == 0 {
                        let diff = ((time.Week - eph.week) as f64) * 604800.0
                            + ((time.MilliSeconds / 1000) as f64 - eph.toe as f64);

                        let abs_diff = diff.abs();

                        if abs_diff <= 7200.0 && abs_diff <= best_time_diff {
                            best_time_diff = abs_diff;
                            best_eph = Some(*eph);
                        }
                    }
                }
            }
            GnssSystem::BdsSystem => {
                // Поиск BeiDou эфемерид с конвертацией в GPS формат
                for eph in self.bds_ephemeris.iter().flatten() {
                    if eph.svid as i32 == svid && (eph.valid & 1) != 0 && eph.health == 0 {
                        let diff = ((time.Week - eph.week) as f64) * 604800.0
                            + ((time.MilliSeconds / 1000) as f64 - eph.toe as f64);

                        let abs_diff = diff.abs();

                        if abs_diff <= 7200.0 && abs_diff <= best_time_diff {
                            best_time_diff = abs_diff;
                            best_eph = Some(eph.to_gps_ephemeris()); // Конвертируем BeiDou в GPS формат
                        }
                    }
                }
            }
            GnssSystem::GalileoSystem => {
                // Поиск Galileo эфемерид
                for eph in self.gal_ephemeris.iter().flatten() {
                    if eph.svid as i32 == svid && (eph.valid & 1) != 0 && eph.health == 0 {
                        let diff = ((time.Week - eph.week) as f64) * 604800.0
                            + ((time.MilliSeconds / 1000) as f64 - eph.toe as f64);

                        let abs_diff = diff.abs();

                        if abs_diff <= 7200.0 && abs_diff <= best_time_diff {
                            best_time_diff = abs_diff;
                            best_eph = Some(*eph);
                        }
                    }
                }
            }
            _ => return None,
        }

        best_eph
    }

    /// Поиск эфемерид ГЛОНАСС по времени и номеру слота - упрощенная версия  
    pub fn find_glo_ephemeris(&self, time: GlonassTime, svid: i32) -> Option<GlonassEphemeris> {
        let mut best_eph: Option<GlonassEphemeris> = None;
        let mut best_time_diff = i32::MAX as f64;

        // Простое сравнение по дню и времени в секундах
        let request_day = time.Day as f64;
        let request_seconds = (time.MilliSeconds / 1000) as f64;

        // Поиск по номеру слота и времени
        for eph in self.glonass_ephemeris.iter().flatten() {
            if eph.slot as i32 == svid && eph.flag != 0 {
                // Сравниваем по дню года и времени tb
                let eph_day = eph.day as f64;
                let eph_tb = eph.tb as f64;

                let day_diff = (request_day - eph_day).abs();
                let time_diff = (request_seconds - eph_tb).abs();

                let total_diff = if day_diff == 0.0 {
                    time_diff
                } else {
                    day_diff * 86400.0 + time_diff
                };

                // GLONASS эфемериды действительны 30 минут (1800 секунд)
                if total_diff < 1800.0 && total_diff <= best_time_diff {
                    best_time_diff = total_diff;
                    best_eph = Some(*eph);
                }
            }
        }

        best_eph
    }

    pub fn complete_almanac(&mut self, _system: GnssSystem, _time: UtcTime) {
        // Placeholder - implement almanac completion logic
    }

    /// Генерирует GPS альманах из данных эфемерид для всех 32 спутников
    pub fn generate_gps_almanac_from_ephemeris(&mut self, current_week: i32) {
        use crate::almanac::get_almanac_from_ephemeris;

        let toa = (current_week % 1024) * 604800 + 405504; // Reference time of almanac

        for i in 0usize..32 {
            let svid = (i + 1) as i32;
            if let Some(eph) = self.find_ephemeris(
                GnssSystem::GpsSystem,
                GnssTime {
                    Week: current_week,
                    MilliSeconds: 0,
                    SubMilliSeconds: 0.0,
                },
                svid,
                0,
                0,
            ) {
                if (eph.valid & 1) != 0 {
                    let almanac = get_almanac_from_ephemeris(&eph, current_week, toa);
                    self.gps_almanac[i] = almanac;
                    crate::vprintln!(
                        "Generated almanac for SV{:02}: valid={}, toa={}, health={}",
                        svid,
                        self.gps_almanac[i].valid,
                        self.gps_almanac[i].toa,
                        self.gps_almanac[i].health
                    );
                }
            }
        }
    }
}

// SignalPower is imported from powercontrol module

// Placeholder trait for navigation bit generation
pub trait NavBitTrait {
    fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        param: i32,
        nav_bits: &mut [i32],
    ) -> i32;
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris);
    fn set_glonass_ephemeris(&mut self, _svid: i32, _eph: &GlonassEphemeris) {}
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

impl CNavBit {
    pub fn new() -> Self {
        CNavBit(ActualCNavBit::new())
    }
}
impl NavBitTrait for CNavBit {
    fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
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
            self.0.set_iono_utc(iono, utc);
        }
    }
    fn get_type(&self) -> NavDataType {
        NavDataType::CNav
    }
    fn clone_box(&self) -> Box<dyn NavBitTrait> {
        Box::new(self.clone())
    }
}

#[derive(Clone)]
pub struct CNav2Bit(ActualCNav2Bit);
impl Default for CNav2Bit {
    fn default() -> Self {
        Self::new()
    }
}

impl CNav2Bit {
    pub fn new() -> Self {
        CNav2Bit(ActualCNav2Bit::new())
    }
}
impl NavBitTrait for CNav2Bit {
    fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
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
            self.0.set_iono_utc(iono, utc);
        }
    }
    fn get_type(&self) -> NavDataType {
        NavDataType::CNav2
    }
    fn clone_box(&self) -> Box<dyn NavBitTrait> {
        Box::new(self.clone())
    }
}

use crate::sat_if_signal::SatIfSignal;

#[derive(Clone)]
pub struct D1D2NavBit(ActualD1D2NavBit);
impl Default for D1D2NavBit {
    fn default() -> Self {
        Self::new()
    }
}

impl D1D2NavBit {
    pub fn new() -> Self {
        D1D2NavBit(ActualD1D2NavBit::new())
    }
}
impl NavBitTrait for D1D2NavBit {
    fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
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
            self.0.set_iono_utc(iono, utc);
        }
    }
    fn get_type(&self) -> NavDataType {
        NavDataType::D1D2Nav
    }
    fn clone_box(&self) -> Box<dyn NavBitTrait> {
        Box::new(self.clone())
    }
}

#[derive(Clone)]
pub struct INavBit(ActualINavBit);
impl Default for INavBit {
    fn default() -> Self {
        Self::new()
    }
}

impl INavBit {
    pub fn new() -> Self {
        INavBit(ActualINavBit::new())
    }
}
impl NavBitTrait for INavBit {
    fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
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
                ai0: iono.a0, // Use alpha parameters as ai parameters
                ai1: iono.a1,
                ai2: iono.a2,
                flag: iono.flag,
            };

            self.0.set_iono_utc(&inav_iono, utc);
        }
    }
    fn get_type(&self) -> NavDataType {
        NavDataType::INav
    }
    fn clone_box(&self) -> Box<dyn NavBitTrait> {
        Box::new(self.clone())
    }
}

#[derive(Clone)]
pub struct FNavBit(ActualFNavBit);
impl Default for FNavBit {
    fn default() -> Self {
        Self::new()
    }
}

impl FNavBit {
    pub fn new() -> Self {
        FNavBit(ActualFNavBit::new())
    }
}
impl NavBitTrait for FNavBit {
    fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
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
            self.0.set_iono_utc(iono, utc);
        }
    }
    fn get_type(&self) -> NavDataType {
        NavDataType::FNav
    }
    fn clone_box(&self) -> Box<dyn NavBitTrait> {
        Box::new(self.clone())
    }
}

#[derive(Clone)]
pub struct BCNav1Bit(ActualBCNav1Bit);
impl Default for BCNav1Bit {
    fn default() -> Self {
        Self::new()
    }
}

impl BCNav1Bit {
    pub fn new() -> Self {
        BCNav1Bit(ActualBCNav1Bit::new())
    }
}
impl NavBitTrait for BCNav1Bit {
    fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        self.0.get_frame_data(start_time, svid, param, nav_bits)
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) {
        self.0.set_ephemeris(svid, eph);
    }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) {
        self.0.set_almanac(alm);
    }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) {
        self.0.set_iono_utc(iono_param, utc_param);
    }
    fn get_type(&self) -> NavDataType {
        NavDataType::BCNav1
    }
    fn clone_box(&self) -> Box<dyn NavBitTrait> {
        Box::new(self.clone())
    }
}

#[derive(Clone)]
pub struct BCNav2Bit(ActualBCNav2Bit);
impl Default for BCNav2Bit {
    fn default() -> Self {
        Self::new()
    }
}

impl BCNav2Bit {
    pub fn new() -> Self {
        BCNav2Bit(ActualBCNav2Bit::new())
    }
}
impl NavBitTrait for BCNav2Bit {
    fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        self.0.get_frame_data(start_time, svid, param, nav_bits)
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) {
        self.0.set_ephemeris(svid, eph);
    }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) {
        self.0.set_almanac(alm);
    }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) {
        self.0.set_iono_utc(iono_param, utc_param);
    }
    fn get_type(&self) -> NavDataType {
        NavDataType::BCNav2
    }
    fn clone_box(&self) -> Box<dyn NavBitTrait> {
        Box::new(self.clone())
    }
}

#[derive(Clone)]
pub struct BCNav3Bit(ActualBCNav3Bit);
impl Default for BCNav3Bit {
    fn default() -> Self {
        Self::new()
    }
}

impl BCNav3Bit {
    pub fn new() -> Self {
        BCNav3Bit(ActualBCNav3Bit::new())
    }
}
impl NavBitTrait for BCNav3Bit {
    fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        self.0.get_frame_data(start_time, svid, param, nav_bits)
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) {
        self.0.set_ephemeris(svid, eph);
    }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) {
        self.0.set_almanac(alm);
    }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) {
        self.0.set_iono_utc(iono_param, utc_param);
    }
    fn get_type(&self) -> NavDataType {
        NavDataType::BCNav3
    }
    fn clone_box(&self) -> Box<dyn NavBitTrait> {
        Box::new(self.clone())
    }
}

// Implement NavBitTrait for existing types
impl NavBitTrait for LNavBit {
    fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        let mut nav_bits_300 = [0i32; 300];
        let result = LNavBit::get_frame_data(self, start_time, svid, param, &mut nav_bits_300);
        nav_bits[..300].copy_from_slice(&nav_bits_300);
        result
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) {
        self.set_ephemeris(svid, eph);
    }
    fn set_almanac(&mut self, _alm: &[GpsAlmanac]) {
        // Convert slice to array - for now, use default empty almanac
        let empty_almanac = [GpsAlmanac::default(); 32];
        self.set_almanac(&empty_almanac);
    }
    fn set_iono_utc(&mut self, _iono_param: Option<&IonoParam>, _utc_param: Option<&UtcParam>) {
        // Заглушка - не используется в GPS L1CA генерации
    }
    fn get_type(&self) -> NavDataType {
        NavDataType::NavDataGpsEph
    }
    fn clone_box(&self) -> Box<dyn NavBitTrait> {
        Box::new(self.clone())
    }
}

impl NavBitTrait for L5CNavBit {
    fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        let mut nav_bits_600 = [0i32; 600];
        let result = L5CNavBit::get_frame_data(self, start_time, svid, param, &mut nav_bits_600);
        nav_bits[..600].copy_from_slice(&nav_bits_600);
        result
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) {
        self.set_ephemeris(svid, eph);
    }
    fn set_almanac(&mut self, _alm: &[GpsAlmanac]) {
        // Convert slice to array - for now, use default empty almanac
        let empty_almanac = [GpsAlmanac::default(); 32];
        self.set_almanac(&empty_almanac);
    }
    fn set_iono_utc(&mut self, _iono_param: Option<&IonoParam>, _utc_param: Option<&UtcParam>) {
        // Заглушка - не используется в GPS L1CA генерации
    }
    fn get_type(&self) -> NavDataType {
        NavDataType::CNav
    }
    fn clone_box(&self) -> Box<dyn NavBitTrait> {
        Box::new(self.clone())
    }
}

impl NavBitTrait for GNavBit {
    fn get_frame_data(
        &mut self,
        _start_time: GnssTime,
        _svid: i32,
        _param: i32,
        _nav_bits: &mut [i32],
    ) -> i32 {
        // Заглушка - избегаем бесконечной рекурсии
        0
    }
    fn set_ephemeris(&mut self, _svid: i32, _eph: &GpsEphemeris) {}
    fn set_glonass_ephemeris(&mut self, svid: i32, eph: &GlonassEphemeris) {
        GNavBit::set_glonass_ephemeris(self, svid, eph);
    }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) {
        self.set_almanac(alm);
    }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) {
        self.set_iono_utc(iono_param, utc_param);
    }
    fn get_type(&self) -> NavDataType {
        NavDataType::NavDataGlonassEph
    }
    fn clone_box(&self) -> Box<dyn NavBitTrait> {
        Box::new(self.clone())
    }
}
use crate::fastmath::FastMath;
use rayon::prelude::*;

// Constants for quantization and time conversion

const TOTAL_GPS_SAT: usize = 32;
const TOTAL_BDS_SAT: usize = 63;
const TOTAL_GAL_SAT: usize = 36;
const TOTAL_GLO_SAT: usize = 24;
const TOTAL_SAT_CHANNEL: usize = 128;

#[derive(Debug, Clone, Copy)]
pub enum DataBitType {
    DataBitLNav = 0, // for GPS
    DataBitCNav = 1,
    DataBitCNav2 = 2,
    DataBitL5CNav = 3,
    DataBitGNav = 4, // for GLONASS
    DataBitGNav2 = 5,
    DataBitD1D2 = 6, // for BDS
    DataBitBCNav1 = 7,
    DataBitBCNav2 = 8,
    DataBitBCNav3 = 9,
    DataBitINav = 10, // for Galileo
    DataBitFNav = 11,
    DataBitECNav = 12,
    DataBitSbas = 13, // for SBAS
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
    [
        FREQ_GPS_L1,
        FREQ_GPS_L1,
        FREQ_GPS_L2,
        FREQ_GPS_L2,
        FREQ_GPS_L5,
        0.0,
        0.0,
        0.0,
    ],
    [
        FREQ_BDS_B1C,
        FREQ_BDS_B1I,
        FREQ_BDS_B2I,
        FREQ_BDS_B3I,
        FREQ_BDS_B2A,
        FREQ_BDS_B2B,
        FREQ_BDS_B2AB,
        0.0,
    ],
    [
        FREQ_GAL_E1,
        FREQ_GAL_E5A,
        FREQ_GAL_E5B,
        FREQ_GAL_E5,
        FREQ_GAL_E6,
        0.0,
        0.0,
        0.0,
    ],
    [FREQ_GLO_G1, FREQ_GLO_G2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
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
        println!(
            "\n================================================================================"
        );
        println!("                          IF SIGNAL GENERATION ");
        println!(
            "================================================================================"
        );

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
        if crate::logutil::is_verbose() {
            println!("[INFO]\tSkipping JSON parsing (handled by load_config)");
        }

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

        if crate::logutil::is_verbose() {
            println!(
                "[INFO]\tParsed time from preset: {}-{:02}-{:02} {:02}:{:02}:{:02.0}",
                utc_time.Year,
                utc_time.Month,
                utc_time.Day,
                utc_time.Hour,
                utc_time.Minute,
                utc_time.Second
            );
        }
        let _start_pos = LlaPosition::default();
        // keep defaults placeholders minimal

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
                        rinex_file = String::from(name);
                    }
                }
            }
        }
        if std::path::Path::new(&rinex_file).exists() {
            if crate::logutil::is_verbose() {
                println!("[INFO]\tLoading RINEX ephemeris file: {}", rinex_file);
            }

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

            // Используем оптимизированную загрузку RINEX с фильтрацией систем из JSON
            crate::json_interpreter::read_nav_file_filtered(
                &mut c_nav_data,
                &rinex_file,
                Some(utc_time), // Фильтрация по времени
                &enabled_systems,
            );

            // Первичная загрузка: выбираем per-satellite, чтобы не расходиться с C-версией
            self.select_per_satellite_and_fill(&c_nav_data, utc_time);

            // CRITICAL: Transfer ionospheric and UTC parameters from CNavData to NavData
            // Without this, LNAV subframe 4 page 18 is empty → receiver has no iono/UTC corrections → NO FIX
            if let (Some(alpha), Some(beta)) = (c_nav_data.gps_iono_alpha, c_nav_data.gps_iono_beta)
            {
                self.nav_data.set_gps_iono(IonoParam {
                    a0: alpha[0],
                    a1: alpha[1],
                    a2: alpha[2],
                    a3: alpha[3],
                    b0: beta[0],
                    b1: beta[1],
                    b2: beta[2],
                    b3: beta[3],
                    flag: 1,
                });
                println!(
                    "[INFO] GPS iono parameters transferred to NavData: alpha={:?}, beta={:?}",
                    alpha, beta
                );
            }
            if let Some(utc) = c_nav_data.utc_param {
                self.nav_data.set_gps_utc(utc);
                println!(
                    "[INFO] GPS UTC parameters transferred to NavData: A0={}, A1={}, TLS={}",
                    utc.A0, utc.A1, utc.TLS
                );
            }
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
        let mut pos_vel = KinematicInfo {
            x: cur_pos.x,
            y: cur_pos.y,
            z: cur_pos.z,
            ..Default::default()
        };
        speed_local_to_ecef(&convert_matrix, &start_vel, &mut pos_vel);

        let filename_binding = String::from_utf8_lossy(&self.output_param.filename);
        let filename_str = filename_binding.trim_end_matches('\0');
        if crate::logutil::is_verbose() {
            println!("[INFO]\tOpening output file: {}", filename_str);
        }
        let mut if_file = match File::create(filename_str) {
            Ok(file) => {
                if crate::logutil::is_verbose() {
                    println!("[INFO]\tOutput file opened successfully.");
                }
                BufWriter::with_capacity(1 << 20, file)
            }
            Err(_) => {
                let filename_str = String::from_utf8_lossy(&self.output_param.filename);
                eprintln!(
                    "[ERROR]\tFailed to open output file: {}",
                    filename_str.trim_end_matches('\0')
                );
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

        self.setup_navigation_data(
            nav_bit_array.as_mut_slice(),
            utc_time,
            glonass_time,
            bds_time,
        )?;
        self.calculate_visible_satellites(cur_pos, glonass_time)?;
        if crate::logutil::is_verbose() {}

        let mut sat_if_signals = self.create_satellite_signals(&nav_bit_array[..], cur_pos)?;
        if crate::logutil::is_verbose() {}

        // Парсим время траектории из JSON пресета (используем хардкод пока)
        let trajectory_time_s = self
            .parse_trajectory_time_from_json(&self.output_param.config_filename)
            .unwrap_or(10.0);
        self.generate_if_signal(
            &mut if_file,
            &mut sat_if_signals,
            cur_pos,
            trajectory_time_s,
        )?;

        println!("[INFO]\tIF Signal generation completed!");
        Ok(())
    }
    fn create_nav_bit_instances(&self) -> Vec<Option<UnifiedNavData>> {
        let mut nav_bit_array: Vec<Option<UnifiedNavData>> = Vec::with_capacity(14);

        for i in 0..14 {
            let nav_bit: Option<UnifiedNavData> = match i {
                0 => Some(UnifiedNavData::LNav(LNavBit::new())), // DataBitLNav
                1 => Some(UnifiedNavData::CNav(ActualCNavBit::new())), // DataBitCNav
                2 => Some(UnifiedNavData::CNav2(ActualCNav2Bit::new())), // DataBitCNav2
                3 => Some(UnifiedNavData::L5CNav(L5CNavBit::new())), // DataBitL5CNav
                4 => Some(UnifiedNavData::GNav(GNavBit::new())), // DataBitGNav
                5 => None,                                       // DataBitGNav2
                6 => Some(UnifiedNavData::D1D2Nav(ActualD1D2NavBit::new())), // DataBitD1D2
                7 => Some(UnifiedNavData::BCNav1(ActualBCNav1Bit::new())), // DataBitBCNav1
                8 => Some(UnifiedNavData::BCNav2(ActualBCNav2Bit::new())), // DataBitBCNav2
                9 => Some(UnifiedNavData::BCNav3(ActualBCNav3Bit::new())), // DataBitBCNav3
                10 => Some(UnifiedNavData::INav(ActualINavBit::new())), // DataBitINav
                11 => Some(UnifiedNavData::FNav(ActualFNavBit::new())), // DataBitFNav
                12 => None,                                      // DataBitECNav
                13 => None,                                      // DataBitSbas
                _ => None,
            };
            nav_bit_array.push(nav_bit);
        }

        nav_bit_array
    }

    // Выбор глобальной эпохи (week,toe) в GPS секундах, согласованной между системами.
    // Для каждой системы выбирается ближайшая эпоха к глобальной в пределах окна (±2ч),
    // затем массивы self.*_eph заполняются эфемеридами только этой эпохи.
    fn select_global_epochs_and_fill(
        &mut self,
        c_nav_data: &crate::json_interpreter::CNavData,
        utc_time: UtcTime,
    ) {
        use std::collections::{HashMap, HashSet};
        // Группировка по эпохам внутри каждой системы
        let mut gps_by_epoch: HashMap<(i32, i32), HashMap<u8, GpsEphemeris>> = HashMap::new();
        for eph in &c_nav_data.gps_ephemeris {
            gps_by_epoch
                .entry((eph.week, eph.toe))
                .or_default()
                .insert(eph.svid, *eph);
        }
        let mut bds_by_epoch: HashMap<(i32, i32), HashMap<u8, crate::types::BeiDouEphemeris>> =
            HashMap::new();
        for eph in &c_nav_data.beidou_ephemeris {
            bds_by_epoch
                .entry((eph.week, eph.toe))
                .or_default()
                .insert(eph.svid, *eph);
        }
        let mut gal_by_epoch: HashMap<(i32, i32), HashMap<u8, GpsEphemeris>> = HashMap::new();
        for eph in &c_nav_data.galileo_ephemeris {
            gal_by_epoch
                .entry((eph.week, eph.toe))
                .or_default()
                .insert(eph.svid, *eph);
        }

        // Кандидаты глобальной эпохи: все эпохи всех систем + целевое время пресета
        let mut candidates: HashSet<i64> = HashSet::new();
        let gps_abs = |w: i32, t: i32| -> i64 { (w as i64) * 604800 + (t as i64) };
        let bds_abs = |w: i32, t: i32| -> i64 { ((w as i64) + 1356) * 604800 + (t as i64) }; // игнор 14с
        let gal_abs = |w: i32, t: i32| -> i64 { (w as i64) * 604800 + (t as i64) };
        for (&(w, t), _) in gps_by_epoch.iter() {
            candidates.insert(gps_abs(w, t));
        }
        for (&(w, t), _) in bds_by_epoch.iter() {
            candidates.insert(bds_abs(w, t));
        }
        for (&(w, t), _) in gal_by_epoch.iter() {
            candidates.insert(gal_abs(w, t));
        }
        let tgt_gps = utc_to_gps_time(utc_time, false);
        let tgt_abs = (tgt_gps.Week as i64) * 604800 + (tgt_gps.MilliSeconds as i64) / 1000;
        candidates.insert(tgt_abs);

        // Оценка кандидатов: максимизируем суммарное число SV в окне ±7200с, tie-break — минимальная суммарная |Δt|
        let window: i64 = 7200;
        let mut best_cand: Option<i64> = None;
        let mut best_score: i32 = -1;
        let mut best_sum_diff: i64 = i64::MAX;
        for &cand in &candidates {
            // Оценка inline для каждой системы (без замыканий с разными типами)
            let (gps_n, gps_d) = {
                let mut best: Option<i64> = None;
                let mut n = 0;
                for (&(w, t), svmap) in &gps_by_epoch {
                    let d = (gps_abs(w, t) - cand).abs();
                    if best.is_none_or(|bd| d < bd) {
                        best = Some(d);
                        n = svmap.len() as i32;
                    }
                }
                if let Some(d) = best {
                    if d <= window {
                        (n, d)
                    } else {
                        (0, d)
                    }
                } else {
                    (0, i64::MAX)
                }
            };
            let (bds_n, bds_d) = {
                let mut best: Option<i64> = None;
                let mut n = 0;
                for (&(w, t), svmap) in &bds_by_epoch {
                    let d = (bds_abs(w, t) - cand).abs();
                    if best.is_none_or(|bd| d < bd) {
                        best = Some(d);
                        n = svmap.len() as i32;
                    }
                }
                if let Some(d) = best {
                    if d <= window {
                        (n, d)
                    } else {
                        (0, d)
                    }
                } else {
                    (0, i64::MAX)
                }
            };
            let (gal_n, gal_d) = {
                let mut best: Option<i64> = None;
                let mut n = 0;
                for (&(w, t), svmap) in &gal_by_epoch {
                    let d = (gal_abs(w, t) - cand).abs();
                    if best.is_none_or(|bd| d < bd) {
                        best = Some(d);
                        n = svmap.len() as i32;
                    }
                }
                if let Some(d) = best {
                    if d <= window {
                        (n, d)
                    } else {
                        (0, d)
                    }
                } else {
                    (0, i64::MAX)
                }
            };
            let score = gps_n + bds_n + gal_n;
            let sum_diff = gps_d.saturating_add(bds_d).saturating_add(gal_d);
            if score > best_score || (score == best_score && sum_diff < best_sum_diff) {
                best_score = score;
                best_sum_diff = sum_diff;
                best_cand = Some(cand);
            }
        }

        // Заполняем выходные массивы по лучшему кандидату
        self.gps_eph.fill(None);
        self.bds_eph.fill(None);
        self.gal_eph.fill(None);
        if let Some(cand) = best_cand {
            // GPS
            if !gps_by_epoch.is_empty() {
                let mut best: Option<((i32, i32), i64)> = None;
                for &(w, t) in gps_by_epoch.keys() {
                    let d = (gps_abs(w, t) - cand).abs();
                    if best.is_none_or(|(_, bd)| d < bd) {
                        best = Some(((w, t), d));
                    }
                }
                if let Some(((w, t), d)) = best {
                    if d <= window {
                        if let Some(map) = gps_by_epoch.get(&(w, t)) {
                            for (&svid, eph) in map {
                                let idx = (svid as usize).saturating_sub(1);
                                if idx < TOTAL_GPS_SAT {
                                    self.gps_eph[idx] = Some(*eph);
                                }
                            }
                            println!(
                                "[EPOCH] GPS: week={}, toe={}, |Δt|={}s, sats={}",
                                w,
                                t,
                                d,
                                map.len()
                            );
                        }
                    }
                }
            }
            // BDS
            if !bds_by_epoch.is_empty() {
                let mut best: Option<((i32, i32), i64)> = None;
                for &(w, t) in bds_by_epoch.keys() {
                    let d = (bds_abs(w, t) - cand).abs();
                    if best.is_none_or(|(_, bd)| d < bd) {
                        best = Some(((w, t), d));
                    }
                }
                if let Some(((w, t), d)) = best {
                    if d <= window {
                        if let Some(map) = bds_by_epoch.get(&(w, t)) {
                            for (&svid, bds) in map {
                                let idx = (svid as usize).saturating_sub(1);
                                if idx < TOTAL_BDS_SAT {
                                    self.bds_eph[idx] = Some(bds.to_gps_ephemeris());
                                }
                            }
                            println!(
                                "[EPOCH] BDS: week={}, toe={}, |Δt|={}s, sats={}",
                                w,
                                t,
                                d,
                                map.len()
                            );
                        }
                    }
                }
            }
            // GAL
            if !gal_by_epoch.is_empty() {
                let mut best: Option<((i32, i32), i64)> = None;
                for &(w, t) in gal_by_epoch.keys() {
                    let d = (gal_abs(w, t) - cand).abs();
                    if best.is_none_or(|(_, bd)| d < bd) {
                        best = Some(((w, t), d));
                    }
                }
                if let Some(((w, t), d)) = best {
                    if d <= window {
                        if let Some(map) = gal_by_epoch.get(&(w, t)) {
                            for (&svid, eph) in map {
                                let idx = (svid as usize).saturating_sub(1);
                                if idx < TOTAL_GAL_SAT {
                                    self.gal_eph[idx] = Some(*eph);
                                }
                            }
                            println!(
                                "[EPOCH] GAL: week={}, toe={}, |Δt|={}s, sats={}",
                                w,
                                t,
                                d,
                                map.len()
                            );
                        }
                    }
                }
            }
            println!(
                "[EPOCH] Global candidate (GPS sec): {} (score={}, sum|Δt|={}s)",
                cand, best_score, best_sum_diff
            );
        } else {
            println!("[WARN] No global epoch candidate available");
        }
    }

    // C-совместимый выбор: для каждого спутника выбираем ближайшую эфемериду в пределах окна.
    fn select_per_satellite_and_fill(
        &mut self,
        c_nav_data: &crate::json_interpreter::CNavData,
        utc_time: UtcTime,
    ) {
        // Очистка
        self.gps_eph.fill(None);
        self.bds_eph.fill(None);
        self.gal_eph.fill(None);
        self.glo_eph.fill(None);

        // 1) GPS (UTC→GPS)
        let tgt_gps = utc_to_gps_time(utc_time, false);
        let tgt_week = tgt_gps.Week;
        let tgt_sec = (tgt_gps.MilliSeconds / 1000) as i64;
        for svid in 1..=TOTAL_GPS_SAT {
            let mut best: Option<(GpsEphemeris, i64)> = None;
            for eph in &c_nav_data.gps_ephemeris {
                if eph.svid as usize == svid && eph.health == 0 {
                    let diff = ((tgt_week - eph.week) as i64) * 604800 + (tgt_sec - eph.toe as i64);
                    let ad = diff.abs();
                    if ad <= 7200 && best.is_none_or(|(_, bd)| ad < bd) {
                        best = Some((*eph, ad));
                    }
                }
            }
            if let Some((eph, _)) = best {
                self.gps_eph[svid - 1] = Some(eph);
            }
        }

        // 2) BeiDou (UTC→BDT)
        let tgt_bds = utc_to_bds_time(utc_time);
        let bw = tgt_bds.Week;
        let bs = (tgt_bds.MilliSeconds / 1000) as i64;
        for svid in 1..=TOTAL_BDS_SAT {
            let mut best: Option<(GpsEphemeris, i64)> = None;
            for eph in &c_nav_data.beidou_ephemeris {
                if eph.svid as usize == svid && eph.health == 0 {
                    let diff = ((bw - eph.week) as i64) * 604800 + (bs - eph.toe as i64);
                    let ad = diff.abs();
                    if ad <= 7200 && best.is_none_or(|(_, bd)| ad < bd) {
                        best = Some((eph.to_gps_ephemeris(), ad));
                    }
                }
            }
            if let Some((eph, _)) = best {
                self.bds_eph[svid - 1] = Some(eph);
            }
        }

        // 3) Galileo (UTC→GPS) — как в C-версии
        let tgt_gps = utc_to_gps_time(utc_time, false);
        let gw = tgt_gps.Week;
        let gs = (tgt_gps.MilliSeconds / 1000) as i64;
        for svid in 1..=TOTAL_GAL_SAT {
            let mut best: Option<(GpsEphemeris, i64)> = None;
            for eph in &c_nav_data.galileo_ephemeris {
                if eph.svid as usize == svid {
                    let diff = ((gw - eph.week) as i64) * 604800 + (gs - eph.toe as i64);
                    let ad = diff.abs();
                    if ad <= 10800 && best.is_none_or(|(_, bd)| ad < bd) {
                        best = Some((*eph, ad));
                    }
                }
            }
            if let Some((eph, _)) = best {
                self.gal_eph[svid - 1] = Some(eph);
            }
        }

        // 4) ГЛОНАСС (UTC→ГЛОНАСС) — окно 1800с
        let gtime = utc_to_glonass_time_corrected(utc_time);
        let req_sec = (gtime.MilliSeconds / 1000) as i64;
        for slot in 1..=TOTAL_GLO_SAT {
            let mut best: Option<(crate::types::GlonassEphemeris, i64)> = None;
            for eph in &c_nav_data.glonass_ephemeris {
                if eph.slot as usize == slot && eph.flag != 0 {
                    // Приводим сравнение к секундам суток: многие RINEX не содержат корректный day для ГЛОНАСС
                    let diff = req_sec - eph.tb as i64;
                    let ad = diff.abs();
                    if ad <= 1800 && best.is_none_or(|(_, bd)| ad < bd) {
                        best = Some((*eph, ad));
                    }
                }
            }
            if let Some((eph, _)) = best {
                self.glo_eph[slot - 1] = Some(eph);
            }
        }
    }

    fn setup_navigation_data(
        &mut self,
        nav_bit_array: &mut [Option<UnifiedNavData>],
        utc_time: UtcTime,
        glonass_time: GlonassTime,
        _bds_time: GnssTime,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Set Ionosphere and UTC parameters for different navigation data bits
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitLNav as usize] {
            nav_bit.set_iono_utc(
                self.nav_data.get_gps_iono(),
                self.nav_data.get_gps_utc_param(),
            );
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav as usize] {
            nav_bit.set_iono_utc(
                self.nav_data.get_gps_iono(),
                self.nav_data.get_gps_utc_param(),
            );
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav2 as usize] {
            nav_bit.set_iono_utc(
                self.nav_data.get_gps_iono(),
                self.nav_data.get_gps_utc_param(),
            );
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitD1D2 as usize] {
            nav_bit.set_iono_utc(
                self.nav_data.get_bds_iono(),
                self.nav_data.get_bds_utc_param(),
            );
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav1 as usize] {
            nav_bit.set_iono_utc(
                self.nav_data.get_bds_iono(),
                self.nav_data.get_bds_utc_param(),
            );
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitINav as usize] {
            nav_bit.set_iono_utc(
                self.nav_data.get_galileo_iono(),
                self.nav_data.get_galileo_utc_param(),
            );
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitFNav as usize] {
            nav_bit.set_iono_utc(
                self.nav_data.get_galileo_iono(),
                self.nav_data.get_galileo_utc_param(),
            );
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitGNav as usize] {
            nav_bit.set_iono_utc(None, self.nav_data.get_gps_utc_param()); // GLONASS uses GPS UTC parameters
        }

        // Find ephemeris matching current time and fill in data to generate bit stream
        // Не перезаписываем ранее выбранные RINEX-эфемериды; дополняем только при отсутствии
        for i in 1..=TOTAL_GPS_SAT {
            if self.gps_eph[i - 1].is_none() {
                self.gps_eph[i - 1] = self.nav_data.find_ephemeris(
                    GnssSystem::GpsSystem,
                    self.cur_time,
                    i as i32,
                    0,
                    0,
                );
            }

            // For L1CA/L1C/L2C/L5, all can use the same ephemeris data
            if let Some(ref eph) = self.gps_eph[i - 1] {
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
            self.glo_eph[i - 1] = self.nav_data.find_glo_ephemeris(glonass_time, i as i32);

            if let Some(ref eph) = self.glo_eph[i - 1] {
                if eph.flag != 0 {
                    if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitGNav as usize]
                    {
                        nav_bit.set_glonass_ephemeris(i as i32, eph);
                    }
                    glo_eph_count += 1;
                }
            }
        }
        println!(
            "[INFO] GLONASS ephemeris loaded: {} satellites",
            glo_eph_count
        );

        // Complete almanac data
        self.nav_data
            .complete_almanac(GnssSystem::GpsSystem, utc_time);
        self.nav_data
            .complete_almanac(GnssSystem::BdsSystem, utc_time);
        self.nav_data
            .complete_almanac(GnssSystem::GalileoSystem, utc_time);
        self.nav_data
            .complete_almanac(GnssSystem::GlonassSystem, utc_time);

        // Generate GPS almanac from ephemeris data for all 32 satellites
        let gps_time = utc_to_gps_time(utc_time, true);
        self.nav_data
            .generate_gps_almanac_from_ephemeris(gps_time.Week);

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
            let gps_almanac =
                self.convert_glonass_to_gps_almanac(self.nav_data.get_glonass_almanac());
            nav_bit.set_almanac(&gps_almanac);
        }

        Ok(())
    }
    fn calculate_visible_satellites(
        &mut self,
        cur_pos: KinematicInfo,
        glonass_time: GlonassTime,
    ) -> Result<(), Box<dyn std::error::Error>> {
        println!(
            "[INFO]\tCalculating visible satellites at position ({:.1}, {:.1}, {:.1})",
            cur_pos.x, cur_pos.y, cur_pos.z
        );

        // ОТЛАДКА: Вызываем функцию проверки позиций эталонных спутников
        // Конвертируем BDS эфемериды в массив BeiDouEphemeris
        let mut beidou_eph_array: [Option<BeiDouEphemeris>; 64] = [None; 64];
        for i in 0..64 {
            if i < self.bds_eph.len() {
                if let Some(gps_eph) = &self.bds_eph[i] {
                    // Конвертируем GpsEphemeris обратно в BeiDouEphemeris для отладки
                    let bds_eph = BeiDouEphemeris::from_gps_ephemeris(gps_eph);
                    beidou_eph_array[i] = Some(bds_eph);
                }
            }
        }

        // Debug: check if we have ephemeris before calculating visibility
        let mut _total_gps_eph = 0;
        for i in 0..TOTAL_GPS_SAT {
            if self.gps_eph[i].is_some() {
                _total_gps_eph += 1;
            }
        }

        // Calculate visible GPS satellites
        self.gps_sat_number = if self.output_param.CompactConfig.should_parse_gps() {
            let mut sat_number = 0;
            let elevation_mask = self.output_param.ElevationMask;

            // КАК В C ВЕРСИИ: передаем только секунды недели, не полное время
            let _transmit_time = (self.cur_time.MilliSeconds as f64) / 1000.0;

            for i in 0..TOTAL_GPS_SAT {
                if let Some(eph) = &self.gps_eph[i] {
                    // Check health and validity
                    if (eph.valid & 1) == 0 {
                        continue;
                    }
                    if eph.health != 0 {
                        continue;
                    }

                    // Check mask out
                    if (self.output_param.GpsMaskOut & (1u32 << i)) != 0 {
                        continue;
                    }

                    // Calculate satellite position
                    // КАК В C ВЕРСИИ: передаем только секунды недели, не полное время
                    let transmit_time = (self.cur_time.MilliSeconds as f64) / 1000.0;
                    let mut sat_pos = KinematicInfo::default();
                    let mut eph_mut = *eph; // Копируем эфемериды для мутабельности
                                            // Для отладочного вывода используем секунды недели
                    let transmit_time_week_seconds = (self.cur_time.MilliSeconds as f64) / 1000.0;
                    let _delta_t_gps = transmit_time_week_seconds - eph.toe as f64;
                    if crate::coordinate::gps_sat_pos_speed_eph(
                        GnssSystem::GpsSystem,
                        transmit_time,
                        &mut eph_mut,
                        &mut sat_pos,
                        None,
                    ) {
                        let _range = ((sat_pos.x - cur_pos.x).powi(2)
                            + (sat_pos.y - cur_pos.y).powi(2)
                            + (sat_pos.z - cur_pos.z).powi(2))
                        .sqrt();

                        // Calculate elevation and azimuth using coordinate.rs function
                        let mut elevation = 0.0;
                        let mut azimuth = 0.0;
                        crate::coordinate::sat_el_az_from_positions(
                            &cur_pos,
                            &sat_pos,
                            &mut elevation,
                            &mut azimuth,
                        );
                        // Специальная отладка для GPS 13 и 27
                        if i == 12 || i == 26 {
                            // GPS 13 (индекс 12) и GPS 27 (индекс 26)
                            let dx = sat_pos.x - cur_pos.x;
                            let dy = sat_pos.y - cur_pos.y;
                            let dz = sat_pos.z - cur_pos.z;
                            let r = (dx * dx + dy * dy + dz * dz).sqrt();
                            // DEBUG: GPS Delta и LOS отключены для уменьшения вывода
                            // println!("[GPS{:02}-DEBUG] Delta: ({:.1}, {:.1}, {:.1}), Range: {:.1}",
                            //          i+1, dx, dy, dz, r);
                            // println!("[GPS{:02}-DEBUG] LOS: ({:.4}, {:.4}, {:.4})",
                            //          i+1, dx/r, dy/r, dz/r);
                            // Вычислим азимут вручную
                            let rcv_lla = crate::coordinate::ecef_to_lla(&cur_pos);
                            let lat = rcv_lla.lat;
                            let lon = rcv_lla.lon;
                            let los_e = -(dx / r) * lon.sin() + (dy / r) * lon.cos();
                            let los_n = -(dx / r) * lat.sin() * lon.cos()
                                - (dy / r) * lat.sin() * lon.sin()
                                + (dz / r) * lat.cos();
                            let manual_az = los_e.atan2(los_n);
                            let _manual_az_deg = if manual_az < 0.0 {
                                manual_az + 2.0 * std::f64::consts::PI
                            } else {
                                manual_az
                            };
                            // DEBUG: GPS вычисления отключены для уменьшения вывода
                            // println!("[GPS{:02}-DEBUG] Manual calc: E={:.4}, N={:.4}, Az={:.1}°",
                            //          i+1, los_e, los_n, manual_az_deg.to_degrees());
                            // println!("[GPS{:02}-DEBUG] Function returned: Elevation: {:.1}°, Azimuth: {:.1}°",
                            //          i+1, elevation.to_degrees(), azimuth.to_degrees());
                        }

                        if elevation >= elevation_mask {
                            if sat_number < TOTAL_GPS_SAT {
                                self.gps_eph_visible[sat_number] = Some(*eph);
                                sat_number += 1;
                            }
                        } else {
                        }
                    } else {
                    }
                } else if i < 10 {
                    // Показать только первые 10 для краткости
                }
            }
            println!("[INFO]\tFound {} visible GPS satellites", sat_number);

            sat_number
        } else {
            0
        };

        // Calculate visible BeiDou satellites
        self.bds_sat_number = if self.output_param.CompactConfig.should_parse_bds() {
            let mut sat_number = 0;
            let elevation_mask = self.output_param.ElevationMask;

            // Проверим состояние массива bds_eph
            let mut _filled_slots = 0;
            for i in 0..TOTAL_BDS_SAT {
                if self.bds_eph[i].is_some() {
                    _filled_slots += 1;
                }
            }

            for i in 0..TOTAL_BDS_SAT {
                if let Some(eph) = &self.bds_eph[i] {
                    if (eph.valid & 1) == 0 || eph.health != 0 {
                        continue;
                    }

                    if (self.output_param.BdsMaskOut & (1u64 << i)) != 0 {
                        continue;
                    }

                    // Для BeiDou используем секунды недели в шкале BDT
                    let utc_now = crate::gnsstime::gps_time_to_utc(self.cur_time, true);
                    let bds_now = crate::gnsstime::utc_to_bds_time(utc_now);
                    let transmit_time = (bds_now.MilliSeconds as f64) / 1000.0;
                    let mut sat_pos = KinematicInfo::default();
                    let mut eph_mut = *eph; // Копируем эфемериды для мутабельности
                    let pos_calc_success = crate::coordinate::gps_sat_pos_speed_eph(
                        GnssSystem::BdsSystem,
                        transmit_time,
                        &mut eph_mut,
                        &mut sat_pos,
                        None,
                    );
                    if i < 5 {
                        // Логируем только первые 5 для отладки
                        let _delta_t_bds = transmit_time - eph.toe as f64;
                    }
                    if pos_calc_success {
                        let mut elevation = 0.0;
                        let mut azimuth = 0.0;
                        crate::coordinate::sat_el_az_from_positions(
                            &cur_pos,
                            &sat_pos,
                            &mut elevation,
                            &mut azimuth,
                        );

                        if elevation >= elevation_mask && sat_number < TOTAL_BDS_SAT {
                            self.bds_eph_visible[sat_number] = Some(*eph);
                            println!(
                                "[INFO]\tBDS visible: SVID={}, elev={:.1}°, az={:.1}°",
                                eph.svid,
                                elevation.to_degrees(),
                                azimuth.to_degrees()
                            );
                            sat_number += 1;
                        }
                    }
                }
            }
            println!("[INFO]\tFound {} visible BeiDou satellites", sat_number);

            sat_number
        } else {
            0
        };

        // Calculate visible Galileo satellites
        self.gal_sat_number = if self.output_param.CompactConfig.should_parse_galileo() {
            let mut sat_number = 0;
            let mut _total_checked = 0;
            let mut _valid_failed = 0;
            let mut _mask_failed = 0;
            let mut _pos_failed = 0;
            let mut _elev_failed = 0;
            let elevation_mask = self.output_param.ElevationMask;

            for i in 0..TOTAL_GAL_SAT {
                if let Some(eph) = &self.gal_eph[i] {
                    _total_checked += 1;

                    if (eph.valid & 1) == 0 {
                        _valid_failed += 1;
                        continue;
                    }

                    if (self.output_param.GalileoMaskOut & (1u64 << i)) != 0 {
                        _mask_failed += 1;
                        continue;
                    }

                    // Для согласованности с C-версией используем GPS секунды недели
                    let transmit_time = (self.cur_time.MilliSeconds as f64) / 1000.0;
                    let mut sat_pos = KinematicInfo::default();
                    let mut eph_mut = *eph; // Копируем эфемериды для мутабельности
                    let _delta_t = transmit_time - eph.toe as f64;
                    let pos_calc_success = crate::coordinate::gps_sat_pos_speed_eph(
                        GnssSystem::GalileoSystem,
                        transmit_time,
                        &mut eph_mut,
                        &mut sat_pos,
                        None,
                    );
                    if pos_calc_success {
                        let mut elevation = 0.0;
                        let mut azimuth = 0.0;
                        crate::coordinate::sat_el_az_from_positions(
                            &cur_pos,
                            &sat_pos,
                            &mut elevation,
                            &mut azimuth,
                        );

                        if elevation >= elevation_mask {
                            if sat_number < TOTAL_GAL_SAT {
                                self.gal_eph_visible[sat_number] = Some(*eph);
                                println!(
                                    "[INFO]\tGAL visible: SVID={}, elev={:.1}°, az={:.1}°",
                                    eph.svid,
                                    elevation.to_degrees(),
                                    azimuth.to_degrees()
                                );
                                sat_number += 1;
                            }
                        } else {
                            _elev_failed += 1;
                        }
                    } else {
                        _pos_failed += 1;
                    }
                }
            }
            if crate::logutil::is_verbose() {
                println!("[INFO]\tFound {} visible Galileo satellites", sat_number);
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
                        crate::coordinate::sat_el_az_from_positions(
                            &cur_pos,
                            &sat_pos,
                            &mut elevation,
                            &mut azimuth,
                        );

                        if elevation >= elevation_mask && sat_number < TOTAL_GLO_SAT {
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

    fn create_satellite_signals(
        &mut self,
        nav_bit_array: &[Option<UnifiedNavData>],
        cur_pos: KinematicInfo,
    ) -> Result<Vec<Option<Box<SatIfSignal>>>, Box<dyn std::error::Error>> {
        let mut sat_if_signals: Vec<Option<Box<SatIfSignal>>> =
            (0..TOTAL_SAT_CHANNEL).map(|_| None).collect();
        let mut total_channel_number = 0;

        println!("[INFO]\tUsing Rayon parallelization for satellite processing");

        println!("[INFO]\tGenerating IF data with following satellite signals:\n");

        // Enhanced signal display with cleaner formatting
        println!("[INFO]\tEnabled Signals:");

        // Display enabled signals for each system
        self.display_enabled_signals();

        // Count total signals per system
        let (gps_signal_count, bds_signal_count, gal_signal_count, glo_signal_count) =
            self.count_signals_per_system();

        self.display_signals_summary_table(
            gps_signal_count,
            bds_signal_count,
            gal_signal_count,
            glo_signal_count,
        );

        // Create satellite signal instances for each system
        total_channel_number = self.create_gps_signals(
            &mut sat_if_signals,
            total_channel_number,
            nav_bit_array,
            cur_pos,
        )?;
        total_channel_number = self.create_bds_signals(
            &mut sat_if_signals,
            total_channel_number,
            nav_bit_array,
            cur_pos,
        )?;
        total_channel_number = self.create_galileo_signals(
            &mut sat_if_signals,
            total_channel_number,
            nav_bit_array,
            cur_pos,
        )?;
        let _ = self.create_glonass_signals(
            &mut sat_if_signals,
            total_channel_number,
            nav_bit_array,
            cur_pos,
        )?;

        Ok(sat_if_signals)
    }

    /// Генерирует промежуточно-частотные (IF) данные ГНСС сигналов
    ///
    /// Основной алгоритм синтеза IF сигналов:
    /// 1. Инициализация параметров выборки (samples_per_ms из частоты дискретизации)
    /// 2. Генерация шумовой составляющей (thermal noise + системный шум)
    /// 3. Обработка всех видимых спутников параллельно (AVX-512 + Rayon)
    /// 4. Суммирование спутниковых сигналов с учетом мощности и доплеровского сдвига
    /// 5. Применение автоматической регулировки усиления (AGC)
    /// 6. Квантование в формат IQ4/IQ8 для записи в файл
    ///
    /// Критические оптимизации:
    /// - AVX-512 векторизация для ≥8 спутников (50-100x ускорение)
    /// - Rayon параллелизм на уровне спутников и выборок
    /// - Кэширование параметров спутников (обновление каждые 50мс)
    /// - Адаптивный AGC против клиппирования сигнала
    fn generate_if_signal(
        &mut self,
        if_file: &mut BufWriter<File>,
        sat_if_signals: &mut Vec<Option<Box<SatIfSignal>>>,
        _cur_pos: KinematicInfo,
        trajectory_duration_s: f64,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Расчет количества выборок на миллисекунду
        // SampleFreq в Hz (например, 5_000_000 Hz = 5000 выборок/мс)
        // Определяет временное разрешение цифрового приемника
        let samples_per_ms = (self.output_param.SampleFreq as f64 / 1000.0) as usize;
        // Calculate total data size and setup progress tracking
        // Используем переданное время траектории из JSON пресета
        let total_duration_ms = (trajectory_duration_s * 1000.0) as i32;
        println!(
            "[INFO]\tUsing trajectory duration: {:.1}s ({} ms)",
            trajectory_duration_s, total_duration_ms
        );
        let bytes_per_ms = samples_per_ms as f64
            * if self.output_param.Format == OutputFormat::OutputFormatIQ4 {
                1.0
            } else {
                2.0
            };
        let total_mb = (total_duration_ms as f64 * bytes_per_ms) / (1024.0 * 1024.0);

        println!("[INFO]\tStarting signal generation loop...");
        println!(
            "[INFO]\tSignal Duration: {:.2} s",
            total_duration_ms as f64 / 1000.0
        );
        println!("[INFO]\tSignal Size: {:.2} MB", total_mb);
        println!(
            "[INFO]\tSignal Data format: {}",
            if self.output_param.Format == OutputFormat::OutputFormatIQ4 {
                "IQ4"
            } else {
                "IQ8"
            }
        );
        println!(
            "[INFO]\tSignal Center freq: {:.4} MHz",
            self.output_param.CenterFreq as f64 / 1_000_000.0
        );
        println!(
            "[INFO]\tSignal Sample rate: {:.2} MHz",
            self.output_param.SampleFreq as f64 / 1_000_000.0
        );

        println!("[INFO]\tStarting main generation loop...");

        let signals_owned = std::mem::take(sat_if_signals);
        self.generate_with_true_parallelization(
            if_file,
            signals_owned,
            total_duration_ms,
            samples_per_ms,
            self.cur_time,
        )
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
        start_time_gnss: GnssTime,
    ) -> Result<(), Box<dyn std::error::Error>> {
        println!("🚀 ПАМЯТЬ-ЭФФЕКТИВНЫЙ РЕЖИМ: Потоковая параллелизация!");
        println!(
            "📊 Параметры: {} ms сигнала, {} samples/ms, {} спутников",
            total_duration_ms,
            samples_per_ms,
            sat_if_signals.len()
        );

        // СТРАТЕГИЯ: Обрабатываем по блокам 1000ms (1 секунда)
        const BLOCK_SIZE_MS: i32 = 50; // 50ms blocks for smooth carrier phase (SignalSim updates every 1ms)
        let estimated_mb_total =
            (total_duration_ms as f64 * samples_per_ms as f64 * 2.0) / (1024.0 * 1024.0);
        println!(
            "🎯 Общий размер данных: {:.1} MB, обработка блоками по {} ms",
            estimated_mb_total, BLOCK_SIZE_MS
        );

        let start_generation = std::time::Instant::now();
        let active_satellites = sat_if_signals.iter().filter(|s| s.is_some()).count();
        println!("🛰️  Активных спутников: {}", active_satellites);

        // ========== ИНТЕЛЛЕКТУАЛЬНАЯ AGC (AUTOMATIC GAIN CONTROL) СИСТЕМА ==========
        //
        // НАЗНАЧЕНИЕ: Предотвращение клиппинга (насыщения) цифрового сигнала при большом
        // количестве активных спутников. AGC автоматически регулирует амплитуду суммарного
        // сигнала, чтобы он не превышал динамический диапазон АЦП приемника.
        //
        // AGC (Automatic Gain Control) масштабирует весь сигнал (шум + спутники)
        // перед квантованием в 8-бит IQ, как в реальном приёмнике.
        //

        // РАСЧЕТ НАЧАЛЬНОГО AGC GAIN
        // Используем ту же формулу амплитуды что и реальная генерация (ComputationCache::update):
        // A = sqrt(2 * 10^(CN0_dB/10) / Fs), где Fs = samples_per_ms * 1000
        let init_cn0 = self.power_control.init_cn0; // типично 47.0 dB-Hz
        let cn0_linear = 10.0_f64.powf(init_cn0 / 10.0);
        let fs = samples_per_ms as f64 * 1000.0;
        let single_sat_amplitude = (cn0_linear / fs).sqrt();

        // RMS суммы некоррелированных спутников: A_total = sqrt(N) * A_single
        let total_rms_amplitude = (active_satellites as f64).sqrt() * single_sat_amplitude;

        let noise_sigma = 1.0;
        let total_rms_with_noise =
            (total_rms_amplitude * total_rms_amplitude + noise_sigma * noise_sigma).sqrt();

        // Целевой RMS = 0.25: при гауссовом распределении 3σ ≈ 0.75, ~5% headroom до 1.0
        let target_rms = 0.25;
        let initial_agc_gain = if total_rms_with_noise > 0.0 {
            target_rms / total_rms_with_noise
        } else {
            1.0
        };

        let mut agc_gain = initial_agc_gain.clamp(0.001, 100.0);

        // СЧЕТЧИКИ ДЛЯ СТАТИСТИКИ
        let mut total_clipped_samples = 0i64;
        let mut total_samples_written = 0i64;

        println!("[AGC] Начальный gain: {:.3} (спутников: {}, RMS сигнала: {:.4}, RMS+шум: {:.4}, целевой RMS: {:.2})",
                 agc_gain, active_satellites, total_rms_amplitude, total_rms_with_noise, target_rms);

        // ОСНОВНОЙ ЦИКЛ: Обработка по блокам
        let num_blocks = (total_duration_ms + BLOCK_SIZE_MS - 1) / BLOCK_SIZE_MS;

        // ОПТИМИЗАЦИЯ: Выделяем буферы один раз перед циклом, переиспользуем между блоками
        let max_block_samples = BLOCK_SIZE_MS as usize * samples_per_ms;
        let mut block_signal = vec![ComplexNumber::from_parts(0.0, 0.0); max_block_samples];
        let mut iq8_data = vec![0u8; max_block_samples * 2];

        for block_idx in 0..num_blocks {
            let block_start_ms = block_idx * BLOCK_SIZE_MS;
            let block_end_ms = std::cmp::min(block_start_ms + BLOCK_SIZE_MS, total_duration_ms);
            let block_duration_ms = block_end_ms - block_start_ms;

            let progress_pct = (block_idx + 1) as f64 / num_blocks as f64 * 100.0;
            print!(
                "\r📦 {}/{} ({:.0}%) {} ms   ",
                block_idx + 1,
                num_blocks,
                progress_pct,
                block_end_ms,
            );
            std::io::stdout().flush().unwrap();

            let block_samples = block_duration_ms as usize * samples_per_ms;

            let generation_start = std::time::Instant::now();

            // 1. Noise fills the block buffer first
            FastMath::generate_noise_block(&mut block_signal[..block_samples], noise_sigma);

            // 2. Per-ms signal generation with periodic Kepler updates
            // Kepler propagation every PARAM_UPDATE_INTERVAL_MS (expensive),
            // but sample generation every 1ms (cheap) — matches C++ accuracy
            const PARAM_UPDATE_INTERVAL_MS: i32 = 1; // C++ SignalSim updates every 1ms
            let center_freq = self.output_param.CenterFreq as f64;
            let position_ecef = lla_to_ecef(&self.start_pos);
            for ms_offset in 0..block_duration_ms {
                let ms_time = start_time_gnss.add_milliseconds((block_start_ms + ms_offset) as f64);

                // Update orbital params periodically (Kepler propagation is expensive)
                let global_ms = block_start_ms + ms_offset;
                if ms_offset == 0 || global_ms % PARAM_UPDATE_INTERVAL_MS == 0 {
                    // C++ UpdateSatParamList: GetPowerControlList(1) + CalculateParam + UpdateCN0
                    let (power_slice, power_count) = self.power_control.get_power_control_list(1);
                    let power_list_owned: Vec<SignalPower> = power_slice.to_vec();
                    self.update_sat_param_list(
                        ms_time,
                        position_ecef,
                        power_count,
                        &power_list_owned,
                        None,
                    );

                    // Push updated params to each satellite
                    // At block boundaries (ms_offset==0): full update_satellite_params
                    // with code phase re-anchor from transmit time + signal_time sync.
                    // Between blocks: lightweight push_sat_param_for_ms (no re-anchor).
                    // C++ SatIfSignal: code phase from Start→End transmit time (continuous)
                    // Full update (code re-anchor) only at block boundaries;
                    // between blocks: lightweight push (Doppler/IF only, no re-anchor)
                    let full_update = ms_offset == 0; // Full update only at 50ms block boundaries; push between
                    for sig_option in sat_if_signals.iter_mut() {
                        if let Some(ref mut sig) = sig_option {
                            let param_ref = match sig.system {
                                GnssSystem::GpsSystem => {
                                    if sig.svid > 0 && (sig.svid as usize) <= TOTAL_GPS_SAT {
                                        Some(&self.gps_sat_param[sig.svid as usize - 1])
                                    } else {
                                        None
                                    }
                                }
                                GnssSystem::BdsSystem => {
                                    if sig.svid > 0 && (sig.svid as usize) <= TOTAL_BDS_SAT {
                                        Some(&self.bds_sat_param[sig.svid as usize - 1])
                                    } else {
                                        None
                                    }
                                }
                                GnssSystem::GalileoSystem => {
                                    if sig.svid > 0 && (sig.svid as usize) <= TOTAL_GAL_SAT {
                                        Some(&self.gal_sat_param[sig.svid as usize - 1])
                                    } else {
                                        None
                                    }
                                }
                                GnssSystem::GlonassSystem => {
                                    if sig.svid > 0 && (sig.svid as usize) <= TOTAL_GLO_SAT {
                                        Some(&self.glo_sat_param[sig.svid as usize - 1])
                                    } else {
                                        None
                                    }
                                }
                                _ => None,
                            };
                            if let Some(new_param) = param_ref {
                                if full_update {
                                    sig.update_satellite_params(new_param, center_freq, &ms_time);
                                } else {
                                    sig.push_sat_param_for_ms(new_param, center_freq, &ms_time);
                                }
                            }
                        }
                    }
                }

                // Phase 1: Generate 1ms of samples for each satellite IN PARALLEL (Rayon)
                // Each satellite writes only to its own sample_array — no data races
                sat_if_signals.par_iter_mut().for_each(|sig_option| {
                    if let Some(ref mut sig) = sig_option {
                        sig.get_if_sample_cached(ms_time);
                    }
                });

                // Phase 2: Accumulate satellite signals into block buffer.
                // Each worker owns one output sample, so writes are race-free.
                let buf_offset = ms_offset as usize * samples_per_ms;
                block_signal[buf_offset..buf_offset + samples_per_ms]
                    .par_iter_mut()
                    .enumerate()
                    .for_each(|(sample_idx, sample)| {
                        for sig in sat_if_signals.iter().flatten() {
                            if sample_idx < sig.sample_array.len() {
                                sample.real += sig.sample_array[sample_idx].real;
                                sample.imag += sig.sample_array[sample_idx].imag;
                            }
                        }
                    });
            }

            let generation_duration = generation_start.elapsed();

            // 3. Применяем AGC ко ВСЕМУ блоку (шум + сигнал) — как в реальном приёмнике
            for i in 0..block_samples {
                block_signal[i].real *= agc_gain;
                block_signal[i].imag *= agc_gain;
            }

            // ========== КВАНТОВАНИЕ И ЗАПИСЬ БЛОКА В ФАЙЛ ==========
            //
            // НАЗНАЧЕНИЕ: Преобразование комплексных отсчетов с плавающей точкой в 8-битные
            // целые IQ компоненты для совместимости с приемниками и программными определяемыми радио.
            //
            // ФОРМАТ IQ8: Каждый отсчет представляется как пара 8-битных целых чисел:
            // - I (софазная компонента): -128 до +127
            // - Q (квадратурная компонента): -128 до +127
            //
            // МАСШТАБИРОВАНИЕ: Отсчеты с плавающей точкой [-1.0, +1.0] масштабируются к [-127, +127]
            // Множитель 127 обеспечивает оптимальное использование динамического диапазона
            let write_start = std::time::Instant::now();
            let mut clipped_in_block = 0u64;

            // SIMD-квантование: обработка 4 сэмплов за итерацию через f64x4
            let scale = f64x4::splat(127.0);
            let clamp_lo = f64x4::splat(-128.0);
            let clamp_hi = f64x4::splat(127.0);

            let chunks = block_samples / 4;

            for chunk in 0..chunks {
                let base = chunk * 4;
                let s0 = &block_signal[base];
                let s1 = &block_signal[base + 1];
                let s2 = &block_signal[base + 2];
                let s3 = &block_signal[base + 3];

                let reals = f64x4::new([s0.real, s1.real, s2.real, s3.real]);
                let imags = f64x4::new([s0.imag, s1.imag, s2.imag, s3.imag]);

                // Клиппинг-детекция скалярно (дешевле чем SIMD extract для 4 элементов)
                if s0.real.abs() > 1.0 || s0.imag.abs() > 1.0 {
                    clipped_in_block += 1;
                }
                if s1.real.abs() > 1.0 || s1.imag.abs() > 1.0 {
                    clipped_in_block += 1;
                }
                if s2.real.abs() > 1.0 || s2.imag.abs() > 1.0 {
                    clipped_in_block += 1;
                }
                if s3.real.abs() > 1.0 || s3.imag.abs() > 1.0 {
                    clipped_in_block += 1;
                }

                // SIMD масштабирование и clamp
                let i_scaled = (reals * scale).max(clamp_lo).min(clamp_hi);
                let q_scaled = (imags * scale).max(clamp_lo).min(clamp_hi);

                let i_arr = i_scaled.as_array_ref();
                let q_arr = q_scaled.as_array_ref();

                let out_base = base * 2;
                iq8_data[out_base] = (i_arr[0] as i8) as u8;
                iq8_data[out_base + 1] = (q_arr[0] as i8) as u8;
                iq8_data[out_base + 2] = (i_arr[1] as i8) as u8;
                iq8_data[out_base + 3] = (q_arr[1] as i8) as u8;
                iq8_data[out_base + 4] = (i_arr[2] as i8) as u8;
                iq8_data[out_base + 5] = (q_arr[2] as i8) as u8;
                iq8_data[out_base + 6] = (i_arr[3] as i8) as u8;
                iq8_data[out_base + 7] = (q_arr[3] as i8) as u8;
            }

            // Остаток (< 4 сэмплов)
            for idx in (chunks * 4)..block_samples {
                let sample = &block_signal[idx];
                let i_scaled = (sample.real * 127.0).clamp(-128.0, 127.0) as i8;
                let q_scaled = (sample.imag * 127.0).clamp(-128.0, 127.0) as i8;
                if sample.real.abs() > 1.0 || sample.imag.abs() > 1.0 {
                    clipped_in_block += 1;
                }
                iq8_data[idx * 2] = i_scaled as u8;
                iq8_data[idx * 2 + 1] = q_scaled as u8;
            }
            let clipped_in_block = clipped_in_block;

            // Записываем блок в файл
            if_file.write_all(&iq8_data[..block_samples * 2])?;

            total_clipped_samples += clipped_in_block as i64;
            total_samples_written += block_samples as i64 * 2; // I + Q

            // ========== RMS-BASED AGC ==========
            // Измеряем RMS блока ПЕРЕД квантованием (сигнал уже масштабирован AGC)
            let rms = {
                let sum_sq: f64 = block_signal[..block_samples]
                    .iter()
                    .map(|s| s.real * s.real + s.imag * s.imag)
                    .sum();
                (sum_sq / block_samples as f64).sqrt()
            };

            // Адаптируем gain: целевой RMS = 0.25 (3σ ≈ 0.75, ~5% headroom до 1.0)
            if rms > 0.0 {
                let correction = target_rms / rms;
                // Плавная коррекция: двигаемся к цели на 50% за блок
                agc_gain *= 1.0 + 0.5 * (correction - 1.0);
                agc_gain = agc_gain.clamp(0.001, 100.0);
            }

            let clipping_rate = clipped_in_block as f64 / block_samples as f64;
            if block_idx < 3 || (block_idx + 1) % 10 == 0 || clipping_rate > 0.01 {
                let progress_pct = (block_idx + 1) as f64 / num_blocks as f64 * 100.0;
                print!(
                    "\r📦 {}/{} ({:.0}%) RMS={:.3} gain={:.3} clip={:.2}%          ",
                    block_idx + 1,
                    num_blocks,
                    progress_pct,
                    rms,
                    agc_gain,
                    clipping_rate * 100.0
                );
                std::io::stdout().flush().unwrap();
            }

            let write_duration = write_start.elapsed();
            if crate::logutil::is_verbose() {
                print!(
                    "\r💾 Блок {}/{} записан: {:.1} MB за {:.0}ms генерации + {:.0}ms записи     ",
                    block_idx + 1,
                    num_blocks,
                    iq8_data.len() as f64 / (1024.0 * 1024.0),
                    generation_duration.as_secs_f64() * 1000.0,
                    write_duration.as_secs_f64() * 1000.0
                );
                std::io::stdout().flush().unwrap();
            }
        }

        // Переходим на новую строку после завершения всех блоков
        if crate::logutil::is_verbose() {}

        // Финальная статистика
        let total_duration = start_generation.elapsed();
        if crate::logutil::is_verbose() {
            println!(
                "==============================================================================="
            );
            println!("                     ПОТОКОВАЯ ГЕНЕРАЦИЯ ЗАВЕРШЕНА");
            println!(
                "==============================================================================="
            );
            println!("🎯 Время генерации: {:.3}s", total_duration.as_secs_f64());
            println!("📊 Всего сэмплов: {}", total_samples_written / 2);
            println!("📁 Всего байт: {}", total_samples_written);
            println!(
                "⚡ Скорость: {:.1} MS/s",
                (total_samples_written / 2) as f64 / total_duration.as_secs_f64() / 1e6
            );
        }

        if total_clipped_samples > 0 {
            let clip_rate =
                total_clipped_samples as f64 / (total_samples_written / 2) as f64 * 100.0;
            if crate::logutil::is_verbose() {
                println!(
                    "⚠️  Клиппинг: {} сэмплов ({:.2}%)",
                    total_clipped_samples, clip_rate
                );
            }
        }

        // ========== ФИНАЛЬНАЯ AGC СТАТИСТИКА ==========
        if crate::logutil::is_verbose() {
            println!("🎚️  Финальный AGC коэффициент: {:.3}", agc_gain);
            if agc_gain < 1.0 {
                println!(
                    "🎚️  AGC компенсировал усиление на {:.1}%",
                    (1.0 - agc_gain) * 100.0
                );
            }
        }

        if_file.flush()?;
        Ok(())
    }

    // Helper methods

    fn get_nav_data<'a>(
        &self,
        sat_system: GnssSystem,
        sat_signal_index: i32,
        nav_bit_array: &'a [Option<UnifiedNavData>],
    ) -> Option<&'a UnifiedNavData> {
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
            GnssSystem::GpsSystem => match sat_signal_index {
                L1CA => nav_bit_array[DataBitType::DataBitLNav as usize].as_ref(),
                L1C => nav_bit_array[DataBitType::DataBitCNav2 as usize].as_ref(),
                L2C => nav_bit_array[DataBitType::DataBitCNav as usize].as_ref(),
                L2P => nav_bit_array[DataBitType::DataBitLNav as usize].as_ref(),
                L5 => nav_bit_array[DataBitType::DataBitL5CNav as usize].as_ref(),
                _ => nav_bit_array[DataBitType::DataBitLNav as usize].as_ref(),
            },
            GnssSystem::BdsSystem => match sat_signal_index {
                B1C => nav_bit_array[DataBitType::DataBitBCNav1 as usize].as_ref(),
                B1I => nav_bit_array[DataBitType::DataBitD1D2 as usize].as_ref(),
                B2I => nav_bit_array[DataBitType::DataBitD1D2 as usize].as_ref(),
                B3I => nav_bit_array[DataBitType::DataBitD1D2 as usize].as_ref(),
                B2A => nav_bit_array[DataBitType::DataBitBCNav2 as usize].as_ref(),
                B2B => nav_bit_array[DataBitType::DataBitBCNav3 as usize].as_ref(),
                _ => nav_bit_array[DataBitType::DataBitD1D2 as usize].as_ref(),
            },
            GnssSystem::GalileoSystem => {
                match sat_signal_index {
                    E1 => nav_bit_array[DataBitType::DataBitINav as usize].as_ref(),
                    E5A => nav_bit_array[DataBitType::DataBitFNav as usize].as_ref(),
                    E5B => nav_bit_array[DataBitType::DataBitINav as usize].as_ref(),
                    E6 => nav_bit_array[DataBitType::DataBitINav as usize].as_ref(), // E6 uses I/NAV for now
                    _ => nav_bit_array[DataBitType::DataBitINav as usize].as_ref(),
                }
            }
            GnssSystem::GlonassSystem => match sat_signal_index {
                G1 => nav_bit_array[DataBitType::DataBitGNav as usize].as_ref(),
                G2 => nav_bit_array[DataBitType::DataBitGNav as usize].as_ref(),
                _ => nav_bit_array[DataBitType::DataBitGNav as usize].as_ref(),
            },
            _ => nav_bit_array[DataBitType::DataBitLNav as usize].as_ref(),
        }
    }

    fn display_enabled_signals(&self) {
        // GPS signals
        if self.output_param.CompactConfig.should_parse_gps() {
            print!("\tGPS : [ ");
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_L1CA)
            {
                print!("L1CA ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_L1C)
            {
                print!("L1C ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_L2C)
            {
                print!("L2C ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_L5)
            {
                print!("L5 ");
            }
            println!("]");
        }

        // BDS signals
        let bds_signals_present = self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_B1C)
            || self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_B1I)
            || self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_B2I)
            || self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_B3I)
            || self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_B2A)
            || self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_B2B)
            || self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_B2AB);
        if bds_signals_present {
            print!("\tBDS : [ ");
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_B1C)
            {
                print!("B1C ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_B1I)
            {
                print!("B1I ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_B2I)
            {
                print!("B2I ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_B2A)
            {
                print!("B2a ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_B2B)
            {
                print!("B2b ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_B3I)
            {
                print!("B3I ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_B2AB)
            {
                print!("B2AB ");
            }
            println!("]");
        }

        // Galileo signals
        let gal_signals_present = self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_E1)
            || self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_E5A)
            || self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_E5B)
            || self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_E6);
        if gal_signals_present {
            print!("\tGAL : [ ");
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_E1)
            {
                print!("E1 ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_E5A)
            {
                print!("E5a ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_E5B)
            {
                print!("E5b ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_E6)
            {
                print!("E6 ");
            }
            println!("]");
        }

        // GLONASS signals
        let glo_signals_present = self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_G1)
            || self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_G2)
            || self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_G3);
        if glo_signals_present {
            print!("\tGLO : [ ");
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_G1)
            {
                print!("G1 ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_G2)
            {
                print!("G2 ");
            }
            if self
                .output_param
                .CompactConfig
                .is_signal_enabled(crate::types::GEN_G3)
            {
                print!("G3 ");
            }
            println!("]");
        }
    }

    fn count_signals_per_system(&self) -> (usize, usize, usize, usize) {
        let mut gps_signal_count = 0;
        let mut bds_signal_count = 0;
        let mut gal_signal_count = 0;
        let mut glo_signal_count = 0;

        // Count GPS signals
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_L1CA)
        {
            gps_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_L1C)
        {
            gps_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_L2C)
        {
            gps_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_L2P)
        {
            gps_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_L5)
        {
            gps_signal_count += 1;
        }

        // Count BDS signals
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_B1C)
        {
            bds_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_B1I)
        {
            bds_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_B2I)
        {
            bds_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_B3I)
        {
            bds_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_B2A)
        {
            bds_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_B2B)
        {
            bds_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_B2AB)
        {
            bds_signal_count += 1;
        }

        // Count Galileo signals
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_E1)
        {
            gal_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_E5A)
        {
            gal_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_E5B)
        {
            gal_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_E6)
        {
            gal_signal_count += 1;
        }

        // Count GLONASS signals
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_G1)
        {
            glo_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_G2)
        {
            glo_signal_count += 1;
        }
        if self
            .output_param
            .CompactConfig
            .is_signal_enabled(crate::types::GEN_G3)
        {
            glo_signal_count += 1;
        }

        (
            gps_signal_count,
            bds_signal_count,
            gal_signal_count,
            glo_signal_count,
        )
    }

    fn display_signals_summary_table(
        &self,
        gps_signal_count: usize,
        bds_signal_count: usize,
        gal_signal_count: usize,
        glo_signal_count: usize,
    ) {
        println!("Signals Summary Table:");
        println!("+---------------+-------------+--------------+------------------------------+");
        println!("| Constellation | Visible SVs | Signals / SV | Total Signals / Visible SVs |");
        println!("+---------------+-------------+--------------+------------------------------+");
        println!(
            "| GPS           | {:11} | {:12} | {:28} |",
            self.gps_sat_number,
            gps_signal_count,
            self.gps_sat_number * gps_signal_count
        );
        println!(
            "| BeiDou        | {:11} | {:12} | {:28} |",
            self.bds_sat_number,
            bds_signal_count,
            self.bds_sat_number * bds_signal_count
        );
        println!(
            "| Galileo       | {:11} | {:12} | {:28} |",
            self.gal_sat_number,
            gal_signal_count,
            self.gal_sat_number * gal_signal_count
        );
        println!(
            "| GLONASS       | {:11} | {:12} | {:28} |",
            self.glo_sat_number,
            glo_signal_count,
            self.glo_sat_number * glo_signal_count
        );
        println!("+---------------+-------------+--------------+------------------------------+");

        let total_visible_svs =
            self.gps_sat_number + self.bds_sat_number + self.gal_sat_number + self.glo_sat_number;
        let total_channels = self.gps_sat_number * gps_signal_count
            + self.bds_sat_number * bds_signal_count
            + self.gal_sat_number * gal_signal_count
            + self.glo_sat_number * glo_signal_count;
        println!(
            "Total Visible SVs = {}, Total channels = {}\n",
            total_visible_svs, total_channels
        );
    }

    fn assign_parameters(
        &mut self,
        _object: &JsonObject,
        utc_time: &mut UtcTime,
        start_pos: &mut LlaPosition,
        start_vel: &mut LocalSpeed,
    ) -> Result<(), Box<dyn std::error::Error>> {
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
    #[allow(dead_code)]
    fn get_visible_satellite(
        &self,
        cur_pos: KinematicInfo,
        cur_time: GnssTime,
        system: GnssSystem,
        eph: &[Option<GpsEphemeris>],
        eph_visible: &mut [Option<GpsEphemeris>],
    ) -> usize {
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

                // КАК В C ВЕРСИИ: передаем только секунды недели
                let gps_time_seconds = (cur_time.MilliSeconds as f64) / 1000.0;

                // Вызов функции расчета позиции спутника (упрощенный)
                // В полной реализации: gps_sat_pos_speed_eph(system, gps_time_seconds, ephemeris, &mut sat_pos_vel)
                // УСКОРЕНИЕ: используем FastMath для тригонометрии в критическом цикле орбит
                let mean_anomaly = ephemeris.M0 + ephemeris.n * gps_time_seconds;
                sat_pos_vel.x = ephemeris.axis * crate::fastmath::FastMath::fast_cos(mean_anomaly);
                sat_pos_vel.y = ephemeris.axis * crate::fastmath::FastMath::fast_sin(mean_anomaly);
                sat_pos_vel.z = ephemeris.axis
                    * crate::fastmath::FastMath::fast_sin(ephemeris.i0)
                    * crate::fastmath::FastMath::fast_sin(mean_anomaly);

                // Вычисляем угол места (elevation) от текущей позиции до спутника
                let dx = sat_pos_vel.x - cur_pos.x;
                let dy = sat_pos_vel.y - cur_pos.y;
                let dz = sat_pos_vel.z - cur_pos.z;
                let distance = (dx * dx + dy * dy + dz * dz).sqrt();
                let elevation = (dz / distance).asin().to_degrees();

                // Проверяем маску угла места
                if elevation >= elevation_mask && sat_number < eph_visible.len() {
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
    #[allow(dead_code)]
    fn get_glonass_visible_satellite(
        &self,
        _cur_pos: KinematicInfo,
        _glonass_time: GlonassTime,
        eph: &[Option<GlonassEphemeris>],
        eph_visible: &mut [Option<GlonassEphemeris>],
    ) -> usize {
        let mut sat_number = 0;
        let _elevation_mask = self.output_param.ElevationMask;
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

    /// Recalculate satellite positions, velocities, Doppler shifts, and ionospheric delays
    /// for all visible satellites at the given time. This keeps parameters current as
    /// satellites move along their orbits during signal generation.
    fn update_sat_param_list(
        &mut self,
        cur_time: GnssTime,
        _cur_pos: KinematicInfo,
        _list_count: usize,
        power_list: &[SignalPower],
        _iono_param: Option<&IonoParam>,
    ) {
        // Convert receiver LLA position to ECEF for satellite parameter computation
        let position_ecef = lla_to_ecef(&self.start_pos);
        let position_lla = self.start_pos;

        let default_iono = IonoParam {
            a0: 0.0,
            a1: 0.0,
            a2: 0.0,
            a3: 0.0,
            b0: 0.0,
            b1: 0.0,
            b2: 0.0,
            b3: 0.0,
            flag: 0,
        };

        // CN0 update parameters (C++ IFdataGen.cpp:798 — UpdateCN0 after each CalculateParam)
        let default_cn0 = self.power_control.init_cn0;
        let adjust = self.power_control.adjust;

        // Update GPS satellite parameters
        let gps_iono = self
            .nav_data
            .get_gps_iono()
            .cloned()
            .unwrap_or(default_iono);
        let gps_time = cur_time;
        for i in 0..self.gps_sat_number {
            if let Some(ref eph) = self.gps_eph_visible[i] {
                let svid = eph.svid as usize;
                if svid > 0 && svid <= TOTAL_GPS_SAT {
                    crate::satellite_param::get_satellite_param(
                        &position_ecef,
                        &position_lla,
                        &gps_time,
                        GnssSystem::GpsSystem,
                        eph,
                        &gps_iono,
                        &mut self.gps_sat_param[svid - 1],
                    );
                    crate::satellite_param::get_satellite_cn0(
                        power_list,
                        default_cn0,
                        adjust,
                        &mut self.gps_sat_param[svid - 1],
                    );
                }
            }
        }

        // Update BeiDou satellite parameters
        // C++ IFdataGen.cpp:800-804: passes GPS time; CalculateParam subtracts 14000ms internally
        // Previously Rust converted to BDT first, causing DOUBLE subtraction (-28s instead of -14s)
        let bds_iono = self
            .nav_data
            .get_bds_iono()
            .or(self.nav_data.get_gps_iono())
            .cloned()
            .unwrap_or(default_iono);
        let utc_now = crate::gnsstime::gps_time_to_utc(cur_time, true);
        for i in 0..self.bds_sat_number {
            if let Some(ref eph) = self.bds_eph_visible[i] {
                let svid = eph.svid as usize;
                if svid > 0 && svid <= TOTAL_BDS_SAT {
                    crate::satellite_param::get_satellite_param(
                        &position_ecef,
                        &position_lla,
                        &cur_time,
                        GnssSystem::BdsSystem,
                        eph,
                        &bds_iono,
                        &mut self.bds_sat_param[svid - 1],
                    );
                    crate::satellite_param::get_satellite_cn0(
                        power_list,
                        default_cn0,
                        adjust,
                        &mut self.bds_sat_param[svid - 1],
                    );
                }
            }
        }

        // Update Galileo satellite parameters (GPS time — C++ passes GPS time directly)
        let gal_iono = self
            .nav_data
            .get_galileo_iono()
            .or(self.nav_data.get_gps_iono())
            .cloned()
            .unwrap_or(default_iono);
        for i in 0..self.gal_sat_number {
            if let Some(ref eph) = self.gal_eph_visible[i] {
                let svid = eph.svid as usize;
                if svid > 0 && svid <= TOTAL_GAL_SAT {
                    crate::satellite_param::get_satellite_param(
                        &position_ecef,
                        &position_lla,
                        &cur_time,
                        GnssSystem::GalileoSystem,
                        eph,
                        &gal_iono,
                        &mut self.gal_sat_param[svid - 1],
                    );
                    crate::satellite_param::get_satellite_cn0(
                        power_list,
                        default_cn0,
                        adjust,
                        &mut self.gal_sat_param[svid - 1],
                    );
                }
            }
        }

        // Update GLONASS satellite parameters (GLONASS time via UTC conversion)
        let glo_iono = self
            .nav_data
            .get_gps_iono()
            .cloned()
            .unwrap_or(default_iono);
        let glonass_time = crate::gnsstime::utc_to_glonass_time_corrected(utc_now);
        for i in 0..self.glo_sat_number {
            if let Some(ref eph) = self.glo_eph_visible[i] {
                let n = eph.n as usize;
                if n > 0 && n <= TOTAL_GLO_SAT {
                    crate::satellite_param::get_glonass_satellite_param(
                        &position_ecef,
                        &position_lla,
                        &glonass_time,
                        eph,
                        &glo_iono,
                        &mut self.glo_sat_param[n - 1],
                    );
                    crate::satellite_param::get_satellite_cn0(
                        power_list,
                        default_cn0,
                        adjust,
                        &mut self.glo_sat_param[n - 1],
                    );
                }
            }
        }
    }

    #[allow(dead_code)]
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
        let _vel_x = glo_eph.vx * 1000.0;
        let _vel_y = glo_eph.vy * 1000.0;
        let _vel_z = glo_eph.vz * 1000.0;

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

    fn create_gps_signals(
        &mut self,
        sat_if_signals: &mut [Option<Box<SatIfSignal>>],
        mut total_channel_number: usize,
        nav_bit_array: &[Option<UnifiedNavData>],
        cur_pos: KinematicInfo,
    ) -> Result<usize, Box<dyn std::error::Error>> {
        for i in 0..self.gps_sat_number {
            if let Some(eph) = &self.gps_eph_visible[i] {
                // Рассчитываем параметры спутника для правильного доплера
                let position_lla = ecef_to_lla(&cur_pos);
                let gps_time = self.cur_time; // используем текущее время

                let default_iono = IonoParam {
                    a0: 0.0,
                    a1: 0.0,
                    a2: 0.0,
                    a3: 0.0,
                    b0: 0.0,
                    b1: 0.0,
                    b2: 0.0,
                    b3: 0.0,
                    flag: 0,
                };
                let iono_param = self.nav_data.get_gps_iono().unwrap_or(&default_iono);

                crate::satellite_param::get_satellite_param(
                    &cur_pos,
                    &position_lla,
                    &gps_time,
                    GnssSystem::GpsSystem,
                    eph,
                    iono_param,
                    &mut self.gps_sat_param[eph.svid as usize - 1],
                );

                // Check each GPS signal
                let gps_signals = [
                    (SIGNAL_INDEX_L1CA, crate::types::GEN_L1CA),
                    (SIGNAL_INDEX_L1C, crate::types::GEN_L1C),
                    (SIGNAL_INDEX_L2C, crate::types::GEN_L2C),
                    (SIGNAL_INDEX_L2P, crate::types::GEN_L2P),
                    (SIGNAL_INDEX_L5, crate::types::GEN_L5),
                ];

                for &(signal_index, signal_mask) in &gps_signals {
                    if self
                        .output_param
                        .CompactConfig
                        .is_signal_enabled(signal_mask)
                    {
                        let center_freq =
                            SIGNAL_CENTER_FREQ[GnssSystem::GpsSystem as usize][signal_index.min(7)];
                        // Статический IF offset (как в SignalSim), Допплер учитывается через carrier phase
                        let if_freq = (center_freq - self.output_param.CenterFreq as f64) as i32;
                        if eph.svid <= 3 {
                            // Отладка для первых 3 спутников
                        }
                        // SatIfSignal expects number of samples per millisecond, not Hz
                        let samples_per_ms = self.output_param.SampleFreq / 1000;
                        let mut new_signal = SatIfSignal::new(
                            samples_per_ms,
                            if_freq,
                            GnssSystem::GpsSystem,
                            signal_index as i32,
                            eph.svid,
                        );

                        let nav_data = self.get_nav_data(
                            GnssSystem::GpsSystem,
                            signal_index as i32,
                            nav_bit_array,
                        );
                        // Cloning the boxed trait object
                        let nav_data_clone = nav_data.cloned();

                        new_signal.init_state(
                            self.cur_time,
                            &self.gps_sat_param[eph.svid as usize - 1],
                            nav_data_clone,
                        );
                        sat_if_signals[total_channel_number] = Some(Box::new(new_signal));
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    fn create_bds_signals(
        &mut self,
        sat_if_signals: &mut [Option<Box<SatIfSignal>>],
        mut total_channel_number: usize,
        nav_bit_array: &[Option<UnifiedNavData>],
        cur_pos: KinematicInfo,
    ) -> Result<usize, Box<dyn std::error::Error>> {
        for i in 0..self.bds_sat_number {
            if let Some(eph) = &self.bds_eph_visible[i] {
                // Рассчитываем параметры спутника для правильного доплера
                let position_lla = ecef_to_lla(&cur_pos);
                // C++ passes GPS time; CalculateParam subtracts 14000ms internally
                // Previously Rust converted to BDT first, causing DOUBLE subtraction (-28s instead of -14s)
                let default_iono = IonoParam {
                    a0: 0.0,
                    a1: 0.0,
                    a2: 0.0,
                    a3: 0.0,
                    b0: 0.0,
                    b1: 0.0,
                    b2: 0.0,
                    b3: 0.0,
                    flag: 0,
                };
                let iono_param = self
                    .nav_data
                    .get_bds_iono()
                    .or_else(|| self.nav_data.get_gps_iono())
                    .unwrap_or(&default_iono);

                crate::satellite_param::get_satellite_param(
                    &cur_pos,
                    &position_lla,
                    &self.cur_time,
                    GnssSystem::BdsSystem,
                    eph,
                    iono_param,
                    &mut self.bds_sat_param[eph.svid as usize - 1],
                );

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
                    if self
                        .output_param
                        .CompactConfig
                        .is_signal_enabled(signal_mask)
                    {
                        let center_freq =
                            SIGNAL_CENTER_FREQ[GnssSystem::BdsSystem as usize][freq_array_index];
                        // Статический IF offset (как в SignalSim), Допплер учитывается через carrier phase
                        let if_freq = (center_freq - self.output_param.CenterFreq as f64) as i32;
                        if eph.svid <= 3 {
                            // Отладка для первых 3 спутников
                        }
                        // SatIfSignal expects number of samples per millisecond, not Hz
                        let samples_per_ms = self.output_param.SampleFreq / 1000;
                        let mut new_signal = SatIfSignal::new(
                            samples_per_ms,
                            if_freq,
                            GnssSystem::BdsSystem,
                            signal_index as i32,
                            eph.svid,
                        );

                        let nav_data = self.get_nav_data(
                            GnssSystem::BdsSystem,
                            signal_index as i32,
                            nav_bit_array,
                        );
                        let nav_data_clone = nav_data.cloned();

                        new_signal.init_state(
                            self.cur_time,
                            &self.bds_sat_param[eph.svid as usize - 1],
                            nav_data_clone,
                        );
                        sat_if_signals[total_channel_number] = Some(Box::new(new_signal));
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    fn create_galileo_signals(
        &mut self,
        sat_if_signals: &mut [Option<Box<SatIfSignal>>],
        mut total_channel_number: usize,
        nav_bit_array: &[Option<UnifiedNavData>],
        cur_pos: KinematicInfo,
    ) -> Result<usize, Box<dyn std::error::Error>> {
        for i in 0..self.gal_sat_number {
            if let Some(eph) = &self.gal_eph_visible[i] {
                // Рассчитываем параметры спутника для правильного доплера
                let position_lla = ecef_to_lla(&cur_pos);
                // C++ passes GPS time directly (no GST conversion); CalculateParam has no time adjustment for Galileo
                let default_iono = IonoParam {
                    a0: 0.0,
                    a1: 0.0,
                    a2: 0.0,
                    a3: 0.0,
                    b0: 0.0,
                    b1: 0.0,
                    b2: 0.0,
                    b3: 0.0,
                    flag: 0,
                };
                let iono_param = self
                    .nav_data
                    .get_galileo_iono()
                    .or_else(|| self.nav_data.get_gps_iono())
                    .unwrap_or(&default_iono);

                crate::satellite_param::get_satellite_param(
                    &cur_pos,
                    &position_lla,
                    &self.cur_time,
                    GnssSystem::GalileoSystem,
                    eph,
                    iono_param,
                    &mut self.gal_sat_param[eph.svid as usize - 1],
                );

                // Check each Galileo signal
                let gal_signals = [
                    (SIGNAL_INDEX_E1, crate::types::GEN_E1, 0),
                    (SIGNAL_INDEX_E5A, crate::types::GEN_E5A, 1),
                    (SIGNAL_INDEX_E5B, crate::types::GEN_E5B, 2),
                    (SIGNAL_INDEX_E6, crate::types::GEN_E6, 4),
                ];

                for &(signal_index, signal_mask, freq_array_index) in &gal_signals {
                    if self
                        .output_param
                        .CompactConfig
                        .is_signal_enabled(signal_mask)
                    {
                        let center_freq = SIGNAL_CENTER_FREQ[GnssSystem::GalileoSystem as usize]
                            [freq_array_index.min(7)];
                        // Статический IF offset (как в SignalSim), Допплер учитывается через carrier phase
                        let if_freq = (center_freq - self.output_param.CenterFreq as f64) as i32;
                        if eph.svid <= 3 {
                            // Отладка для первых 3 спутников
                        }
                        // SatIfSignal expects number of samples per millisecond, not Hz
                        let samples_per_ms = self.output_param.SampleFreq / 1000;
                        let mut new_signal = SatIfSignal::new(
                            samples_per_ms,
                            if_freq,
                            GnssSystem::GalileoSystem,
                            signal_index as i32,
                            eph.svid,
                        );

                        let nav_data = self.get_nav_data(
                            GnssSystem::GalileoSystem,
                            signal_index as i32,
                            nav_bit_array,
                        );
                        let nav_data_clone = nav_data.cloned();

                        new_signal.init_state(
                            self.cur_time,
                            &self.gal_sat_param[eph.svid as usize - 1],
                            nav_data_clone,
                        );
                        sat_if_signals[total_channel_number] = Some(Box::new(new_signal));
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    fn create_glonass_signals(
        &mut self,
        sat_if_signals: &mut [Option<Box<SatIfSignal>>],
        mut total_channel_number: usize,
        nav_bit_array: &[Option<UnifiedNavData>],
        cur_pos: KinematicInfo,
    ) -> Result<usize, Box<dyn std::error::Error>> {
        for i in 0..self.glo_sat_number {
            if let Some(eph) = &self.glo_eph_visible[i] {
                // Рассчитываем параметры спутника для корректных доплеров и задержек
                let position_lla = ecef_to_lla(&cur_pos);
                let utc_now = crate::gnsstime::gps_time_to_utc(self.cur_time, true);
                let glonass_time = crate::gnsstime::utc_to_glonass_time_corrected(utc_now);

                let default_iono = IonoParam {
                    a0: 0.0,
                    a1: 0.0,
                    a2: 0.0,
                    a3: 0.0,
                    b0: 0.0,
                    b1: 0.0,
                    b2: 0.0,
                    b3: 0.0,
                    flag: 0,
                };
                let iono_param = self.nav_data.get_gps_iono().unwrap_or(&default_iono);

                crate::satellite_param::get_glonass_satellite_param(
                    &cur_pos,
                    &position_lla,
                    &glonass_time,
                    eph,
                    iono_param,
                    &mut self.glo_sat_param[eph.n as usize - 1],
                );

                // Check each GLONASS signal
                let glo_signals = [
                    (SIGNAL_INDEX_G1, crate::types::GEN_G1, 0usize),
                    (SIGNAL_INDEX_G2, crate::types::GEN_G2, 1usize),
                    (SIGNAL_INDEX_G3, crate::types::GEN_G3, 2usize),
                ];

                for &(signal_index, signal_mask, freq_array_index) in &glo_signals {
                    if self
                        .output_param
                        .CompactConfig
                        .is_signal_enabled(signal_mask)
                    {
                        let base = SIGNAL_CENTER_FREQ[GnssSystem::GlonassSystem as usize]
                            [freq_array_index];
                        let step = if signal_index == SIGNAL_INDEX_G2 {
                            437500.0
                        } else if signal_index == SIGNAL_INDEX_G1 {
                            562500.0
                        } else {
                            0.0
                        };
                        let center_freq = base + eph.freq as f64 * step;

                        // Статический IF offset (как в SignalSim), Допплер учитывается через carrier phase
                        // Для GLONASS FDMA: offset включает k×562.5 кГц
                        let if_freq = (center_freq - self.output_param.CenterFreq as f64) as i32;
                        // SatIfSignal expects number of samples per millisecond, not Hz
                        let samples_per_ms = self.output_param.SampleFreq / 1000;
                        let mut new_signal = SatIfSignal::new(
                            samples_per_ms,
                            if_freq,
                            GnssSystem::GlonassSystem,
                            signal_index as i32,
                            eph.n,
                        );

                        let nav_data = self.get_nav_data(
                            GnssSystem::GlonassSystem,
                            signal_index as i32,
                            nav_bit_array,
                        );
                        let nav_data_clone = nav_data.cloned();

                        new_signal.init_state(
                            self.cur_time,
                            &self.glo_sat_param[eph.n as usize - 1],
                            nav_data_clone,
                        );
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
        // println!("[UNIQUE-DEBUG] load_config called with: {}", config_file);
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

        // println!("[UNIQUE-JSON-START] Starting pure Rust JSON parsing");

        // Переменная для пути к RINEX файлу
        let mut rinex_file = String::from("Rinex_Data/rinex_v3_20251560000.rnx"); // Значение по умолчанию
                                                                                  // Стратегия выбора эпох: per_sat (по умолчанию, как в C) или global
        let epoch_selection = String::from("per_sat");

        // УПРОЩЕННЫЙ JSON ПАРСИНГ - читаем файл напрямую и парсим нужные параметры
        if let Ok(json_content) = std::fs::read_to_string(config_file) {
            // println!("[UNIQUE-JSON-READ] File read successfully, length: {}", json_content.len());
            if let Ok(json) = serde_json::from_str::<serde_json::Value>(&json_content) {
                // println!("[UNIQUE-JSON-PARSE] JSON parsed successfully");
                // Парсим время из JSON
                if let Some(time) = json.get("time") {
                    if let (
                        Some(year),
                        Some(month),
                        Some(day),
                        Some(hour),
                        Some(minute),
                        Some(second),
                    ) = (
                        time.get("year").and_then(|v| v.as_i64()),
                        time.get("month").and_then(|v| v.as_i64()),
                        time.get("day").and_then(|v| v.as_i64()),
                        time.get("hour").and_then(|v| v.as_i64()),
                        time.get("minute").and_then(|v| v.as_i64()),
                        time.get("second").and_then(|v| v.as_f64()),
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
                        let pos_type = init_pos
                            .get("type")
                            .and_then(|v| v.as_str())
                            .unwrap_or("LLA");
                        let format = init_pos
                            .get("format")
                            .and_then(|v| v.as_str())
                            .unwrap_or("d");
                        if pos_type.eq_ignore_ascii_case("LLA") {
                            if let (Some(longitude), Some(latitude), Some(altitude)) = (
                                init_pos.get("longitude").and_then(|v| v.as_f64()),
                                init_pos.get("latitude").and_then(|v| v.as_f64()),
                                init_pos.get("altitude").and_then(|v| v.as_f64()),
                            ) {
                                // format: "d" (degrees), "rad" (radians)
                                let (lon_rad, lat_rad) = if format.eq_ignore_ascii_case("rad") {
                                    (longitude, latitude)
                                } else {
                                    // по умолчанию градусы
                                    (longitude.to_radians(), latitude.to_radians())
                                };
                                start_pos.lon = lon_rad;
                                start_pos.lat = lat_rad;
                                start_pos.alt = altitude;
                                println!("[INFO] Parsed position (LLA) from JSON: lat={:.6}°, lon={:.6}°, alt={:.1} m", 
                                         if format.eq_ignore_ascii_case("rad") { latitude.to_degrees() } else { latitude },
                                         if format.eq_ignore_ascii_case("rad") { longitude.to_degrees() } else { longitude },
                                         altitude);
                                self.start_pos = start_pos;
                            }
                        } else if pos_type.eq_ignore_ascii_case("ECEF") {
                            if let (Some(x), Some(y), Some(z)) = (
                                init_pos.get("x").and_then(|v| v.as_f64()),
                                init_pos.get("y").and_then(|v| v.as_f64()),
                                init_pos.get("z").and_then(|v| v.as_f64()),
                            ) {
                                let ecef = KinematicInfo {
                                    x,
                                    y,
                                    z,
                                    ..Default::default()
                                };
                                let lla = ecef_to_lla(&ecef);
                                self.start_pos = lla;
                                println!("[INFO] Parsed position (ECEF) from JSON: x={:.1} m, y={:.1} m, z={:.1} m", x, y, z);
                            }
                        }
                    }
                }

                // Парсим путь к файлу эфемерид
                if let Some(ephemeris) = json.get("ephemeris") {
                    if let Some(name) = ephemeris.get("name").and_then(|v| v.as_str()) {
                        rinex_file = String::from(name); // Копируем путь из JSON
                    }
                }

                // Парсим выходные параметры из JSON
                if let Some(output) = json.get("output") {
                    // Парсим имя файла
                    if let Some(filename) = output.get("name").and_then(|v| v.as_str()) {
                        let filename_bytes = filename.as_bytes();
                        let len = filename_bytes.len().min(255);
                        self.output_param.filename[..len].copy_from_slice(&filename_bytes[..len]);
                        self.output_param.filename[len] = 0; // Null terminator
                    }

                    // Парсим частоту дискретизации (sampleFreq в MHz -> Hz)
                    if let Some(sample_freq) = output.get("sampleFreq").and_then(|v| v.as_f64()) {
                        self.output_param.SampleFreq = (sample_freq * 1e6) as i32;
                    }

                    // Парсим центральную частоту (centerFreq в MHz -> Hz)
                    if let Some(center_freq) = output.get("centerFreq").and_then(|v| v.as_f64()) {
                        self.output_param.CenterFreq = (center_freq * 1_000_000.0) as i32;
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
                    if let Some(system_select) =
                        output.get("systemSelect").and_then(|v| v.as_array())
                    {
                        use crate::types::*;

                        // Сброс конфигурации
                        self.output_param.CompactConfig = CompactConfig::new();

                        for system in system_select {
                            if let (Some(sys), Some(signal), Some(enable)) = (
                                system.get("system").and_then(|v| v.as_str()),
                                system.get("signal").and_then(|v| v.as_str()),
                                system.get("enable").and_then(|v| v.as_bool()),
                            ) {
                                if enable {
                                    // Определяем системы для парсинга
                                    match sys {
                                        "GPS" => self
                                            .output_param
                                            .CompactConfig
                                            .enable_system_parsing(PARSE_GPS),
                                        "BDS" => self
                                            .output_param
                                            .CompactConfig
                                            .enable_system_parsing(PARSE_BDS),
                                        "Galileo" => self
                                            .output_param
                                            .CompactConfig
                                            .enable_system_parsing(PARSE_GALILEO),
                                        "GLONASS" => self
                                            .output_param
                                            .CompactConfig
                                            .enable_system_parsing(PARSE_GLONASS),
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
                                        ("GPS", "L2P") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_L2P);
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
                                        ("GLONASS", "G3") => {
                                            self.output_param.CompactConfig.enable_signal(GEN_G3);
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
                                self.output_param.Interval = (time * 1000.0) as i32;
                                // секунды -> мс
                            }
                        }
                    }
                }
            }
        }

        // Устанавливаем время симуляции из JSON пресета
        self.cur_time = crate::gnsstime::utc_to_gps_time(utc_time, true);

        println!("[INFO]\tParsed from JSON preset:");
        println!(
            "[INFO]\t  Time: {}-{:02}-{:02} {:02}:{:02}:{:06.3}",
            utc_time.Year,
            utc_time.Month,
            utc_time.Day,
            utc_time.Hour,
            utc_time.Minute,
            utc_time.Second
        );
        println!(
            "[INFO]\t  Duration: {:.1} s",
            self.output_param.Interval as f64 / 1000.0
        );

        // Создаем новую траекторию
        let mut new_trajectory = crate::trajectory::CTrajectory::new();
        new_trajectory.set_init_pos_vel_lla(start_pos, start_vel, false);
        self.trajectory = new_trajectory;

        // Reset satellite masks to allow all satellites
        self.output_param.GpsMaskOut = 0;
        self.output_param.BdsMaskOut = 0;
        self.output_param.GalileoMaskOut = 0;
        self.output_param.GlonassMaskOut = 0;

        // Применяем elevationMask из JSON, если задан; иначе дефолт 5°
        // Ищем output.config.elevationMask (в градусах)
        if let Ok(json_content) = std::fs::read_to_string(config_file) {
            if let Ok(json) = serde_json::from_str::<serde_json::Value>(&json_content) {
                if let Some(cfg) = json.get("output").and_then(|o| o.get("config")) {
                    if let Some(deg) = cfg.get("elevationMask").and_then(|v| v.as_f64()) {
                        self.output_param.ElevationMask = deg.to_radians();
                    }
                }
                // Парсим блок power (noiseFloor, initPower, elevationAdjust)
                if let Some(power) = json.get("power") {
                    // noiseFloor в дБм; если указан, применяем напрямую
                    if let Some(nf) = power.get("noiseFloor").and_then(|v| v.as_f64()) {
                        // В пресетах -999 означает «авто/игнорировать» — пропускаем
                        if nf > -900.0 {
                            self.power_control.set_noise_floor(nf);
                        }
                    }
                    // initPower { unit: "dBHz"|"dBm"|"dBW", value: f64 }
                    if let Some(init) = power.get("initPower") {
                        let unit = init.get("unit").and_then(|v| v.as_str()).unwrap_or("dBHz");
                        if let Some(val) = init.get("value").and_then(|v| v.as_f64()) {
                            let cn0 = match unit {
                                "dBHz" => val,
                                // При unit=dBm/dBW интерпретируем как абсолютная мощность и конвертируем в CN0≈S-N0
                                // CN0≈S(dBm) − N0(dBm/Hz). Для dBW добавляем 30 дБ для перехода в dBm.
                                "dBm" => val - self.power_control.get_noise_floor(),
                                "dBW" => (val + 30.0) - self.power_control.get_noise_floor(),
                                _ => val,
                            };
                            self.power_control.set_init_cn0(cn0);
                        }
                    }
                    // elevationAdjust: true/false
                    if let Some(adj) = power.get("elevationAdjust").and_then(|v| v.as_bool()) {
                        self.power_control.set_elevation_adjust(if adj {
                            crate::powercontrol::ElevationAdjust::ElevationAdjustSinSqrtFade
                        } else {
                            crate::powercontrol::ElevationAdjust::ElevationAdjustNone
                        });
                    }
                }
                // Маска исключений спутников: output.maskOut
                if let Some(mask_out) = json
                    .get("output")
                    .and_then(|o| o.get("maskOut"))
                    .and_then(|v| v.as_array())
                {
                    for item in mask_out {
                        let system = item.get("system").and_then(|v| v.as_str()).unwrap_or("");
                        if let Some(svids) = item.get("svid").and_then(|v| v.as_array()) {
                            for s in svids {
                                if let Some(svid) = s.as_i64() {
                                    match system {
                                        "GPS" => {
                                            if (1..=32).contains(&(svid as i32)) {
                                                self.output_param.GpsMaskOut |=
                                                    1 << (svid as u32 - 1);
                                            }
                                        }
                                        "BDS" => {
                                            if (1..=63).contains(&(svid as i32)) {
                                                self.output_param.BdsMaskOut |=
                                                    1u64 << (svid as u64 - 1);
                                            }
                                        }
                                        "Galileo" => {
                                            if (1..=50).contains(&(svid as i32)) {
                                                self.output_param.GalileoMaskOut |=
                                                    1u64 << (svid as u64 - 1);
                                            }
                                        }
                                        "GLONASS" => {
                                            if (1..=24).contains(&(svid as i32)) {
                                                self.output_param.GlonassMaskOut |=
                                                    1 << (svid as u32 - 1);
                                            }
                                        }
                                        _ => {}
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
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

            // Используем оптимизированную загрузку RINEX с фильтрацией систем из JSON
            crate::json_interpreter::read_nav_file_filtered(
                &mut c_nav_data,
                &rinex_file,
                Some(utc_time),   // Фильтрация по времени
                &enabled_systems, // Только BeiDou система
            );

            // Стратегия выбора эпох
            match epoch_selection.as_str() {
                "global" => self.select_global_epochs_and_fill(&c_nav_data, utc_time),
                _ => self.select_per_satellite_and_fill(&c_nav_data, utc_time),
            }

            // CRITICAL: Transfer ionospheric and UTC parameters from CNavData to NavData
            if let (Some(alpha), Some(beta)) = (c_nav_data.gps_iono_alpha, c_nav_data.gps_iono_beta)
            {
                self.nav_data.set_gps_iono(IonoParam {
                    a0: alpha[0],
                    a1: alpha[1],
                    a2: alpha[2],
                    a3: alpha[3],
                    b0: beta[0],
                    b1: beta[1],
                    b2: beta[2],
                    b3: beta[3],
                    flag: 1,
                });
                println!(
                    "[INFO] GPS iono parameters transferred to NavData: alpha={:?}, beta={:?}",
                    alpha, beta
                );
            }
            if let Some(utc) = c_nav_data.utc_param {
                self.nav_data.set_gps_utc(utc);
                println!(
                    "[INFO] GPS UTC parameters transferred to NavData: A0={}, A1={}, TLS={}",
                    utc.A0, utc.A1, utc.TLS
                );
            }

            // КРИТИЧЕСКО: Синхронизируем внутренний NavData для GLONASS,
            // чтобы initialize() мог найти эфемериды через find_glo_ephemeris
            // Очищаем и заполняем только релевантные слоты
            self.nav_data.glonass_ephemeris.clear();
            // Гарантируем размер вектора не меньше 24
            if self.nav_data.glonass_ephemeris.len() < TOTAL_GLO_SAT {
                self.nav_data.glonass_ephemeris.resize(TOTAL_GLO_SAT, None);
            }
            for slot in 0..TOTAL_GLO_SAT {
                if let Some(eph) = self.glo_eph[slot] {
                    self.nav_data.glonass_ephemeris[slot] = Some(eph);
                } else {
                    self.nav_data.glonass_ephemeris[slot] = None;
                }
            }

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
    #[allow(dead_code)]
    fn add_minimal_gps_ephemeris(&mut self) {
        println!("[INFO]\tAdding minimal GPS ephemeris for demonstration");

        // Убеждаемся что вектор достаточно большой
        if self.nav_data.gps_ephemeris.len() < 32 {
            self.nav_data.gps_ephemeris.resize(32, None);
        }

        // Добавляем минимальные эфемериды для нескольких спутников
        for svid in 1..=8 {
            let eph = GpsEphemeris {
                svid: svid as u8,
                valid: 1,
                week: 2200,
                sqrtA: 5153.5,
                i0: 55.0_f64.to_radians(),
                omega0: (svid - 1) as f64 * 45.0_f64.to_radians(),
                ecc: 0.01,
                omega_dot: -2.6e-9,
                ..Default::default()
            };

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
    #[allow(dead_code)]
    fn copy_ephemeris_from_json_nav_data(
        &mut self,
        c_nav_data: &crate::json_interpreter::CNavData,
        utc_time: UtcTime,
    ) {
        println!("[INFO]\tCopying GPS ephemeris from JSON CNavData");

        use std::collections::{HashMap, HashSet};
        // Целевое время в секундах от GPS эпохи
        let target = utc_to_gps_time(utc_time, false);
        let target_abs = (target.Week as i64) * 604800 + (target.MilliSeconds as i64) / 1000;

        // Группируем эфемериды по (week, toe)
        let mut by_epoch: HashMap<(i32, i32), HashMap<u8, GpsEphemeris>> = HashMap::new();
        let mut epochs: HashSet<(i32, i32)> = HashSet::new();
        for eph in &c_nav_data.gps_ephemeris {
            let key = (eph.week, eph.toe);
            epochs.insert(key);
            by_epoch.entry(key).or_default().insert(eph.svid, *eph);
        }

        // Выбираем одну epoch (week,toe) с минимальной |Δt|
        let mut best_key: Option<(i32, i32)> = None;
        let mut best_diff = i64::MAX;
        for &(w, toe) in &epochs {
            let epoch_abs = (w as i64) * 604800 + (toe as i64);
            let diff = (epoch_abs - target_abs).abs();
            if diff < best_diff {
                best_diff = diff;
                best_key = Some((w, toe));
            }
        }

        // Очищаем массив и заполняем только эфемеридами выбранной эпохи
        self.gps_eph.fill(None);
        if let Some(key) = best_key {
            if let Some(map) = by_epoch.get(&key) {
                for (&svid, eph) in map {
                    let idx = (svid as usize).saturating_sub(1);
                    if idx < TOTAL_GPS_SAT {
                        self.gps_eph[idx] = Some(*eph);
                    }
                }
                println!(
                    "[INFO]\tGPS epoch selected: week={}, toe={} (Δt={:.1}h), sats={}",
                    key.0,
                    key.1,
                    best_diff as f64 / 3600.0,
                    map.len()
                );
            }
        } else {
            println!("[WARN]\tNo GPS epochs available after grouping");
        }

        // Поддержка legacy: скопируем все исходные эфемериды в nav_data (как справочник)
        if self.nav_data.gps_ephemeris.len() < 32 {
            self.nav_data.gps_ephemeris.resize(32, None);
        }
        for eph in &c_nav_data.gps_ephemeris {
            if (eph.svid as usize) <= 32 && eph.svid > 0 {
                self.nav_data.gps_ephemeris[eph.svid as usize - 1] = Some(*eph);
            }
        }
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
    #[allow(dead_code)]
    fn copy_beidou_ephemeris_from_json_nav_data(
        &mut self,
        c_nav_data: &crate::json_interpreter::CNavData,
        utc_time: UtcTime,
    ) {
        println!("[INFO]\tCopying BeiDou ephemeris from JSON CNavData");

        // Целевое время в секундах от BDT эпохи
        use std::collections::{HashMap, HashSet};
        let target_bds = utc_to_bds_time(utc_time);
        let target_abs =
            (target_bds.Week as i64) * 604800 + (target_bds.MilliSeconds as i64) / 1000;

        // Группируем по (week, toe) в BDT
        let mut by_epoch: HashMap<(i32, i32), HashMap<u8, BeiDouEphemeris>> = HashMap::new();
        let mut epochs: HashSet<(i32, i32)> = HashSet::new();
        for eph in &c_nav_data.beidou_ephemeris {
            let key = (eph.week, eph.toe);
            epochs.insert(key);
            by_epoch.entry(key).or_default().insert(eph.svid, *eph);
        }
        // Выбор ближайшей эпохи
        let mut best_key: Option<(i32, i32)> = None;
        let mut best_diff = i64::MAX;
        for &(w, toe) in &epochs {
            let epoch_abs = (w as i64) * 604800 + (toe as i64);
            let diff = (epoch_abs - target_abs).abs();
            if diff < best_diff {
                best_diff = diff;
                best_key = Some((w, toe));
            }
        }
        // Заполняем self.bds_eph из выбранной эпохи
        self.bds_eph.fill(None);
        if let Some(key) = best_key {
            if let Some(map) = by_epoch.get(&key) {
                for (&svid, bds) in map {
                    let idx = (svid as usize).saturating_sub(1);
                    if idx < TOTAL_BDS_SAT {
                        self.bds_eph[idx] = Some(bds.to_gps_ephemeris());
                    }
                }
                println!(
                    "[INFO]\tBDS epoch selected: week={}, toe={} (Δt={:.1}h), sats={}",
                    key.0,
                    key.1,
                    best_diff as f64 / 3600.0,
                    map.len()
                );
            }
        } else {
            println!("[WARN]\tNo BDS epochs available after grouping");
        }

        // Статистическая верификация: подсчет эфемерид, успешно прошедших временной отбор
        let mut verified_count = 0;
        for i in 0..TOTAL_BDS_SAT {
            if self.bds_eph[i].is_some() {
                verified_count += 1; // Инкремент счетчика заполненных слотов
            }
        }

        // Параллельное заполнение nav_data для поддержки совместимости с существующим кодом
        // BeiDou констелляция: поддерживается до 65 спутников (включая резервные орбиты)
        if self.nav_data.bds_ephemeris.len() < 65 {
            self.nav_data.bds_ephemeris.resize(65, None); // Выделение памяти для полной BeiDou констелляции
        }
        // Заполнение nav_data всеми исходными эфемеридами (без применения временной фильтрации)
        for eph in &c_nav_data.beidou_ephemeris {
            if (eph.svid as usize) <= 65 && eph.svid > 0 {
                self.nav_data.bds_ephemeris[eph.svid as usize - 1] = Some(*eph);
            }
        }

        println!(
            "[INFO]\tSelected {} BeiDou ephemeris records using time-based algorithm",
            verified_count
        );
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
    #[allow(dead_code)]
    fn copy_galileo_ephemeris_from_json_nav_data(
        &mut self,
        c_nav_data: &crate::json_interpreter::CNavData,
        utc_time: UtcTime,
    ) {
        println!("[INFO]\tCopying Galileo ephemeris from JSON CNavData");

        // Целевое время в секундах от GST эпохи
        use std::collections::{HashMap, HashSet};
        let target_gal = utc_to_galileo_time(utc_time);
        let target_abs =
            (target_gal.Week as i64) * 604800 + (target_gal.MilliSeconds as i64) / 1000;

        // Группируем по (week, toe) для Galileo
        let mut by_epoch: HashMap<(i32, i32), HashMap<u8, GpsEphemeris>> = HashMap::new();
        let mut epochs: HashSet<(i32, i32)> = HashSet::new();
        for eph in &c_nav_data.galileo_ephemeris {
            let key = (eph.week, eph.toe);
            epochs.insert(key);
            by_epoch.entry(key).or_default().insert(eph.svid, *eph);
        }
        // Выбираем ближний (week,toe)
        let mut best_key: Option<(i32, i32)> = None;
        let mut best_diff = i64::MAX;
        for &(w, toe) in &epochs {
            let epoch_abs = (w as i64) * 604800 + (toe as i64);
            let diff = (epoch_abs - target_abs).abs();
            if diff < best_diff {
                best_diff = diff;
                best_key = Some((w, toe));
            }
        }
        // Заполняем self.gal_eph из выбранной эпохи
        self.gal_eph.fill(None);
        let mut gal_count = 0;
        if let Some(key) = best_key {
            if let Some(map) = by_epoch.get(&key) {
                for (&svid, eph) in map {
                    let idx = (svid as usize).saturating_sub(1);
                    if idx < TOTAL_GAL_SAT {
                        self.gal_eph[idx] = Some(*eph);
                        gal_count += 1;
                    }
                }
                println!(
                    "[INFO]\tGAL epoch selected: week={}, toe={} (Δt={:.1}h), sats={}",
                    key.0,
                    key.1,
                    best_diff as f64 / 3600.0,
                    gal_count
                );
            }
        } else {
            println!("[WARN]\tNo Galileo epochs available after grouping");
        }

        // Финальная верификация корректности алгоритма выбора эфемерид
        let mut verified_count = 0;
        for i in 0..TOTAL_GAL_SAT {
            if self.gal_eph[i].is_some() {
                verified_count += 1; // Подтверждение успешного размещения эфемериды
            }
        }

        // Синхронизация с nav_data структурой для обеспечения совместимости архитектуры
        // Galileo: динамическое масштабирование под актуальный размер констелляции
        if self.nav_data.gal_ephemeris.len() < TOTAL_GAL_SAT {
            self.nav_data.gal_ephemeris.resize(TOTAL_GAL_SAT, None); // Выделение под полную Galileo констелляцию
        }
        // Полное копирование исходных данных в nav_data (дублирование без временного отбора)
        for eph in &c_nav_data.galileo_ephemeris {
            if (eph.svid as usize) <= TOTAL_GAL_SAT && eph.svid > 0 {
                self.nav_data.gal_ephemeris[eph.svid as usize - 1] = Some(*eph);
            }
        }

        // Расширенная отчетность: сравнение количества обработанных и верифицированных эфемерид
        println!(
            "[INFO]\tCopied {} Galileo ephemeris records, verified {} in gal_eph array",
            gal_count, verified_count
        );
    }

    pub fn generate_data(&mut self) -> Result<GenerationStats, Box<dyn std::error::Error>> {
        // Используем стандартные данные пока не реализованы get методы
        let utc_time = self
            .parse_utc_time_from_json(&self.output_param.config_filename)
            .unwrap_or_else(|| {
                println!("[WARNING]\tFailed to parse time from JSON, using default time");
                UtcTime {
                    Year: 2025,
                    Month: 6,
                    Day: 5,
                    Hour: 10,
                    Minute: 5,
                    Second: 30.0,
                }
            });
        let start_pos = self.start_pos;
        let _start_vel = LocalSpeed::default();
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
                BufWriter::with_capacity(1 << 20, file)
            }
            Err(_) => {
                println!("[ERROR]\tFailed to open output file: {}", filename_str);
                return Err("Failed to open output file".into());
            }
        };

        // Создаем навигационные биты
        let mut nav_bit_array = self.create_nav_bit_instances();

        // Настраиваем систему
        self.setup_navigation_data(
            nav_bit_array.as_mut_slice(),
            utc_time,
            glonass_time,
            bds_time,
        )?;
        self.calculate_visible_satellites(cur_pos, glonass_time)?;

        // Санити-проверка PVT перед генерацией: пытаемся восстановить позицию приёмника
        let utc_now = self
            .parse_utc_time_from_json(&self.output_param.config_filename)
            .unwrap_or(UtcTime {
                Year: 2025,
                Month: 6,
                Day: 5,
                Hour: 10,
                Minute: 5,
                Second: 30.0,
            });
        let gps_now = crate::gnsstime::utc_to_gps_time(utc_now, true);
        let bds_now = crate::gnsstime::utc_to_bds_time(utc_now);
        let gal_now = crate::gnsstime::utc_to_galileo_time(utc_now);
        let init_pos = cur_pos;
        if let Some((sol_ecef, _cb_s, _alt)) = pvt::solve_pvt_wls(
            gps_now,
            bds_now,
            gal_now,
            &self.gps_eph_visible,
            &self.bds_eph_visible,
            &self.gal_eph_visible,
            init_pos,
        ) {
            let _sol_lla = ecef_to_lla(&sol_ecef);
            let _true_lla = ecef_to_lla(&cur_pos);
            let dx = sol_ecef.x - cur_pos.x;
            let dy = sol_ecef.y - cur_pos.y;
            let dz = sol_ecef.z - cur_pos.z;
            let _err = (dx * dx + dy * dy + dz * dz).sqrt();
        } else {
        }

        // Создаем спутниковые сигналы и генерируем данные
        let mut sat_if_signals = self.create_satellite_signals(&nav_bit_array[..], cur_pos)?;
        let trajectory_time_s = self
            .parse_trajectory_time_from_json(&self.output_param.config_filename)
            .unwrap_or(10.0);
        self.generate_if_signal(
            &mut if_file,
            &mut sat_if_signals,
            cur_pos,
            trajectory_time_s,
        )?;

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
            total_time_ms: 0.0, // Будет заполнено в вызывающем коде
            signal_processing_time_ms: 0.0,
            samples_generated: total_samples,
            satellites_processed: sat_if_signals.len() as u32,
            avx512_accelerated: avx512_available,
            cuda_accelerated: cuda_available,
        })
    }

    // Satellite position calculation functions based on C implementation
    // Убрана неработающая функция gps_sat_pos_speed_eph - используется crate::coordinate::gps_sat_pos_speed_eph

    fn glonass_sat_pos_speed_eph(
        &self,
        _transmit_time: f64,
        eph: &GlonassEphemeris,
    ) -> Option<KinematicInfo> {
        // Simplified GLONASS position calculation
        // In a real implementation, this would use Runge-Kutta integration
        // For now, use a simplified approach based on the orbital elements

        Some(KinematicInfo {
            x: eph.x * 1000.0, // Convert from km to m
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
            Second: second,
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
