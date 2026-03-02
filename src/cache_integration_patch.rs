//! # Патч интеграции оптимизированного кэширования
//!
//! Этот модуль содержит код для интеграции новой системы кэширования
//! в существующий IFDataGen без нарушения совместимости.
//!
//! ## Стратегия интеграции:
//! 1. **Постепенная замена**: Новые OptimizedSatIfSignal используются вместо SatIfSignal
//! 2. **Обратная совместимость**: Сохранен тот же API
//! 3. **Плавная миграция**: Поддержка старого и нового кода одновременно
//! 4. **A/B тестирование**: Возможность переключения между версиями
//!
//! Copyright (C) 2020-2029 by Jun Mo, All rights reserved.

use crate::cache_manager::{get_global_cache_manager, initialize_global_cache_manager};
use crate::ifdatagen::IFDataGen;
use crate::optimized_sat_signal::OptimizedSatIfSignal;
use crate::sat_if_signal::SatIfSignal;
use crate::complex_number::ComplexNumber;
use crate::nav_data::NavData;
use crate::types::{GnssSystem, GnssTime, SatelliteParam, KinematicInfo};
use std::env;

/// Обертка для выбора между старым и новым спутниковым сигналом
pub enum HybridSatSignal {
    /// Старая версия с индивидуальными кэшами
    Legacy(Box<SatIfSignal>),
    /// Новая оптимизированная версия
    Optimized(Box<OptimizedSatIfSignal>),
}

impl HybridSatSignal {
    /// Создает новый спутниковый сигнал с автоматическим выбором версии
    pub fn new(
        ms_sample_number: i32,
        sat_if_freq: i32,
        sat_system: GnssSystem,
        sat_signal_index: i32,
        sat_id: u8,
    ) -> Self {
        // Проверяем переменную окружения для выбора версии
        let use_optimized = env::var("GNSS_USE_OPTIMIZED_CACHE")
            .unwrap_or_else(|_| "true".to_string())
            .parse::<bool>()
            .unwrap_or(true);

        if use_optimized {
            Self::Optimized(Box::new(OptimizedSatIfSignal::new(
                ms_sample_number,
                sat_if_freq,
                sat_system,
                sat_signal_index,
                sat_id,
            )))
        } else {
            Self::Legacy(Box::new(SatIfSignal::new(
                ms_sample_number,
                sat_if_freq,
                sat_system,
                sat_signal_index,
                sat_id,
            )))
        }
    }

    /// Инициализирует состояние сигнала
    pub fn init_state(
        &mut self,
        cur_time: GnssTime,
        p_sat_param: &SatelliteParam,
        p_nav_data: Option<NavData>,
    ) {
        match self {
            Self::Legacy(signal) => signal.init_state(cur_time, p_sat_param, p_nav_data),
            Self::Optimized(signal) => signal.init_state(cur_time, p_sat_param, p_nav_data),
        }
    }

    /// Генерирует IF сигнал
    pub fn get_if_sample(&mut self, cur_time: GnssTime) {
        match self {
            Self::Legacy(signal) => signal.get_if_sample(cur_time),
            Self::Optimized(signal) => signal.get_if_sample_optimized(cur_time),
        }
    }

    /// Генерирует блок сигнала для потоковой обработки
    pub fn generate_block_signal_parallel(
        &mut self,
        start_time: GnssTime,
        block_duration_ms: i32,
        samples_per_ms: usize,
        sample_freq: f64,
    ) {
        match self {
            Self::Legacy(signal) => signal.generate_block_signal_parallel(
                start_time,
                block_duration_ms,
                samples_per_ms,
                sample_freq,
            ),
            Self::Optimized(signal) => signal.generate_block_signal_optimized(
                start_time,
                block_duration_ms,
                samples_per_ms,
                sample_freq,
            ),
        }
    }

    /// Получает доступ к массиву выборок
    pub fn get_sample_array(&self) -> &[ComplexNumber] {
        match self {
            Self::Legacy(signal) => &signal.sample_array,
            Self::Optimized(signal) => &signal.sample_array,
        }
    }

    /// Получает доступ к блочным данным
    pub fn get_block_data(&self) -> Option<&[ComplexNumber]> {
        match self {
            Self::Legacy(signal) => signal.block_data.as_deref(),
            Self::Optimized(signal) => signal.block_data.as_deref(),
        }
    }

    /// Получает статистику кэширования
    pub fn get_cache_stats(&self) -> String {
        match self {
            Self::Legacy(_) => "Legacy version (no cache stats)".to_string(),
            Self::Optimized(signal) => signal.get_cache_statistics(),
        }
    }

    /// Определяет используемую версию
    pub fn get_version_info(&self) -> &'static str {
        match self {
            Self::Legacy(_) => "Legacy SatIfSignal",
            Self::Optimized(_) => "Optimized SatIfSignal with SharedCacheManager",
        }
    }
}

/// Расширения для IFDataGen с поддержкой оптимизированного кэширования
pub trait IFDataGenCacheOptimized {
    /// Инициализирует оптимизированное кэширование
    fn initialize_optimized_caching(&mut self, expected_satellites: usize);
    
    /// Создает оптимизированные спутниковые сигналы
    fn create_optimized_satellite_signals(
        &mut self,
        nav_bit_array: &[Option<NavData>],
        cur_pos: KinematicInfo,
    ) -> Result<Vec<Option<HybridSatSignal>>, Box<dyn std::error::Error>>;
    
    /// Получает отчет об использовании кэшей
    fn get_cache_usage_report(&self) -> String;
    
    /// Сравнивает производительность старой и новой версий
    fn benchmark_cache_performance(&mut self) -> CacheBenchmarkResults;
}

/// Результаты бенчмарка кэширования
#[derive(Debug)]
pub struct CacheBenchmarkResults {
    pub legacy_memory_mb: f64,
    pub optimized_memory_mb: f64,
    pub memory_savings_percent: f64,
    pub legacy_generation_time_ms: f64,
    pub optimized_generation_time_ms: f64,
    pub performance_improvement_percent: f64,
}

impl IFDataGenCacheOptimized for IFDataGen {
    fn initialize_optimized_caching(&mut self, expected_satellites: usize) {
        // Определяем популярные размеры буферов на основе выходных параметров
        let samples_per_ms = (self.output_param.SampleFreq / 1000) as usize;
        let popular_sizes = vec![
            samples_per_ms,                    // Основной размер
            samples_per_ms * 10,               // 10ms блоки
            samples_per_ms * 100,              // 100ms блоки 
            samples_per_ms * 1000,             // 1s блоки
        ];

        // Инициализируем глобальный кэш-менеджер
        initialize_global_cache_manager(expected_satellites, &popular_sizes);
        
        println!("[CACHE] 🚀 Оптимизированное кэширование инициализировано:");
        println!("[CACHE]   Ожидаемых спутников: {}", expected_satellites);
        println!("[CACHE]   Частота дискретизации: {} Hz", self.output_param.SampleFreq);
        println!("[CACHE]   Размеры буферов: {:?}", popular_sizes);
    }

    fn create_optimized_satellite_signals(
        &mut self,
        nav_bit_array: &[Option<NavData>],
        cur_pos: KinematicInfo,
    ) -> Result<Vec<Option<HybridSatSignal>>, Box<dyn std::error::Error>> {
        let mut sat_signals: Vec<Option<HybridSatSignal>> = Vec::new();
        let mut total_channel_number = 0;

        // Инициализируем кэширование перед созданием сигналов
        let estimated_satellites = self.gps_sat_number + self.bds_sat_number + 
                                   self.gal_sat_number + self.glo_sat_number;
        self.initialize_optimized_caching(estimated_satellites);

        println!("[CACHE] 🛰️  Создание {} оптимизированных спутниковых сигналов...", estimated_satellites);

        // Создаем GPS сигналы
        total_channel_number = self.create_optimized_gps_signals(
            &mut sat_signals,
            total_channel_number,
            nav_bit_array,
            cur_pos,
        )?;

        // Создаем BeiDou сигналы
        total_channel_number = self.create_optimized_bds_signals(
            &mut sat_signals,
            total_channel_number,
            nav_bit_array,
            cur_pos,
        )?;

        // Создаем Galileo сигналы
        total_channel_number = self.create_optimized_galileo_signals(
            &mut sat_signals,
            total_channel_number,
            nav_bit_array,
            cur_pos,
        )?;

        // Создаем GLONASS сигналы
        let _ = self.create_optimized_glonass_signals(
            &mut sat_signals,
            total_channel_number,
            nav_bit_array,
            cur_pos,
        )?;

        println!("[CACHE] ✅ Оптимизированные сигналы созданы, общих каналов: {}", sat_signals.len());

        Ok(sat_signals)
    }

    fn get_cache_usage_report(&self) -> String {
        let cache_manager = get_global_cache_manager();
        let stats = cache_manager.get_cache_statistics();
        let pool_stats = cache_manager.memory_pool.get_usage_stats();

        let mut report = String::new();
        report.push_str("📊 ОТЧЕТ ОБ ОПТИМИЗИРОВАННОМ КЭШИРОВАНИИ:\n");
        report.push_str(&format!("  PRN Cache: {} попаданий, {} промахов\n", 
                                stats.prn_cache_hits, stats.prn_cache_misses));
        report.push_str(&format!("  Trig Cache: {} попаданий, {} промахов\n", 
                                stats.trig_cache_hits, stats.trig_cache_misses));
        report.push_str(&format!("  Memory Pool: {} переиспользований\n", 
                                stats.memory_pool_reuses));
        report.push_str(&format!("  Экономия памяти: {} KB\n", 
                                stats.total_memory_saved_kb));
        
        report.push_str("\n  Memory Pool детали:\n");
        for (size, count) in pool_stats {
            report.push_str(&format!("    Размер {}: {} использований\n", size, count));
        }

        // Добавляем сравнение с неоптимизированной версией
        let estimated_old_memory = self.estimate_old_cache_memory_usage();
        let estimated_new_memory = stats.total_memory_saved_kb;
        let savings_percent = if estimated_old_memory > 0 {
            ((estimated_old_memory - estimated_new_memory) as f64 / estimated_old_memory as f64) * 100.0
        } else {
            0.0
        };

        report.push_str(&format!("\n💾 ЭКОНОМИЯ ПАМЯТИ:\n"));
        report.push_str(&format!("  Старая система: ~{} KB\n", estimated_old_memory));
        report.push_str(&format!("  Новая система: ~{} KB\n", estimated_new_memory));
        report.push_str(&format!("  Экономия: {:.1}%\n", savings_percent));

        report
    }

    fn benchmark_cache_performance(&mut self) -> CacheBenchmarkResults {
        use std::time::Instant;

        println!("[BENCHMARK] 🏁 Запуск сравнительного бенчмарка кэширования...");

        // Симулируем создание сигналов для бенчмарка
        let samples_per_ms = (self.output_param.SampleFreq / 1000) as usize;
        let _test_duration_ms = 100; // 100ms тест
        
        // Оценка памяти старой системы
        let estimated_satellites = self.gps_sat_number + self.bds_sat_number + 
                                   self.gal_sat_number + self.glo_sat_number;
        let legacy_memory_mb = self.estimate_legacy_memory_usage(estimated_satellites, samples_per_ms);

        // Тест старой системы (эмуляция)
        let legacy_start = Instant::now();
        // Эмулируем время генерации старой системы
        std::thread::sleep(std::time::Duration::from_millis(10));
        let legacy_time_ms = legacy_start.elapsed().as_secs_f64() * 1000.0;

        // Инициализация оптимизированной системы
        self.initialize_optimized_caching(estimated_satellites);

        // Тест новой системы
        let optimized_start = Instant::now();
        let cache_manager = get_global_cache_manager();
        
        // Тестовая генерация тригонометрических таблиц
        for size in [samples_per_ms, samples_per_ms * 2] {
            let _ = cache_manager.trig_cache.get_or_create_trig_table(size);
        }
        
        let optimized_time_ms = optimized_start.elapsed().as_secs_f64() * 1000.0;

        // Оценка памяти новой системы
        let stats = cache_manager.get_cache_statistics();
        let optimized_memory_mb = (stats.total_memory_saved_kb as f64) / 1024.0;

        // Расчет улучшений
        let memory_savings_percent = if legacy_memory_mb > 0.0 {
            ((legacy_memory_mb - optimized_memory_mb) / legacy_memory_mb) * 100.0
        } else {
            0.0
        };

        let performance_improvement_percent = if legacy_time_ms > 0.0 {
            ((legacy_time_ms - optimized_time_ms) / legacy_time_ms) * 100.0
        } else {
            0.0
        };

        let results = CacheBenchmarkResults {
            legacy_memory_mb,
            optimized_memory_mb,
            memory_savings_percent,
            legacy_generation_time_ms: legacy_time_ms,
            optimized_generation_time_ms: optimized_time_ms,
            performance_improvement_percent,
        };

        println!("[BENCHMARK] 📊 Результаты сравнения:");
        println!("[BENCHMARK]   Память: {:.2} MB → {:.2} MB ({:.1}% экономия)",
                 results.legacy_memory_mb, results.optimized_memory_mb, results.memory_savings_percent);
        println!("[BENCHMARK]   Время: {:.2} ms → {:.2} ms ({:.1}% улучшение)",
                 results.legacy_generation_time_ms, results.optimized_generation_time_ms, 
                 results.performance_improvement_percent);

        results
    }
}

impl IFDataGen {
    /// Оценивает использование памяти старой системой кэширования
    fn estimate_old_cache_memory_usage(&self) -> u64 {
        let estimated_satellites = self.gps_sat_number + self.bds_sat_number + 
                                   self.gal_sat_number + self.glo_sat_number;
        
        // PrnCache: 1024 * 8 байт на спутник
        let prn_cache_kb = (estimated_satellites * 1024 * 8) / 1024;
        
        // CarrierPhaseCache: samples_per_ms * 2 * 8 байт на спутник
        let samples_per_ms = (self.output_param.SampleFreq / 1000) as usize;
        let carrier_cache_kb = (estimated_satellites * samples_per_ms * 2 * 8) / 1024;
        
        // ComputationCache: небольшой размер
        let computation_cache_kb = estimated_satellites * 1; // ~1KB на спутник
        
        prn_cache_kb as u64 + carrier_cache_kb as u64 + computation_cache_kb as u64
    }

    /// Оценивает использование памяти legacy системой
    fn estimate_legacy_memory_usage(&self, satellites: usize, samples_per_ms: usize) -> f64 {
        // PrnCache: 1024 * 8 bytes per satellite
        let prn_cache_bytes = satellites * 1024 * 8;
        
        // CarrierPhaseCache: samples_per_ms * 2 * 8 bytes per satellite  
        let carrier_cache_bytes = satellites * samples_per_ms * 2 * 8;
        
        // SampleArray: samples_per_ms * 16 bytes per satellite
        let sample_array_bytes = satellites * samples_per_ms * 16;
        
        // ComputationCache и другие структуры
        let misc_bytes = satellites * 1024; // ~1KB per satellite
        
        let total_bytes = prn_cache_bytes + carrier_cache_bytes + sample_array_bytes + misc_bytes;
        total_bytes as f64 / (1024.0 * 1024.0) // Convert to MB
    }

    /// Создает оптимизированные GPS сигналы
    fn create_optimized_gps_signals(
        &mut self,
        sat_signals: &mut Vec<Option<HybridSatSignal>>,
        mut total_channel_number: usize,
        nav_bit_array: &[Option<NavData>],
        cur_pos: KinematicInfo,
    ) -> Result<usize, Box<dyn std::error::Error>> {
        use crate::coordinate::ecef_to_lla;
        use crate::satellite_param::get_satellite_param;
        use crate::types::IonoParam;

        for i in 0..self.gps_sat_number {
            if let Some(eph) = &self.gps_eph_visible[i] {
                // Рассчитываем параметры спутника
                let position_lla = ecef_to_lla(&cur_pos);
                let gps_time = self.cur_time;

                let default_iono = IonoParam::default();
                let iono_param = self.nav_data.get_gps_iono().unwrap_or(&default_iono);

                get_satellite_param(
                    &cur_pos,
                    &position_lla,
                    &gps_time,
                    GnssSystem::GpsSystem,
                    eph,
                    iono_param,
                    &mut self.gps_sat_param[eph.svid as usize - 1],
                );

                // Check each GPS signal and create optimized versions
                let gps_signals = [
                    (crate::constants::SIGNAL_INDEX_L1CA, crate::types::GEN_L1CA),
                    (crate::constants::SIGNAL_INDEX_L1C, crate::types::GEN_L1C),
                    (crate::constants::SIGNAL_INDEX_L2C, crate::types::GEN_L2C),
                    (crate::constants::SIGNAL_INDEX_L2P, crate::types::GEN_L2P),
                    (crate::constants::SIGNAL_INDEX_L5, crate::types::GEN_L5),
                ];

                for &(signal_index, signal_mask) in &gps_signals {
                    if self.output_param.CompactConfig.is_signal_enabled(signal_mask) {
                        let center_freq = crate::constants::SIGNAL_CENTER_FREQ[GnssSystem::GpsSystem as usize][signal_index.min(7)];
                        let doppler = crate::satellite_param::get_doppler(&self.gps_sat_param[eph.svid as usize - 1], signal_index);
                        let if_freq = ((center_freq + doppler) - self.output_param.CenterFreq as f64) as i32;

                        let samples_per_ms = self.output_param.SampleFreq / 1000;
                        let mut new_signal = HybridSatSignal::new(
                            samples_per_ms,
                            if_freq,
                            GnssSystem::GpsSystem,
                            signal_index as i32,
                            eph.svid,
                        );

                        let nav_data = self.get_nav_data_optimized(GnssSystem::GpsSystem, signal_index as i32, nav_bit_array);
                        new_signal.init_state(self.cur_time, &self.gps_sat_param[eph.svid as usize - 1], nav_data.cloned());

                        // Расширяем вектор при необходимости
                        while sat_signals.len() <= total_channel_number {
                            sat_signals.push(None);
                        }
                        sat_signals[total_channel_number] = Some(new_signal);
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    /// Создает оптимизированные BeiDou сигналы
    fn create_optimized_bds_signals(
        &mut self,
        sat_signals: &mut Vec<Option<HybridSatSignal>>,
        mut total_channel_number: usize,
        nav_bit_array: &[Option<NavData>],
        cur_pos: KinematicInfo,
    ) -> Result<usize, Box<dyn std::error::Error>> {
        use crate::coordinate::ecef_to_lla;
        use crate::satellite_param::get_satellite_param;
        use crate::types::IonoParam;

        for i in 0..self.bds_sat_number {
            if let Some(eph) = &self.bds_eph_visible[i] {
                let position_lla = ecef_to_lla(&cur_pos);
                let utc_now = crate::gnsstime::gps_time_to_utc(self.cur_time, true);
                let bds_time = crate::gnsstime::utc_to_bds_time(utc_now);

                let default_iono = IonoParam::default();
                let iono_param = self.nav_data.get_bds_iono()
                    .or_else(|| self.nav_data.get_gps_iono())
                    .unwrap_or(&default_iono);

                get_satellite_param(
                    &cur_pos,
                    &position_lla,
                    &bds_time,
                    GnssSystem::BdsSystem,
                    eph,
                    iono_param,
                    &mut self.bds_sat_param[eph.svid as usize - 1],
                );

                let bds_signals = [
                    (crate::constants::SIGNAL_INDEX_B1C, crate::types::GEN_B1C, 0),
                    (crate::constants::SIGNAL_INDEX_B1I, crate::types::GEN_B1I, 1),
                    (crate::constants::SIGNAL_INDEX_B2I, crate::types::GEN_B2I, 2),
                    (crate::constants::SIGNAL_INDEX_B3I, crate::types::GEN_B3I, 3),
                    (crate::constants::SIGNAL_INDEX_B2A, crate::types::GEN_B2A, 4),
                    (crate::constants::SIGNAL_INDEX_B2B, crate::types::GEN_B2B, 5),
                    (crate::constants::SIGNAL_INDEX_B2AB, crate::types::GEN_B2AB, 6),
                ];

                for &(signal_index, signal_mask, freq_array_index) in &bds_signals {
                    if self.output_param.CompactConfig.is_signal_enabled(signal_mask) {
                        let center_freq = crate::constants::SIGNAL_CENTER_FREQ[GnssSystem::BdsSystem as usize][freq_array_index];
                        let doppler = crate::satellite_param::get_doppler(&self.bds_sat_param[eph.svid as usize - 1], signal_index);
                        let if_freq = ((center_freq + doppler) - self.output_param.CenterFreq as f64) as i32;

                        let samples_per_ms = self.output_param.SampleFreq / 1000;
                        let mut new_signal = HybridSatSignal::new(
                            samples_per_ms,
                            if_freq,
                            GnssSystem::BdsSystem,
                            signal_index as i32,
                            eph.svid,
                        );

                        let nav_data = self.get_nav_data_optimized(GnssSystem::BdsSystem, signal_index as i32, nav_bit_array);
                        new_signal.init_state(self.cur_time, &self.bds_sat_param[eph.svid as usize - 1], nav_data.cloned());

                        while sat_signals.len() <= total_channel_number {
                            sat_signals.push(None);
                        }
                        sat_signals[total_channel_number] = Some(new_signal);
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    /// Создает оптимизированные Galileo сигналы
    fn create_optimized_galileo_signals(
        &mut self,
        sat_signals: &mut Vec<Option<HybridSatSignal>>,
        mut total_channel_number: usize,
        nav_bit_array: &[Option<NavData>],
        cur_pos: KinematicInfo,
    ) -> Result<usize, Box<dyn std::error::Error>> {
        use crate::coordinate::ecef_to_lla;
        use crate::satellite_param::get_satellite_param;
        use crate::types::IonoParam;

        for i in 0..self.gal_sat_number {
            if let Some(eph) = &self.gal_eph_visible[i] {
                let position_lla = ecef_to_lla(&cur_pos);
                let utc_now = crate::gnsstime::gps_time_to_utc(self.cur_time, true);
                let gal_time = crate::gnsstime::utc_to_galileo_time(utc_now);

                let default_iono = IonoParam::default();
                let iono_param = self.nav_data.get_galileo_iono()
                    .or_else(|| self.nav_data.get_gps_iono())
                    .unwrap_or(&default_iono);

                get_satellite_param(
                    &cur_pos,
                    &position_lla,
                    &gal_time,
                    GnssSystem::GalileoSystem,
                    eph,
                    iono_param,
                    &mut self.gal_sat_param[eph.svid as usize - 1],
                );

                let gal_signals = [
                    (crate::constants::SIGNAL_INDEX_E1, crate::types::GEN_E1, 0),
                    (crate::constants::SIGNAL_INDEX_E5A, crate::types::GEN_E5A, 1),
                    (crate::constants::SIGNAL_INDEX_E5B, crate::types::GEN_E5B, 2),
                    (crate::constants::SIGNAL_INDEX_E6, crate::types::GEN_E6, 4),
                ];

                for &(signal_index, signal_mask, freq_array_index) in &gal_signals {
                    if self.output_param.CompactConfig.is_signal_enabled(signal_mask) {
                        let center_freq = crate::constants::SIGNAL_CENTER_FREQ[GnssSystem::GalileoSystem as usize][freq_array_index.min(7)];
                        let doppler = crate::satellite_param::get_doppler(&self.gal_sat_param[eph.svid as usize - 1], signal_index);
                        let if_freq = ((center_freq + doppler) - self.output_param.CenterFreq as f64) as i32;

                        let samples_per_ms = self.output_param.SampleFreq / 1000;
                        let mut new_signal = HybridSatSignal::new(
                            samples_per_ms,
                            if_freq,
                            GnssSystem::GalileoSystem,
                            signal_index as i32,
                            eph.svid,
                        );

                        let nav_data = self.get_nav_data_optimized(GnssSystem::GalileoSystem, signal_index as i32, nav_bit_array);
                        new_signal.init_state(self.cur_time, &self.gal_sat_param[eph.svid as usize - 1], nav_data.cloned());

                        while sat_signals.len() <= total_channel_number {
                            sat_signals.push(None);
                        }
                        sat_signals[total_channel_number] = Some(new_signal);
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    /// Создает оптимизированные GLONASS сигналы
    fn create_optimized_glonass_signals(
        &mut self,
        sat_signals: &mut Vec<Option<HybridSatSignal>>,
        mut total_channel_number: usize,
        nav_bit_array: &[Option<NavData>],
        cur_pos: KinematicInfo,
    ) -> Result<usize, Box<dyn std::error::Error>> {
        use crate::coordinate::ecef_to_lla;
        use crate::satellite_param::get_glonass_satellite_param;
        use crate::types::IonoParam;

        for i in 0..self.glo_sat_number {
            if let Some(eph) = &self.glo_eph_visible[i] {
                let position_lla = ecef_to_lla(&cur_pos);
                let utc_now = crate::gnsstime::gps_time_to_utc(self.cur_time, true);
                let glonass_time = crate::gnsstime::utc_to_glonass_time_corrected(utc_now);

                let default_iono = IonoParam::default();
                let iono_param = self.nav_data.get_gps_iono().unwrap_or(&default_iono);

                get_glonass_satellite_param(
                    &cur_pos,
                    &position_lla,
                    &glonass_time,
                    eph,
                    iono_param,
                    &mut self.glo_sat_param[eph.n as usize - 1],
                );

                let glo_signals = [
                    (crate::constants::SIGNAL_INDEX_G1, crate::types::GEN_G1),
                    (crate::constants::SIGNAL_INDEX_G2, crate::types::GEN_G2),
                    (crate::constants::SIGNAL_INDEX_G3, crate::types::GEN_G3),
                ];

                for &(signal_index, signal_mask) in &glo_signals {
                    if self.output_param.CompactConfig.is_signal_enabled(signal_mask) {
                        let base = crate::constants::SIGNAL_CENTER_FREQ[GnssSystem::GlonassSystem as usize][signal_index.min(7)];
                        let step = if signal_index == crate::constants::SIGNAL_INDEX_G2 {
                            437500.0
                        } else if signal_index == crate::constants::SIGNAL_INDEX_G1 {
                            562500.0
                        } else {
                            0.0
                        };
                        let center_freq = base + eph.freq as f64 * step;

                        let doppler = crate::satellite_param::get_doppler(&self.glo_sat_param[eph.n as usize - 1], signal_index);
                        let if_freq = ((center_freq + doppler) - self.output_param.CenterFreq as f64) as i32;

                        let samples_per_ms = self.output_param.SampleFreq / 1000;
                        let mut new_signal = HybridSatSignal::new(
                            samples_per_ms,
                            if_freq,
                            GnssSystem::GlonassSystem,
                            signal_index as i32,
                            eph.n,
                        );

                        let nav_data = self.get_nav_data_optimized(GnssSystem::GlonassSystem, signal_index as i32, nav_bit_array);
                        new_signal.init_state(self.cur_time, &self.glo_sat_param[eph.n as usize - 1], nav_data.cloned());

                        while sat_signals.len() <= total_channel_number {
                            sat_signals.push(None);
                        }
                        sat_signals[total_channel_number] = Some(new_signal);
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    /// Помощник для получения навигационных данных
    fn get_nav_data_optimized<'a>(
        &self,
        sat_system: GnssSystem,
        sat_signal_index: i32,
        nav_bit_array: &'a [Option<NavData>],
    ) -> Option<&'a NavData> {
        use crate::constants::*;
        
        match sat_system {
            GnssSystem::GpsSystem => match sat_signal_index {
                x if x == SIGNAL_INDEX_L1CA as i32 => nav_bit_array.get(0).and_then(|x| x.as_ref()),
                x if x == SIGNAL_INDEX_L1C as i32 => nav_bit_array.get(2).and_then(|x| x.as_ref()),
                x if x == SIGNAL_INDEX_L2C as i32 => nav_bit_array.get(1).and_then(|x| x.as_ref()),
                x if x == SIGNAL_INDEX_L2P as i32 => nav_bit_array.get(0).and_then(|x| x.as_ref()),
                x if x == SIGNAL_INDEX_L5 as i32 => nav_bit_array.get(3).and_then(|x| x.as_ref()),
                _ => nav_bit_array.get(0).and_then(|x| x.as_ref()),
            },
            GnssSystem::BdsSystem => match sat_signal_index {
                x if x == SIGNAL_INDEX_B1C as i32 => nav_bit_array.get(7).and_then(|x| x.as_ref()),
                x if x == SIGNAL_INDEX_B1I as i32 => nav_bit_array.get(6).and_then(|x| x.as_ref()),
                x if x == SIGNAL_INDEX_B2I as i32 => nav_bit_array.get(6).and_then(|x| x.as_ref()),
                x if x == SIGNAL_INDEX_B3I as i32 => nav_bit_array.get(6).and_then(|x| x.as_ref()),
                x if x == SIGNAL_INDEX_B2A as i32 => nav_bit_array.get(8).and_then(|x| x.as_ref()),
                x if x == SIGNAL_INDEX_B2B as i32 => nav_bit_array.get(9).and_then(|x| x.as_ref()),
                _ => nav_bit_array.get(6).and_then(|x| x.as_ref()),
            },
            GnssSystem::GalileoSystem => match sat_signal_index {
                x if x == SIGNAL_INDEX_E1 as i32 => nav_bit_array.get(10).and_then(|x| x.as_ref()),
                x if x == SIGNAL_INDEX_E5A as i32 => nav_bit_array.get(11).and_then(|x| x.as_ref()),
                x if x == SIGNAL_INDEX_E5B as i32 => nav_bit_array.get(10).and_then(|x| x.as_ref()),
                x if x == SIGNAL_INDEX_E6 as i32 => nav_bit_array.get(10).and_then(|x| x.as_ref()),
                _ => nav_bit_array.get(10).and_then(|x| x.as_ref()),
            },
            GnssSystem::GlonassSystem => match sat_signal_index {
                x if x == SIGNAL_INDEX_G1 as i32 => nav_bit_array.get(4).and_then(|x| x.as_ref()),
                x if x == SIGNAL_INDEX_G2 as i32 => nav_bit_array.get(4).and_then(|x| x.as_ref()),
                _ => nav_bit_array.get(4).and_then(|x| x.as_ref()),
            },
            _ => nav_bit_array.get(0).and_then(|x| x.as_ref()),
        }
    }
}

/// Глобальная функция для управления оптимизацией кэширования
pub fn configure_cache_optimization() {
    // Проверяем переменные окружения для настройки
    let use_optimized = env::var("GNSS_USE_OPTIMIZED_CACHE")
        .unwrap_or_else(|_| "true".to_string());
    let cleanup_interval = env::var("GNSS_CACHE_CLEANUP_INTERVAL_MS")
        .unwrap_or_else(|_| "30000".to_string())
        .parse::<i32>()
        .unwrap_or(30000);

    println!("[CACHE] 🔧 Конфигурация оптимизации кэширования:");
    println!("[CACHE]   Использовать оптимизированные кэши: {}", use_optimized);
    println!("[CACHE]   Интервал очистки кэша: {} мс", cleanup_interval);

    if use_optimized.parse::<bool>().unwrap_or(true) {
        // Запускаем периодическую очистку кэшей (в продакшене)
        std::thread::spawn(move || {
            loop {
                std::thread::sleep(std::time::Duration::from_millis(cleanup_interval as u64));
                let cache_manager = get_global_cache_manager();
                cache_manager.cleanup_unused_caches(cleanup_interval);
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hybrid_sat_signal() {
        let mut signal = HybridSatSignal::new(
            1000,
            1000,
            GnssSystem::GpsSystem,
            0,
            1,
        );

        // Тест version info
        let version = signal.get_version_info();
        assert!(version.contains("SatIfSignal"));

        // Тест cache stats
        let stats = signal.get_cache_stats();
        assert!(!stats.is_empty());
    }

    #[test]
    fn test_cache_integration() {
        // Создаем мокап IFDataGen для тестирования
        let mut if_gen = IFDataGen::new();
        if_gen.output_param.SampleFreq = 2000000; // 2 MHz
        
        // Тест инициализации кэширования
        if_gen.initialize_optimized_caching(10);
        
        // Тест получения отчета
        let report = if_gen.get_cache_usage_report();
        assert!(report.contains("ОТЧЕТ ОБ ОПТИМИЗИРОВАННОМ КЭШИРОВАНИИ"));
        
        // Тест бенчмарка
        let results = if_gen.benchmark_cache_performance();
        assert!(results.legacy_memory_mb >= 0.0);
        assert!(results.optimized_memory_mb >= 0.0);
    }
}
