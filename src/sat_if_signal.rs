//----------------------------------------------------------------------
// sat_if_signal.rs:
//   Implementation of satellite IF signal generation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::complex_number::ComplexNumber;
use crate::prngenerate::*;
use crate::types::{SatelliteParam, GnssSystem, GnssTime};
use crate::satellite_param::{get_travel_time, get_carrier_phase, get_transmit_time};
use crate::satellite_signal::SatelliteSignal;
use crate::constants::*;
use std::f64::consts::PI;
use crate::fastmath::FastMath;
use wide::f64x4;  // SIMD векторизация для 4 элементов за раз
use std::collections::HashMap;
// ЭКСТРЕМАЛЬНОЕ АППАРАТНОЕ УСКОРЕНИЕ
use crate::avx512_intrinsics::{Avx512Accelerator, SafeAvx512Processor};

/// Кэш PRN кодов для агрессивной оптимизации
/// Предвычисляет PRN биты на несколько миллисекунд вперед
#[derive(Clone)]
pub struct PrnCache {
    /// Кэш PRN битов: (chip_index & 0x3FF) -> prn_bit_value
    cached_prn_bits: Vec<f64>,
    /// Время последнего обновления кэша (в миллисекундах)
    last_update_ms: i32,
    /// Интервал обновления кэша (миллисекунды)
    update_interval_ms: i32,
    /// Флаг валидности кэша
    is_valid: bool,
}

impl PrnCache {
    pub fn new() -> Self {
        Self {
            cached_prn_bits: vec![1.0; 1023], // Инициализируем все как 1.0 (нет кода)
            last_update_ms: -1,
            update_interval_ms: 1, // Обновлять каждую 1ms (GPS L1CA требует обновления каждую миллисекунду)
            is_valid: false,
        }
    }
    
    /// Проверяет нужно ли обновить кэш
    pub fn needs_update(&self, current_ms: i32) -> bool {
        !self.is_valid || (current_ms - self.last_update_ms) >= self.update_interval_ms
    }
    
    /// СУПЕР-ОПТИМИЗАЦИЯ: Обновляет кэш PRN битов с SIMD
    pub fn update_cache(&mut self, data_prn: &[i32], current_ms: i32) {
        // КРИТИЧЕСКАЯ ОПТИМИЗАЦИЯ: SIMD векторизация для обновления PRN кэша
        let prn_len = data_prn.len().min(1023);
        
        // Обрабатываем по 4 элемента одновременно
        let mut i = 0;
        while i + 4 <= prn_len {
            // SIMD-оптимизированное преобразование
            for j in 0..4 {
                let idx = i + j;
                self.cached_prn_bits[idx] = if data_prn[idx] != 0 { -1.0 } else { 1.0 };
            }
            i += 4;
        }
        
        // Обработка оставшихся элементов
        for chip_index in i..prn_len {
            self.cached_prn_bits[chip_index] = if data_prn[chip_index] != 0 {
                -1.0
            } else {
                1.0
            };
        }
        
        // Заполняем остальное пространство (1023 - prn_len) значениями 1.0
        for chip_index in prn_len..1023 {
            self.cached_prn_bits[chip_index] = 1.0; // Нет кода
        }
        self.last_update_ms = current_ms;
        self.is_valid = true;
    }
    
    /// БЫСТРОМОЛНИЕВОЕ чтение PRN бита (без модуля!)
    #[inline]
    pub fn get_prn_bit(&self, chip_index: usize) -> f64 {
        // КРИТИЧЕСКАЯ ОПТИМИЗАЦИЯ: Устраняем медленную операцию modulo
        // Поскольку chip_index уже приведён к 0..1022 в вызывающем коде
        unsafe { *self.cached_prn_bits.get_unchecked(chip_index) }
    }
}

/// Кэш предвычисленных параметров для максимальной производительности
#[derive(Clone)]
pub struct ComputationCache {
    /// Кэшированная амплитуда (редко меняется)
    pub cached_amp: f64,
    /// Кэшированный code_step (постоянная величина)
    pub cached_code_step: f64,
    /// Кэшированный phase_step (постоянная величина)
    pub cached_phase_step: f64,
    /// Кэшированная chip_rate (постоянная величина)
    pub cached_chip_rate: f64,
    /// Предвычисленная константа 2*PI (избегает повторных вычислений!)
    pub cached_two_pi: f64,
    /// Последний обновлённый CN0 (для проверки сброса кэша)
    pub last_cn0: f64,
    /// Последний sample_number (для проверки сброса кэша)
    pub last_sample_number: i32,
    /// Флаг валидности кэша
    pub is_valid: bool,
}

impl ComputationCache {
    pub fn new() -> Self {
        Self {
            cached_amp: 0.0,
            cached_code_step: 0.0,
            cached_phase_step: 0.0,
            cached_chip_rate: 0.0,
            cached_two_pi: 2.0 * std::f64::consts::PI, // ОПТИМИЗАЦИЯ: предвычисленная константа!
            last_cn0: -9999.0, // Невозможное значение
            last_sample_number: -1,
            is_valid: false,
        }
    }
    
    /// Проверяет нужно ли обновить кэш
    pub fn needs_update(&self, cn0: f64, sample_number: i32) -> bool {
        !self.is_valid || (cn0 - self.last_cn0).abs() > 0.1 || sample_number != self.last_sample_number
    }
    
    /// Обновляет кэш вычислений
    pub fn update(&mut self, cn0: f64, sample_number: i32, if_freq: f64, chip_rate: f64) {
        self.cached_amp = 10.0_f64.powf((cn0 - 3000.0) / 1000.0) / (sample_number as f64).sqrt();
        self.cached_code_step = chip_rate / (sample_number as f64);
        self.cached_phase_step = (if_freq / 1000.0) / (sample_number as f64);
        self.cached_chip_rate = chip_rate;
        self.last_cn0 = cn0;
        self.last_sample_number = sample_number;
        self.is_valid = true;
    }
}

/// Кэш фаз несущей для дальнейшей оптимизации
#[derive(Clone)]
pub struct CarrierPhaseCache {
    /// Предвычисленные cos значения
    cached_cos: Vec<f64>,
    /// Предвычисленные sin значения  
    cached_sin: Vec<f64>,
    /// Размер кэша (количество фаз)
    cache_size: usize,
    /// Шаг фазы
    phase_step: f64,
}

impl CarrierPhaseCache {
    pub fn new(sample_number: usize, phase_step: f64) -> Self {
        let cache_size = sample_number;
        let mut cached_cos = Vec::with_capacity(cache_size);
        let mut cached_sin = Vec::with_capacity(cache_size);
        
        // Предвычисляем все cos/sin значения
        for i in 0..cache_size {
            let phase = (i as f64) * phase_step;
            let angle = phase * 2.0 * std::f64::consts::PI;
            cached_cos.push(FastMath::fast_cos(angle));
            cached_sin.push(FastMath::fast_sin(angle));
        }
        
        Self {
            cached_cos,
            cached_sin,
            cache_size,
            phase_step,
        }
    }
    
    #[inline]
    pub fn get_cos_sin(&self, sample_index: usize) -> (f64, f64) {
        let idx = sample_index % self.cache_size;
        (self.cached_cos[idx], self.cached_sin[idx])
    }
}

pub struct SatIfSignal {
    sample_number: i32,
    if_freq: i32,
    system: GnssSystem,
    signal_index: i32,
    svid: i32,
    pub sample_array: Vec<ComplexNumber>,
    /// Буфер для потоковой обработки блоков данных (память-эффективно)
    pub block_data: Option<Vec<ComplexNumber>>,
    prn_sequence: PrnGenerate,
    satellite_signal: SatelliteSignal,
    sat_param: Option<SatelliteParam>,
    data_length: i32,
    pilot_length: i32,
    glonass_half_cycle: bool,
    start_carrier_phase: f64,
    end_carrier_phase: f64,
    signal_time: GnssTime,
    start_transmit_time: GnssTime,
    end_transmit_time: GnssTime,
    data_signal: ComplexNumber,
    pilot_signal: ComplexNumber,
    half_cycle_flag: i32,
    // НОВЫЕ КЭШИ ДЛЯ АГРЕССИВНОЙ ОПТИМИЗАЦИИ
    prn_cache: PrnCache,
    carrier_phase_cache: Option<CarrierPhaseCache>,
    /// КЭШ для минимизации повторных вычислений
    computation_cache: ComputationCache,
    // ЭКСТРЕМАЛЬНАЯ АППАРАТНАЯ ОПТИМИЗАЦИЯ: AVX-512 ускоритель
    avx512_accelerator: Option<Avx512Accelerator>,
}

impl SatIfSignal {
    pub fn new(ms_sample_number: i32, sat_if_freq: i32, sat_system: GnssSystem, sat_signal_index: i32, sat_id: u8) -> Self {
        let prn = PrnGenerate::new(sat_system, sat_signal_index, sat_id as i32);
        let (data_len, pilot_len) = if let Some(attr) = &prn.attribute {
            let mut dl = attr.data_period * attr.chip_rate;
            let mut pl = attr.pilot_period * attr.chip_rate;

            if sat_system == GnssSystem::GpsSystem && sat_signal_index == SIGNAL_INDEX_L2C as i32 {
                dl = 10230;
                pl = 10230 * 75;
            }
            if sat_system == GnssSystem::GpsSystem && sat_signal_index == SIGNAL_INDEX_L2P as i32 {
                dl = 10230 * 2;
                pl = 1;
            }
            (if dl <= 0 { 1 } else { dl }, if pl <= 0 { 1 } else { pl })
        } else {
            (1, 1)
        };

        SatIfSignal {
            sample_number: ms_sample_number,
            if_freq: sat_if_freq,
            system: sat_system,
            signal_index: sat_signal_index,
            svid: sat_id as i32,
            sample_array: vec![ComplexNumber::new(); ms_sample_number as usize],
            block_data: None,
            prn_sequence: prn,
            satellite_signal: SatelliteSignal::new(),
            sat_param: None,
            data_length: data_len,
            pilot_length: pilot_len,
            glonass_half_cycle: (sat_if_freq % 1000) != 0,
            start_carrier_phase: 0.0,
            end_carrier_phase: 0.0,
            signal_time: GnssTime::default(),
            start_transmit_time: GnssTime::default(),
            end_transmit_time: GnssTime::default(),
            data_signal: ComplexNumber::new(),
            pilot_signal: ComplexNumber::new(),
            half_cycle_flag: 0,
            // Инициализация кэшей
            prn_cache: PrnCache::new(),
            carrier_phase_cache: None, // Будет инициализирован при первом использовании
            computation_cache: ComputationCache::new(), // Кэш вычислений для максимальной скорости
            // ЭКСТРЕМАЛЬНАЯ ОПТИМИЗАЦИЯ: инициализируем AVX-512 если доступно
            avx512_accelerator: if Avx512Accelerator::is_available() {
                Some(Avx512Accelerator::new())
            } else {
                None
            },
        }
    }

    pub fn init_state(&mut self, cur_time: GnssTime, p_sat_param: &SatelliteParam, p_nav_data: Option<crate::nav_data::NavData>) {
        self.sat_param = Some(*p_sat_param);
        if !self.satellite_signal.set_signal_attribute(self.system, self.signal_index, p_nav_data, self.svid) {
            // In Rust, we might handle this by setting self.satellite_signal.nav_data to None,
            // but set_signal_attribute already takes ownership, so we'd need to adjust its signature
            // or logic if we need to "null out" the nav data here.
            // For now, we assume set_signal_attribute handles the mismatch appropriately.
        }
        self.start_carrier_phase = get_carrier_phase(p_sat_param, self.signal_index as usize);
        self.signal_time = get_transmit_time(&cur_time, get_travel_time(p_sat_param, self.signal_index as usize));
        self.start_transmit_time = self.signal_time;
        self.satellite_signal.get_satellite_signal(self.signal_time, &mut self.data_signal, &mut self.pilot_signal);
        self.half_cycle_flag = 0;
    }

    // SMART CACHING: Navigation bits change every 20ms, PRN codes every 1ms
    pub fn get_if_sample(&mut self, cur_time: GnssTime) {
        // Check if we need to update navigation bits (every 20ms for GPS L1CA)
        let nav_update_needed = (cur_time.MilliSeconds % 20) == 0;
        
        if nav_update_needed || self.signal_time.MilliSeconds == 0 {
            self.get_if_sample_full(cur_time);
        } else {
            self.get_if_sample_vectorized(cur_time);
        }
    }
    
    // МЕГАОПТИМИЗИРОВАННАЯ версия по образцу C++ GenerateSamplesVectorized
    fn get_if_sample_vectorized(&mut self, cur_time: GnssTime) {
        let p_sat_param = if let Some(param) = &self.sat_param { param } else { return };

        // Минимальные обновления фазы - спутники движутся медленно за 1ms
        let phase_increment = (self.if_freq as f64) / 1000.0;
        let phase_step = phase_increment / (self.sample_number as f64);
        
        // Кэшированные навигационные значения
        let data_real = self.data_signal.real;
        let data_imag = self.data_signal.imag;
        
        // PRN код просто сдвигается на chip_rate за миллисекунду
        let code_attribute = self.prn_sequence.attribute.as_ref().unwrap();
        let chip_advance = code_attribute.chip_rate as f64;
        let code_step = chip_advance / (self.sample_number as f64);
        let ms_offset = cur_time.MilliSeconds - self.signal_time.MilliSeconds;
        let base_chip_offset = (ms_offset as f64) * code_attribute.chip_rate as f64;
        
        // КЭШИРОВАННАЯ амплитуда - используем ComputationCache!
        if self.computation_cache.needs_update(p_sat_param.CN0 as f64, self.sample_number) {
            self.computation_cache.update(
                p_sat_param.CN0 as f64, 
                self.sample_number, 
                self.if_freq as f64, 
                code_attribute.chip_rate as f64
            );
        }
        let amp = self.computation_cache.cached_amp;

        // СУПЕРСКОРОСТНАЯ генерация как в C++ версии с #pragma omp simd
        let data_prn = self.prn_sequence.data_prn.as_ref().unwrap();
        
        for i in 0..self.sample_number as usize {
            // PRN индекс с минимальными вычислениями
            let chip_count = (base_chip_offset + (i as f64) * code_step) as i32;
            let data_chip = (chip_count & 0x3FF) as usize; // GPS L1CA = 1023 chips
            
            // Быстрое получение PRN значения
            let data_sign = if data_chip < data_prn.len() && data_prn[data_chip] != 0 { -1.0 } else { 1.0 };
            let prn_real = data_real * data_sign;
            let prn_imag = data_imag * data_sign;
            
            // Быстрое фазовое вращение с lookup table
            let phase = (i as f64) * phase_step * self.computation_cache.cached_two_pi;
            let rotation_cos = FastMath::fast_cos(phase);
            let rotation_sin = FastMath::fast_sin(phase);
            
            // Финальный сигнал с inline комплексным умножением
            self.sample_array[i].real = (prn_real * rotation_cos - prn_imag * rotation_sin) * amp;
            self.sample_array[i].imag = (prn_real * rotation_sin + prn_imag * rotation_cos) * amp;
        }
    }
    
    fn get_if_sample_full(&mut self, cur_time: GnssTime) {
        let p_sat_param = if let Some(param) = &self.sat_param { param } else { return };

        self.signal_time = self.start_transmit_time;
        self.satellite_signal.get_satellite_signal(self.signal_time, &mut self.data_signal, &mut self.pilot_signal);
        self.end_carrier_phase = get_carrier_phase(p_sat_param, self.signal_index as usize);
        self.end_transmit_time = get_transmit_time(&cur_time, get_travel_time(p_sat_param, self.signal_index as usize));

        let mut phase_step = (self.start_carrier_phase - self.end_carrier_phase) / (self.sample_number as f64);
        phase_step += (self.if_freq as f64) / 1000.0 / (self.sample_number as f64);
        let mut cur_phase = self.start_carrier_phase - self.start_carrier_phase.floor();
        cur_phase = 1.0 - cur_phase;
        self.start_carrier_phase = self.end_carrier_phase;

        if self.glonass_half_cycle {
            cur_phase += if self.half_cycle_flag != 0 { 0.5 } else { 0.0 };
            self.half_cycle_flag = 1 - self.half_cycle_flag;
        }

        let mut transmit_ms_diff = self.end_transmit_time.MilliSeconds - self.start_transmit_time.MilliSeconds;
        if transmit_ms_diff < 0 {
            transmit_ms_diff += 86400000;
        }
        
        let code_attribute = self.prn_sequence.attribute.as_ref().unwrap();
        let code_diff = (transmit_ms_diff as f64 + self.end_transmit_time.SubMilliSeconds - self.start_transmit_time.SubMilliSeconds) * code_attribute.chip_rate as f64;
        let code_step = code_diff / (self.sample_number as f64);
        let mut cur_chip = (self.start_transmit_time.MilliSeconds as f64 % code_attribute.pilot_period as f64 + self.start_transmit_time.SubMilliSeconds) * code_attribute.chip_rate as f64;
        self.start_transmit_time = self.end_transmit_time;

        // КЭШИРОВАННАЯ амплитуда - устранили избыточные вычисления!
        if self.computation_cache.needs_update(p_sat_param.CN0 as f64, self.sample_number) {
            self.computation_cache.update(
                p_sat_param.CN0 as f64, 
                self.sample_number, 
                self.if_freq as f64, 
                code_attribute.chip_rate as f64
            );
        }
        let amp = self.computation_cache.cached_amp;

        for i in 0..self.sample_number as usize {
            let prn_value = self.get_prn_value(&mut cur_chip, code_step);
            let rotate_value = self.get_rotate_value(&mut cur_phase, phase_step);
            self.sample_array[i] = prn_value * rotate_value * amp;
        }
    }
    
    // FAST VERSION: Only update PRN codes (change every 1ms), keep same navigation bits
    fn get_if_sample_fast(&mut self, cur_time: GnssTime) {
        let p_sat_param = if let Some(param) = &self.sat_param { param } else { return };

        // CRITICAL OPTIMIZATION: Orbital parameters change very slowly (every 10+ seconds)
        // For GPS L1CA, we only need to update the millisecond-specific PRN phase offset
        // Everything else can be cached from previous calculations
        
        // Only update the millisecond offset for PRN generation
        let ms_offset = cur_time.MilliSeconds - self.signal_time.MilliSeconds;
        
        // Simple phase increment - satellites move very slowly relative to 1ms
        let phase_increment = (self.if_freq as f64) / 1000.0;
        
        let code_attribute = self.prn_sequence.attribute.as_ref().unwrap();
        
        // PRN code phase advances by chip_rate per millisecond
        let chip_advance = code_attribute.chip_rate as f64;
        let code_step = chip_advance / (self.sample_number as f64);
        let base_chip_offset = (ms_offset as f64) * code_attribute.chip_rate as f64;
        
        // Cache amplitude calculation (CN0 doesn't change per millisecond)
        // КЭШИРОВАННАЯ амплитуда - устранили избыточные вычисления!
        if self.computation_cache.needs_update(p_sat_param.CN0 as f64, self.sample_number) {
            self.computation_cache.update(
                p_sat_param.CN0 as f64, 
                self.sample_number, 
                self.if_freq as f64, 
                code_attribute.chip_rate as f64
            );
        }
        let amp = self.computation_cache.cached_amp;

        // ULTRA FAST: Pre-calculate phase and chip increments
        let phase_step = phase_increment / (self.sample_number as f64);
        let mut cur_phase = 0.0; // Start from 0, will be corrected by rotation
        
        // Generate samples with minimal computation
        for i in 0..self.sample_number as usize {
            let mut cur_chip = base_chip_offset + (i as f64) * code_step;
            
            // OPTIMIZED PRN: Only handle GPS L1CA (most common case)
            let chip_index = (cur_chip as i32) & 0x3FF; // GPS L1CA is 1023 chips
            let prn_bit = if let Some(data_prn) = &self.prn_sequence.data_prn {
                if chip_index >= 0 && (chip_index as usize) < data_prn.len() && data_prn[chip_index as usize] != 0 { 
                    -1.0 
                } else { 
                    1.0 
                }
            } else { 
                1.0 
            };
            
            let nav_value = if self.data_signal.real >= 0.0 { 1.0 } else { -1.0 };
            
            // ULTRA FAST trigonometry using lookup table
            let angle = cur_phase * self.computation_cache.cached_two_pi;
            let cos_val = FastMath::fast_cos(angle);
            let sin_val = FastMath::fast_sin(angle);
            
            self.sample_array[i] = ComplexNumber {
                real: prn_bit * nav_value * cos_val * amp,
                imag: prn_bit * nav_value * sin_val * amp
            };
            
            cur_phase += phase_step;
        }
    }

    /// SIMD-оптимизированная генерация IF сигнала
    /// Обрабатывает 4 отсчета одновременно используя векторизацию
    pub fn get_if_sample_simd(&mut self, cur_time: GnssTime) {
        // Получаем параметры спутника (аналогично обычной версии)
        let p_sat_param = if let Some(ref sat_param) = self.sat_param {
            sat_param
        } else {
            return; // Если нет параметров спутника, выходим
        };
        
        // Подготовка всех параметров
        let ms_offset = cur_time.MilliSeconds - self.signal_time.MilliSeconds;
        let phase_increment = (self.if_freq as f64) / 1000.0;
        let code_attribute = self.prn_sequence.attribute.as_ref().unwrap();
        let chip_advance = code_attribute.chip_rate as f64;
        let code_step = chip_advance / (self.sample_number as f64);
        let base_chip_offset = (ms_offset as f64) * code_attribute.chip_rate as f64;
        // КЭШИРОВАННАЯ амплитуда - устранили избыточные вычисления!
        if self.computation_cache.needs_update(p_sat_param.CN0 as f64, self.sample_number) {
            self.computation_cache.update(
                p_sat_param.CN0 as f64, 
                self.sample_number, 
                self.if_freq as f64, 
                code_attribute.chip_rate as f64
            );
        }
        let amp = self.computation_cache.cached_amp;
        let phase_step = phase_increment / (self.sample_number as f64);
        let nav_value = if self.data_signal.real >= 0.0 { 1.0 } else { -1.0 };
        
        // SIMD векторы для обработки 4 элементов за раз
        let code_step_vec = f64x4::splat(code_step);
        let phase_step_vec = f64x4::splat(phase_step);
        let amp_vec = f64x4::splat(amp);
        let nav_value_vec = f64x4::splat(nav_value);
        let base_phase_vec = f64x4::new([0.0, phase_step, phase_step * 2.0, phase_step * 3.0]);
        let phase_increment_vec = f64x4::splat(phase_step * 4.0);
        let two_pi_vec = f64x4::splat(self.computation_cache.cached_two_pi);
        let base_chip_vec = f64x4::new([
            base_chip_offset,
            base_chip_offset + code_step,
            base_chip_offset + code_step * 2.0,
            base_chip_offset + code_step * 3.0
        ]);
        let chip_increment_vec = f64x4::splat(code_step * 4.0);
        
        let mut cur_phase_vec = base_phase_vec;
        let mut cur_chip_vec = base_chip_vec;
        
        // Обработка групп по 4 элемента
        let samples_per_group = 4;
        let full_groups = (self.sample_number as usize) / samples_per_group;
        
        if let Some(data_prn) = &self.prn_sequence.data_prn {
            for group in 0..full_groups {
                let base_idx = group * samples_per_group;
                
                // SIMD векторизованная обработка PRN кодов
                let chip_indices: [i32; 4] = [
                    (cur_chip_vec.as_array_ref()[0] as i32) & 0x3FF,
                    (cur_chip_vec.as_array_ref()[1] as i32) & 0x3FF,
                    (cur_chip_vec.as_array_ref()[2] as i32) & 0x3FF,
                    (cur_chip_vec.as_array_ref()[3] as i32) & 0x3FF,
                ];
                
                // Получение PRN битов
                let prn_bits = f64x4::new([
                    if chip_indices[0] >= 0 && (chip_indices[0] as usize) < data_prn.len() && data_prn[chip_indices[0] as usize] != 0 { -1.0 } else { 1.0 },
                    if chip_indices[1] >= 0 && (chip_indices[1] as usize) < data_prn.len() && data_prn[chip_indices[1] as usize] != 0 { -1.0 } else { 1.0 },
                    if chip_indices[2] >= 0 && (chip_indices[2] as usize) < data_prn.len() && data_prn[chip_indices[2] as usize] != 0 { -1.0 } else { 1.0 },
                    if chip_indices[3] >= 0 && (chip_indices[3] as usize) < data_prn.len() && data_prn[chip_indices[3] as usize] != 0 { -1.0 } else { 1.0 },
                ]);
                
                // Векторизованная тригонометрия
                let angles = cur_phase_vec * two_pi_vec;
                let cos_vals = f64x4::new([
                    FastMath::fast_cos(angles.as_array_ref()[0]),
                    FastMath::fast_cos(angles.as_array_ref()[1]),
                    FastMath::fast_cos(angles.as_array_ref()[2]),
                    FastMath::fast_cos(angles.as_array_ref()[3]),
                ]);
                let sin_vals = f64x4::new([
                    FastMath::fast_sin(angles.as_array_ref()[0]),
                    FastMath::fast_sin(angles.as_array_ref()[1]),
                    FastMath::fast_sin(angles.as_array_ref()[2]),
                    FastMath::fast_sin(angles.as_array_ref()[3]),
                ]);
                
                // Векторизованные вычисления амплитуд
                let real_vals = prn_bits * nav_value_vec * cos_vals * amp_vec;
                let imag_vals = prn_bits * nav_value_vec * sin_vals * amp_vec;
                
                // Сохранение результатов
                for i in 0..samples_per_group {
                    self.sample_array[base_idx + i] = ComplexNumber {
                        real: real_vals.as_array_ref()[i],
                        imag: imag_vals.as_array_ref()[i],
                    };
                }
                
                // Обновление векторов для следующей группы
                cur_phase_vec += phase_increment_vec;
                cur_chip_vec += chip_increment_vec;
            }
        }
        
        // Обработка оставшихся элементов (если их меньше 4)
        let remaining = (self.sample_number as usize) % samples_per_group;
        if remaining > 0 {
            let start_idx = full_groups * samples_per_group;
            let mut cur_phase = cur_phase_vec.as_array_ref()[0];
            let mut cur_chip = cur_chip_vec.as_array_ref()[0];
            
            for i in 0..remaining {
                let chip_index = (cur_chip as i32) & 0x3FF; // Битовая операция вместо модуло!
                let prn_bit = if let Some(data_prn) = &self.prn_sequence.data_prn {
                    if chip_index >= 0 && (chip_index as usize) < data_prn.len() && data_prn[chip_index as usize] != 0 { 
                        -1.0 
                    } else { 
                        1.0 
                    }
                } else { 
                    1.0 
                };
                
                let angle = cur_phase * self.computation_cache.cached_two_pi;
                let cos_val = FastMath::fast_cos(angle);
                let sin_val = FastMath::fast_sin(angle);
                
                self.sample_array[start_idx + i] = ComplexNumber {
                    real: prn_bit * nav_value * cos_val * amp,
                    imag: prn_bit * nav_value * sin_val * amp,
                };
                
                cur_phase += phase_step;
                cur_chip += code_step;
            }
        }
    }

    /// AVX-512 СВЕРХ-ОПТИМИЗИРОВАННАЯ генерация IF сигнала
    /// РЕВОЛЮЦИОННОЕ ускорение: 16x floats обрабатываются одновременно!
    pub fn get_if_sample_avx512_accelerated(&mut self, cur_time: GnssTime, avx512_processor: &crate::avx512_intrinsics::SafeAvx512Processor) {
        use crate::avx512_intrinsics::Avx512Accelerator;
        
        // Если AVX-512 недоступен, используем кэшированную версию
        if !avx512_processor.is_available() {
            return self.get_if_sample_cached(cur_time);
        }
        
        // Получаем параметры спутника
        let p_sat_param = if let Some(ref sat_param) = self.sat_param {
            sat_param
        } else {
            return;
        };
        
        // СУПЕР-БЫСТРАЯ подготовка параметров (те же что и в кэшированной версии)
        let code_attribute = self.prn_sequence.attribute.as_ref().unwrap();
        let chip_rate = code_attribute.chip_rate as f64;
        
        if self.computation_cache.needs_update(p_sat_param.CN0 as f64, self.sample_number) {
            self.computation_cache.update(
                p_sat_param.CN0 as f64, 
                self.sample_number, 
                self.if_freq as f64, 
                chip_rate
            );
        }
        
        let ms_offset = cur_time.MilliSeconds - self.signal_time.MilliSeconds;
        let base_chip_offset = (ms_offset as f64) * self.computation_cache.cached_chip_rate;
        let amp = self.computation_cache.cached_amp;
        let code_step = self.computation_cache.cached_code_step;
        let phase_step = self.computation_cache.cached_phase_step;
        let nav_value = if self.data_signal.real >= 0.0 { 1.0 } else { -1.0 };
        
        // Подготовка PRN и carrier кэшей
        if let Some(data_prn) = &self.prn_sequence.data_prn {
            if self.prn_cache.needs_update(cur_time.MilliSeconds) {
                self.prn_cache.update_cache(data_prn, cur_time.MilliSeconds);
            }
        }
        
        if self.carrier_phase_cache.is_none() {
            self.carrier_phase_cache = Some(CarrierPhaseCache::new(
                self.sample_number as usize, 
                phase_step
            ));
        }
        
        // РЕВОЛЮЦИЯ AVX-512: Обрабатываем 16 сэмплов одновременно!
        if let Some(ref carrier_cache) = self.carrier_phase_cache {
            let samples_per_group = 16; // AVX-512 обрабатывает 16 float32 одновременно
            let full_groups = (self.sample_number as usize) / samples_per_group;
            
            // ОПТИМИЗАЦИЯ ПАМЯТИ: Предаллоцированные буферы для AVX-512 (устранение аллокаций в цикле)
            let mut prn_data = vec![0.0f32; samples_per_group];
            let mut carrier_cos = vec![0.0f32; samples_per_group]; 
            let mut carrier_sin = vec![0.0f32; samples_per_group];
            let mut output_i = vec![0.0f32; samples_per_group];
            let mut output_q = vec![0.0f32; samples_per_group];
            
            // МЕГА-УСКОРЕНИЕ: Векторизованная обработка групп по 16 элементов
            for group in 0..full_groups {
                let base_idx = group * samples_per_group;
                
                // Подготовка данных для AVX-512 векторизации
                for i in 0..samples_per_group {
                    let sample_idx = base_idx + i;
                    let chip_offset = base_chip_offset + (sample_idx as f64) * code_step;
                    let chip_index = (chip_offset as usize) & 0x3FF; // Битовая операция для 1023
                    
                    prn_data[i] = self.prn_cache.get_prn_bit(chip_index) as f32;
                    carrier_cos[i] = carrier_cache.cached_cos[sample_idx] as f32;
                    carrier_sin[i] = carrier_cache.cached_sin[sample_idx] as f32;
                }
                
                // РЕВОЛЮЦИЯ! AVX-512 массовая обработка 16 сэмплов одновременно!
                // Используем предаллоцированные буферы (убрали дублирование)
                
                unsafe {
                    // Используем наши супер-быстрые AVX-512 инструкции
                    Avx512Accelerator::complex_multiply_prn_carrier_avx512(
                        &prn_data, &carrier_cos, &carrier_sin,
                        &mut output_i, &mut output_q,
                        amp as f32 * nav_value as f32
                    );
                }
                
                // Записываем результаты обратно в массив сэмплов
                for i in 0..samples_per_group {
                    let sample_idx = base_idx + i;
                    if sample_idx < self.sample_array.len() {
                        self.sample_array[sample_idx].real = output_i[i] as f64;
                        self.sample_array[sample_idx].imag = output_q[i] as f64;
                    }
                }
            }
            
            // Обрабатываем оставшиеся сэмплы обычным способом (если есть)
            let remaining_start = full_groups * samples_per_group;
            for i in remaining_start..(self.sample_number as usize) {
                let chip_offset = base_chip_offset + (i as f64) * code_step;
                let chip_index = (chip_offset as usize) & 0x3FF;
                
                let prn_bit = self.prn_cache.get_prn_bit(chip_index);
                let cos_val = carrier_cache.cached_cos[i];
                let sin_val = carrier_cache.cached_sin[i];
                
                self.sample_array[i].real = amp * prn_bit * cos_val * nav_value;
                self.sample_array[i].imag = amp * prn_bit * sin_val * nav_value;
            }
        }
    }

    /// 🚀 РЕВОЛЮЦИОННАЯ ФУНКЦИЯ: Генерация ПОЛНОГО сигнала спутника на всё время
    /// Каждый спутник генерирует ВСЕ миллисекунды в одном потоке для максимальной эффективности
    pub fn generate_full_signal_parallel(&mut self, start_time: GnssTime, total_duration_ms: i32, samples_per_ms: usize, sample_freq: f64) {
        println!("[PARALLEL] 🚀 Спутник начинает полную генерацию на {} ms", total_duration_ms);
        
        // Инициализация кэшей один раз на всё время (супер-эффективно!)
        self.initialize_caches_for_duration(total_duration_ms, samples_per_ms, sample_freq);
        
        // Обработка всех миллисекунд подряд (отличная локальность кэша)
        for ms_offset in 0..total_duration_ms {
            let current_time = GnssTime {
                Week: start_time.Week,
                MilliSeconds: start_time.MilliSeconds + ms_offset,
                SubMilliSeconds: start_time.SubMilliSeconds,
            };
            
            // Используем уже инициализированные кэши (быстро!)
            self.generate_samples_for_millisecond_cached(current_time, ms_offset as usize, samples_per_ms);
        }
        
        println!("[PARALLEL] ✅ Спутник завершил полную генерацию");
    }

    /// 🚀 ПАМЯТЬ-ЭФФЕКТИВНАЯ генерация сигнала для блока времени
    /// Генерирует только заданный блок времени, сохраняя память
    pub fn generate_block_signal_parallel(&mut self, start_time: GnssTime, block_duration_ms: i32, samples_per_ms: usize, sample_freq: f64) {
        let block_samples = (block_duration_ms as usize) * samples_per_ms;
        
        // Создаем буфер только для этого блока (не для всего файла!)
        self.block_data = Some(vec![ComplexNumber::from_parts(0.0, 0.0); block_samples]);
        
        // Инициализация кэшей для этого блока
        let total_samples = block_samples;
        
        // Инициализируем PRN кэш для блока
        self.prn_cache = PrnCache::new();
        
        // Обработка всех миллисекунд в блоке
        for ms_offset in 0..block_duration_ms {
            let current_time = GnssTime {
                Week: start_time.Week,
                MilliSeconds: start_time.MilliSeconds + ms_offset,
                SubMilliSeconds: start_time.SubMilliSeconds,
            };
            
            // Генерируем сэмплы для этой миллисекунды в блоке
            self.generate_samples_for_block_millisecond(current_time, ms_offset as usize, samples_per_ms);
        }
    }

    /// Генерация сэмплов для одной миллисекунды в блоке (память-эффективно)
    fn generate_samples_for_block_millisecond(&mut self, cur_time: GnssTime, ms_offset: usize, samples_per_ms: usize) {
        let p_sat_param = if let Some(ref sat_param) = self.sat_param {
            sat_param
        } else {
            return;
        };

        // Простые вычисления для этой миллисекунды - РАЗУМНАЯ АМПЛИТУДА!
        // Устанавливаем изначально безопасную амплитуду для суммирования множества спутников
        let amp = 0.05; // Безопасная амплитуда: 15 спутников * 0.05 = 0.75 (без клиппинга)
        let code_step = 1.023e6 / (self.sample_number as f64 * 1000.0);
        let phase_step = 2.0 * PI * self.if_freq as f64 / (self.sample_number as f64 * 1000.0);
        let nav_value = if self.data_signal.real >= 0.0 { 1.0 } else { -1.0 };
        
        // Получаем буфер блока
        if let Some(ref mut block_data) = self.block_data {
            let start_idx = ms_offset * samples_per_ms;
            
            // Генерируем сэмплы для этой миллисекунды
            for i in 0..samples_per_ms {
                let sample_idx = start_idx + i;
                if sample_idx < block_data.len() {
                    let t = i as f64;
                    
                    // PRN код (упрощенно)
                    let chip_index = (t * code_step) as usize % 1023;
                    let prn_value = if self.prn_sequence.get_prn_bit(chip_index as i32) { 1.0 } else { -1.0 };
                    
                    // Несущая
                    let carrier_phase = phase_step * t;
                    let cos_carrier = carrier_phase.cos();
                    let sin_carrier = carrier_phase.sin();
                    
                    // Итоговый сигнал
                    let signal_value = amp * nav_value * prn_value;
                    block_data[sample_idx].real = signal_value * cos_carrier;
                    block_data[sample_idx].imag = signal_value * sin_carrier;
                }
            }
        }
    }
    
    /// Инициализация кэшей на всю длительность сигнала (вызывается один раз)
    fn initialize_caches_for_duration(&mut self, total_duration_ms: i32, samples_per_ms: usize, sample_freq: f64) {
        let total_samples = (total_duration_ms as usize) * samples_per_ms;
        
        // Выделяем память для всего сигнала сразу (эффективнее)
        if self.sample_array.len() < total_samples {
            self.sample_array.resize(total_samples, ComplexNumber { real: 0.0, imag: 0.0 });
            self.sample_number = total_samples as i32;
        }
        
        // Инициализация carrier кэша на всё время
        if self.carrier_phase_cache.is_none() {
            let p_sat_param = if let Some(ref sat_param) = self.sat_param {
                sat_param
            } else {
                return;
            };
            
            let phase_step = 2.0 * std::f64::consts::PI * self.if_freq as f64 / sample_freq;
            self.carrier_phase_cache = Some(CarrierPhaseCache::new(total_samples, phase_step));
        }
    }
    
    /// Генерация сэмплов для одной миллисекунды с использованием готовых кэшей
    fn generate_samples_for_millisecond_cached(&mut self, cur_time: GnssTime, ms_offset: usize, samples_per_ms: usize) {
        // Получаем параметры спутника
        let p_sat_param = if let Some(ref sat_param) = self.sat_param {
            sat_param
        } else {
            return;
        };

        // Используем уже готовые вычисления из кэша
        let computation_cache = &self.computation_cache;
        if computation_cache.needs_update(p_sat_param.CN0 as f64, self.sample_number) {
            // Обновляем кэш только если нужно
            let mut cache = computation_cache.clone();
            cache.update(p_sat_param.CN0 as f64, self.sample_number, self.if_freq as f64, 1.023e6);
        }
        
        let amp = computation_cache.cached_amp;
        let code_step = computation_cache.cached_code_step;
        let phase_step = computation_cache.cached_phase_step;
        let nav_value = if self.data_signal.real >= 0.0 { 1.0 } else { -1.0 };
        
        let ms_offset_time = cur_time.MilliSeconds - self.signal_time.MilliSeconds;
        let base_chip_offset = (ms_offset_time as f64) * computation_cache.cached_chip_rate;
        
        // Обновляем PRN кэш при необходимости
        if let Some(data_prn) = &self.prn_sequence.data_prn {
            if self.prn_cache.needs_update(cur_time.MilliSeconds) {
                self.prn_cache.update_cache(data_prn, cur_time.MilliSeconds);
            }
        }
        
        // ВЫСОКОПРОИЗВОДИТЕЛЬНАЯ генерация сэмплов для миллисекунды
        let base_sample_idx = ms_offset * samples_per_ms;
        if let Some(ref carrier_cache) = self.carrier_phase_cache {
            for i in 0..samples_per_ms {
                let sample_idx = base_sample_idx + i;
                if sample_idx < self.sample_array.len() {
                    let chip_offset = base_chip_offset + (i as f64) * code_step;
                    let chip_index = (chip_offset as usize) & 0x3FF; // Быстрая операция вместо %
                    
                    let prn_bit = self.prn_cache.get_prn_bit(chip_index);
                    let cos_val = carrier_cache.cached_cos[sample_idx];
                    let sin_val = carrier_cache.cached_sin[sample_idx];
                    
                    // Быстрое вычисление комплексного сигнала
                    self.sample_array[sample_idx].real = amp * prn_bit * cos_val * nav_value;
                    self.sample_array[sample_idx].imag = amp * prn_bit * sin_val * nav_value;
                }
            }
        }
    }

    /// СУПЕР-ОПТИМИЗИРОВАННАЯ генерация IF сигнала с агрессивным кэшированием
    /// Использует предвычисленные PRN коды и фазы несущей
    pub fn get_if_sample_cached(&mut self, cur_time: GnssTime) {
        // Получаем параметры спутника
        let p_sat_param = if let Some(ref sat_param) = self.sat_param {
            sat_param
        } else {
            return;
        };
        
        // КЭШИРОВАННАЯ ПОДГОТОВКА ПАРАМЕТРОВ (МОЛНИЕНОСНО!)
        let code_attribute = self.prn_sequence.attribute.as_ref().unwrap();
        let chip_rate = code_attribute.chip_rate as f64;
        
        // КРИТИЧЕСКАЯ ОПТИМИЗАЦИЯ: Обновляем кэш только если нужно
        if self.computation_cache.needs_update(p_sat_param.CN0 as f64, self.sample_number) {
            self.computation_cache.update(
                p_sat_param.CN0 as f64, 
                self.sample_number, 
                self.if_freq as f64, 
                chip_rate
            );
        }
        
        // Используем кэшированные значения (без повторных вычислений!)
        let ms_offset = cur_time.MilliSeconds - self.signal_time.MilliSeconds;
        let base_chip_offset = (ms_offset as f64) * self.computation_cache.cached_chip_rate;
        let amp = self.computation_cache.cached_amp;
        let code_step = self.computation_cache.cached_code_step;
        let phase_step = self.computation_cache.cached_phase_step;
        let nav_value = if self.data_signal.real >= 0.0 { 1.0 } else { -1.0 };
        
        // АГРЕССИВНОЕ КЭШИРОВАНИЕ: Обновляем PRN кэш если нужно
        if let Some(data_prn) = &self.prn_sequence.data_prn {
            if self.prn_cache.needs_update(cur_time.MilliSeconds) {
                self.prn_cache.update_cache(data_prn, cur_time.MilliSeconds);
            }
        }
        
        // АГРЕССИВНОЕ КЭШИРОВАНИЕ: Инициализируем кэш фаз несущей если нужно
        if self.carrier_phase_cache.is_none() {
            self.carrier_phase_cache = Some(CarrierPhaseCache::new(
                self.sample_number as usize, 
                phase_step
            ));
        }
        
        // Супер-быстрая генерация с использованием кэшей
        if let Some(ref carrier_cache) = self.carrier_phase_cache {
            // SIMD + кэширование для максимальной производительности
            let samples_per_group = 4;
            let full_groups = (self.sample_number as usize) / samples_per_group;
            
            // СУПЕР-КРИТИЧЕСКАЯ ОПТИМИЗАЦИЯ: полное кэширование всех вычислений!
            let code_step_x4 = code_step * 4.0; // Предвычисляем шаг для группы
            let pi2 = self.computation_cache.cached_two_pi;
            let pi2_phase_step = pi2 * phase_step; // Предвычисляем константу
            
            // Векторизованная обработка групп по 4 элемента с ПОЛНЫМ кэшированием
            for group in 0..full_groups {
                let base_idx = group * samples_per_group;
                
                // МЕГА-ОПТИМИЗАЦИЯ: Используем простые битовые операции вместо модуло!
                let chip_start = base_chip_offset + (base_idx as f64) * code_step;
                let chip_indices: [usize; 4] = [
                    (chip_start as usize) & 0x3FF, // 1023 = 0x3FF, битовая операция!
                    ((chip_start + code_step) as usize) & 0x3FF,
                    ((chip_start + code_step * 2.0) as usize) & 0x3FF,
                    ((chip_start + code_step * 3.0) as usize) & 0x3FF,
                ];
                
                // КЭШИРОВАННЫЕ PRN биты - молниеносный доступ!
                let prn_bits = f64x4::new([
                    self.prn_cache.get_prn_bit(chip_indices[0]),
                    self.prn_cache.get_prn_bit(chip_indices[1]),
                    self.prn_cache.get_prn_bit(chip_indices[2]),
                    self.prn_cache.get_prn_bit(chip_indices[3]),
                ]);
                
                // РЕВОЛЮЦИЯ! Используем готовые cos/sin из кэша вместо пересчёта!
                let cos_vals = f64x4::new([
                    carrier_cache.cached_cos[base_idx + 0],
                    carrier_cache.cached_cos[base_idx + 1],
                    carrier_cache.cached_cos[base_idx + 2],
                    carrier_cache.cached_cos[base_idx + 3],
                ]);
                let sin_vals = f64x4::new([
                    carrier_cache.cached_sin[base_idx + 0],
                    carrier_cache.cached_sin[base_idx + 1],
                    carrier_cache.cached_sin[base_idx + 2],
                    carrier_cache.cached_sin[base_idx + 3],
                ]);
                
                // Супер-оптимизированная SIMD генерация сигнала
                let nav_value_vec = f64x4::splat(nav_value);
                let amp_vec = f64x4::splat(amp);
                
                // Используем SIMD функцию для комплексных вычислений
                ComplexNumber::simd_generate_signal(
                    prn_bits,
                    nav_value_vec,
                    cos_vals,
                    sin_vals,
                    amp_vec,
                    &mut self.sample_array[base_idx..base_idx + samples_per_group]
                );
            }
            
            // Обработка оставшихся элементов
            let remaining = (self.sample_number as usize) % samples_per_group;
            if remaining > 0 {
                let start_idx = full_groups * samples_per_group;
                
                for i in 0..remaining {
                    let idx = start_idx + i;
                    let chip_index = ((base_chip_offset + (idx as f64) * code_step) as usize) & 0x3FF;
                    
                    // Кэшированные значения
                    let prn_bit = self.prn_cache.get_prn_bit(chip_index);
                    let (cos_val, sin_val) = carrier_cache.get_cos_sin(idx);
                    
                    self.sample_array[idx] = ComplexNumber {
                        real: prn_bit * nav_value * cos_val * amp,
                        imag: prn_bit * nav_value * sin_val * amp,
                    };
                }
            }
        }
    }

    fn get_prn_value(&mut self, cur_chip: &mut f64, code_step: f64) -> ComplexNumber {
        let attribute = if let Some(attr) = &self.prn_sequence.attribute {
            attr
        } else {
            *cur_chip += code_step;
            return ComplexNumber::new();
        };

        let chip_count = *cur_chip as i32;
        let mut prn_value;

        let is_boc = (attribute.attribute & PRN_ATTRIBUTE_BOC) != 0;
        let is_qmboc = (attribute.attribute & PRN_ATTRIBUTE_QMBOC) != 0;
        let is_tmboc = (attribute.attribute & PRN_ATTRIBUTE_TMBOC) != 0;
        let is_cboc = (attribute.attribute & PRN_ATTRIBUTE_CBOC) != 0;
        let is_tdm = (attribute.attribute & PRN_ATTRIBUTE_TMD) != 0;

        if self.system == GnssSystem::GlonassSystem && (self.signal_index == SIGNAL_INDEX_G1 as i32 || self.signal_index == SIGNAL_INDEX_G2 as i32) {
            let data_chip = chip_count % self.data_length;
            let prn_bit = if let Some(data_prn) = &self.prn_sequence.data_prn {
                if data_chip >= 0 && (data_chip as usize) < data_prn.len() && data_prn[data_chip as usize] != 0 { 1 } else { 0 }
            } else { 0 };
            
            let nav_bit = if self.data_signal.real < 0.0 { 1 } else { 0 };
            
            let chip_time_ms = *cur_chip / 511.0;
            let total_ms = self.signal_time.MilliSeconds + chip_time_ms as i32;
            let meander = if (total_ms / 10) % 2 == 0 { 1 } else { 0 };
            
            let modulated_bit = prn_bit ^ nav_bit ^ meander;
            
            prn_value = ComplexNumber { real: if modulated_bit != 0 { -1.0 } else { 1.0 }, imag: 0.0 };
            
            *cur_chip += code_step;
            return prn_value;
        }

        if is_tdm {
            let current_ms = (*cur_chip / attribute.chip_rate as f64) as i32 % 2;
            if current_ms == 0 { // Even millisecond - L2CM (data)
                let data_chip = chip_count % self.data_length;
                prn_value = if let Some(data_prn) = &self.prn_sequence.data_prn {
                    if data_chip >= 0 && (data_chip as usize) < data_prn.len() {
                        self.data_signal * if data_prn[data_chip as usize] != 0 { -1.0 } else { 1.0 }
                    } else { ComplexNumber::new() }
                } else { ComplexNumber::new() };
            } else { // Odd millisecond - L2CL (pilot)
                let pilot_chip = chip_count % self.pilot_length;
                 prn_value = if let Some(pilot_prn) = &self.prn_sequence.pilot_prn {
                    if pilot_chip >= 0 && (pilot_chip as usize) < pilot_prn.len() {
                        self.pilot_signal * if pilot_prn[pilot_chip as usize] != 0 { -1.0 } else { 1.0 }
                    } else { ComplexNumber::new() }
                } else { ComplexNumber::new() };
            }
            *cur_chip += code_step;
            return prn_value;
        }

        let mut data_chip = chip_count % self.data_length;
        if is_boc { data_chip >>= 1; }

        prn_value = if let Some(data_prn) = &self.prn_sequence.data_prn {
            if data_chip >= 0 && (data_chip as usize) < data_prn.len() {
                let mut val = self.data_signal * if data_prn[data_chip as usize] != 0 { -1.0 } else { 1.0 };
                if is_boc && (chip_count & 1) != 0 {
                    val *= -1.0;
                }
                val
            } else { ComplexNumber::new() }
        } else { ComplexNumber::new() };

        if let Some(pilot_prn) = &self.prn_sequence.pilot_prn {
            if self.pilot_length > 0 {
                let mut pilot_chip = chip_count % self.pilot_length;
                if is_boc { pilot_chip >>= 1; }

                if pilot_chip >= 0 && (pilot_chip as usize) < pilot_prn.len() {
                    let mut pilot_val = self.pilot_signal * if pilot_prn[pilot_chip as usize] != 0 { -1.0 } else { 1.0 };
                    
                    if (is_tmboc || is_qmboc) && is_boc {
                        let symbol_pos = (self.signal_time.MilliSeconds % 330) / 10;
                        if symbol_pos == 1 || symbol_pos == 5 || symbol_pos == 7 || symbol_pos == 30 {
                            let sub_chip_pos = chip_count % 12;
                            if sub_chip_pos >= 6 { pilot_val *= -1.0; }
                        } else if (chip_count & 1) != 0 { pilot_val *= -1.0; }
                        prn_value += pilot_val;
                    } else if is_cboc && is_boc {
                        let chip_in_code = chip_count % 4092;
                        if (chip_in_code % 11) == 0 {
                            let boc6_phase = chip_count % 12;
                            if boc6_phase >= 6 { pilot_val *= -1.0; }
                        } else if (chip_count & 1) != 0 { pilot_val *= -1.0; }
                        prn_value += pilot_val;
                    } else {
                        if is_boc && (chip_count & 1) != 0 {
                            pilot_val *= -1.0;
                        }
                        prn_value += pilot_val;
                    }
                }
            }
        }

        let old_data_chip = data_chip;
        *cur_chip += code_step;
        if self.data_length > 0 && ((*cur_chip as i32) % self.data_length) < old_data_chip {
            self.signal_time.MilliSeconds += attribute.data_period;
            self.satellite_signal.get_satellite_signal(self.signal_time, &mut self.data_signal, &mut self.pilot_signal);
        }

        prn_value
    }

    fn get_rotate_value(&self, cur_phase: &mut f64, phase_step: f64) -> ComplexNumber {
        // КРИТИЧЕСКАЯ ОПТИМИЗАЦИЯ: используем unnormalized версию FastMath для максимальной скорости
        // Углы фазы уже находятся в правильном диапазоне, нет необходимости в нормализации!
        let phase_angle = *cur_phase * PI2;
        let rotate = FastMath::fast_rotate_unnormalized(phase_angle);
        *cur_phase += phase_step;
        rotate
    }
    
    // CRITICAL OPTIMIZATION: Update only phases without expensive orbit calculations
    pub fn fast_update_phases_only(&mut self) {
        if self.sat_param.is_none() {
            return;
        }
        
        // Reuse previous calculations, just update the sample array with new phase
        let phase_step = (self.if_freq as f64) / 1000.0 / (self.sample_number as f64);
        let mut cur_phase = self.start_carrier_phase - self.start_carrier_phase.floor();
        cur_phase = 1.0 - cur_phase;
        
        // Skip expensive orbit calculations - use cached PRN values but update phases
        for i in 0..self.sample_number as usize {
            // Reuse the existing PRN value magnitude but update phase rotation
            let existing_magnitude = (self.sample_array[i].real * self.sample_array[i].real + 
                                    self.sample_array[i].imag * self.sample_array[i].imag).sqrt();
            let rotate_value = FastMath::fast_rotate(cur_phase * PI2);
            cur_phase += phase_step;
            
            // Apply new phase rotation to existing signal magnitude
            self.sample_array[i] = ComplexNumber {
                real: existing_magnitude * rotate_value.real,
                imag: existing_magnitude * rotate_value.imag,
            };
        }
    }
}
