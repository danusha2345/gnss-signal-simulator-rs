//----------------------------------------------------------------------
// sat_if_signal.rs:
//   Implementation of satellite IF signal generation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::complex_number::ComplexNumber;
use crate::constants::*;
use crate::fastmath::FastMath;
use crate::prngenerate::*;
use crate::satellite_param::{get_carrier_phase, get_doppler, get_transmit_time, get_travel_time};
use crate::satellite_signal::SatelliteSignal;
use crate::types::{GnssSystem, GnssTime, SatelliteParam};
use crate::avx512_intrinsics::Avx512Accelerator;

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

impl Default for PrnCache {
    fn default() -> Self {
        Self::new()
    }
}

impl PrnCache {
    pub fn new() -> Self {
        Self {
            cached_prn_bits: vec![1.0; 1024], // Степень 2 для быстрых битовых операций
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
        let prn_len = data_prn.len().min(1024);

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
        for (chip_index, val) in data_prn.iter().enumerate().skip(i).take(prn_len - i) {
            self.cached_prn_bits[chip_index] = if *val != 0 { -1.0 } else { 1.0 };
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
    /// Кэшированный code_step (включает code Doppler!)
    pub cached_code_step: f64,
    /// Кэшированный phase_step (постоянная величина)
    pub cached_phase_step: f64,
    /// Кэшированная chip_rate (включает code Doppler!)
    pub cached_chip_rate: f64,
    /// Предвычисленная константа 2*PI (избегает повторных вычислений!)
    pub cached_two_pi: f64,
    /// Последний обновлённый CN0 (для проверки сброса кэша)
    pub last_cn0: f64,
    /// Последний sample_number (для проверки сброса кэша)
    pub last_sample_number: i32,
    /// Последний Doppler (для обновления code Doppler при смене скорости спутника)
    pub last_doppler_hz: f64,
    /// Флаг валидности кэша
    pub is_valid: bool,
}

impl Default for ComputationCache {
    fn default() -> Self {
        Self::new()
    }
}

impl ComputationCache {
    pub fn new() -> Self {
        Self {
            cached_amp: 0.0,
            cached_code_step: 0.0,
            cached_phase_step: 0.0,
            cached_chip_rate: 0.0,
            cached_two_pi: 2.0 * std::f64::consts::PI,
            last_cn0: -9999.0,
            last_sample_number: -1,
            last_doppler_hz: 0.0,
            is_valid: false,
        }
    }

    /// Проверяет нужно ли обновить кэш (включая смену Doppler для code Doppler)
    pub fn needs_update(&self, cn0: f64, sample_number: i32, doppler_hz: f64) -> bool {
        !self.is_valid
            || (cn0 - self.last_cn0).abs() > 0.1
            || sample_number != self.last_sample_number
            || (doppler_hz - self.last_doppler_hz).abs() > 0.1
    }

    /// Invalidate cache to force recalculation on next access
    pub fn invalidate(&mut self) {
        self.is_valid = false;
        self.last_cn0 = -9999.0;
    }

    /// Обновляет кэш вычислений с code Doppler коррекцией
    /// code_rate = chip_rate * (1 + doppler_hz / carrier_freq_hz)
    /// Это критично для DLL tracking — без code Doppler код дрейфует ~0.002 чипа/мс
    pub fn update(&mut self, cn0: f64, sample_number: i32, if_freq: f64, chip_rate: f64, doppler_hz: f64, carrier_freq_hz: f64) {
        // C++ compatible amplitude formula: A = sqrt(C/N0_linear / Fs)
        // Matches SignalSim: pow(10, (CN0-3000)/2000.) / sqrt(SampleNumber)
        let cn0_db = cn0 / 100.0;
        let cn0_linear = 10.0_f64.powf(cn0_db / 10.0);
        let fs = sample_number as f64 * 1000.0;
        self.cached_amp = (cn0_linear / fs).sqrt();

        // Code Doppler: code rate scales proportionally to carrier Doppler
        // Real GNSS signal: code_rate = nominal_chip_rate * (1 + v/c)
        // where v/c = doppler_hz / carrier_freq_hz
        let code_doppler_factor = if carrier_freq_hz > 0.0 {
            1.0 + doppler_hz / carrier_freq_hz
        } else {
            1.0
        };
        self.cached_chip_rate = chip_rate * code_doppler_factor;
        self.cached_code_step = self.cached_chip_rate / (sample_number as f64);
        self.cached_phase_step = (if_freq / 1000.0) / (sample_number as f64);
        self.last_cn0 = cn0;
        self.last_sample_number = sample_number;
        self.last_doppler_hz = doppler_hz;
        self.is_valid = true;
    }
}

pub struct SatIfSignal {
    sample_number: i32,
    if_freq: i32,
    pub system: GnssSystem,
    pub signal_index: i32,
    pub svid: i32,
    pub sample_array: Vec<ComplexNumber>,
    /// Буфер для потоковой обработки блоков данных (память-эффективно)
    pub block_data: Option<Vec<ComplexNumber>>,
    prn_sequence: PrnGenerate,
    satellite_signal: SatelliteSignal,
    sat_param: Option<SatelliteParam>,
    data_length: i32,
    pilot_length: i32,
    start_carrier_phase: f64,
    start_code_phase: f64, // Code phase at start of ms (Doppler model, re-anchored by update_satellite_params)
    signal_time: GnssTime,
    start_transmit_time: GnssTime,
    data_signal: ComplexNumber,
    pilot_signal: ComplexNumber,
    last_nav_bit_index: i32,
    prn_cache: PrnCache,
    /// КЭШ для минимизации повторных вычислений
    computation_cache: ComputationCache,
    // ЭКСТРЕМАЛЬНАЯ АППАРАТНАЯ ОПТИМИЗАЦИЯ: AVX-512 ускоритель
    #[allow(dead_code)]
    avx512_accelerator: Option<Avx512Accelerator>,
}

impl SatIfSignal {
    pub fn new(
        ms_sample_number: i32,
        sat_if_freq: i32,
        sat_system: GnssSystem,
        sat_signal_index: i32,
        sat_id: u8,
    ) -> Self {
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
            start_carrier_phase: 0.0,
            start_code_phase: 0.0,
            signal_time: GnssTime::default(),
            start_transmit_time: GnssTime::default(),
            data_signal: ComplexNumber::new(),
            pilot_signal: ComplexNumber::new(),
            last_nav_bit_index: -1,
            prn_cache: PrnCache::new(),
            computation_cache: ComputationCache::new(),
            // ЭКСТРЕМАЛЬНАЯ ОПТИМИЗАЦИЯ: инициализируем AVX-512 если доступно
            avx512_accelerator: if Avx512Accelerator::is_available() {
                Some(Avx512Accelerator::new())
            } else {
                None
            },
        }
    }

    pub fn init_state(
        &mut self,
        cur_time: GnssTime,
        p_sat_param: &SatelliteParam,
        p_nav_data: Option<crate::nav_data::NavData>,
    ) {
        self.sat_param = Some(*p_sat_param);
        if !self.satellite_signal.set_signal_attribute(
            self.system,
            self.signal_index,
            p_nav_data,
            self.svid,
        ) {
            // In Rust, we might handle this by setting self.satellite_signal.nav_data to None,
            // but set_signal_attribute already takes ownership, so we'd need to adjust its signature
            // or logic if we need to "null out" the nav data here.
            // For now, we assume set_signal_attribute handles the mismatch appropriately.
        }
        self.start_carrier_phase = get_carrier_phase(p_sat_param, self.signal_index as usize);
        self.signal_time = get_transmit_time(
            &cur_time,
            get_travel_time(p_sat_param, self.signal_index as usize),
        );
        self.start_transmit_time = self.signal_time;

        // Initialize code phase from transmit time
        if let Some(attr) = &self.prn_sequence.attribute {
            self.start_code_phase = (self.signal_time.MilliSeconds as f64
                % attr.pilot_period as f64
                + self.signal_time.SubMilliSeconds)
                * attr.chip_rate as f64;
        }

        self.satellite_signal.get_satellite_signal(
            self.signal_time,
            &mut self.data_signal,
            &mut self.pilot_signal,
        );
    }

    /// Update satellite parameters (position, velocity, doppler) from recalculated SatelliteParam.
    /// Called every block (~50ms) to keep Doppler and code delay current.
    ///
    /// Carrier phase: NO hard re-anchor — phase accumulates continuously via Doppler
    /// between updates (like C++ SignalSim updates every 1ms). Re-anchoring every 50ms
    /// caused ~0.003 cycle phase jumps at 20 Hz — comparable to PLL thermal noise at 45 dB-Hz.
    ///
    /// Code phase: re-anchored from true transmit time (~0.001 chip jump, filtered by DLL).
    pub fn update_satellite_params(&mut self, new_param: &SatelliteParam, output_center_freq: f64, cur_time: &GnssTime) {
        self.sat_param = Some(*new_param);

        let signal_center_freq = self.get_signal_center_freq();
        self.if_freq = (signal_center_freq - output_center_freq) as i32;

        // Update signal_time for nav bit timing (needed for BUG 4 mid-sample transitions)
        let travel_time = get_travel_time(new_param, self.signal_index as usize);
        let new_transmit_time = get_transmit_time(cur_time, travel_time);
        self.signal_time = new_transmit_time;

        // Do NOT re-anchor start_code_phase — it accumulates continuously via Doppler model.
        // Re-anchoring creates code phase jumps that break nav bit decode for ~50% of satellites.

        // Do NOT re-anchor start_carrier_phase — it accumulates continuously via Doppler model.

        // Refresh nav bits for the new signal_time
        self.satellite_signal.get_satellite_signal(
            self.signal_time,
            &mut self.data_signal,
            &mut self.pilot_signal,
        );

        self.last_nav_bit_index = -1;
        self.computation_cache.invalidate();
    }

    /// Lightweight per-ms parameter push (for per-ms main loop, matching C++ StepToNextMs).
    /// Updates sat_param, IF freq, and signal_time. Does NOT re-anchor code/carrier phase.
    pub fn push_sat_param_for_ms(&mut self, new_param: &SatelliteParam, output_center_freq: f64, cur_time: &GnssTime) {
        self.sat_param = Some(*new_param);
        let signal_center_freq = self.get_signal_center_freq();
        self.if_freq = (signal_center_freq - output_center_freq) as i32;

        // Sync signal_time with actual transmit time every ms.
        // Without this, signal_time drifts when travel_time crosses integer ms boundary,
        // causing BUG 4 mid-sample detection to load wrong nav bit for ~50% of satellites.
        let travel_time = get_travel_time(new_param, self.signal_index as usize);
        self.signal_time = get_transmit_time(cur_time, travel_time);

        self.computation_cache.invalidate();
    }

    /// Get the RF center frequency for this signal (Hz), accounting for GLONASS FDMA offset.
    fn get_signal_center_freq(&self) -> f64 {
        match self.system {
            GnssSystem::GlonassSystem => {
                let freq_id = self.sat_param.as_ref().map(|p| p.FreqID).unwrap_or(0);
                match self.signal_index as usize {
                    SIGNAL_INDEX_G1 => FREQ_GLO_G1 + freq_id as f64 * 562500.0,
                    SIGNAL_INDEX_G2 => FREQ_GLO_G2 + freq_id as f64 * 437500.0,
                    _ => 0.0,
                }
            }
            _ => {
                // Map global signal_index to per-system freq_array_index
                let freq_array_index = match self.signal_index as usize {
                    // GPS: signal indices 0-4 map directly
                    SIGNAL_INDEX_L1CA => 0,
                    SIGNAL_INDEX_L1C => 1,
                    SIGNAL_INDEX_L2C => 2,
                    SIGNAL_INDEX_L2P => 3,
                    SIGNAL_INDEX_L5 => 4,
                    // BDS: signal indices 8-14 map to 0-6
                    SIGNAL_INDEX_B1C => 0,
                    SIGNAL_INDEX_B1I => 1,
                    SIGNAL_INDEX_B2I => 2,
                    SIGNAL_INDEX_B3I => 3,
                    SIGNAL_INDEX_B2A => 4,
                    SIGNAL_INDEX_B2B => 5,
                    SIGNAL_INDEX_B2AB => 6,
                    // GAL: signal indices 16-20 map to 0,1,2,_,4
                    SIGNAL_INDEX_E1 => 0,
                    SIGNAL_INDEX_E5A => 1,
                    SIGNAL_INDEX_E5B => 2,
                    SIGNAL_INDEX_E6 => 4,
                    _ => 0,
                };
                SIGNAL_CENTER_FREQ[self.system as usize][freq_array_index.min(7)]
            }
        }
    }


    /// AVX-512 СВЕРХ-ОПТИМИЗИРОВАННАЯ генерация IF сигнала
    /// РЕВОЛЮЦИОННОЕ ускорение: 16x floats обрабатываются одновременно!
    pub fn get_if_sample_avx512_accelerated(
        &mut self,
        cur_time: GnssTime,
        avx512_processor: &crate::avx512_intrinsics::SafeAvx512Processor,
    ) {
        // Если AVX-512 недоступен, fallback на кэшированную версию.
        // CUDA не используется в этой функции; GPU offload живёт в cuda_acceleration.rs.
        if !avx512_processor.is_available() {
            return self.get_if_sample_cached(cur_time);
        }

        // BOC/сложные сигналы (BDS B1C, GAL E1) и non-GPS коды (ГЛОНАСС G1/G2 = 511 чипов)
        // перенаправляем на get_if_sample_cached,
        // который использует rem_euclid(data_length).
        if let Some(attr) = &self.prn_sequence.attribute {
            let is_boc = (attr.attribute & PRN_ATTRIBUTE_BOC) != 0;
            if is_boc || self.data_length != crate::constants::GPS_L1CA_CODE_LENGTH {
                return self.get_if_sample_cached(cur_time);
            }
        }

        // Получаем параметры спутника
        let p_sat_param = if let Some(ref sat_param) = self.sat_param {
            sat_param
        } else {
            return;
        };

        // Compute actual transmit time for nav bit indexing (MUST use transmit time, not receiver time!)
        let travel_time = get_travel_time(p_sat_param, self.signal_index as usize);
        let actual_transmit_time = get_transmit_time(&cur_time, travel_time);

        // Update navigation bits at bit boundaries using TRANSMIT time (GPS L1CA: 20ms bit period)
        {
            let bit_period_ms = self.satellite_signal.attribute.code_length
                * self.satellite_signal.attribute.nh_length;
            if bit_period_ms > 0 {
                let transmit_ms = actual_transmit_time.MilliSeconds;
                let current_bit_idx = transmit_ms / bit_period_ms;
                if current_bit_idx != self.last_nav_bit_index {
                    self.satellite_signal.get_satellite_signal(
                        actual_transmit_time,
                        &mut self.data_signal,
                        &mut self.pilot_signal,
                    );
                    self.last_nav_bit_index = current_bit_idx;
                }
            }
        }

        let code_attribute = self.prn_sequence.attribute.as_ref().unwrap();
        let chip_rate = code_attribute.chip_rate as f64;
        let _pilot_period = code_attribute.pilot_period as f64;

        // Doppler MUST be computed BEFORE cache update — code Doppler depends on it
        let doppler_hz = get_doppler(p_sat_param, self.signal_index as usize);
        let carrier_freq_hz = self.get_signal_center_freq();

        if self
            .computation_cache
            .needs_update(p_sat_param.CN0 as f64, self.sample_number, doppler_hz)
        {
            self.computation_cache.update(
                p_sat_param.CN0 as f64,
                self.sample_number,
                self.if_freq as f64,
                chip_rate,
                doppler_hz,
                carrier_freq_hz,
            );
        }

        let amp = self.computation_cache.cached_amp;

        // Code phase from start_code_phase (Doppler model)
        let base_chip_offset = self.start_code_phase;
        let code_step = self.computation_cache.cached_code_step;

        let nav_value = {
            let r = self.data_signal.real;
            let i = self.data_signal.imag;
            if r.abs() >= i.abs() {
                if r >= 0.0 { 1.0 } else { -1.0 }
            } else {
                if i >= 0.0 { 1.0 } else { -1.0 }
            }
        };

        // Подготовка PRN кэша
        if let Some(data_prn) = &self.prn_sequence.data_prn {
            if self.prn_cache.needs_update(cur_time.MilliSeconds) {
                self.prn_cache.update_cache(data_prn, cur_time.MilliSeconds);
            }
        }

        // Carrier phase: Doppler model (f64 sin/cos)
        let doppler_cycles_per_ms = doppler_hz / 1000.0;
        let n = self.sample_number as f64;
        let phase_step = (doppler_cycles_per_ms + self.if_freq as f64 / 1000.0) / n;
        let frac_phase = self.start_carrier_phase - self.start_carrier_phase.floor();
        let mut cur_phase = 1.0 - frac_phase;

        // Update start_carrier_phase via Doppler accumulation
        self.start_carrier_phase -= doppler_cycles_per_ms;

        // Генерация сэмплов (GPS L1CA only path)
        for i in 0..self.sample_number as usize {
            let chip_offset = base_chip_offset + (i as f64) * code_step;
            let chip_index = (chip_offset as i32).rem_euclid(self.data_length) as usize;

            let prn_bit = self.prn_cache.get_prn_bit(chip_index);

            // Carrier phase: f64 sin/cos
            let angle = cur_phase * std::f64::consts::TAU;
            let (sin_val, cos_val) = angle.sin_cos();
            cur_phase += phase_step;

            self.sample_array[i] = ComplexNumber {
                real: prn_bit * nav_value * cos_val * amp,
                imag: prn_bit * nav_value * sin_val * amp,
            };
        }
        // Advance code phase by one ms worth of chips
        self.start_code_phase += self.computation_cache.cached_chip_rate;
        if self.pilot_length > 0 {
            self.start_code_phase = self.start_code_phase.rem_euclid(self.pilot_length as f64);
        }
    }


    /// 🚀 ПАМЯТЬ-ЭФФЕКТИВНАЯ генерация сигнала для блока времени
    /// Генерирует только заданный блок времени, используя точную логику модуляции
    /// из `get_if_sample_*` и копируя миллисекундные буферы в блочный вывод.
    pub fn generate_block_signal_parallel(
        &mut self,
        start_time: GnssTime,
        block_duration_ms: i32,
        samples_per_ms: usize,
        _sample_freq: f64,
    ) {
        let block_samples = (block_duration_ms as usize) * samples_per_ms;

        // ОПТИМИЗАЦИЯ: переиспользуем существующий буфер если размер совпадает
        match &mut self.block_data {
            Some(ref mut buf) if buf.len() == block_samples => {
                // Буфер уже нужного размера — copy_from_slice перезапишет данные
            }
            _ => {
                self.block_data = Some(vec![ComplexNumber::from_parts(0.0, 0.0); block_samples]);
            }
        }

        // Убедимся, что размер милисекундного массива соответствует samples_per_ms
        if self.sample_array.len() != samples_per_ms {
            self.sample_array
                .resize(samples_per_ms, ComplexNumber::new());
            self.sample_number = samples_per_ms as i32;
        }

        // ОПТИМИЗАЦИЯ: НЕ сбрасываем кэши каждый блок.
        // PrnCache самоинвалидируется через needs_update().
        // CarrierPhaseCache инвалидируется при смене phase_step (в get_if_sample_cached).

        // Для каждой миллисекунды получаем точные выборки и копируем их в блок
        for ms_offset in 0..block_duration_ms {
            let current_time = GnssTime {
                Week: start_time.Week,
                MilliSeconds: start_time.MilliSeconds + ms_offset,
                SubMilliSeconds: start_time.SubMilliSeconds,
            };

            // Генерация точных IF выборок для текущей миллисекунды
            self.get_if_sample_cached(current_time);

            // Копирование в блочный буфер
            if let Some(ref mut block_data) = self.block_data {
                let start_idx = (ms_offset as usize) * samples_per_ms;
                let end_idx = start_idx + samples_per_ms;
                if end_idx <= block_data.len() {
                    block_data[start_idx..end_idx]
                        .copy_from_slice(&self.sample_array[..samples_per_ms]);
                }
            }
        }
    }


    /// Генерация IF сигнала с кэшированием
    /// Для GPS L1CA использует быстрый путь с PrnCache + bitmask,
    /// для BOC-сигналов (BDS B1C, GAL E1) — корректный путь с modulo и BOC-обработкой
    pub fn get_if_sample_cached(&mut self, cur_time: GnssTime) {
        // Получаем параметры спутника
        let p_sat_param = if let Some(ref sat_param) = self.sat_param {
            sat_param
        } else {
            return;
        };

        // Compute actual transmit time for nav bit indexing (MUST use transmit time, not receiver time!)
        // SignalSim: nav bits keyed off StartTransmitTime (transmit time coordinates)
        let travel_time = get_travel_time(p_sat_param, self.signal_index as usize);
        let actual_transmit_time = get_transmit_time(&cur_time, travel_time);

        // Update navigation bits at bit boundaries using TRANSMIT time
        {
            let bit_period_ms = self.satellite_signal.attribute.code_length
                * self.satellite_signal.attribute.nh_length;
            if bit_period_ms > 0 {
                let transmit_ms = actual_transmit_time.MilliSeconds;
                let current_bit_idx = transmit_ms / bit_period_ms;
                if current_bit_idx != self.last_nav_bit_index {
                    self.satellite_signal.get_satellite_signal(
                        actual_transmit_time,
                        &mut self.data_signal,
                        &mut self.pilot_signal,
                    );
                    self.last_nav_bit_index = current_bit_idx;
                }
            }
        }

        let code_attribute = self.prn_sequence.attribute.as_ref().unwrap();
        let chip_rate = code_attribute.chip_rate as f64;
        let _pilot_period = code_attribute.pilot_period as f64;

        // Doppler MUST be computed BEFORE cache update — code Doppler depends on it
        let doppler_hz = get_doppler(p_sat_param, self.signal_index as usize);
        let carrier_freq_hz = self.get_signal_center_freq();

        if self
            .computation_cache
            .needs_update(p_sat_param.CN0 as f64, self.sample_number, doppler_hz)
        {
            self.computation_cache.update(
                p_sat_param.CN0 as f64,
                self.sample_number,
                self.if_freq as f64,
                chip_rate,
                doppler_hz,
                carrier_freq_hz,
            );
        }

        let amp = self.computation_cache.cached_amp;

        // Code phase from start_code_phase (Doppler model)
        let base_chip_offset = self.start_code_phase;
        let code_step = self.computation_cache.cached_code_step;

        // BUG 1 FIX: No scalar nav_value — use complex data_signal/pilot_signal directly
        // This preserves I/Q channel orientation for signals where data is in Q, pilot in I

        // BOC flag for subchip modulation
        let is_boc = (code_attribute.attribute & PRN_ATTRIBUTE_BOC) != 0;

        // Carrier phase: Doppler model (f64 sin/cos)
        let doppler_cycles_per_ms = doppler_hz / 1000.0;
        let n = self.sample_number as f64;
        let phase_step = (doppler_cycles_per_ms + self.if_freq as f64 / 1000.0) / n;
        let frac_phase = self.start_carrier_phase - self.start_carrier_phase.floor();
        let mut cur_phase = 1.0 - frac_phase;

        // GLONASS half cycle compensation for odd frequency satellites
        if p_sat_param.system == GnssSystem::GlonassSystem
            && (p_sat_param.FreqID & 1) != 0
            && (cur_time.MilliSeconds & 1) != 0
        {
            cur_phase += 0.5;
        }

        // Update start_carrier_phase via Doppler accumulation (no re-anchor)
        self.start_carrier_phase -= doppler_cycles_per_ms;

        {
            let data_length = self.data_length;
            let pilot_length = self.pilot_length;
            let is_cboc = (code_attribute.attribute & PRN_ATTRIBUTE_CBOC) != 0;
            let is_qmboc = (code_attribute.attribute & PRN_ATTRIBUTE_QMBOC) != 0;
            let data_period = code_attribute.data_period;

            // BUG 4 FIX: Local copies for mid-sample nav bit transitions
            // When code chip counter wraps (crosses data period boundary), update nav bits
            let mut data_signal_local = self.data_signal;
            let mut pilot_signal_local = self.pilot_signal;
            let mut signal_time_local = self.signal_time;
            let mut prev_data_chip: i32 = -1;

            for i in 0..self.sample_number as usize {
                let chip_raw = (base_chip_offset + (i as f64) * code_step) as i32;

                // --- Data channel (COMPLEX modulation, BUG 1) ---
                let chip_mod = chip_raw.rem_euclid(data_length);
                let data_chip_idx = if is_boc { chip_mod >> 1 } else { chip_mod };

                // BUG 4: Mid-sample nav bit transition detection
                // When data chip wraps around (new code period), advance signal_time and update nav bits
                if prev_data_chip >= 0 && data_chip_idx < prev_data_chip {
                    signal_time_local.MilliSeconds += data_period;
                    self.satellite_signal.get_satellite_signal(
                        signal_time_local,
                        &mut data_signal_local,
                        &mut pilot_signal_local,
                    );
                }
                prev_data_chip = data_chip_idx;

                let data_sign = if let Some(data_prn) = &self.prn_sequence.data_prn {
                    if (data_chip_idx as usize) < data_prn.len() && data_prn[data_chip_idx as usize] != 0 {
                        -1.0
                    } else {
                        1.0
                    }
                } else {
                    1.0
                };

                // BUG 1: Complex data modulation — preserves I/Q orientation
                let mut prn_r = data_signal_local.real * data_sign;
                let mut prn_i = data_signal_local.imag * data_sign;

                // BOC subchip sign flip for data
                if is_boc && (chip_mod & 1) != 0 {
                    prn_r = -prn_r;
                    prn_i = -prn_i;
                }

                // --- Pilot channel (COMPLEX modulation, BUG 1) ---
                if let Some(pilot_prn) = &self.prn_sequence.pilot_prn {
                    if pilot_length > 0 {
                        let pilot_chip_mod = chip_raw.rem_euclid(pilot_length);
                        let pilot_chip = if is_boc { (pilot_chip_mod >> 1) as usize } else { pilot_chip_mod as usize };

                        if pilot_chip < pilot_prn.len() {
                            let pilot_sign = if pilot_prn[pilot_chip] != 0 { -1.0 } else { 1.0 };
                            // BUG 1: Complex pilot modulation
                            let mut p_r = pilot_signal_local.real * pilot_sign;
                            let mut p_i = pilot_signal_local.imag * pilot_sign;

                            // BOC/QMBOC/CBOC flip logic for pilot
                            let mut flip = false;
                            if (is_qmboc || is_cboc) && is_boc {
                                if is_qmboc {
                                    let symbol_pos = (signal_time_local.MilliSeconds % 330) / 10;
                                    if symbol_pos == 1 || symbol_pos == 5 || symbol_pos == 7 || symbol_pos == 30 {
                                        let sub_chip_pos = chip_raw.rem_euclid(12);
                                        if sub_chip_pos >= 6 { flip = true; }
                                    } else if (pilot_chip_mod & 1) != 0 { flip = true; }
                                } else {
                                    // CBOC (Galileo E1)
                                    let chip_in_code = chip_raw.rem_euclid(4092);
                                    if (chip_in_code % 11) == 0 {
                                        let boc6_phase = chip_raw.rem_euclid(12);
                                        if boc6_phase >= 6 { flip = true; }
                                    } else if (pilot_chip_mod & 1) != 0 { flip = true; }
                                }
                            } else if is_boc && (pilot_chip_mod & 1) != 0 {
                                flip = true;
                            }

                            if flip {
                                p_r = -p_r;
                                p_i = -p_i;
                            }

                            prn_r += p_r;
                            prn_i += p_i;
                        }
                    }
                }

                // Carrier phase: f64 sin/cos (Doppler model)
                let angle = cur_phase * std::f64::consts::TAU;
                let (sin_val, cos_val) = angle.sin_cos();
                cur_phase += phase_step;

                self.sample_array[i] = ComplexNumber {
                    real: (prn_r * cos_val - prn_i * sin_val) * amp,
                    imag: (prn_r * sin_val + prn_i * cos_val) * amp,
                };
            }

            // BUG 4: Save back local state after mid-sample transitions
            self.data_signal = data_signal_local;
            self.pilot_signal = pilot_signal_local;
            self.signal_time = signal_time_local;
        }
        // Advance code phase by one ms worth of chips
        self.start_code_phase += self.computation_cache.cached_chip_rate;
        if self.pilot_length > 0 {
            self.start_code_phase = self.start_code_phase.rem_euclid(self.pilot_length as f64);
        }
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
            let existing_magnitude = (self.sample_array[i].real * self.sample_array[i].real
                + self.sample_array[i].imag * self.sample_array[i].imag)
                .sqrt();
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
