//! # Модуль управления мощностью сигналов
//!
//! Этот модуль отвечает за управление мощностью спутниковых сигналов в ГНСС системах.
//! Основные функции:
//! - Контроль уровня мощности сигналов различных ГНСС систем
//! - Регулировка отношения сигнал/шум (C/N0) в зависимости от условий
//! - Коррекция мощности по углу места спутника
//! - Моделирование ослабления сигнала в различных условиях приема
//! - Управление временными изменениями мощности сигнала
//!
//! Модуль критически важен для реалистичного моделирования условий
//! приема ГНСС сигналов и тестирования приемников.

//----------------------------------------------------------------------
// powercontrol.rs:
//   Definition of signal power control class
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

/// Signal power control structure
/// Структура для управления мощностью сигнала спутника
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct SignalPower {
    pub system: i32, // GNSS система (GPS=0, BDS=1, Galileo=2, GLONASS=3)
    pub svid: i32,   // ID спутника
    pub time: i32,   // Время в миллисекундах
    pub cn0: f64,    // Отношение сигнал/шум в дБ-Гц
}

/// Elevation adjustment types
/// Типы коррекции по углу места
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ElevationAdjust {
    ElevationAdjustNone = 0,        // Без коррекции
    ElevationAdjustSinSqrtFade = 1, // Коррекция с затуханием sin(sqrt(elevation))
}

/// Power control class for managing satellite signal power
/// Класс управления мощностью сигналов спутников
pub struct CPowerControl {
    /// Array size / Размер массива
    pub array_size: usize,

    /// Next index for processing / Следующий индекс для обработки
    pub next_index: usize,

    /// Power control array / Массив управления мощностью
    pub power_control_array: Vec<SignalPower>,

    /// Time elapsed in milliseconds / Прошедшее время в миллисекундах
    pub time_elaps_ms: i32,

    /// Elevation adjustment type / Тип коррекции по углу места
    pub adjust: ElevationAdjust,

    /// Noise floor in dBm / Уровень шума в дБм
    pub noise_floor: f64,

    /// Initial CN0 in dB-Hz / Начальное отношение сигнал/шум в дБ-Гц
    pub init_cn0: f64,
}

impl CPowerControl {
    /// Constructor / Конструктор
    pub fn new() -> Self {
        CPowerControl {
            array_size: 0,
            power_control_array: Vec::new(),
            adjust: ElevationAdjust::ElevationAdjustNone,
            noise_floor: -172.0,
            init_cn0: 47.0,
            next_index: 0,
            time_elaps_ms: 0,
        }
    }

    /// Add control element to the array
    /// Добавить элемент управления в массив
    pub fn add_control_element(&mut self, control_element: &SignalPower) {
        // Allocate array or expand size (grows by 100 elements at a time like in C++)
        // Выделить массив или расширить размер (растет по 100 элементов как в C++)
        if self.power_control_array.is_empty() || (self.array_size % 100) == 0 {
            self.power_control_array.reserve(100);
        }

        self.power_control_array.push(*control_element);
        self.array_size += 1;
    }

    /// Sort the power control array by time
    /// Сортировать массив управления мощностью по времени
    pub fn sort(&mut self) {
        // Selection sort implementation (same as C++ version)
        // Реализация сортировки выбором (как в C++ версии)
        for i in 0..self.array_size.saturating_sub(1) {
            let mut min_index = i;

            for j in (i + 1)..self.array_size {
                if self.power_control_array[j].time < self.power_control_array[min_index].time {
                    min_index = j;
                }
            }

            // Swap min time element to position i
            // Поменять местами элемент с минимальным временем с позицией i
            if min_index != i {
                self.power_control_array.swap(i, min_index);
            }
        }
    }

    /// Reset time counter
    /// Сбросить счетчик времени
    pub fn reset_time(&mut self) {
        self.time_elaps_ms = 0;
        self.next_index = 0;
    }

    /// Get power control list for the given time step
    /// Получить список управления мощностью для заданного временного шага
    ///
    /// # Arguments
    /// * `time_step_ms` - Time step in milliseconds / Временной шаг в миллисекундах
    ///
    /// # Returns
    /// * `(&[SignalPower], usize)` - Tuple of (power list slice, count) / Кортеж (срез списка мощности, количество)
    pub fn get_power_control_list(&mut self, time_step_ms: i32) -> (&[SignalPower], usize) {
        let init_index = self.next_index;

        self.time_elaps_ms += time_step_ms;

        while self.next_index < self.array_size {
            if self.power_control_array[self.next_index].time > self.time_elaps_ms {
                break;
            }
            self.next_index += 1;
        }

        let count = self.next_index - init_index;
        let slice = if count > 0 && init_index < self.power_control_array.len() {
            &self.power_control_array[init_index..self.next_index]
        } else {
            &[]
        };

        (slice, count)
    }

    /// Get noise floor value
    /// Получить значение уровня шума
    pub fn get_noise_floor(&self) -> f64 {
        self.noise_floor
    }

    /// Set noise floor value
    /// Установить значение уровня шума
    pub fn set_noise_floor(&mut self, noise_floor: f64) {
        self.noise_floor = noise_floor;
    }

    /// Get initial CN0 value
    /// Получить начальное значение CN0
    pub fn get_init_cn0(&self) -> f64 {
        self.init_cn0
    }

    /// Alias for get_power_control_list for compatibility
    /// Псевдоним для get_power_control_list для совместимости
    pub fn get_power_list(&mut self, time_step_ms: i32) -> (&[SignalPower], usize) {
        self.get_power_control_list(time_step_ms)
    }

    /// Set initial CN0 value
    /// Установить начальное значение CN0
    pub fn set_init_cn0(&mut self, init_cn0: f64) {
        self.init_cn0 = init_cn0;
    }

    /// Set elevation adjustment type
    /// Установить тип коррекции по углу места
    pub fn set_elevation_adjust(&mut self, adjust: ElevationAdjust) {
        self.adjust = adjust;
    }

    /// Get elevation adjustment type
    /// Получить тип коррекции по углу места
    pub fn get_elevation_adjust(&self) -> ElevationAdjust {
        self.adjust
    }
}

impl Default for CPowerControl {
    fn default() -> Self {
        Self::new()
    }
}

// C-compatible functions for FFI (if needed)
// C-совместимые функции для FFI (если нужно)

/// Set noise floor (C-compatible)
#[no_mangle]
pub extern "C" fn set_noise_floor(power_control: &mut CPowerControl, noise_floor: f64) {
    power_control.set_noise_floor(noise_floor);
}

/// Get noise floor (C-compatible)
#[no_mangle]
pub extern "C" fn get_noise_floor(power_control: &CPowerControl) -> f64 {
    power_control.get_noise_floor()
}

/// Set initial CN0 (C-compatible)
#[no_mangle]
pub extern "C" fn set_init_cn0(power_control: &mut CPowerControl, cn0: f64) {
    power_control.set_init_cn0(cn0);
}

/// Get initial CN0 (C-compatible)
#[no_mangle]
pub extern "C" fn get_init_cn0(power_control: &CPowerControl) -> f64 {
    power_control.get_init_cn0()
}

/// Set elevation adjust (C-compatible)
#[no_mangle]
pub extern "C" fn set_elevation_adjust(power_control: &mut CPowerControl, adjust: i32) {
    let elevation_adjust = match adjust {
        0 => ElevationAdjust::ElevationAdjustNone,
        1 => ElevationAdjust::ElevationAdjustSinSqrtFade,
        _ => ElevationAdjust::ElevationAdjustNone,
    };
    power_control.set_elevation_adjust(elevation_adjust);
}

/// Add control element (C-compatible)
#[no_mangle]
pub extern "C" fn add_control_element(
    power_control: &mut CPowerControl,
    signal_power: &SignalPower,
) {
    power_control.add_control_element(signal_power);
}
