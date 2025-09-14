//! ЭКСТРЕМАЛЬНЫЕ AVX-512 ОПТИМИЗАЦИИ ДЛЯ AMD RYZEN 9 7950X
//! Максимальное использование 512-bit векторных инструкций процессора

#[cfg(target_arch = "x86_64")]
use core::arch::x86_64::*;

/// AVX-512 оптимизированная версия GNSS сигнальной обработки
///
/// КРИТИЧЕСКОЕ УСКОРЕНИЕ: Обрабатывает 16 double значений одновременно!
/// (512 bit / 32 bit = 16 float или 512 bit / 64 bit = 8 double)
pub struct Avx512Accelerator {
    #[allow(dead_code)]
    initialized: bool,
}

impl Default for Avx512Accelerator {
    fn default() -> Self {
        Self::new()
    }
}

impl Avx512Accelerator {
    /// Создать новый AVX-512 ускоритель
    pub fn new() -> Self {
        Self { initialized: false }
    }

    /// Проверить доступность AVX-512 на данном процессоре
    #[cfg(target_arch = "x86_64")]
    pub fn is_available() -> bool {
        is_x86_feature_detected!("avx512f")
            && is_x86_feature_detected!("avx512dq")
            && is_x86_feature_detected!("avx512bw")
    }

    /// МЕГАОПТИМИЗАЦИЯ: Обработка PRN кодов с AVX-512
    /// Обрабатывает 16 float значений за одну инструкцию!
    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx512f")]
    /// # Safety
    /// Requires AVX‑512 support. Slices must have length >= 16 and be valid
    /// for read/write; they must not alias in undefined ways.
    pub unsafe fn process_prn_codes_avx512(prn_data: &[f32], samples: &mut [f32], amplitude: f32) {
        assert!(prn_data.len() >= 16);
        assert!(samples.len() >= 16);

        // РЕВОЛЮЦИОННАЯ СКОРОСТЬ: загружаем 16 float за раз
        let prn_vec = _mm512_loadu_ps(prn_data.as_ptr());
        let amp_vec = _mm512_set1_ps(amplitude);

        // МОЛНИЕНОСНОЕ умножение 16 элементов одновременно
        let result = _mm512_mul_ps(prn_vec, amp_vec);

        // Сохраняем результат
        _mm512_storeu_ps(samples.as_mut_ptr(), result);
    }

    /// ЭКСТРЕМАЛЬНАЯ ОПТИМИЗАЦИЯ: Комплексные вычисления с AVX-512
    /// Обрабатывает 8 комплексных чисел (16 float) одновременно
    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx512f")]
    /// # Safety
    /// Caller must ensure CPU supports AVX‑512 (checked via `is_available()`),
    /// and that all input/output slices have length >= 16 and do not overlap in
    /// undefined ways for concurrent reads/writes. Pointers are read/written via
    /// unaligned loads/stores (`_mm512_loadu_ps/_storeu_ps`).
    pub unsafe fn complex_multiply_avx512(
        real_a: &[f32],
        imag_a: &[f32],
        real_b: &[f32],
        imag_b: &[f32],
        real_out: &mut [f32],
        imag_out: &mut [f32],
    ) {
        assert_eq!(real_a.len(), 16);
        assert_eq!(imag_a.len(), 16);

        // Загружаем 16 элементов каждого массива
        let ra = _mm512_loadu_ps(real_a.as_ptr());
        let ia = _mm512_loadu_ps(imag_a.as_ptr());
        let rb = _mm512_loadu_ps(real_b.as_ptr());
        let ib = _mm512_loadu_ps(imag_b.as_ptr());

        // Комплексное умножение: (a + ib) * (c + id) = (ac - bd) + i(ad + bc)
        // real = ra * rb - ia * ib
        let real_part = _mm512_fmsub_ps(ra, rb, _mm512_mul_ps(ia, ib));

        // imag = ra * ib + ia * rb
        let imag_part = _mm512_fmadd_ps(ra, ib, _mm512_mul_ps(ia, rb));

        // Сохраняем результаты
        _mm512_storeu_ps(real_out.as_mut_ptr(), real_part);
        _mm512_storeu_ps(imag_out.as_mut_ptr(), imag_part);
    }

    /// ОПТИМИЗИРОВАННАЯ функция для PRN * carrier с амплитудой
    /// Специально для случая генерации GNSS сигналов
    #[target_feature(enable = "avx512f")]
    /// # Safety
    /// Requires AVX‑512 support and all slices length >= 16. Output slices must
    /// be distinct from inputs or aliasing must be safe for simultaneous reads/writes.
    pub unsafe fn complex_multiply_prn_carrier_avx512(
        prn_data: &[f32],
        carrier_cos: &[f32],
        carrier_sin: &[f32],
        output_i: &mut [f32],
        output_q: &mut [f32],
        amplitude: f32,
    ) {
        assert!(prn_data.len() >= 16);
        assert!(carrier_cos.len() >= 16);
        assert!(carrier_sin.len() >= 16);
        assert!(output_i.len() >= 16);
        assert!(output_q.len() >= 16);

        let prn = _mm512_loadu_ps(prn_data.as_ptr());
        let cos_val = _mm512_loadu_ps(carrier_cos.as_ptr());
        let sin_val = _mm512_loadu_ps(carrier_sin.as_ptr());
        let amp = _mm512_set1_ps(amplitude);

        // PRN * carrier_cos * amplitude для I канала
        let i_result = _mm512_mul_ps(_mm512_mul_ps(prn, cos_val), amp);
        // PRN * carrier_sin * amplitude для Q канала
        let q_result = _mm512_mul_ps(_mm512_mul_ps(prn, sin_val), amp);

        _mm512_storeu_ps(output_i.as_mut_ptr(), i_result);
        _mm512_storeu_ps(output_q.as_mut_ptr(), q_result);
    }

    /// СУПЕР-КРИТИЧЕСКАЯ ОПТИМИЗАЦИЯ: Тригонометрия с AVX-512
    /// Обрабатывает 16 углов одновременно используя lookup таблицы
    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx512f")]
    /// # Safety
    /// Requires AVX‑512 support. All provided buffers must be length >= 16.
    /// Indices computed are masked to LUT size; caller ensures LUTs are valid.
    pub unsafe fn fast_sin_cos_avx512(
        angles: &[f32],
        sin_lut: &[f32; 65536],
        cos_lut: &[f32; 65536],
        sin_out: &mut [f32],
        cos_out: &mut [f32],
    ) {
        assert_eq!(angles.len(), 16);

        // Загружаем 16 углов
        let angle_vec = _mm512_loadu_ps(angles.as_ptr());

        // Константы для преобразования в индексы lookup таблицы
        let scale = _mm512_set1_ps(65536.0 / (2.0 * std::f32::consts::PI));
        let mask = _mm512_set1_epi32(65535); // 0xFFFF для маскирования

        // Преобразуем углы в индексы
        let scaled = _mm512_mul_ps(angle_vec, scale);
        let indices = _mm512_cvtps_epi32(scaled);
        let masked_indices = _mm512_and_epi32(indices, mask);

        // К сожалению, AVX-512 не имеет прямой gather инструкции для float lookup
        // Используем альтернативный подход с обработкой каждого элемента
        let indices_array = [0i32; 16];
        _mm512_storeu_si512(indices_array.as_ptr() as *mut __m512i, masked_indices);

        for i in 0..16 {
            let idx = indices_array[i] as usize;
            sin_out[i] = sin_lut[idx];
            cos_out[i] = cos_lut[idx];
        }
    }

    /// МАКСИМАЛЬНАЯ ПРОИЗВОДИТЕЛЬНОСТЬ: Векторное накопление сигналов
    /// Суммирует 16 комплексных сигналов одновременно
    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx512f")]
    /// # Safety
    /// Requires AVX‑512 support. All slices length >= 16. Accumulators must be
    /// valid for mutable writes and may alias neither each other nor inputs.
    pub unsafe fn accumulate_signals_avx512(
        signal_real: &[f32],
        signal_imag: &[f32],
        accumulator_real: &mut [f32],
        accumulator_imag: &mut [f32],
    ) {
        assert_eq!(signal_real.len(), 16);

        // Загружаем данные
        let sig_r = _mm512_loadu_ps(signal_real.as_ptr());
        let sig_i = _mm512_loadu_ps(signal_imag.as_ptr());
        let acc_r = _mm512_loadu_ps(accumulator_real.as_ptr());
        let acc_i = _mm512_loadu_ps(accumulator_imag.as_ptr());

        // Накапливаем 16 значений одновременно
        let new_acc_r = _mm512_add_ps(acc_r, sig_r);
        let new_acc_i = _mm512_add_ps(acc_i, sig_i);

        // Сохраняем результаты
        _mm512_storeu_ps(accumulator_real.as_mut_ptr(), new_acc_r);
        _mm512_storeu_ps(accumulator_imag.as_mut_ptr(), new_acc_i);
    }

    /// ЭКСТРЕМАЛЬНАЯ ОПТИМИЗАЦИЯ: Параллельная обработка нескольких спутников
    /// Использует полную ширину AVX-512 для обработки данных от 16 спутников
    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx512f")]
    /// # Safety
    /// Requires AVX‑512 support. All slice references must be valid and sized
    /// exactly for 16 elements.
    pub unsafe fn parallel_satellite_processing_avx512(
        satellite_phases: &mut [f32; 16],
        phase_steps: &[f32; 16],
        prn_values: &[f32; 16],
        output_real: &mut [f32; 16],
        output_imag: &mut [f32; 16],
    ) {
        // Загружаем данные для всех 16 спутников одновременно
        let phases = _mm512_loadu_ps(satellite_phases.as_ptr());
        let steps = _mm512_loadu_ps(phase_steps.as_ptr());
        let prn = _mm512_loadu_ps(prn_values.as_ptr());

        // Обновляем фазы всех спутников параллельно
        let new_phases = _mm512_add_ps(phases, steps);
        _mm512_storeu_ps(satellite_phases.as_mut_ptr(), new_phases);

        // Быстрая тригонометрия через приближение (можно заменить на lookup)
        // Для демонстрации используем простое приближение
        let sin_approx = _mm512_mul_ps(new_phases, prn);
        let cos_approx = _mm512_sqrt_ps(_mm512_sub_ps(
            _mm512_set1_ps(1.0),
            _mm512_mul_ps(sin_approx, sin_approx),
        ));

        // Сохраняем результаты для всех спутников
        _mm512_storeu_ps(output_real.as_mut_ptr(), cos_approx);
        _mm512_storeu_ps(output_imag.as_mut_ptr(), sin_approx);
    }

    /// AVX-512 векторное сложение двух массивов float32
    /// СУПЕР-БЫСТРОЕ сложение 16 элементов одновременно!
    #[target_feature(enable = "avx512f")]
    /// # Safety
    /// Requires AVX‑512 support and all slices length >= 16. Result must be
    /// valid for writes and not alias `a`/`b` in undefined ways.
    pub unsafe fn vector_add_avx512(a: &[f32], b: &[f32], result: &mut [f32]) {
        assert!(a.len() >= 16 && b.len() >= 16 && result.len() >= 16);

        let vec_a = _mm512_loadu_ps(a.as_ptr());
        let vec_b = _mm512_loadu_ps(b.as_ptr());
        let vec_result = _mm512_add_ps(vec_a, vec_b);

        _mm512_storeu_ps(result.as_mut_ptr(), vec_result);
    }

    /// AVX-512 векторное умножение на скаляр
    /// Умножает массив float32 на скалярное значение
    #[target_feature(enable = "avx512f")]
    /// # Safety
    /// Requires AVX‑512 support and all slices length >= 16. `scalar` is
    /// expected to contain 16 identical values.
    pub unsafe fn vector_multiply_scalar_avx512(
        input: &[f32],
        scalar: &[f32], // Для совместимости с API (должен содержать 16 одинаковых значений)
        result: &mut [f32],
    ) {
        assert!(input.len() >= 16 && scalar.len() >= 16 && result.len() >= 16);

        let vec_input = _mm512_loadu_ps(input.as_ptr());
        let vec_scalar = _mm512_loadu_ps(scalar.as_ptr());
        let vec_result = _mm512_mul_ps(vec_input, vec_scalar);

        _mm512_storeu_ps(result.as_mut_ptr(), vec_result);
    }
}

/// Безопасная обертка для использования AVX-512 оптимизаций
pub struct SafeAvx512Processor;

impl Default for SafeAvx512Processor {
    fn default() -> Self {
        Self::new()
    }
}

impl SafeAvx512Processor {
    pub fn new() -> Self {
        SafeAvx512Processor
    }

    pub fn is_available(&self) -> bool {
        Avx512Accelerator::is_available()
    }
}

impl SafeAvx512Processor {
    /// Обработать PRN коды с максимальной скоростью
    pub fn process_prn_batch(prn_data: &[f32], amplitude: f32) -> Vec<f32> {
        let mut result = vec![0.0f32; prn_data.len()];

        #[cfg(target_arch = "x86_64")]
        {
            if Avx512Accelerator::is_available() {
                unsafe {
                    // Обрабатываем блоками по 16 элементов
                    for chunk in prn_data.chunks(16) {
                        if chunk.len() == 16 {
                            let start_idx =
                                (chunk.as_ptr() as usize - prn_data.as_ptr() as usize) / 4;
                            Avx512Accelerator::process_prn_codes_avx512(
                                chunk,
                                &mut result[start_idx..start_idx + 16],
                                amplitude,
                            );
                        } else {
                            // Fallback для последнего неполного блока
                            for (i, &prn_val) in chunk.iter().enumerate() {
                                let idx =
                                    (chunk.as_ptr() as usize - prn_data.as_ptr() as usize) / 4 + i;
                                result[idx] = prn_val * amplitude;
                            }
                        }
                    }
                }
                return result;
            }
        }

        // Fallback для систем без AVX-512
        for (i, &prn_val) in prn_data.iter().enumerate() {
            result[i] = prn_val * amplitude;
        }

        result
    }
}
