use std::env;
use std::f64::consts::PI;
use std::fs::File;
use std::io::{BufReader, Read};

/// Структура для комплексного числа
#[derive(Clone, Copy, Debug)]
struct Complex {
    real: f64,
    imag: f64,
}

impl Complex {
    fn new(real: f64, imag: f64) -> Self {
        Complex { real, imag }
    }

    fn magnitude(&self) -> f64 {
        (self.real * self.real + self.imag * self.imag).sqrt()
    }
}

/// Быстрое преобразование Фурье (FFT) - алгоритм Кули-Тьюки
fn fft(data: &mut [Complex]) {
    let n = data.len();
    if n <= 1 {
        return;
    }

    // Проверка, что размер - степень двойки
    if n & (n - 1) != 0 {
        panic!("FFT размер должен быть степенью 2, получено: {}", n);
    }

    // Бит-реверсивная перестановка
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;

        if i < j {
            data.swap(i, j);
        }
    }

    // FFT по Кули-Тьюки
    let mut len = 2;
    while len <= n {
        let angle = -2.0 * PI / len as f64;
        let wlen = Complex::new(angle.cos(), angle.sin());

        let mut i = 0;
        while i < n {
            let mut w = Complex::new(1.0, 0.0);

            for j in 0..len / 2 {
                let u = data[i + j];
                let v = Complex {
                    real: data[i + j + len / 2].real * w.real - data[i + j + len / 2].imag * w.imag,
                    imag: data[i + j + len / 2].real * w.imag + data[i + j + len / 2].imag * w.real,
                };

                data[i + j] = Complex {
                    real: u.real + v.real,
                    imag: u.imag + v.imag,
                };

                data[i + j + len / 2] = Complex {
                    real: u.real - v.real,
                    imag: u.imag - v.imag,
                };

                // w = w * wlen
                let w_new = Complex {
                    real: w.real * wlen.real - w.imag * wlen.imag,
                    imag: w.real * wlen.imag + w.imag * wlen.real,
                };
                w = w_new;
            }
            i += len;
        }
        len <<= 1;
    }
}

/// Чтение IQ данных из файла
fn read_iq_data(
    filename: &str,
    format: &str,
    max_samples: usize,
) -> Result<Vec<Complex>, Box<dyn std::error::Error>> {
    let mut file = BufReader::new(File::open(filename)?);
    let mut samples = Vec::new();

    match format {
        "iq4" => {
            // IQ4: 4 бита I + 4 бита Q в одном байте
            let mut buffer = [0u8; 1024];
            let mut total_read = 0;

            while total_read < max_samples {
                let bytes_read = file.read(&mut buffer)?;
                if bytes_read == 0 {
                    break;
                }

                for &byte in &buffer[..bytes_read] {
                    if samples.len() >= max_samples {
                        break;
                    }

                    // Распаковка 4-битных значений со знаком
                    let i_val = ((byte & 0xF0) as i8) >> 4; // Старшие 4 бита
                    let q_val = ((byte & 0x0F) << 4) as i8 >> 4; // Младшие 4 бита с расширением знака

                    samples.push(Complex::new(
                        i_val as f64 / 8.0, // Нормализация к [-1, 1]
                        q_val as f64 / 8.0,
                    ));
                }
                total_read = samples.len();
            }
        }
        "iq8" => {
            // IQ8: 8 бит I + 8 бит Q = 2 байта на отсчёт
            let mut buffer = [0u8; 2048];
            let mut total_read = 0;

            while total_read < max_samples {
                let bytes_read = file.read(&mut buffer)?;
                if bytes_read == 0 {
                    break;
                }

                for chunk in buffer[..bytes_read].chunks_exact(2) {
                    if samples.len() >= max_samples {
                        break;
                    }

                    let i_val = chunk[0] as i8;
                    let q_val = chunk[1] as i8;

                    samples.push(Complex::new(
                        i_val as f64 / 128.0, // Нормализация к [-1, 1]
                        q_val as f64 / 128.0,
                    ));
                }
                total_read = samples.len();
            }
        }
        _ => return Err(format!("Неизвестный формат: {}", format).into()),
    }

    Ok(samples)
}

/// Расчёт спектральной плотности мощности (PSD) для IF сигнала
fn calculate_psd(samples: &[Complex], sample_rate: f64) -> (Vec<f64>, Vec<f64>) {
    // Размер FFT - ближайшая степень 2
    let fft_size = samples.len().next_power_of_two();

    // Подготовка данных для FFT с дополнением нулями
    let mut fft_data: Vec<Complex> = samples.to_vec();
    fft_data.resize(fft_size, Complex::new(0.0, 0.0));

    // Применение окна Хэмминга для уменьшения спектральных утечек
    for (i, val) in fft_data.iter_mut().enumerate().take(samples.len()) {
        let window = 0.54 - 0.46 * (2.0 * PI * i as f64 / (samples.len() - 1) as f64).cos();
        val.real *= window;
        val.imag *= window;
    }

    // Выполнение FFT
    fft(&mut fft_data);

    // Расчёт PSD с учётом отрицательных и положительных частот
    let mut frequencies = Vec::new();
    let mut psd = Vec::new();

    let freq_resolution = sample_rate / fft_size as f64;

    // ВАЖНО: Для IF сигнала нужен полный спектр от -fs/2 до +fs/2
    // FFT выдаёт: [0...fs/2, -fs/2...0]
    // Переупорядочиваем для отображения [-fs/2...0...+fs/2]

    // Сначала отрицательные частоты (вторая половина FFT)
    for (offset, val) in fft_data[fft_size / 2..].iter().enumerate() {
        let i = offset + fft_size / 2;
        let freq = (i as f64 - fft_size as f64) * freq_resolution;
        frequencies.push(freq);

        let magnitude = val.magnitude();
        let power_db = if magnitude > 0.0 {
            20.0 * (magnitude / fft_size as f64).log10()
        } else {
            -200.0 // Минимальное значение для нулевой мощности
        };
        psd.push(power_db);
    }

    // Затем положительные частоты (первая половина FFT)
    for (i, val) in fft_data[..fft_size / 2].iter().enumerate() {
        let freq = i as f64 * freq_resolution;
        frequencies.push(freq);

        let magnitude = val.magnitude();
        let power_db = if magnitude > 0.0 {
            20.0 * (magnitude / fft_size as f64).log10()
        } else {
            -200.0 // Минимальное значение для нулевой мощности
        };
        psd.push(power_db);
    }

    (frequencies, psd)
}

/// Вывод спектра в текстовом виде (ASCII график)
fn plot_spectrum_ascii(frequencies: &[f64], psd: &[f64], center_freq: f64, width: usize) {
    // Найти минимум и максимум PSD
    let min_psd = psd.iter().fold(f64::INFINITY, |a, &b| a.min(b));
    let max_psd = psd.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
    let range = max_psd - min_psd;

    println!("\n════════════════════════════════════════════════════════════════════");
    println!("                    СПЕКТР GNSS СИГНАЛА");
    println!("════════════════════════════════════════════════════════════════════");
    println!("Центральная частота: {:.3} МГц", center_freq / 1e6);
    println!("Динамический диапазон: {:.1} дБ", range);
    println!("Максимум: {:.1} дБ, Минимум: {:.1} дБ", max_psd, min_psd);
    println!("────────────────────────────────────────────────────────────────────");

    // Высота графика
    let height = 20;

    // Создание ASCII графика
    for h in (0..height).rev() {
        let threshold = min_psd + (h as f64 / height as f64) * range;

        // Метка уровня
        if h == height - 1 || h == height / 2 || h == 0 {
            print!("{:6.1} дБ │", threshold);
        } else {
            print!("          │");
        }

        // График
        for i in (0..psd.len()).step_by(psd.len() / width) {
            if psd[i] >= threshold {
                print!("█");
            } else if psd[i] >= threshold - range / height as f64 {
                print!("▄");
            } else {
                print!(" ");
            }
        }
        println!();
    }

    // Ось частот
    println!("          └{}", "─".repeat(width));
    print!("           ");
    let freq_min = frequencies[0] / 1e3;
    let freq_max = frequencies[frequencies.len() - 1] / 1e3;
    let freq_center = (freq_min + freq_max) / 2.0;
    println!(
        "{:<.1} кГц          {:.1} кГц          {:.1} кГц",
        freq_min, freq_center, freq_max
    );

    // Поиск пиков (спутниковых сигналов)
    println!("\n═══════════════════════════════════════════════════════════════════");
    println!("                    ОБНАРУЖЕННЫЕ СИГНАЛЫ");
    println!("═══════════════════════════════════════════════════════════════════");

    // Простой детектор пиков
    let noise_floor = min_psd + 10.0; // Порог на 10 дБ выше минимума
    let mut peaks = Vec::new();

    for i in 1..psd.len() - 1 {
        if psd[i] > noise_floor && psd[i] > psd[i - 1] && psd[i] > psd[i + 1] {
            peaks.push((frequencies[i], psd[i]));
        }
    }

    // Сортировка по мощности
    peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

    // Вывод топ-10 пиков
    println!("№  │ Частота (кГц) │ Доплер (Гц) │ Мощность (дБ) │ SNR (дБ)");
    println!("───┼───────────────┼─────────────┼───────────────┼──────────");
    for (idx, &(freq, power)) in peaks.iter().take(10).enumerate() {
        let doppler = freq - center_freq;
        let snr = power - min_psd;
        println!(
            "{:2} │ {:13.3} │ {:+11.1} │ {:13.1} │ {:8.1}",
            idx + 1,
            freq / 1e3,
            doppler,
            power,
            snr
        );
    }

    if peaks.is_empty() {
        println!("Сигналы не обнаружены. Проверьте уровень сигнала и формат данных.");
    } else {
        println!("\nВсего обнаружено сигналов: {}", peaks.len());
    }
}

/// Генерация CSV файла со спектром
fn save_spectrum_csv(
    filename: &str,
    frequencies: &[f64],
    psd: &[f64],
) -> Result<(), Box<dyn std::error::Error>> {
    use std::io::Write;

    let mut file = File::create(filename)?;
    writeln!(file, "Frequency_Hz,Power_dB")?;

    for (freq, power) in frequencies.iter().zip(psd.iter()) {
        writeln!(file, "{},{}", freq, power)?;
    }

    println!("\nСпектр сохранён в файл: {}", filename);
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        eprintln!(
            "Использование: {} <if_data_file> [format] [sample_rate] [center_freq]",
            args[0]
        );
        eprintln!("  format: iq4 или iq8 (по умолчанию: iq8)");
        eprintln!("  sample_rate: частота дискретизации в Гц (по умолчанию: 5000000)");
        eprintln!("  center_freq: центральная частота в Гц (по умолчанию: 0)");
        eprintln!("\nПример: {} if_data.bin iq8 5000000 0", args[0]);
        return Ok(());
    }

    let filename = &args[1];
    let format = args.get(2).map(|s| s.as_str()).unwrap_or("iq8");
    let sample_rate: f64 = args
        .get(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(5_000_000.0);
    let center_freq: f64 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(0.0);

    println!("Анализ спектра GNSS сигнала");
    println!("Файл: {}", filename);
    println!("Формат: {}", format.to_uppercase());
    println!("Частота дискретизации: {:.3} МГц", sample_rate / 1e6);
    println!("Центральная частота: {:.3} МГц", center_freq / 1e6);

    // Чтение данных (ограничиваем размер для FFT)
    let max_samples = 65536; // 64K для быстрого FFT
    println!("\nЧтение {} отсчётов...", max_samples);
    let samples = read_iq_data(filename, format, max_samples)?;
    println!("Прочитано {} отсчётов", samples.len());

    // Расчёт спектра
    println!("Вычисление спектра (FFT)...");
    let (frequencies, psd) = calculate_psd(&samples, sample_rate);

    // Вывод результатов
    plot_spectrum_ascii(&frequencies, &psd, center_freq, 70);

    // Сохранение в CSV
    if args.len() > 5 && args[5] == "--csv" {
        save_spectrum_csv("spectrum.csv", &frequencies, &psd)?;
    }

    println!("\n💡 Подсказка: используйте --csv для сохранения спектра в файл");

    Ok(())
}
