//! # GNSS IF Data Generator
//!
//! Главная программа для генерации промежуточных частотных (IF) данных спутниковых сигналов.
//! Портирована с C++ версии IFdataGen для обеспечения совместимости.
//!
//! ## Основные функции:
//! - Чтение конфигурации из JSON файлов
//! - Генерация IF данных для множественных ГНСС систем
//! - Поддержка различных форматов вывода (IQ4, IQ8)
//! - Моделирование траекторий и спутниковых параметров
//! - Управление мощностью сигналов и уровнем шума
//!
//! ## Использование:
//! ```bash
//! cargo run [JSON_CONFIG_FILE]
//! cargo run -- --help
//! ```
//!
//! Copyright (C) 2020-2029 by Jun Mo, All rights reserved.

use std::env;
use std::time::Instant;
use std::path::Path;
use gnss_rust::*;
use gnss_rust::types::*;
use gnss_rust::gnsstime::*;

const DEFAULT_CONFIG: &str = "config.json";

// Тест конвертации времени
fn test_utc_to_gps_conversion() {
    println!("=== ТЕСТ КОНВЕРТАЦИИ UTC → GPS ВРЕМЯ ===");
    
    let utc_time = UtcTime {
        Year: 2025,
        Month: 6,
        Day: 5,
        Hour: 10,
        Minute: 5,
        Second: 30.0,
    };

    println!("Тестируемое UTC время: 2025-06-05 10:05:30");
    
    // Тестируем текущую функцию
    let gps_time = utc_to_gps_time(utc_time, false);
    println!("Результат utc_to_gps_time: Week {}, MilliSeconds {}", 
             gps_time.Week, gps_time.MilliSeconds);
    
    // Ожидаемый результат
    let expected_week = 2369;
    let expected_ms = 36330000; // четверг 10:05:30
    println!("Ожидается: Week {}, MilliSeconds {}", expected_week, expected_ms);
    
    // Проверим промежуточное GLONASS время (базовый год 1992)
    let glonass_time = utc_to_glonass_time(utc_time);
    println!("Промежуточное GLONASS время (1992): LeapYear {}, Day {}, MilliSeconds {}", 
             glonass_time.LeapYear, glonass_time.Day, glonass_time.MilliSeconds);
    
    // Проверим исправленную функцию (базовый год 1996) 
    let glonass_time_corrected = utc_to_glonass_time_corrected(utc_time);
    println!("Исправленное GLONASS время (1996): LeapYear {}, Day {}, MilliSeconds {}", 
             glonass_time_corrected.LeapYear, glonass_time_corrected.Day, glonass_time_corrected.MilliSeconds);
    
    // Анализ дней от GPS эпохи
    println!("\n=== АНАЛИЗ РАСЧЁТА ДНЕЙ ===");
    
    // Дни от GPS эпохи (6 января 1980) до промежуточной эпохи (1992)
    let days_1980_to_1992 = (1992 - 1980) * 365 + 3; // 3 високосных года (1980, 1984, 1988)
    println!("Дни от GPS эпохи (1980) до промежуточной (1992): {}", days_1980_to_1992);
    
    // Дни от промежуточной эпохи до целевой даты
    let days_1992_to_2025 = glonass_time.LeapYear * (366 + 365 * 3) + glonass_time.Day;
    println!("Дни от промежуточной эпохи (1992) до 2025-06-05: {}", days_1992_to_2025);
    
    // Общие дни от GPS эпохи
    let total_days = days_1980_to_1992 + days_1992_to_2025 - 6; // -6 дней от 1 января до 6 января 1980
    println!("Общее количество дней от GPS эпохи: {}", total_days);
    println!("GPS недель: {}", total_days / 7);
    println!("Остаток дней (день недели): {}", total_days % 7);
    
    // Проверим какой день недели
    let day_names = ["воскресенье", "понедельник", "вторник", "среда", "четверг", "пятница", "суббота"];
    println!("День недели: {}", day_names[(total_days % 7) as usize]);
    
    // Разница в результатах
    let week_diff = gps_time.Week - expected_week;
    let ms_diff = gps_time.MilliSeconds - expected_ms;
    println!("\n=== ОШИБКИ ===");
    println!("Ошибка в неделях: {} (должно быть 0)", week_diff);
    println!("Ошибка в миллисекундах: {} (должно быть 0)", ms_diff);
    println!("Ошибка в днях: {:.2}", ms_diff as f64 / 86400000.0);
    
    // Анализ дня недели
    println!("\n=== АНАЛИЗ ДНЯ НЕДЕЛИ ===");
    println!("GPS миллисекунды: {} мс", gps_time.MilliSeconds);
    println!("Это соответствует дню недели: {}", gps_time.MilliSeconds / 86400000);
    println!("Часы в этом дне: {}", (gps_time.MilliSeconds % 86400000) / 3600000);
    println!("Минуты в этом часе: {}", ((gps_time.MilliSeconds % 86400000) % 3600000) / 60000);
    println!("Секунды в этой минуте: {}", (((gps_time.MilliSeconds % 86400000) % 3600000) % 60000) / 1000);
    
    println!("Ожидаемые GPS миллисекунды: {} мс", expected_ms);
    println!("Это соответствует дню недели: {}", expected_ms / 86400000);
    println!("Часы в этом дне: {}", (expected_ms % 86400000) / 3600000);
    
    let day_names = ["воскресенье", "понедельник", "вторник", "среда", "четверг", "пятница", "суббота"];
    println!("Получили день: {}", day_names[(gps_time.MilliSeconds / 86400000) as usize]);
    println!("Ожидаем день: {}", day_names[(expected_ms / 86400000) as usize]);
    
    // Дополнительная проверка: правильный расчёт вручную
    println!("\n=== РУЧНОЙ РАСЧЁТ ДЛЯ ПРОВЕРКИ ===");
    
    // Дни от 6 января 1980 до 5 июня 2025
    let years = 2025 - 1980; // 45 лет
    let leap_years = (1980..=2024).filter(|&y| is_leap_year(y)).count(); // високосные годы
    let normal_years = years - leap_years;
    let total_days_manual = normal_years * 365 + leap_years * 366;
    
    // Дни с 1 января до 5 июня 2025 = 31+28+31+30+31+5 = 156 дней (2025 не високосный)
    let days_in_2025 = 31 + 28 + 31 + 30 + 31 + 5;
    
    // Дни с 6 января до конца 1980 года = 366 - 6 = 360 дней (1980 високосный)
    let days_in_1980 = 366 - 6;
    
    let manual_total = days_in_1980 + (years - 1) * 365 + (leap_years - 1) * 1 + days_in_2025;
    println!("Ручной расчёт общих дней: {}", manual_total);
    println!("Ручной расчёт GPS недель: {}", manual_total / 7);
    println!("Ручной день недели: {}", day_names[(manual_total % 7) as usize]);
}

fn is_leap_year(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Запускаем тест конвертации времени перед основной программой
    test_utc_to_gps_conversion();
    println!("\n");
    
    println!("================================================================================");
    println!("                          IF SIGNAL GENERATION");
    println!("================================================================================");
    
    // Обработка аргументов командной строки
    let args: Vec<String> = env::args().collect();
    let config_file = if args.len() > 1 {
        if args[1] == "--help" {
            print_help(&args[0]);
            return Ok(());
        }
        &args[1]
    } else {
        DEFAULT_CONFIG
    };

    // Проверка существования файла конфигурации
    if !Path::new(config_file).exists() {
        eprintln!("[ERROR]\tConfiguration file not found: {}", config_file);
        eprintln!("[INFO]\tCreating example configuration file...");
        create_example_config(config_file)?;
        println!("[INFO]\tExample configuration created: {}", config_file);
        println!("[INFO]\tPlease edit the configuration and run again.");
        return Ok(());
    }

    println!("[INFO]\tLoading JSON configuration: {}", config_file);
    
    // ============= ДЕТАЛЬНЫЕ ИЗМЕРЕНИЯ ПРОИЗВОДИТЕЛЬНОСТИ =============
    let total_start = Instant::now();
    println!("[TIMING]\tStarting performance measurement...");
    
    // ЭТАП 1: Создание компонентов
    let init_start = Instant::now();
    let mut if_data_gen = IFDataGen::new();
    let init_duration = init_start.elapsed();
    println!("[TIMING]\tComponent initialization: {:.3}s", init_duration.as_secs_f64());
    
    // ЭТАП 2: Загрузка конфигурации
    let config_start = Instant::now();
    match if_data_gen.load_config(config_file) {
        Ok(_) => {
            let config_duration = config_start.elapsed();
            println!("[TIMING]\tConfiguration loading: {:.3}s", config_duration.as_secs_f64());
            println!("[INFO]\tConfiguration loaded successfully");
        }
        Err(e) => {
            eprintln!("[ERROR]\tFailed to load configuration: {}", e);
            return Err(e);
        }
    }
    
    // ЭТАП 3: Инициализация системы (включая парсинг RINEX)
    let system_init_start = Instant::now();
    if_data_gen.initialize()?;
    let system_init_duration = system_init_start.elapsed();
    println!("[TIMING]\tSystem initialization (RINEX parsing): {:.3}s", system_init_duration.as_secs_f64());
    println!("[INFO]\tSystem initialized");
    
    // ЭТАП 4: Генерация данных (основная работа)
    println!("[INFO]\tStarting IF data generation...");
    let generation_start = Instant::now();
    let result = if_data_gen.generate_data();
    let generation_duration = generation_start.elapsed();
    
    let total_duration = total_start.elapsed();
    
    // ============= АНАЛИЗ ВРЕМЕНИ ПО ЭТАПАМ =============
    println!("================================================================================");
    println!("                          DETAILED TIMING BREAKDOWN");
    println!("================================================================================");
    println!("[TIMING]\tComponent init:     {:.3}s ({:.1}%)", 
        init_duration.as_secs_f64(),
        init_duration.as_secs_f64() / total_duration.as_secs_f64() * 100.0);
    println!("[TIMING]\tConfig loading:     {:.3}s ({:.1}%)", 
        config_start.elapsed().as_secs_f64(),
        config_start.elapsed().as_secs_f64() / total_duration.as_secs_f64() * 100.0);
    println!("[TIMING]\tSystem init/RINEX:  {:.3}s ({:.1}%)", 
        system_init_duration.as_secs_f64(),
        system_init_duration.as_secs_f64() / total_duration.as_secs_f64() * 100.0);
    println!("[TIMING]\tSignal generation:  {:.3}s ({:.1}%)", 
        generation_duration.as_secs_f64(),
        generation_duration.as_secs_f64() / total_duration.as_secs_f64() * 100.0);
    println!("[TIMING]\tTotal time:         {:.3}s (100.0%)", total_duration.as_secs_f64());
    
    let duration = total_duration;
    
    // Обработка результатов
    match result {
        Ok(stats) => {
            println!("================================================================================");
            println!("                          GENERATION COMPLETED");
            println!("================================================================================");
            println!("[INFO]\tTotal samples: {}", stats.total_samples);
            println!("[INFO]\tTotal time: {:.2} seconds", duration.as_secs_f64());
            println!("[INFO]\tGeneration rate: {:.2} MS/s", 
                stats.total_samples as f64 / duration.as_secs_f64() / 1e6);
            
            if let Some(file_size) = stats.file_size_mb {
                println!("[INFO]\tOutput file size: {:.2} MB", file_size);
                println!("[INFO]\tData rate: {:.2} MB/s", 
                    file_size / duration.as_secs_f64());
            }
            
            if stats.clipped_samples > 0 {
                let clip_rate = stats.clipped_samples as f64 / stats.total_samples as f64 * 100.0;
                println!("[INFO]\tClipped samples: {} ({:.4}%)", 
                    stats.clipped_samples, clip_rate);
                
                if clip_rate > 5.0 {
                    println!("[WARNING]\tHigh clipping rate! Consider reducing initPower in config.");
                }
            }
            
            println!("================================================================================");
        }
        Err(e) => {
            eprintln!("[ERROR]\tGeneration failed: {}", e);
            eprintln!("[INFO]\tTime before failure: {:.2} seconds", duration.as_secs_f64());
            return Err(e);
        }
    }
    
    Ok(())
}

fn print_help(program_name: &str) {
    println!("GNSS IF Data Generator - Rust Implementation");
    println!();
    println!("Usage: {} [OPTIONS] [CONFIG_FILE]", program_name);
    println!();
    println!("Arguments:");
    println!("  CONFIG_FILE    JSON configuration file path (default: config.json)");
    println!();
    println!("Options:");
    println!("  --help         Show this help message");
    println!();
    println!("Examples:");
    println!("  {}                          # Use default config.json", program_name);
    println!("  {} my_config.json          # Use custom configuration", program_name);
    println!("  {} presets/GPS_L1_only.json # Use preset configuration", program_name);
    println!();
    println!("Configuration file format:");
    println!("  The JSON configuration should specify time, trajectory, ephemeris,");
    println!("  output settings, and power parameters. See example configurations");
    println!("  in the presets/ directory.");
}

fn create_example_config(filename: &str) -> Result<(), Box<dyn std::error::Error>> {
    use std::fs::File;
    use std::io::Write;
    
    let example_config = r#"{
    "version": 1.0,
    "description": "Example GNSS IF Data Generation Configuration",
    "time": {
        "type": "UTC",
        "year": 2025,
        "month": 1,
        "day": 1,
        "hour": 12,
        "minute": 0,
        "second": 0
    },
    "trajectory": {
        "name": "Static position test",
        "initPosition": {
            "type": "LLA",
            "format": "d",
            "longitude": 0.0,
            "latitude": 0.0,
            "altitude": 100.0
        },
        "initVelocity": {
            "type": "SCU",
            "speed": 0.0,
            "course": 0.0
        },
        "trajectoryList": [
            {
                "type": "Const",
                "time": 10.0
            }
        ]
    },
    "ephemeris": {
        "type": "RINEX",
        "name": "ephemeris.rnx"
    },
    "output": {
        "type": "IFdata",
        "format": "IQ8",
        "sampleFreq": 5.0,
        "centerFreq": 1575.42,
        "name": "output.dat",
        "config": {
            "elevationMask": 5
        },
        "systemSelect": [
            {
                "system": "GPS",
                "signal": "L1CA",
                "enable": true
            }
        ]
    },
    "power": {
        "noiseFloor": -174,
        "initPower": {
            "unit": "dBHz",
            "value": 45
        },
        "elevationAdjust": true
    }
}"#;
    
    let mut file = File::create(filename)?;
    file.write_all(example_config.as_bytes())?;
    Ok(())
}