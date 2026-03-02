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

use gnss_rust::*;
use std::env;
use std::path::Path;
use std::time::Instant;

const DEFAULT_CONFIG: &str = "config.json";

use gnss_rust::logutil::{is_quiet, is_verbose, set_level, LogLevel};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Обработка аргументов командной строки (до любых логов)
    let mut args_iter = env::args();
    let program_name = args_iter.next().unwrap_or_else(|| "gnss_rust".into());
    let mut level_override: Option<LogLevel> = None;
    let mut config_file: Option<String> = None;

    for arg in args_iter {
        match arg.as_str() {
            "-h" | "--help" => {
                print_help(&program_name);
                return Ok(());
            }
            "-v" | "--verbose" => level_override = Some(LogLevel::Verbose),
            "-q" | "--quiet" => level_override = Some(LogLevel::Quiet),
            _ if arg.starts_with('-') => {
                eprintln!("[WARN]\tUnknown option: {}", arg);
            }
            _ => {
                if config_file.is_none() {
                    config_file = Some(arg);
                } else {
                    eprintln!("[WARN]\tIgnoring extra argument: {}", arg);
                }
            }
        }
    }

    if let Some(lvl) = level_override {
        set_level(lvl);
    }

    if is_verbose() {
        println!(
            "================================================================================"
        );
        println!("                          IF SIGNAL GENERATION ");
        println!(
            "================================================================================"
        );
    }

    let config_file = config_file.as_deref().unwrap_or(DEFAULT_CONFIG);

    // Проверка существования файла конфигурации
    if !Path::new(config_file).exists() {
        eprintln!("[ERROR]\tConfiguration file not found: {}", config_file);
        if is_verbose() {
            eprintln!("[INFO]\tCreating example configuration file...");
        }
        create_example_config(config_file)?;
        if is_verbose() {
            println!("[INFO]\tExample configuration created: {}", config_file);
            println!("[INFO]\tPlease edit the configuration and run again.");
        }
        return Ok(());
    }

    if is_verbose() {
        println!("[INFO]\tLoading JSON configuration: {}", config_file);
    }

    let total_start = Instant::now();
    if is_verbose() {
        println!("[TIMING]\tStarting performance measurement...");
    }

    // ЭТАП 1: Создание компонентов
    let init_start = Instant::now();
    let mut if_data_gen = IFDataGen::new();
    let init_duration = init_start.elapsed();
    if is_verbose() {
        println!(
            "[TIMING]\tComponent initialization: {:.3}s",
            init_duration.as_secs_f64()
        );
    }

    // ЭТАП 2: Загрузка конфигурации
    let config_start = Instant::now();
    match if_data_gen.load_config(config_file) {
        Ok(_) => {
            let config_duration = config_start.elapsed();
            if is_verbose() {
                println!(
                    "[TIMING]\tConfiguration loading: {:.3}s",
                    config_duration.as_secs_f64()
                );
                println!("[INFO]\tConfiguration loaded successfully");
            }
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
    if is_verbose() {
        println!(
            "[TIMING]\tSystem initialization (RINEX parsing): {:.3}s",
            system_init_duration.as_secs_f64()
        );
        println!("[INFO]\tSystem initialized");
    }

    // ЭТАП 4: Генерация данных (основная работа)
    if is_verbose() {
        println!("[INFO]\tStarting IF data generation...");
    }
    let generation_start = Instant::now();
    let result = if_data_gen.generate_data();
    let generation_duration = generation_start.elapsed();

    let total_duration = total_start.elapsed();

    // ============= АНАЛИЗ ВРЕМЕНИ ПО ЭТАПАМ =============
    if is_verbose() {
        println!(
            "================================================================================"
        );
        println!("                          DETAILED TIMING BREAKDOWN");
        println!(
            "================================================================================"
        );
        println!(
            "[TIMING]\tComponent init:     {:.3}s ({:.1}%)",
            init_duration.as_secs_f64(),
            init_duration.as_secs_f64() / total_duration.as_secs_f64() * 100.0
        );
        println!(
            "[TIMING]\tConfig loading:     {:.3}s ({:.1}%)",
            config_start.elapsed().as_secs_f64(),
            config_start.elapsed().as_secs_f64() / total_duration.as_secs_f64() * 100.0
        );
        println!(
            "[TIMING]\tSystem init/RINEX:  {:.3}s ({:.1}%)",
            system_init_duration.as_secs_f64(),
            system_init_duration.as_secs_f64() / total_duration.as_secs_f64() * 100.0
        );
        println!(
            "[TIMING]\tSignal generation:  {:.3}s ({:.1}%)",
            generation_duration.as_secs_f64(),
            generation_duration.as_secs_f64() / total_duration.as_secs_f64() * 100.0
        );
        println!(
            "[TIMING]\tTotal time:         {:.3}s (100.0%)",
            total_duration.as_secs_f64()
        );
    }

    let duration = total_duration;

    // Обработка результатов
    match result {
        Ok(stats) => {
            if !is_quiet() {
                println!("================================================================================");
                println!("                          GENERATION COMPLETED");
                println!("================================================================================");
                println!("[INFO]\tTotal samples: {}", stats.total_samples);
                println!("[INFO]\tTotal time: {:.2} seconds", duration.as_secs_f64());
                println!(
                    "[INFO]\tGeneration rate: {:.2} MS/s",
                    stats.total_samples as f64 / duration.as_secs_f64() / 1e6
                );

                if let Some(file_size) = stats.file_size_mb {
                    println!("[INFO]\tOutput file size: {:.2} MB", file_size);
                    println!(
                        "[INFO]\tData rate: {:.2} MB/s",
                        file_size / duration.as_secs_f64()
                    );
                }

                if stats.clipped_samples > 0 {
                    let clip_rate =
                        stats.clipped_samples as f64 / stats.total_samples as f64 * 100.0;
                    println!(
                        "[INFO]\tClipped samples: {} ({:.4}%)",
                        stats.clipped_samples, clip_rate
                    );

                    if clip_rate > 5.0 {
                        println!(
                            "[WARNING]\tHigh clipping rate! Consider reducing initPower in config."
                        );
                    }
                }

                println!("================================================================================");
            }
        }
        Err(e) => {
            eprintln!("[ERROR]\tGeneration failed: {}", e);
            eprintln!(
                "[INFO]\tTime before failure: {:.2} seconds",
                duration.as_secs_f64()
            );
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
    println!("  -h, --help     Show this help message");
    println!("  -v, --verbose  Verbose logs (overrides GNSS_VERBOSE)");
    println!("  -q, --quiet    Quiet mode (errors only)");
    println!();
    println!("Examples:");
    println!(
        "  {}                          # Use default config.json",
        program_name
    );
    println!(
        "  {} my_config.json          # Use custom configuration",
        program_name
    );
    println!(
        "  {} presets/GPS_L1_only.json # Use preset configuration",
        program_name
    );
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
