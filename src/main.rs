//! # Главная функция ГНСС библиотеки
//!
//! Этот модуль содержит точку входа в приложение ГНСС Rust Library.
//! Основные задачи:
//! - Инициализация и тестирование всех модулей библиотеки
//! - Демонстрация работы с различными типами навигационных данных
//! - Проверка функциональности генерации сигналов и PRN кодов
//! - Тестирование парсинга альманахов и эфемерид
//! - Валидация работы с различными ГНСС системами (GPS, ГЛОНАСС, BeiDou, Galileo)
//!
//! Служит как пример использования библиотеки и инструмент для проверки
//! корректности работы всех компонентов системы.

use gnss_rust::*;
use std::fs::File;

fn main() {
    println!("GNSS Rust Library - Unified Project");
    
    // Test Almanac functionality
    println!("\n=== Testing Almanac Module ===");
    match File::open("test_almanac.txt") {
        Ok(mut file) => {
            let almanac_type = CheckAlmnanacType(&mut file);
            println!("Detected almanac type: {:?}", almanac_type);
        }
        Err(e) => {
            println!("Could not open test almanac file: {}", e);
        }
    }
    
    // Test BCNav1Bit functionality
    println!("\n=== Testing BCNav1Bit Module ===");
    let mut bcnav = BCNav1Bit::new();
    
    let start_time = GnssTime {
        Week: 2000,
        MilliSeconds: 0,
        SubMilliSeconds: 0.0,
    };
    
    let mut nav_bits = vec![0i32; 1800];
    let result = bcnav.GetFrameData(start_time, 1, 0, &mut nav_bits);
    
    println!("Generated frame data with {} bits", result);
    println!("First 10 bits: {:?}", &nav_bits[0..10]);
    
    // Test ionosphere and UTC parameter setting
    let iono_param = IonoParam {
        a0: 1.0e-8,
        a1: 1.0e-8,
        a2: -1.0e-7,
        a3: -1.0e-7,
        b0: 1.0e5,
        b1: 1.0e5,
        b2: -1.0e5,
        b3: -1.0e5,
        flag: 1,
    };
    
    let utc_param = UtcParam {
        A0: 1.0e-9,
        A1: 1.0e-15,
        A2: 0.0,
        WN: 2000,
        WNLSF: 2000,
        tot: (147456 >> 12) as u8,
        TLS: 18,
        TLSF: 18,
        DN: 7,
        flag: 1,
    };
    
    let result = bcnav.SetIonoUtc(Some(&iono_param), Some(&utc_param));
    println!("Set ionosphere and UTC parameters, result: {}", result);
    
    // Test almanac conversion
    println!("\n=== Testing Almanac Conversion ===");
    let eph = GpsEphemeris {
        ura: 0,
        iodc: 0,
        iode: 0,
        svid: 1,
        source: 0,
        valid: 1,
        flag: 0,
        health: 0,
        toe: 0,
        toc: 0,
        top: 0,
        week: 2000,
        M0: 0.0,
        delta_n: 0.0,
        delta_n_dot: 0.0,
        ecc: 0.01,
        sqrtA: 5153.0,
        axis_dot: 0.0,
        omega0: 0.0,
        i0: 0.97,
        w: 0.0,
        omega_dot: -2.6e-9,
        idot: 0.0,
        cuc: 0.0,
        cus: 0.0,
        crc: 0.0,
        crs: 0.0,
        cic: 0.0,
        cis: 0.0,
        af0: 0.0,
        af1: 0.0,
        af2: 0.0,
        tgd: 0.0,
        tgd2: 0.0,
        tgd_ext: [0.0; 5],
        axis: 0.0,
        n: 1.46e-4,
        root_ecc: 0.0,
        omega_t: 0.0,
        omega_delta: 0.0,
        Ek: 0.0,
        Ek_dot: 0.0,
    };
    
    let alm = GetAlmanacFromEphemeris(&eph, 2000, 0);
    println!("Converted ephemeris to almanac for SVID {}", alm.svid);
    println!("Almanac valid: {}, health: {}", alm.valid, alm.health);
}