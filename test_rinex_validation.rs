use gnss_rust::json_interpreter::{CNavData, read_nav_file_limited};
use gnss_rust::navdata::{CNavData as CNavDataOrig};
use gnss_rust::types::{GnssTime, GlonassTime, GpsEphemeris, GlonassEphemeris, GnssSystem};
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== RINEX Data Validation Test ===");
    println!();
    
    let rinex_file = "/mnt/c/msys64/home/Daniil/gnss_rust/Rinex_Data/rinex_v3_20251560000.rnx";
    
    if !Path::new(rinex_file).exists() {
        eprintln!("ERROR: RINEX file not found: {}", rinex_file);
        return Err("RINEX file not found".into());
    }
    
    println!("[INFO] Validating RINEX data structures in: {}", rinex_file);
    println!("[INFO] Reading limited sample from each GNSS system for validation");
    println!();
    
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, rinex_file, 3); // Читаем по 3 эфемериды от каждой системы
    
    // Проверяем GPS эфемериды
    if let Some(first_gps) = nav_data.get_first_gps_ephemeris() {
        println!("=== GPS Ephemeris Validation (First Entry) ===");
        println!("SVID: {}", first_gps.svid);
        println!("Week: {}", first_gps.week);
        println!("TOE: {} sec", first_gps.toe);
        println!("TOC: {} sec", first_gps.toc);
        println!("sqrt(A): {:.6} m^1/2", first_gps.sqrtA);
        println!("Semi-major axis: {:.3} km", first_gps.axis / 1000.0);
        println!("Eccentricity: {:.9}", first_gps.ecc);
        println!("Inclination (i0): {:.9} rad ({:.3}°)", first_gps.i0, first_gps.i0.to_degrees());
        println!("RAAN (Omega0): {:.9} rad ({:.3}°)", first_gps.omega0, first_gps.omega0.to_degrees());
        println!("Arg of Perigee (w): {:.9} rad ({:.3}°)", first_gps.w, first_gps.w.to_degrees());
        println!("Mean Anomaly (M0): {:.9} rad ({:.3}°)", first_gps.M0, first_gps.M0.to_degrees());
        println!("Mean Motion Diff (Δn): {:.12} rad/s", first_gps.delta_n);
        println!("RAAN Rate (Ω̇): {:.12} rad/s", first_gps.omega_dot);
        println!("Inclination Rate (İ): {:.12} rad/s", first_gps.idot);
        println!();
        
        // Корректировочные коэффициенты
        println!("=== Correction Coefficients ===");
        println!("Crs: {:.6} m", first_gps.crs);
        println!("Crc: {:.6} m", first_gps.crc);
        println!("Cus: {:.9} rad", first_gps.cus);
        println!("Cuc: {:.9} rad", first_gps.cuc);
        println!("Cis: {:.9} rad", first_gps.cis);
        println!("Cic: {:.9} rad", first_gps.cic);
        println!();
        
        // Часовые поправки
        println!("=== Clock Corrections ===");
        println!("af0: {:.12e} s", first_gps.af0);
        println!("af1: {:.12e} s/s", first_gps.af1);
        println!("af2: {:.12e} s/s²", first_gps.af2);
        println!("TGD: {:.12e} s", first_gps.tgd);
        println!();
        
        // Статусные параметры
        println!("=== Status Parameters ===");
        println!("IODE: {}", first_gps.iode);
        println!("IODC: {}", first_gps.iodc);
        println!("URA: {}", first_gps.ura);
        println!("Health: {}", first_gps.health);
        println!("Valid: {}", first_gps.valid);
        println!("Flag: {}", first_gps.flag);
        println!();
        
        // Проверим физические ограничения
        let mut errors = Vec::new();
        
        // Проверка орбитальных параметров
        if first_gps.sqrtA < 5000.0 || first_gps.sqrtA > 6500.0 {
            errors.push(format!("Неправильная sqrt(A): {:.1} (должна быть 5000-6500)", first_gps.sqrtA));
        }
        
        if first_gps.ecc < 0.0 || first_gps.ecc > 0.1 {
            errors.push(format!("Неправильная эксцентричность: {:.6} (должна быть 0-0.1)", first_gps.ecc));
        }
        
        let inclination_deg = first_gps.i0.to_degrees();
        if inclination_deg < 50.0 || inclination_deg > 60.0 {
            errors.push(format!("Неправильная наклонность: {:.1}° (должна быть 50-60°)", inclination_deg));
        }
        
        // Проверка часовых поправок
        if first_gps.af0.abs() > 1e-3 {
            errors.push(format!("Слишком большая af0: {:.3e} (должна быть < 1e-3)", first_gps.af0));
        }
        
        if errors.is_empty() {
            println!("✅ GPS ephemeris data looks physically reasonable");
        } else {
            println!("❌ GPS ephemeris validation errors:");
            for error in &errors {
                println!("   - {}", error);
            }
        }
    }
    
    println!();
    println!("{}", "=".repeat(80));
    
    // Тестируем функцию выбора эфемерид по времени
    println!("=== Testing Ephemeris Selection by Time ===");
    
    // Создаем тестовое время - 5 июня 2025, 12:00:00 GPS
    let test_time = GnssTime {
        Week: 2316,  // GPS week для 5 июня 2025
        MilliSeconds: 43200000,  // 12:00:00 в миллисекундах от начала недели
    };
    
    println!("Test time: Week {}, Seconds {:.0}", test_time.Week, test_time.MilliSeconds as f64 / 1000.0);
    
    // Тестируем поиск GPS эфемерид
    let mut test_eph = GpsEphemeris::default();
    let result = nav_data.get_ephemeris_by_time(GnssSystem::GpsSystem, 1, test_time, &mut test_eph);
    
    match result {
        0 => {
            println!("✅ GPS ephemeris found for SVID 1 at test time");
            println!("   Selected ephemeris TOE: {} sec", test_eph.toe);
            println!("   Time difference: {:.0} sec", (test_time.MilliSeconds / 1000) as f64 - test_eph.toe as f64);
        },
        1 => println!("❌ No GPS ephemeris found for SVID 1 at test time"),
        _ => println!("❌ Error in ephemeris search for SVID 1"),
    }
    
    // Тестируем поиск для другого спутника  
    let mut test_eph2 = GpsEphemeris::default();
    let result2 = nav_data.get_ephemeris_by_time(GnssSystem::GpsSystem, 2, test_time, &mut test_eph2);
    
    match result2 {
        0 => {
            println!("✅ GPS ephemeris found for SVID 2 at test time");
            println!("   Selected ephemeris TOE: {} sec", test_eph2.toe);
            println!("   Time difference: {:.0} sec", (test_time.MilliSeconds / 1000) as f64 - test_eph2.toe as f64);
        },
        1 => println!("❌ No GPS ephemeris found for SVID 2 at test time"),
        _ => println!("❌ Error in ephemeris search for SVID 2"),
    }
    
    // Тестируем поиск GLONASS эфемерид
    let mut test_glo_eph = GlonassEphemeris::default();
    let glonass_time = GlonassTime {
        Week: 2316,
        MilliSeconds: 43200000,
    };
    
    let glo_result = nav_data.get_glonass_ephemeris_by_time(1, glonass_time, &mut test_glo_eph);
    
    match glo_result {
        0 => {
            println!("✅ GLONASS ephemeris found for slot 1 at test time");
            println!("   Selected GLONASS tk: {} ({}h {}m)", test_glo_eph.tk, 
                     (test_glo_eph.tk >> 7) & 0x1F, 
                     (test_glo_eph.tk >> 1) & 0x3F);
        },
        1 => println!("❌ No GLONASS ephemeris found for slot 1 at test time"),
        _ => println!("❌ Error in GLONASS ephemeris search for slot 1"),
    }
    
    println!();
    println!("=== Summary ===");
    println!("Total GPS ephemeris loaded: {}", nav_data.get_gps_ephemeris_count());
    println!("Total GLONASS ephemeris loaded: {}", nav_data.get_glonass_ephemeris_count());
    println!("Total BeiDou ephemeris loaded: {}", nav_data.get_beidou_ephemeris_count());
    println!("Total Galileo ephemeris loaded: {}", nav_data.get_galileo_ephemeris_count());
    
    // Проверяем ионосферные параметры
    if nav_data.has_gps_iono() {
        println!("✅ GPS ionospheric parameters are present");
    } else {
        println!("❌ GPS ionospheric parameters are missing");
    }
    
    Ok(())
}