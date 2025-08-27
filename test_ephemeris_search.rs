use gnss_rust::json_interpreter::{CNavData, read_nav_file_limited};
use gnss_rust::ifdatagen::NavData;
use gnss_rust::types::{GnssTime, GlonassTime, GnssSystem};
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Test Ephemeris Search by Time ===");
    println!();
    
    let rinex_file = "/mnt/c/msys64/home/Daniil/gnss_rust/Rinex_Data/rinex_v3_20251560000.rnx";
    
    if !Path::new(rinex_file).exists() {
        eprintln!("ERROR: RINEX file not found: {}", rinex_file);
        return Err("RINEX file not found".into());
    }
    
    // Загружаем навигационные данные
    let mut nav_data_parser = CNavData::new();
    read_nav_file_limited(&mut nav_data_parser, rinex_file, 10);
    
    // Создаем NavData для поиска эфемерид
    let mut nav_data = NavData::new();
    
    // Копируем данные из парсера в NavData (упрощенно)
    for gps_eph in nav_data_parser.get_gps_ephemeris().iter().take(10) {
        nav_data.add_gps_ephemeris(gps_eph);
    }
    
    for glo_eph in nav_data_parser.get_glonass_ephemeris().iter().take(10) {
        nav_data.add_glonass_ephemeris(glo_eph);
    }
    
    println!("Loaded ephemeris data:");
    println!("GPS ephemeris count: {}", nav_data_parser.get_gps_ephemeris_count());
    println!("GLONASS ephemeris count: {}", nav_data_parser.get_glonass_ephemeris_count());
    println!();
    
    // Тест 1: Поиск GPS эфемерид по точному времени TOE
    println!("=== Test 1: GPS Ephemeris Search by Exact TOE Time ===");
    if let Some(first_gps) = nav_data_parser.get_first_gps_ephemeris() {
        let exact_time = GnssTime {
            Week: first_gps.week,
            MilliSeconds: (first_gps.toe as i32) * 1000,
        };
        
        println!("Looking for GPS SVID {} at exact TOE time:", first_gps.svid);
        println!("  Target time: Week {}, TOE {} sec", exact_time.Week, exact_time.MilliSeconds / 1000);
        
        if let Some(found_eph) = nav_data.find_ephemeris(GnssSystem::GpsSystem, exact_time, first_gps.svid as i32, 0, 0) {
            println!("  ✅ Found ephemeris!");
            println!("  Found SVID: {}, TOE: {} sec", found_eph.svid, found_eph.toe);
            println!("  Time difference: {} sec", (found_eph.toe - exact_time.MilliSeconds / 1000).abs());
        } else {
            println!("  ❌ No ephemeris found");
        }
    }
    println!();
    
    // Тест 2: Поиск GPS эфемерид с временным смещением
    println!("=== Test 2: GPS Ephemeris Search with Time Offset ===");
    if let Some(first_gps) = nav_data_parser.get_first_gps_ephemeris() {
        let offset_time = GnssTime {
            Week: first_gps.week,
            MilliSeconds: (first_gps.toe as i32 + 3600) * 1000, // +1 час
        };
        
        println!("Looking for GPS SVID {} at TOE + 1 hour:", first_gps.svid);
        println!("  Target time: Week {}, Seconds {} sec", offset_time.Week, offset_time.MilliSeconds / 1000);
        
        if let Some(found_eph) = nav_data.find_ephemeris(GnssSystem::GpsSystem, offset_time, first_gps.svid as i32, 0, 0) {
            println!("  ✅ Found ephemeris!");
            println!("  Found SVID: {}, TOE: {} sec", found_eph.svid, found_eph.toe);
            println!("  Time difference: {} sec", (found_eph.toe - offset_time.MilliSeconds / 1000).abs());
        } else {
            println!("  ❌ No ephemeris found (expected - 1 hour offset may be too large)");
        }
    }
    println!();
    
    // Тест 3: Поиск GPS эфемерид со слишком большим смещением (должен не найти)
    println!("=== Test 3: GPS Ephemeris Search with Large Time Offset (Should Fail) ===");
    if let Some(first_gps) = nav_data_parser.get_first_gps_ephemeris() {
        let large_offset_time = GnssTime {
            Week: first_gps.week,
            MilliSeconds: (first_gps.toe as i32 + 10800) * 1000, // +3 часа (больше лимита 2 часа)
        };
        
        println!("Looking for GPS SVID {} at TOE + 3 hours (should fail):", first_gps.svid);
        println!("  Target time: Week {}, Seconds {} sec", large_offset_time.Week, large_offset_time.MilliSeconds / 1000);
        
        if let Some(found_eph) = nav_data.find_ephemeris(GnssSystem::GpsSystem, large_offset_time, first_gps.svid as i32, 0, 0) {
            println!("  ⚠️  Unexpected: Found ephemeris!");
            println!("  Found SVID: {}, TOE: {} sec", found_eph.svid, found_eph.toe);
        } else {
            println!("  ✅ Correctly rejected - time offset too large (>2 hours limit)");
        }
    }
    println!();
    
    // Тест 4: Поиск GLONASS эфемерид
    println!("=== Test 4: GLONASS Ephemeris Search ===");
    if let Some(first_glo) = nav_data_parser.get_glonass_ephemeris().first() {
        let glo_time = GlonassTime {
            Week: 2316,  // Примерное время
            MilliSeconds: 43200000,  // 12:00:00
        };
        
        println!("Looking for GLONASS slot {}:", first_glo.slot);
        println!("  Target time: Week {}, MilliSeconds {}", glo_time.Week, glo_time.MilliSeconds);
        
        if let Some(found_eph) = nav_data.find_glo_ephemeris(glo_time, first_glo.slot as i32) {
            println!("  ✅ Found GLONASS ephemeris!");
            println!("  Found slot: {}, day: {}", found_eph.slot, found_eph.day);
        } else {
            println!("  ❌ No GLONASS ephemeris found");
        }
    }
    println!();
    
    // Тест 5: Поиск несуществующего спутника
    println!("=== Test 5: Search for Non-existent Satellite ===");
    let test_time = GnssTime {
        Week: 2369,
        MilliSeconds: 345600000,
    };
    
    println!("Looking for non-existent GPS SVID 99:");
    if let Some(_) = nav_data.find_ephemeris(GnssSystem::GpsSystem, test_time, 99, 0, 0) {
        println!("  ⚠️  Unexpected: Found ephemeris for non-existent satellite!");
    } else {
        println!("  ✅ Correctly returned None for non-existent satellite");
    }
    
    println!();
    println!("=== Ephemeris Search Test Complete ===");
    Ok(())
}