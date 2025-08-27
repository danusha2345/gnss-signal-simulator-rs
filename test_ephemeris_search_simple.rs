use gnss_rust::json_interpreter::{CNavData, read_nav_file_limited};
use gnss_rust::ifdatagen::NavData;
use gnss_rust::types::{GnssTime, GnssSystem};
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
    read_nav_file_limited(&mut nav_data_parser, rinex_file, 5);
    
    // Создаем NavData для поиска эфемерид
    let mut nav_data = NavData::new();
    
    // Копируем GPS данные
    for gps_eph in nav_data_parser.get_gps_ephemeris().iter().take(5) {
        nav_data.add_gps_ephemeris(gps_eph);
    }
    
    println!("Loaded GPS ephemeris count: {}", nav_data_parser.get_gps_ephemeris_count());
    println!();
    
    // Тест: Поиск GPS эфемерид по точному времени TOE
    println!("=== Test GPS Ephemeris Search ===");
    if let Some(first_gps) = nav_data_parser.get_first_gps_ephemeris() {
        let exact_time = GnssTime {
            Week: first_gps.week,
            MilliSeconds: (first_gps.toe as i32) * 1000,
            SubMilliSeconds: 0.0,
        };
        
        println!("Looking for GPS SVID {} at exact TOE time:", first_gps.svid);
        println!("  Target time: Week {}, TOE {} sec", exact_time.Week, exact_time.MilliSeconds / 1000);
        println!("  Expected: SVID {}, TOE {} sec", first_gps.svid, first_gps.toe);
        
        if let Some(found_eph) = nav_data.find_ephemeris(GnssSystem::GpsSystem, exact_time, first_gps.svid as i32, 0, 0) {
            println!("  ✅ Found ephemeris!");
            println!("  Found SVID: {}, TOE: {} sec", found_eph.svid, found_eph.toe);
            println!("  Time difference: {} sec", (found_eph.toe - exact_time.MilliSeconds / 1000).abs());
            
            if found_eph.svid == first_gps.svid && found_eph.toe == first_gps.toe {
                println!("  ✅ PERFECT MATCH - Algorithm working correctly!");
            } else {
                println!("  ⚠️  Different ephemeris returned - algorithm may have issues");
            }
        } else {
            println!("  ❌ No ephemeris found - algorithm has issues");
        }
    }
    println!();
    
    // Тест с небольшим временным смещением  
    println!("=== Test GPS Ephemeris Search with Small Offset ===");
    if let Some(first_gps) = nav_data_parser.get_first_gps_ephemeris() {
        let offset_time = GnssTime {
            Week: first_gps.week,
            MilliSeconds: (first_gps.toe as i32 + 1800) * 1000, // +30 минут
        };
        
        println!("Looking for GPS SVID {} at TOE + 30 minutes:", first_gps.svid);
        println!("  Target time: Week {}, Seconds {} sec", offset_time.Week, offset_time.MilliSeconds / 1000);
        
        if let Some(found_eph) = nav_data.find_ephemeris(GnssSystem::GpsSystem, offset_time, first_gps.svid as i32, 0, 0) {
            println!("  ✅ Found ephemeris!");
            println!("  Found SVID: {}, TOE: {} sec", found_eph.svid, found_eph.toe);
            println!("  Time difference: {} sec", (found_eph.toe - offset_time.MilliSeconds / 1000).abs());
        } else {
            println!("  ❌ No ephemeris found (30 minutes should be within 2-hour limit)");
        }
    }
    println!();
    
    // Тест со слишком большим смещением (должен не найти)
    println!("=== Test GPS Ephemeris Search with Large Offset (Should Fail) ===");
    if let Some(first_gps) = nav_data_parser.get_first_gps_ephemeris() {
        let large_offset_time = GnssTime {
            Week: first_gps.week,
            MilliSeconds: (first_gps.toe as i32 + 10800) * 1000, // +3 часа (больше лимита 2 часа)
        };
        
        println!("Looking for GPS SVID {} at TOE + 3 hours (should fail):", first_gps.svid);
        
        if let Some(_) = nav_data.find_ephemeris(GnssSystem::GpsSystem, large_offset_time, first_gps.svid as i32, 0, 0) {
            println!("  ⚠️  Unexpected: Found ephemeris (algorithm may have wrong time limits)");
        } else {
            println!("  ✅ Correctly rejected - time offset too large (>2 hours limit)");
        }
    }
    
    println!();
    println!("=== Ephemeris Search Test Complete ===");
    Ok(())
}