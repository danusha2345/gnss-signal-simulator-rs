use crate::coordinate::*;
use crate::types::*;

pub fn debug_satellite_positions(
    gps_eph: &[Option<GpsEphemeris>; 32],
    beidou_eph: &[Option<BeiDouEphemeris>; 64],
    galileo_eph: &[Option<GpsEphemeris>; 36],
    cur_time: &GnssTime,
    receiver_pos: &KinematicInfo,
) {
    println!("\n=== ОТЛАДКА ПОЗИЦИЙ СПУТНИКОВ ===");
    println!("Время: GPS Week {}, MS {}", cur_time.Week, cur_time.MilliSeconds);
    let transmit_time_full = (cur_time.Week as f64) * 604800.0 + (cur_time.MilliSeconds as f64) / 1000.0;
    let transmit_time_week = (cur_time.MilliSeconds as f64) / 1000.0;
    println!("transmit_time полное: {:.1} сек, секунды недели: {:.1}", transmit_time_full, transmit_time_week);
    println!("Позиция приёмника: ({:.1}, {:.1}, {:.1})", 
             receiver_pos.x, receiver_pos.y, receiver_pos.z);
    
    let receiver_lla = ecef_to_lla(receiver_pos);
    println!("Приёмник LLA: lat={:.6}°, lon={:.6}°, alt={:.1}m",
             receiver_lla.lat.to_degrees(), 
             receiver_lla.lon.to_degrees(),
             receiver_lla.alt);
    
    // Эталонные GPS спутники из C версии
    let c_version_gps = vec![1, 2, 7, 8, 9, 13, 14, 17, 22, 27, 30];
    let c_version_beidou = vec![19, 23, 27, 28, 37, 43];
    let c_version_galileo = vec![2, 7, 8, 15, 27, 29, 30, 34, 36];
    
    println!("\n--- GPS спутники (эталон из C: {:?}) ---", c_version_gps);
    
    // ИСПРАВЛЕНИЕ: используем только секунды недели, а не полное время
    let transmit_time_week_secs = (cur_time.MilliSeconds as f64) / 1000.0;
    let elevation_mask = 5.0_f64.to_radians();
    
    for svid in &c_version_gps {
        let idx = (svid - 1) as usize;
        if idx >= 32 { continue; }
        
        print!("GPS{:02}: ", svid);
        
        if let Some(eph) = &gps_eph[idx] {
            let mut sat_pos = KinematicInfo::default();
            let mut eph_mut = *eph;
            
            // DEBUG: GPS01 детальный анализ отключен для уменьшения вывода
            // if *svid == 1 {
            //     println!("  [GPS01 DEBUG] toe={}, week={}", eph.toe, eph.week);
            //     println!("  [GPS01 DEBUG] M0={:.6}, n={:.10e}", eph.M0, eph.n);
            //     println!("  [GPS01 DEBUG] sqrtA={:.6}, ecc={:.10e}", eph.sqrtA, eph.ecc);
            //     println!("  [GPS01 DEBUG] omega0={:.6}, i0={:.6}, w={:.6}", eph.omega0, eph.i0, eph.w);
            //     let delta_t = transmit_time_week - eph.toe as f64;
            //     println!("  [GPS01 DEBUG] delta_t={:.1} сек ({:.2} часов)", delta_t, delta_t/3600.0);
            // }
            
            // ИСПРАВЛЕНО: передаём только секунды недели, а не полное время
            if gps_sat_pos_speed_eph(GnssSystem::GpsSystem, transmit_time_week_secs, &mut eph_mut, &mut sat_pos, None) {
                let mut elevation = 0.0;
                let mut azimuth = 0.0;
                sat_el_az_from_positions(receiver_pos, &sat_pos, &mut elevation, &mut azimuth);
                
                let range = ((sat_pos.x - receiver_pos.x).powi(2) + 
                             (sat_pos.y - receiver_pos.y).powi(2) + 
                             (sat_pos.z - receiver_pos.z).powi(2)).sqrt();
                
                let visible = elevation >= elevation_mask;
                let status = if visible { "ВИДИМ ✓" } else { "НЕ ВИДИМ ✗" };
                
                println!("{} | El={:5.1}° Az={:5.1}° Range={:.0}km | toe={} health={}", 
                         status,
                         elevation.to_degrees(), 
                         azimuth.to_degrees(),
                         range / 1000.0,
                         eph.toe,
                         eph.health);
            } else {
                println!("Ошибка расчёта позиции");
            }
        } else {
            println!("Нет эфемерид");
        }
    }
    
    println!("\n--- BeiDou спутники (эталон из C: {:?}) ---", c_version_beidou);
    
    for svid in &c_version_beidou {
        let idx = (svid - 1) as usize;
        if idx >= 64 { continue; }
        
        print!("C{:02}: ", svid);
        
        if let Some(eph) = &beidou_eph[idx] {
            let mut sat_pos = KinematicInfo::default();
            // Конвертируем BeiDouEphemeris в GpsEphemeris для расчёта
            let mut gps_eph = eph.to_gps_ephemeris();
            
            // BeiDou использует BDT время - передаём только секунды недели
            // Эфемериды уже в GPS времени после конверсии в RINEX
            
            if gps_sat_pos_speed_eph(GnssSystem::BdsSystem, transmit_time_week_secs, &mut gps_eph, &mut sat_pos, None) {
                let mut elevation = 0.0;
                let mut azimuth = 0.0;
                sat_el_az_from_positions(receiver_pos, &sat_pos, &mut elevation, &mut azimuth);
                
                let range = ((sat_pos.x - receiver_pos.x).powi(2) + 
                             (sat_pos.y - receiver_pos.y).powi(2) + 
                             (sat_pos.z - receiver_pos.z).powi(2)).sqrt();
                
                let visible = elevation >= elevation_mask;
                let status = if visible { "ВИДИМ ✓" } else { "НЕ ВИДИМ ✗" };
                
                println!("{} | El={:5.1}° Az={:5.1}° Range={:.0}km | toe={} health={}", 
                         status,
                         elevation.to_degrees(), 
                         azimuth.to_degrees(),
                         range / 1000.0,
                         gps_eph.toe,
                         gps_eph.health);
            } else {
                println!("Ошибка расчёта позиции");
            }
        } else {
            println!("Нет эфемерид");
        }
    }
    
    println!("\n--- Galileo спутники (эталон из C: {:?}) ---", c_version_galileo);
    
    for svid in &c_version_galileo {
        let idx = (svid - 1) as usize;
        if idx >= 36 { continue; }
        
        print!("E{:02}: ", svid);
        
        if let Some(eph) = &galileo_eph[idx] {
            let mut sat_pos = KinematicInfo::default();
            let mut eph_mut = *eph;
            
            if gps_sat_pos_speed_eph(GnssSystem::GalileoSystem, transmit_time_week_secs, &mut eph_mut, &mut sat_pos, None) {
                let mut elevation = 0.0;
                let mut azimuth = 0.0;
                sat_el_az_from_positions(receiver_pos, &sat_pos, &mut elevation, &mut azimuth);
                
                let range = ((sat_pos.x - receiver_pos.x).powi(2) + 
                             (sat_pos.y - receiver_pos.y).powi(2) + 
                             (sat_pos.z - receiver_pos.z).powi(2)).sqrt();
                
                let visible = elevation >= elevation_mask;
                let status = if visible { "ВИДИМ ✓" } else { "НЕ ВИДИМ ✗" };
                
                println!("{} | El={:5.1}° Az={:5.1}° Range={:.0}km | toe={} health={}", 
                         status,
                         elevation.to_degrees(), 
                         azimuth.to_degrees(),
                         range / 1000.0,
                         eph.toe,
                         eph.health);
            } else {
                println!("Ошибка расчёта позиции");
            }
        } else {
            println!("Нет эфемерид");
        }
    }
    
    println!("\n=== КОНЕЦ ОТЛАДКИ ===\n");
}