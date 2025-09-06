//! Тест для проверки генерации навигационных данных
//! Создает минимальный GPS L1CA сигнал и проверяет навигационные данные

use gnss_rust::types::*;
use gnss_rust::nav_data::NavData;
use gnss_rust::lnavbit::LNavBit;

fn main() {
    println!("=== Тест генерации навигационных данных GPS L1CA ===");
    
    // Создаём минимальные эфемериды GPS
    let mut gps_eph = GpsEphemeris::default();
    gps_eph.svid = 1; // GPS PRN 1
    gps_eph.af0 = 0.0;
    gps_eph.af1 = 0.0;
    gps_eph.af2 = 0.0;
    gps_eph.toe = 345600; // seconds in week
    gps_eph.toc = 345600; // seconds in week
    gps_eph.week = 2369; // GPS week (June 2025)
    gps_eph.iode = 1;
    gps_eph.iodc = 1;
    
    // Создаём LNavBit для GPS L1CA
    let mut lnav = LNavBit::new();
    
    // Устанавливаем эфемериды
    let result = lnav.set_ephemeris(1, &gps_eph);
    println!("Установка эфемерид GPS SV01: {}", result);
    
    // Создаём навигационные данные через enum
    let mut nav_data = NavData::LNav(lnav);
    
    // Тестовое время: 2025-06-05 10:05:30 UTC
    let test_time = GnssTime {
        Week: 2369,
        MilliSeconds: 345630000, // 10:05:30 in milliseconds
        SubMilliSeconds: 0.0
    };
    
    // Получаем навигационные биты
    let mut nav_bits = [0i32; 4096];
    let bits_generated = nav_data.get_frame_data(test_time, 1, 0, &mut nav_bits);
    
    println!("Сгенерированных навигационных битов: {}", bits_generated);
    
    // Проверяем первые 50 битов
    let non_zero_count = nav_bits[..300].iter().filter(|&&x| x != 0).count();
    println!("Ненулевых битов из первых 300: {} ({:.1}%)", 
             non_zero_count, (non_zero_count as f32 / 300.0) * 100.0);
    
    // Показываем первые 20 битов
    print!("Первые 20 битов: ");
    for i in 0..20 {
        print!("{}", if nav_bits[i] != 0 { "1" } else { "0" });
    }
    println!();
    
    // Показываем биты из разных субфреймов
    print!("Биты 50-70:      ");
    for i in 50..70 {
        print!("{}", if nav_bits[i] != 0 { "1" } else { "0" });
    }
    println!();
    
    print!("Биты 100-120:    ");
    for i in 100..120 {
        print!("{}", if nav_bits[i] != 0 { "1" } else { "0" });
    }
    println!();
    
    if non_zero_count == 0 {
        println!("❌ ПРОБЛЕМА: Навигационные данные не генерируются!");
        println!("   Все биты равны нулю - это объясняет узкий спектр");
    } else {
        println!("✅ Навигационные данные генерируются правильно");
        println!("   Должна быть модуляция данных в сигнале");
    }
}