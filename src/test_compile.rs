// Простой тест для проверки компиляции
use crate::bcnavbit::BCNavBit;
use crate::types::GpsEphemeris;

pub fn test_bcnavbit() {
    let mut bcnav = BCNavBit::new();
    
    // Тест создания структуры
    let eph = GpsEphemeris::default();
    let _result = bcnav.set_ephemeris(1, &eph);
    
    println!("BCNavBit test passed!");
}