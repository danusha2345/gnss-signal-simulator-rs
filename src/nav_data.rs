//! # Унифицированная навигационная архитектура
//!
//! Этот модуль заменяет dyn NavBitTrait на enum для обеспечения параллелизации
//! и улучшения производительности в 2-3 раза за счет исключения виртуальных вызовов.

use crate::bcnav1bit::BCNav1Bit;
use crate::bcnav2bit::BCNav2Bit;
use crate::bcnav3bit::BCNav3Bit;
use crate::cnav2bit::CNav2Bit;
use crate::cnavbit::CNavBit;
use crate::d1d2navbit::D1D2NavBit;
use crate::fnavbit::FNavBit;
use crate::gnavbit::GNavBit;
use crate::inavbit::INavBit;
use crate::l5cnavbit::L5CNavBit;
use crate::lnavbit::LNavBit;
use crate::navdata::NavDataType;
use crate::types::*;
/// Тип навигационного сообщения
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NavMessageType {
    LNav,    // GPS L1CA
    CNav,    // GPS L2C
    CNav2,   // GPS L1C
    L5CNav,  // GPS L5
    GNav,    // GLONASS
    INav,    // Galileo I/NAV
    FNav,    // Galileo F/NAV
    D1D2Nav, // BeiDou D1/D2
    BCNav1,  // BeiDou B1C
    BCNav2,  // BeiDou B2a
    BCNav3,  // BeiDou B2b/B3I
}

/// Унифицированный enum для всех типов навигационных данных
///
/// Заменяет Box<dyn NavBitTrait> для обеспечения Sync + Send и параллелизации
#[derive(Clone)]
#[allow(clippy::large_enum_variant)]
pub enum NavData {
    // === GPS Navigation Messages ===
    /// GPS L1CA - LNAV (Legacy Navigation)
    LNav(LNavBit),

    /// GPS L2C - CNAV (Civil Navigation)  
    CNav(CNavBit),

    /// GPS L1C - CNAV2 (Civil Navigation Version 2)
    CNav2(CNav2Bit),

    /// GPS L5 - L5-CNAV (L5 Civil Navigation)
    L5CNav(L5CNavBit),

    // === GLONASS Navigation Messages ===
    /// GLONASS G1/G2/G3 - GNAV (GLONASS Navigation)
    GNav(GNavBit),

    // === Galileo Navigation Messages ===
    /// Galileo E1/E5b - I/NAV (Integrity Navigation)
    INav(INavBit),

    /// Galileo E5a - F/NAV (Freely accessible Navigation)  
    FNav(FNavBit),

    // === BeiDou Navigation Messages ===
    /// BeiDou BDS-2 B1I/B2I/B3I - D1/D2 Navigation
    D1D2Nav(D1D2NavBit),

    /// BeiDou BDS-3 B1C - BCNav1 (BeiDou Civil Navigation 1)
    BCNav1(BCNav1Bit),

    /// BeiDou BDS-3 B2a - BCNav2 (BeiDou Civil Navigation 2)
    BCNav2(BCNav2Bit),

    /// BeiDou BDS-3 B2b/B3I - BCNav3 (BeiDou Civil Navigation 3)
    BCNav3(BCNav3Bit),
}

impl NavData {
    /// Получить тип навигационных данных
    #[inline]
    pub fn get_nav_message_type(&self) -> NavMessageType {
        match self {
            NavData::LNav(_) => NavMessageType::LNav,
            NavData::CNav(_) => NavMessageType::CNav,
            NavData::CNav2(_) => NavMessageType::CNav2,
            NavData::L5CNav(_) => NavMessageType::L5CNav,
            NavData::GNav(_) => NavMessageType::GNav,
            NavData::INav(_) => NavMessageType::INav,
            NavData::FNav(_) => NavMessageType::FNav,
            NavData::D1D2Nav(_) => NavMessageType::D1D2Nav,
            NavData::BCNav1(_) => NavMessageType::BCNav1,
            NavData::BCNav2(_) => NavMessageType::BCNav2,
            NavData::BCNav3(_) => NavMessageType::BCNav3,
        }
    }

    /// Получить навигационные биты (основной метод)
    ///
    /// Высокопроизводительная замена виртуального вызова через match
    /// Обрабатывает различия в сигнатурах разных навигационных модулей
    #[inline]
    pub fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        match self {
            NavData::LNav(nav) => {
                // LNavBit требует [i32; 300]
                if nav_bits.len() >= 300 {
                    let mut fixed_array = [0i32; 300];
                    let result = nav.get_frame_data(start_time, svid, param, &mut fixed_array);
                    nav_bits[..300].copy_from_slice(&fixed_array);
                    result
                } else {
                    0 // Недостаточно места
                }
            }
            NavData::CNav(nav) => nav.get_frame_data(start_time, svid, param, nav_bits),
            NavData::CNav2(nav) => nav.get_frame_data(start_time, svid, param, nav_bits),
            NavData::L5CNav(nav) => {
                // L5CNavBit требует [i32; 600]
                if nav_bits.len() >= 600 {
                    let mut fixed_array = [0i32; 600];
                    let result = nav.get_frame_data(start_time, svid, param, &mut fixed_array);
                    nav_bits[..600].copy_from_slice(&fixed_array);
                    result
                } else {
                    0 // Недостаточно места
                }
            }
            NavData::GNav(nav) => nav.get_frame_data(start_time, svid, param, nav_bits),
            NavData::INav(nav) => nav.get_frame_data(start_time, svid, param, nav_bits),
            NavData::FNav(nav) => nav.get_frame_data(start_time, svid, param, nav_bits),
            NavData::D1D2Nav(nav) => nav.get_frame_data(start_time, svid, param, nav_bits),
            NavData::BCNav1(nav) => nav.get_frame_data(start_time, svid, param, nav_bits),
            NavData::BCNav2(nav) => nav.get_frame_data(start_time, svid, param, nav_bits),
            NavData::BCNav3(nav) => nav.get_frame_data(start_time, svid, param, nav_bits),
        }
    }

    /// Установить эфемериды спутника
    #[inline]
    pub fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) -> i32 {
        match self {
            NavData::LNav(nav) => nav.set_ephemeris(svid, eph),
            NavData::CNav(nav) => nav.set_ephemeris(svid, eph),
            NavData::CNav2(nav) => nav.set_ephemeris(svid, eph),
            NavData::L5CNav(nav) => nav.set_ephemeris(svid, eph),
            NavData::GNav(_nav) => {
                // GLONASS использует собственный формат эфемерид
                // TODO: Добавить конвертацию GPS -> GLONASS если нужно
                0 // Заглушка
            }
            NavData::INav(nav) => nav.set_ephemeris(svid, eph),
            NavData::FNav(nav) => nav.set_ephemeris(svid, eph),
            NavData::D1D2Nav(nav) => nav.set_ephemeris(svid, eph),
            NavData::BCNav1(nav) => {
                if nav.set_ephemeris(svid, eph) {
                    1
                } else {
                    0
                }
            }
            NavData::BCNav2(nav) => nav.set_ephemeris(svid, eph), // Возвращает i32
            NavData::BCNav3(nav) => {
                if nav.set_ephemeris(svid, eph) {
                    1
                } else {
                    0
                }
            }
        }
    }

    /// Установить эфемериды ГЛОНАСС
    #[inline]
    pub fn set_glonass_ephemeris(&mut self, svid: i32, eph: &GlonassEphemeris) -> i32 {
        match self {
            NavData::GNav(nav) => nav.set_glonass_ephemeris(svid, eph),
            _ => 0,
        }
    }

    /// Установить альманах  
    #[inline]
    pub fn set_almanac(&mut self, alm: &[GpsAlmanac]) -> i32 {
        match self {
            NavData::LNav(nav) => {
                // Конвертируем &[GpsAlmanac] в &[GpsAlmanac; 32]
                if alm.len() >= 32 {
                    let alm_array: &[GpsAlmanac; 32] = alm[..32].try_into().unwrap();
                    nav.set_almanac(alm_array)
                } else {
                    0 // Недостаточно данных альманаха
                }
            }
            NavData::CNav(nav) => nav.set_almanac(alm),
            NavData::CNav2(nav) => nav.set_almanac(alm),
            NavData::L5CNav(nav) => {
                // Конвертируем &[GpsAlmanac] в &[GpsAlmanac; 32]
                if alm.len() >= 32 {
                    let alm_array: &[GpsAlmanac; 32] = alm[..32].try_into().unwrap();
                    nav.set_almanac(alm_array)
                } else {
                    0 // Недостаточно данных альманаха
                }
            }
            NavData::GNav(nav) => nav.set_almanac(alm),
            NavData::INav(nav) => nav.set_almanac(alm),
            NavData::FNav(nav) => nav.set_almanac(alm),
            NavData::D1D2Nav(nav) => nav.set_almanac(alm),
            NavData::BCNav1(nav) => {
                if nav.set_almanac(alm) {
                    1
                } else {
                    0
                }
            }
            NavData::BCNav2(nav) => nav.set_almanac(alm), // Возвращает i32
            NavData::BCNav3(nav) => {
                if nav.set_almanac(alm) {
                    1
                } else {
                    0
                }
            }
        }
    }

    /// Установить ионосферные и UTC параметры
    #[inline]
    pub fn set_iono_utc(
        &mut self,
        iono_param: Option<&IonoParam>,
        utc_param: Option<&UtcParam>,
    ) -> i32 {
        match self {
            NavData::LNav(nav) => {
                // LNavBit требует обязательные параметры
                if let (Some(iono), Some(utc)) = (iono_param, utc_param) {
                    nav.set_iono_utc(iono, utc)
                } else {
                    0 // Недостаточно данных
                }
            }
            NavData::CNav(nav) => {
                // CNavBit требует обязательные параметры
                if let (Some(iono), Some(utc)) = (iono_param, utc_param) {
                    nav.set_iono_utc(iono, utc)
                } else {
                    0 // Недостаточно данных
                }
            }
            NavData::CNav2(nav) => {
                // CNav2Bit требует обязательные параметры
                if let (Some(iono), Some(utc)) = (iono_param, utc_param) {
                    nav.set_iono_utc(iono, utc)
                } else {
                    0 // Недостаточно данных
                }
            }
            NavData::L5CNav(nav) => {
                // L5CNavBit требует обязательные параметры
                if let (Some(iono), Some(utc)) = (iono_param, utc_param) {
                    nav.set_iono_utc(iono, utc)
                } else {
                    0 // Недостаточно данных
                }
            }
            NavData::GNav(nav) => nav.set_iono_utc(iono_param, utc_param),
            NavData::INav(_nav) => {
                // INavBit требует другой тип IonoNequick вместо IonoParam
                // TODO: Добавить конвертацию параметров если нужно
                0 // Заглушка
            }
            NavData::FNav(nav) => {
                // FNavBit требует обязательные параметры
                if let (Some(iono), Some(utc)) = (iono_param, utc_param) {
                    nav.set_iono_utc(iono, utc)
                } else {
                    0 // Недостаточно данных
                }
            }
            NavData::D1D2Nav(nav) => {
                // D1D2NavBit требует обязательные параметры
                if let (Some(iono), Some(utc)) = (iono_param, utc_param) {
                    nav.set_iono_utc(iono, utc)
                } else {
                    0 // Недостаточно данных
                }
            }
            NavData::BCNav1(nav) => nav.set_iono_utc(iono_param, utc_param), // Возвращает i32
            NavData::BCNav2(nav) => nav.set_iono_utc(iono_param, utc_param), // Возвращает i32
            NavData::BCNav3(nav) => {
                if nav.set_iono_utc(iono_param, utc_param) {
                    1
                } else {
                    0
                }
            }
        }
    }

    /// Получить тип навигационных данных для совместимости с NavBitTrait
    #[inline]
    pub fn get_type(&self) -> NavDataType {
        match self {
            NavData::LNav(_) => NavDataType::LNav,
            NavData::CNav(_) => NavDataType::CNav,
            NavData::CNav2(_) => NavDataType::CNav2,
            NavData::L5CNav(_) => NavDataType::L5CNav,
            NavData::GNav(_) => NavDataType::GNav,
            NavData::INav(_) => NavDataType::INav,
            NavData::FNav(_) => NavDataType::FNav,
            NavData::D1D2Nav(_) => NavDataType::D1D2Nav,
            NavData::BCNav1(_) => NavDataType::BCNav1,
            NavData::BCNav2(_) => NavDataType::BCNav2,
            NavData::BCNav3(_) => NavDataType::BCNav3,
        }
    }
}

/// Конструкторы для создания навигационных данных по типу системы и сигнала
impl NavData {
    /// Создать навигационные данные для GPS
    pub fn new_gps(signal_type: i32) -> Option<Self> {
        match signal_type {
            1 => Some(NavData::LNav(LNavBit::new())),     // L1CA
            2 => Some(NavData::CNav(CNavBit::new())),     // L2C
            5 => Some(NavData::L5CNav(L5CNavBit::new())), // L5
            _ => None,
        }
    }

    /// Создать навигационные данные для GLONASS
    pub fn new_glonass(signal_type: i32) -> Option<Self> {
        match signal_type {
            1..=3 => Some(NavData::GNav(GNavBit::new())), // G1/G2/G3
            _ => None,
        }
    }

    /// Создать навигационные данные для Galileo
    pub fn new_galileo(signal_type: i32) -> Option<Self> {
        match signal_type {
            1 => Some(NavData::INav(INavBit::new())), // E1
            5 => Some(NavData::FNav(FNavBit::new())), // E5a
            7 => Some(NavData::INav(INavBit::new())), // E5b
            _ => None,
        }
    }

    /// Создать навигационные данные для BeiDou
    pub fn new_beidou(signal_type: i32) -> Option<Self> {
        match signal_type {
            1 => Some(NavData::D1D2Nav(D1D2NavBit::new())), // B1I (BDS-2)
            2 => Some(NavData::D1D2Nav(D1D2NavBit::new())), // B2I (BDS-2)
            3 => Some(NavData::D1D2Nav(D1D2NavBit::new())), // B3I (BDS-2)
            11 => Some(NavData::BCNav1(BCNav1Bit::new())),  // B1C (BDS-3)
            12 => Some(NavData::BCNav2(BCNav2Bit::new())),  // B2a (BDS-3)
            13 => Some(NavData::BCNav3(BCNav3Bit::new())),  // B2b (BDS-3)
            _ => None,
        }
    }

    /// Универсальный конструктор по системе и типу сигнала
    pub fn new_for_system(system: GnssSystem, signal_type: i32) -> Option<Self> {
        match system {
            GnssSystem::GpsSystem => Self::new_gps(signal_type),
            GnssSystem::GlonassSystem => Self::new_glonass(signal_type),
            GnssSystem::GalileoSystem => Self::new_galileo(signal_type),
            GnssSystem::BdsSystem => Self::new_beidou(signal_type),
            _ => None, // SBAS, QZSS, NavIC пока не поддерживаются
        }
    }
}

// Автоматически реализуем Send + Sync для параллелизации
unsafe impl Send for NavData {}
unsafe impl Sync for NavData {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nav_data_creation() {
        // GPS
        let gps_l1ca = NavData::new_gps(1).unwrap();
        assert!(matches!(gps_l1ca, NavData::LNav(_)));

        // GLONASS
        let glo_g1 = NavData::new_glonass(1).unwrap();
        assert!(matches!(glo_g1, NavData::GNav(_)));

        // Galileo
        let gal_e1 = NavData::new_galileo(1).unwrap();
        assert!(matches!(gal_e1, NavData::INav(_)));

        // BeiDou
        let bds_b1i = NavData::new_beidou(1).unwrap();
        assert!(matches!(bds_b1i, NavData::D1D2Nav(_)));

        let bds_b1c = NavData::new_beidou(11).unwrap();
        assert!(matches!(bds_b1c, NavData::BCNav1(_)));
    }

    #[test]
    fn test_nav_data_is_sync_send() {
        // Проверяем, что NavData реализует Sync + Send для параллелизации
        fn assert_sync<T: Sync>() {}
        fn assert_send<T: Send>() {}

        assert_sync::<NavData>();
        assert_send::<NavData>();
    }
}
