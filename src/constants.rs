//! # Константы ГНСС систем
//!
//! Модуль содержит математические и физические константы, используемые
//! в расчетах ГНСС сигналов и навигационных данных.
//!
//! ## Основные группы констант:
//! - Математические константы (PI, скорости света)
//! - Физические константы WGS84 и других систем координат
//! - Частоты несущих всех ГНСС систем
//! - Индексы сигналов для массивов и структур данных
//! - Константы времени и эпох для разных систем
//! - Параметры ионосферных и тропосферных моделей
//!
//! ## Поддерживаемые системы:
//! - GPS (L1, L2, L5)
//! - ГЛОНАСС (G1, G2, G3) 
//! - BeiDou (B1, B2, B3)
//! - Galileo (E1, E5a, E5b, E6)

use std::f64::consts::PI;

// Mathematical constants
pub const LIGHT_SPEED: f64 = 299792458.0;
pub const PI2: f64 = 2.0 * PI;
pub const RAD2DEG: fn(f64) -> f64 = |r| r * (180.0 / PI);
pub const DEG2RAD: fn(f64) -> f64 = |d| d * (PI / 180.0);

// WGS84 constants
pub const WGS_PI: f64 = 3.1415926535898;
pub const WGS_AXIS_A: f64 = 6378137.0;
pub const WGS_AXIS_B: f64 = 6_356_752.314_245_179;
pub const WGS_E1_SQR: f64 = 0.006694379990141317;
pub const WGS_E2_SQR: f64 = 0.006739496742276435;
pub const WGS_SQRT_GM: f64 = 19_964_981.843_217_388;
pub const WGS_OMEGDOTE: f64 = 7.2921151467e-5;
pub const WGS_F_GTR: f64 = -4.442807633e-10;

// CGCS2000 constants
pub const CGCS2000_SQRT_GM: f64 = 19_964_980.385_665_298;
pub const CGCS2000_OMEGDOTE: f64 = 7.292115e-5;

// PZ90 constants
pub const PZ90_AE: f64 = 6378136.0;
pub const PZ90_AE2: f64 = PZ90_AE * PZ90_AE;
pub const EARTH_GM: f64 = 3.986004418e+14;  // Unified Earth GM constant
pub const PZ90_GM: f64 = EARTH_GM;  // PZ90 uses the same GM value
pub const PZ90_C20: f64 = 1082.63e-6;
pub const PZ90_C20AE2: f64 = PZ90_C20 * PZ90_AE2;
pub const PZ90_OMEGDOTE: f64 = 7.292115e-5;

// Almanac constants
pub const SQRT_A0: f64 = 5_440.588_203_494_177;
pub const NORMINAL_I0: f64 = 0.977_384_381_116_824_6;

// Signal frequency constants (in Hz)
pub const FREQ_GPS_L1: f64 = 1575420000.0;
pub const FREQ_GPS_L2: f64 = 1227600000.0;
pub const FREQ_GPS_L5: f64 = 1176450000.0;
pub const FREQ_BDS_B1C: f64 = 1575420000.0;
pub const FREQ_BDS_B1I: f64 = 1561098000.0;
pub const FREQ_BDS_B2I: f64 = 1207140000.0;
pub const FREQ_BDS_B3I: f64 = 1268520000.0;
pub const FREQ_BDS_B2A: f64 = 1176450000.0;
pub const FREQ_BDS_B2B: f64 = 1207140000.0;
pub const FREQ_BDS_B2AB: f64 = 1191795000.0;
pub const FREQ_GAL_E1: f64 = 1575420000.0;
pub const FREQ_GAL_E5A: f64 = 1176450000.0;
pub const FREQ_GAL_E5B: f64 = 1207140000.0;
pub const FREQ_GAL_E5: f64 = 1191795000.0;
pub const FREQ_GAL_E6: f64 = 1278750000.0;
pub const FREQ_GLO_G1: f64 = 1602000000.0;
pub const FREQ_GLO_G2: f64 = 1246000000.0;
pub const FREQ_GLO_G3: f64 = 1202025000.0;


// Global signal index constants (different from types.rs which are per-system)
pub const SIGNAL_INDEX_L1CA: usize = 0;
pub const SIGNAL_INDEX_L1C: usize = 1;
pub const SIGNAL_INDEX_L2C: usize = 2;
pub const SIGNAL_INDEX_L2P: usize = 3;
pub const SIGNAL_INDEX_L5: usize = 4;

pub const SIGNAL_INDEX_B1C: usize = 8;
pub const SIGNAL_INDEX_B1I: usize = 9;
pub const SIGNAL_INDEX_B2I: usize = 10;
pub const SIGNAL_INDEX_B3I: usize = 11;
pub const SIGNAL_INDEX_B2A: usize = 12;
pub const SIGNAL_INDEX_B2B: usize = 13;
pub const SIGNAL_INDEX_B2AB: usize = 14;

pub const SIGNAL_INDEX_E1: usize = 16;
pub const SIGNAL_INDEX_E5A: usize = 17;
pub const SIGNAL_INDEX_E5B: usize = 18;
pub const SIGNAL_INDEX_E6: usize = 20;

pub const SIGNAL_INDEX_G1: usize = 24;
pub const SIGNAL_INDEX_G2: usize = 25;
pub const SIGNAL_INDEX_G3: usize = 26;

// BCNav1Bit constants
pub const B1C_SUBFRAME2_SYMBOL_LENGTH: usize = 100;
pub const B1C_SUBFRAME3_SYMBOL_LENGTH: usize = 44;


// Macro for composing bits
#[macro_export]
macro_rules! COMPOSE_BITS {
    ($value:expr, $start:expr, $length:expr) => {
        (($value as u32) & ((1u32 << $length) - 1)) << $start
    };
}