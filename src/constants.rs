use std::f64::consts::PI;

// Mathematical constants
pub const PI2: f64 = 2.0 * PI;

// Almanac constants
pub const SQRT_A0: f64 = 5440.588203494177338011974948823;
pub const NORMINAL_I0: f64 = 0.97738438111682456307726683035362;

// GNSS system constants
pub const PZ90_OMEGDOTE: f64 = 7.2921151467e-5;
pub const PZ90_GM: f64 = 3.9860044e14;
pub const PZ90_C20: f64 = -1.08263e-3;
pub const PZ90_AE2: f64 = 4.0680631590769e13;

pub const CGCS2000_OMEGDOTE: f64 = 7.2921150e-5;
pub const CGCS2000_SQRT_GM: f64 = 6.3140131e7;

pub const WGS_SQRT_GM: f64 = 6.3140131e7;
pub const WGS_OMEGDOTE: f64 = 7.2921151467e-5;

// WGS84 constants
pub const WGS_AXIS_A: f64 = 6378137.0;
pub const WGS_AXIS_B: f64 = 6356752.314245;
pub const WGS_E1_SQR: f64 = 0.00669437999014;
pub const WGS_E2_SQR: f64 = 0.00673949674228;

// PZ90 constants
pub const PZ90_C20AE2: f64 = PZ90_C20 * PZ90_AE2;

// Speed of light
pub const LIGHT_SPEED: f64 = 299792458.0;

// GPS frequency constants (Hz)
pub const FREQ_GPS_L1: f64 = 1575420000.0;
pub const FREQ_GPS_L2: f64 = 1227600000.0;
pub const FREQ_GPS_L5: f64 = 1176450000.0;

// BDS frequency constants (Hz)
pub const FREQ_BDS_B1C: f64 = 1575420000.0;
pub const FREQ_BDS_B1I: f64 = 1561098000.0;
pub const FREQ_BDS_B2I: f64 = 1207140000.0;
pub const FREQ_BDS_B3I: f64 = 1268520000.0;
pub const FREQ_BDS_B2A: f64 = 1176450000.0;
pub const FREQ_BDS_B2B: f64 = 1207140000.0;
pub const FREQ_BDS_B2AB: f64 = 1191795000.0;

// Galileo frequency constants (Hz)
pub const FREQ_GAL_E1: f64 = 1575420000.0;
pub const FREQ_GAL_E5A: f64 = 1176450000.0;
pub const FREQ_GAL_E5B: f64 = 1207140000.0;
pub const FREQ_GAL_E5: f64 = 1191795000.0;
pub const FREQ_GAL_E6: f64 = 1278750000.0;

// GLONASS frequency constants (Hz)
pub const FREQ_GLO_G1: f64 = 1602000000.0;
pub const FREQ_GLO_G2: f64 = 1246000000.0;
pub const FREQ_GLO_G3: f64 = 1202025000.0;

// Signal index constants
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

pub const SIGNAL_INDEX_E1: usize = 16;
pub const SIGNAL_INDEX_E5A: usize = 17;
pub const SIGNAL_INDEX_E5B: usize = 18;
pub const SIGNAL_INDEX_E6: usize = 20;

pub const SIGNAL_INDEX_G1: usize = 24;
pub const SIGNAL_INDEX_G2: usize = 25;

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