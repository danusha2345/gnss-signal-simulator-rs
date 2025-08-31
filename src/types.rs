//! # Модуль основных типов данных ГНСС
//!
//! Этот модуль содержит базовые структуры данных и типы, используемые во всей ГНСС системе.
//! Включает в себя:
//! - Перечисления для различных ГНСС систем (GPS, ГЛОНАСС, BeiDou, Galileo)
//! - Типы альманахов для разных спутниковых систем
//! - Общие структуры данных для времени, координат и параметров спутников
//! - Константы и базовые типы для работы с ГНСС данными
//!
//! Модуль служит основой для всех других компонентов системы и обеспечивает
//! единообразное представление данных между различными модулями.

// Common GNSS types
pub type Bool = i32;
pub const TRUE: Bool = 1;
pub const FALSE: Bool = 0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AlmanacType {
    AlmanacGps,
    AlmanacBds,
    AlmanacGalileo,
    AlmanacGlonass,
    AlmanacUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[derive(Default)]
pub enum GnssSystem {
    #[default]
    GpsSystem,
    BdsSystem,
    GalileoSystem,
    GlonassSystem,
    SbasSystem,
    QzssSystem,
    NavICSystem,
}


// Signal index constants moved to constants.rs

// Velocity structures
#[derive(Debug, Clone, Copy, Default)]
pub struct LocalSpeed {
    pub ve: f64,
    pub vn: f64,
    pub vu: f64,
    pub speed: f64,
    pub course: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct KinematicInfo {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct LlaPosition {
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ConvertMatrix {
    pub x2e: f64,
    pub y2e: f64,
    pub x2n: f64,
    pub y2n: f64,
    pub z2n: f64,
    pub x2u: f64,
    pub y2u: f64,
    pub z2u: f64,
}

// Time structures
#[derive(Debug, Clone, Copy, Default)]
pub struct GnssTime {
    pub Week: i32,
    pub MilliSeconds: i32,
    pub SubMilliSeconds: f64,
}

impl GnssTime {
    pub fn add_milliseconds(&self, ms: f64) -> GnssTime {
        let mut new_ms = self.MilliSeconds as f64 + ms;
        let mut new_week = self.Week;
        
        // Handle week overflow (604800000 ms = 1 week)
        while new_ms >= 604800000.0 {
            new_ms -= 604800000.0;
            new_week += 1;
        }
        
        GnssTime {
            Week: new_week,
            MilliSeconds: new_ms as i32,
            SubMilliSeconds: self.SubMilliSeconds,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct UtcTime {
    pub Year: i32,
    pub Month: i32,
    pub Day: i32,
    pub Hour: i32,
    pub Minute: i32,
    pub Second: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct GlonassTime {
    pub LeapYear: i32,
    pub Day: i32,
    pub MilliSeconds: i32,
    pub SubMilliSeconds: f64,
}

// GPS ephemeris (also used by BDS, Galileo, QZSS and NavIC)
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsEphemeris {
    pub ura: i16,
    pub iodc: u16,
    pub iode: u8,
    pub svid: u8,
    pub source: u8,
    pub valid: u8,
    pub flag: u16,
    pub health: u16,
    pub toe: i32,
    pub toc: i32,
    pub top: i32,
    pub week: i32,
    // orbit parameters
    pub M0: f64,
    pub delta_n: f64,
    pub delta_n_dot: f64,
    pub ecc: f64,
    pub sqrtA: f64,
    pub axis_dot: f64,
    pub omega0: f64,
    pub i0: f64,
    pub w: f64,
    pub omega_dot: f64,
    pub idot: f64,
    pub cuc: f64,
    pub cus: f64,
    pub crc: f64,
    pub crs: f64,
    pub cic: f64,
    pub cis: f64,
    // clock and delay parameters
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub tgd: f64,
    pub tgd2: f64,
    pub tgd_ext: [f64; 5],
    // derived variables
    pub axis: f64,
    pub n: f64,
    pub root_ecc: f64,
    pub omega_t: f64,
    pub omega_delta: f64,
    pub Ek: f64,
    pub Ek_dot: f64,
}

// Definitions for source field
pub const EPH_SOURCE_LNAV: u8 = 0;
pub const EPH_SOURCE_D1D2: u8 = 0;
pub const EPH_SOURCE_INAV: u8 = 0;
pub const EPH_SOURCE_CNAV: u8 = 1;
pub const EPH_SOURCE_CNV1: u8 = 1;
pub const EPH_SOURCE_FNAV: u8 = 1;
pub const EPH_SOURCE_CNV2: u8 = 2;
pub const EPH_SOURCE_CNV3: u8 = 3;

// GPS almanac
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsAlmanac {
    pub valid: u8,
    pub flag: u8,
    pub health: u8,
    pub svid: u8,
    pub toa: i32,
    pub week: i32,
    pub M0: f64,
    pub ecc: f64,
    pub sqrtA: f64,
    pub omega0: f64,
    pub i0: f64,
    pub w: f64,
    pub omega_dot: f64,
    pub af0: f64,
    pub af1: f64,
}

// GLONASS ephemeris
#[derive(Debug, Clone, Copy, Default)]
pub struct GlonassEphemeris {
    pub valid: u8,
    pub flag: u8,
    pub freq: i8,
    pub slot: u8,
    pub P: u8,
    pub M: u8,
    pub Ft: u8,
    pub n: u8,
    pub Bn: u8,
    pub En: u8,
    pub tb: u32,
    pub day: u16,
    pub tk: u16,
    pub gamma: f64,
    pub tn: f64,
    pub dtn: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub ax: f64,
    pub ay: f64,
    pub az: f64,
    // derived variables
    pub tc: f64,
    pub PosVelT: KinematicInfo,
}

// GLONASS almanac
#[derive(Debug, Clone, Copy, Default)]
pub struct GlonassAlmanac {
    pub flag: u8,
    pub freq: i8,
    pub leap_year: i16,
    pub day: i16,
    pub t: f64,
    pub lambda: f64,
    pub di: f64,
    pub ecc: f64,
    pub w: f64,
    pub dt: f64,
    pub dt_dot: f64,
    pub clock_error: f64,
}

// Ionospheric parameters
#[derive(Debug, Clone, Copy, Default)]
pub struct IonoParam {
    pub a0: f64,
    pub a1: f64,
    pub a2: f64,
    pub a3: f64,
    pub b0: f64,
    pub b1: f64,
    pub b2: f64,
    pub b3: f64,
    pub flag: u32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct IonoNequick {
    pub ai0: f64,
    pub ai1: f64,
    pub ai2: f64,
    pub flag: u32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct IonoBdgim {
    pub alpha1: f64,
    pub alpha2: f64,
    pub alpha3: f64,
    pub alpha4: f64,
    pub alpha5: f64,
    pub alpha6: f64,
    pub alpha7: f64,
    pub alpha8: f64,
    pub alpha9: f64,
    pub flag: u32,
}

// UTC parameters
#[derive(Debug, Clone, Copy, Default)]
pub struct UtcParam {
    pub A0: f64,
    pub A1: f64,
    pub A2: f64,
    pub WN: i16,
    pub WNLSF: i16,
    pub tot: u8,
    pub TLS: i8,
    pub TLSF: i8,
    pub DN: u8,
    pub flag: u32,
}

pub const MAX_OBS_NUMBER: usize = 6;

#[derive(Debug, Clone, Copy, Default)]
pub struct SatObservation {
    pub system: i32,
    pub svid: i32,
    pub ValidMask: u32,
    pub PseudoRange: [f64; MAX_OBS_NUMBER],
    pub CarrierPhase: [f64; MAX_OBS_NUMBER],
    pub Doppler: [f64; MAX_OBS_NUMBER],
    pub CN0: [f64; MAX_OBS_NUMBER],
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[derive(Default)]
pub enum OutputType {
    #[default]
    OutputTypePosition,
    OutputTypeObservation,
    OutputTypeIFdata,
    OutputTypeBaseband,
}


#[derive(Debug, Clone, Copy, PartialEq)]
#[derive(Default)]
pub enum OutputFormat {
    #[default]
    OutputFormatEcef,
    OutputFormatLla,
    OutputFormatNmea,
    OutputFormatKml,
    OutputFormatRinex,
    OutputFormatIQ8,
    OutputFormatIQ4,
}


#[derive(Debug, Clone)]
pub struct OutputParam {
    pub filename: [u8; 256],
    pub config_filename: String, // Путь к файлу конфигурации
    pub Type: OutputType,
    pub Format: OutputFormat,
    pub GpsMaskOut: u32,
    pub GlonassMaskOut: u32,
    pub BdsMaskOut: u64,
    pub GalileoMaskOut: u64,
    pub ElevationMask: f64,
    pub Interval: i32,
    pub SampleFreq: i32,
    pub CenterFreq: i32,
    pub CompactConfig: CompactConfig, // 32-битная конфигурация
}

impl Default for OutputParam {
    fn default() -> Self {
        Self {
            filename: [0u8; 256],
            config_filename: String::new(),
            Type: OutputType::default(),
            Format: OutputFormat::default(),
            GpsMaskOut: 0,
            GlonassMaskOut: 0,
            BdsMaskOut: 0,
            GalileoMaskOut: 0,
            ElevationMask: 0.0,
            Interval: 0,
            SampleFreq: 0,
            CenterFreq: 0,
            CompactConfig: CompactConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct DelayConfig {
    pub SystemDelay: [f64; 4],
    pub ReceiverDelay: [[f64; 8]; 4],
}

#[derive(Debug, Clone, Copy, Default)]
pub struct SatelliteParam {
    pub system: GnssSystem,
    pub svid: i32,
    pub FreqID: i32,
    pub CN0: i32,
    pub PosTimeTag: i32,
    pub PosVel: KinematicInfo,
    pub Acc: [f64; 3],
    pub TravelTime: f64,
    pub IonoDelay: f64,
    pub GroupDelay: [f64; 8],
    pub Elevation: f64,
    pub Azimuth: f64,
    pub RelativeSpeed: f64,
    pub LosVector: [f64; 3],
}

// Compact configuration structure - 32-bit approach
#[derive(Debug, Clone, Copy, Default)]
pub struct CompactConfig {
    pub config: u32,
}

// System parsing bits (0-3)
pub const PARSE_GPS: u32     = 1 << 0;  // 0x1
pub const PARSE_BDS: u32     = 1 << 1;  // 0x2
pub const PARSE_GALILEO: u32 = 1 << 2;  // 0x4
pub const PARSE_GLONASS: u32 = 1 << 3;  // 0x8

// Signal generation bits (4-31) - mapped from SIGNAL_INDEX + 4
pub const GEN_L1CA: u32  = 1 << 4;   // GPS L1CA  (SIGNAL_INDEX_L1CA + 4)
pub const GEN_L1C: u32   = 1 << 5;   // GPS L1C   (SIGNAL_INDEX_L1C + 4)
pub const GEN_L2C: u32   = 1 << 6;   // GPS L2C   (SIGNAL_INDEX_L2C + 4)
pub const GEN_L2P: u32   = 1 << 7;   // GPS L2P   (SIGNAL_INDEX_L2P + 4)
pub const GEN_L5: u32    = 1 << 8;   // GPS L5    (SIGNAL_INDEX_L5 + 4)
pub const GEN_B1C: u32   = 1 << 12;  // BDS B1C   (SIGNAL_INDEX_B1C + 4)
pub const GEN_B1I: u32   = 1 << 13;  // BDS B1I   (SIGNAL_INDEX_B1I + 4)
pub const GEN_B2I: u32   = 1 << 14;  // BDS B2I   (SIGNAL_INDEX_B2I + 4)
pub const GEN_B3I: u32   = 1 << 15;  // BDS B3I   (SIGNAL_INDEX_B3I + 4)
pub const GEN_B2A: u32   = 1 << 16;  // BDS B2A   (SIGNAL_INDEX_B2A + 4)
pub const GEN_B2B: u32   = 1 << 17;  // BDS B2B   (SIGNAL_INDEX_B2B + 4)
pub const GEN_B2AB: u32  = 1 << 18;  // BDS B2AB  (SIGNAL_INDEX_B2AB + 4)
pub const GEN_E1: u32    = 1 << 20;  // GAL E1    (SIGNAL_INDEX_E1 + 4)
pub const GEN_E5A: u32   = 1 << 21;  // GAL E5A   (SIGNAL_INDEX_E5A + 4)
pub const GEN_E5B: u32   = 1 << 22;  // GAL E5B   (SIGNAL_INDEX_E5B + 4)
pub const GEN_E6: u32    = 1 << 24;  // GAL E6    (SIGNAL_INDEX_E6 + 4)
pub const GEN_G1: u32    = 1 << 28;  // GLO G1    (SIGNAL_INDEX_G1 + 4)
pub const GEN_G2: u32    = 1 << 29;  // GLO G2    (SIGNAL_INDEX_G2 + 4)
pub const GEN_G3: u32    = 1 << 30;  // GLO G3    (SIGNAL_INDEX_G3 + 4)

impl CompactConfig {
    pub fn new() -> Self {
        Self { config: 0 }
    }

    // Check if system parsing is enabled
    pub fn should_parse_gps(&self) -> bool {
        self.config & PARSE_GPS != 0
    }

    pub fn should_parse_bds(&self) -> bool {
        self.config & PARSE_BDS != 0
    }

    pub fn should_parse_galileo(&self) -> bool {
        self.config & PARSE_GALILEO != 0
    }

    pub fn should_parse_glonass(&self) -> bool {
        self.config & PARSE_GLONASS != 0
    }

    // Check if signal generation is enabled
    pub fn is_signal_enabled(&self, signal_bit: u32) -> bool {
        self.config & signal_bit != 0
    }

    // Enable system parsing
    pub fn enable_system_parsing(&mut self, system_bit: u32) {
        self.config |= system_bit;
    }

    // Enable signal generation
    pub fn enable_signal(&mut self, signal_bit: u32) {
        self.config |= signal_bit;
    }

    // Convert SIGNAL_INDEX to generation bit
    pub fn signal_index_to_gen_bit(signal_index: usize) -> Option<u32> {
        match signal_index {
            0 => Some(GEN_L1CA),   // SIGNAL_INDEX_L1CA
            1 => Some(GEN_L1C),    // SIGNAL_INDEX_L1C
            2 => Some(GEN_L2C),    // SIGNAL_INDEX_L2C
            3 => Some(GEN_L2P),    // SIGNAL_INDEX_L2P
            4 => Some(GEN_L5),     // SIGNAL_INDEX_L5
            8 => Some(GEN_B1C),    // SIGNAL_INDEX_B1C
            9 => Some(GEN_B1I),    // SIGNAL_INDEX_B1I
            10 => Some(GEN_B2I),   // SIGNAL_INDEX_B2I
            11 => Some(GEN_B3I),   // SIGNAL_INDEX_B3I
            12 => Some(GEN_B2A),   // SIGNAL_INDEX_B2A
            13 => Some(GEN_B2B),   // SIGNAL_INDEX_B2B
            14 => Some(GEN_B2AB),  // SIGNAL_INDEX_B2AB
            16 => Some(GEN_E1),    // SIGNAL_INDEX_E1
            17 => Some(GEN_E5A),   // SIGNAL_INDEX_E5A
            18 => Some(GEN_E5B),   // SIGNAL_INDEX_E5B
            20 => Some(GEN_E6),    // SIGNAL_INDEX_E6
            24 => Some(GEN_G1),    // SIGNAL_INDEX_G1
            25 => Some(GEN_G2),    // SIGNAL_INDEX_G2
            26 => Some(GEN_G3),    // SIGNAL_INDEX_G3
            _ => None,
        }
    }
}

// Type aliases for backwards compatibility
pub type Ephemeris = GpsEphemeris;

impl KinematicInfo {
    pub fn pos_vel(&self) -> [f64; 6] {
        [self.x, self.y, self.z, self.vx, self.vy, self.vz]
    }
}

impl GpsEphemeris {
    pub fn new() -> Self {
        Self::default()
    }
}

impl GlonassEphemeris {
    pub fn new() -> Self {
        Self::default()
    }
}