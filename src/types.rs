// use std::f64::consts::PI;

// Common GNSS types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AlmanacType {
    AlmanacGps,
    AlmanacBds,
    AlmanacGalileo,
    AlmanacGlonass,
    AlmanacUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GnssSystem {
    GpsSystem,
    GlonassSystem,
    GalileoSystem,
    BdsSystem,
    QzssSystem,
    IrnssSystem,
    UnknownSystem,
}

// Time structures
#[derive(Debug, Clone, Copy, Default)]
pub struct GnssTime {
    pub Week: i32,
    pub MilliSeconds: i32,
    pub SubMilliSeconds: f64,
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

// Almanac structures
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsAlmanac {
    pub valid: u8,
    pub health: u8,
    pub svid: u8,
    pub flag: u8,
    pub week: i32,
    pub toa: i32,
    pub ecc: f64,
    pub sqrtA: f64,
    pub omega0: f64,
    pub omega_dot: f64,
    pub w: f64,
    pub M0: f64,
    pub i0: f64,
    pub af0: f64,
    pub af1: f64,
}

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

// Ephemeris structures
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

impl GpsEphemeris {
    pub fn new() -> Self {
        Self::default()
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct GlonassEphemeris {
    pub valid: u8,
    pub slot: u8,
    pub flag: u8,
    pub freq: i8,
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
    // calculated values
    pub tc: f64,
    pub pos_vel_t: KinematicInfo,
}

impl GlonassEphemeris {
    pub fn new() -> Self {
        Self::default()
    }
}

// Kinematic information
#[derive(Debug, Clone, Copy, Default)]
pub struct KinematicInfo {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
}

impl KinematicInfo {
    pub fn pos_vel(&self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct LlaPosition {
    pub lat: f64,  // latitude in radians
    pub lon: f64,  // longitude in radians
    pub alt: f64,  // altitude in meters
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

#[derive(Debug, Clone, Copy, Default)]
pub struct LocalSpeed {
    pub ve: f64,     // east velocity
    pub vn: f64,     // north velocity
    pub vu: f64,     // up velocity
    pub speed: f64,  // horizontal speed
    pub course: f64, // course angle
}

// Ionosphere and UTC parameters
#[derive(Debug, Clone, Copy, Default)]
pub struct IonoParam {
    pub a0: f64,  // 2^-30
    pub a1: f64,  // 2^-27
    pub a2: f64,  // 2^-24
    pub a3: f64,  // 2^-24
    pub b0: f64,  // 2^11
    pub b1: f64,  // 2^14
    pub b2: f64,  // 2^16
    pub b3: f64,  // 2^16
    pub flag: u32, // bit0:1 available, bit8~13: svid
}

#[derive(Debug, Clone, Copy, Default)]
pub struct UtcParam {
    pub A0: f64,  // second
    pub A1: f64,  // second/second
    pub A2: f64,  // second/second^2
    pub WN: i16,
    pub WNLSF: i16,
    pub tot: u8,  // scale factor 2^12 for GPS/BDS and 3600 for Galileo
    pub TLS: i8,  // leap second
    pub TLSF: i8,
    pub DN: u8,
    pub flag: u32, // bit0: UTC parameter available, bit1: leap second available
}

// Type aliases for compatibility with C++ code
pub type PionoParam = *const IonoParam;
pub type PutcParam = *const UtcParam;