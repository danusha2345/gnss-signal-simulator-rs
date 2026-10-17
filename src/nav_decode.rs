#![allow(non_snake_case)]

//! Navigation message decoder module for GNSS diagnostics.
//!
//! Provides reverse-decoding functions for GPS LNAV, Galileo I-NAV, and BeiDou B-CNAV
//! navigation messages, plus parity/CRC verification.

use std::f64::consts::PI;

use crate::types::GpsEphemeris;

// ---------------------------------------------------------------------------
// GPS LNAV parity constants (copied from lnavbit.rs)
// ---------------------------------------------------------------------------

const PARITY_TABLE: [[u8; 16]; 6] = [
    [
        0x00, 0x13, 0x25, 0x36, 0x0B, 0x18, 0x2E, 0x3D, 0x16, 0x05, 0x33, 0x20, 0x1D, 0x0E,
        0x38, 0x2B,
    ],
    [
        0x00, 0x2C, 0x19, 0x35, 0x32, 0x1E, 0x2B, 0x07, 0x26, 0x0A, 0x3F, 0x13, 0x14, 0x38,
        0x0D, 0x21,
    ],
    [
        0x00, 0x0E, 0x1F, 0x11, 0x3E, 0x30, 0x21, 0x2F, 0x3D, 0x33, 0x22, 0x2C, 0x03, 0x0D,
        0x1C, 0x12,
    ],
    [
        0x00, 0x38, 0x31, 0x09, 0x23, 0x1B, 0x12, 0x2A, 0x07, 0x3F, 0x36, 0x0E, 0x24, 0x1C,
        0x15, 0x2D,
    ],
    [
        0x00, 0x0D, 0x1A, 0x17, 0x37, 0x3A, 0x2D, 0x20, 0x2F, 0x22, 0x35, 0x38, 0x18, 0x15,
        0x02, 0x0F,
    ],
    [
        0x00, 0x1C, 0x3B, 0x27, 0x34, 0x28, 0x0F, 0x13, 0x2A, 0x36, 0x11, 0x0D, 0x1E, 0x02,
        0x25, 0x39,
    ],
];

#[allow(dead_code)]
const PARITY_ADJUST: [u32; 4] = [0x00, 0xa5, 0xf6, 0x53];

// ---------------------------------------------------------------------------
// CRC24Q table (copied from crc24q.rs)
// ---------------------------------------------------------------------------

const CRC24Q_TABLE: [u32; 256] = [
    0x00000000, 0x01864CFB, 0x028AD50D, 0x030C99F6, 0x0493E6E1, 0x0515AA1A, 0x061933EC,
    0x079F7F17, 0x08A18139, 0x0927CDC2, 0x0A2B5434, 0x0BAD18CF, 0x0C3267D8, 0x0DB42B23,
    0x0EB8B2D5, 0x0F3EFE2E, 0x10C54E89, 0x11430272, 0x124F9B84, 0x13C9D77F, 0x1456A868,
    0x15D0E493, 0x16DC7D65, 0x175A319E, 0x1864CFB0, 0x19E2834B, 0x1AEE1ABD, 0x1B685646,
    0x1CF72951, 0x1D7165AA, 0x1E7DFC5C, 0x1FFBB0A7, 0x200CD1E9, 0x218A9D12, 0x228604E4,
    0x2300481F, 0x249F3708, 0x25197BF3, 0x2615E205, 0x2793AEFE, 0x28AD50D0, 0x292B1C2B,
    0x2A2785DD, 0x2BA1C926, 0x2C3EB631, 0x2DB8FACA, 0x2EB4633C, 0x2F322FC7, 0x30C99F60,
    0x314FD39B, 0x32434A6D, 0x33C50696, 0x345A7981, 0x35DC357A, 0x36D0AC8C, 0x3756E077,
    0x38681E59, 0x39EE52A2, 0x3AE2CB54, 0x3B6487AF, 0x3CFBF8B8, 0x3D7DB443, 0x3E712DB5,
    0x3FF7614E, 0x4019A3D2, 0x419FEF29, 0x429376DF, 0x43153A24, 0x448A4533, 0x450C09C8,
    0x4600903E, 0x4786DCC5, 0x48B822EB, 0x493E6E10, 0x4A32F7E6, 0x4BB4BB1D, 0x4C2BC40A,
    0x4DAD88F1, 0x4EA11107, 0x4F275DFC, 0x50DCED5B, 0x515AA1A0, 0x52563856, 0x53D074AD,
    0x544F0BBA, 0x55C94741, 0x56C5DEB7, 0x5743924C, 0x587D6C62, 0x59FB2099, 0x5AF7B96F,
    0x5B71F594, 0x5CEE8A83, 0x5D68C678, 0x5E645F8E, 0x5FE21375, 0x6015723B, 0x61933EC0,
    0x629FA736, 0x6319EBCD, 0x648694DA, 0x6500D821, 0x660C41D7, 0x678A0D2C, 0x68B4F302,
    0x6932BFF9, 0x6A3E260F, 0x6BB86AF4, 0x6C2715E3, 0x6DA15918, 0x6EADC0EE, 0x6F2B8C15,
    0x70D03CB2, 0x71567049, 0x725AE9BF, 0x73DCA544, 0x7443DA53, 0x75C596A8, 0x76C90F5E,
    0x774F43A5, 0x7871BD8B, 0x79F7F170, 0x7AFB6886, 0x7B7D247D, 0x7CE25B6A, 0x7D641791,
    0x7E688E67, 0x7FEEC29C, 0x803347A4, 0x81B50B5F, 0x82B992A9, 0x833FDE52, 0x84A0A145,
    0x8526EDBE, 0x862A7448, 0x87AC38B3, 0x8892C69D, 0x89148A66, 0x8A181390, 0x8B9E5F6B,
    0x8C01207C, 0x8D876C87, 0x8E8BF571, 0x8F0DB98A, 0x90F6092D, 0x917045D6, 0x927CDC20,
    0x93FA90DB, 0x9465EFCC, 0x95E3A337, 0x96EF3AC1, 0x9769763A, 0x98578814, 0x99D1C4EF,
    0x9ADD5D19, 0x9B5B11E2, 0x9CC46EF5, 0x9D42220E, 0x9E4EBBF8, 0x9FC8F703, 0xA03F964D,
    0xA1B9DAB6, 0xA2B54340, 0xA3330FBB, 0xA4AC70AC, 0xA52A3C57, 0xA626A5A1, 0xA7A0E95A,
    0xA89E1774, 0xA9185B8F, 0xAA14C279, 0xAB928E82, 0xAC0DF195, 0xAD8BBD6E, 0xAE872498,
    0xAF016863, 0xB0FAD8C4, 0xB17C943F, 0xB2700DC9, 0xB3F64132, 0xB4693E25, 0xB5EF72DE,
    0xB6E3EB28, 0xB765A7D3, 0xB85B59FD, 0xB9DD1506, 0xBAD18CF0, 0xBB57C00B, 0xBCC8BF1C,
    0xBD4EF3E7, 0xBE426A11, 0xBFC426EA, 0xC02AE476, 0xC1ACA88D, 0xC2A0317B, 0xC3267D80,
    0xC4B90297, 0xC53F4E6C, 0xC633D79A, 0xC7B59B61, 0xC88B654F, 0xC90D29B4, 0xCA01B042,
    0xCB87FCB9, 0xCC1883AE, 0xCD9ECF55, 0xCE9256A3, 0xCF141A58, 0xD0EFAAFF, 0xD169E604,
    0xD2657FF2, 0xD3E33309, 0xD47C4C1E, 0xD5FA00E5, 0xD6F69913, 0xD770D5E8, 0xD84E2BC6,
    0xD9C8673D, 0xDAC4FECB, 0xDB42B230, 0xDCDDCD27, 0xDD5B81DC, 0xDE57182A, 0xDFD154D1,
    0xE026359F, 0xE1A07964, 0xE2ACE092, 0xE32AAC69, 0xE4B5D37E, 0xE5339F85, 0xE63F0673,
    0xE7B94A88, 0xE887B4A6, 0xE901F85D, 0xEA0D61AB, 0xEB8B2D50, 0xEC145247, 0xED921EBC,
    0xEE9E874A, 0xEF18CBB1, 0xF0E37B16, 0xF16537ED, 0xF269AE1B, 0xF3EFE2E0, 0xF4709DF7,
    0xF5F6D10C, 0xF6FA48FA, 0xF77C0401, 0xF842FA2F, 0xF9C4B6D4, 0xFAC82F22, 0xFB4E63D9,
    0xFCD11CCE, 0xFD575035, 0xFE5BC9C3, 0xFFDD8538,
];

// ===========================================================================
// 1. Rescale primitives (inverse of unscale)
// ===========================================================================

/// Inverse of `unscale_int`: recovers floating-point value from quantized integer.
///
/// `unscale_int(value, scale) = round(value * 2^(-scale))`, so the inverse is:
/// `rescale_int(encoded, scale) = encoded * 2^scale`.
pub fn rescale_int(value: i32, scale: i32) -> f64 {
    (value as f64) * 2.0_f64.powi(scale)
}

/// Unsigned variant of `rescale_int`.
pub fn rescale_uint(value: u32, scale: i32) -> f64 {
    (value as f64) * 2.0_f64.powi(scale)
}

/// Inverse of `compose_bits`: extracts `length` bits starting at bit `position`.
///
/// `compose_bits(value, position, length)` places the lowest `length` bits of `value`
/// at bit offset `position`. This function reverses that operation.
pub fn extract_bits(word: u32, position: i32, length: i32) -> u32 {
    // Width-safe mask: `1u32 << 32` is undefined (panics in debug, wraps in release). Mirror the
    // guard in sign_extend so a full-width extract works even though current callers stay <= 24.
    let mask = if length >= 32 {
        u32::MAX
    } else {
        (1u32 << length) - 1
    };
    (word >> position) & mask
}

/// Sign-extends a `bits`-wide two's-complement value to i32.
pub fn sign_extend(value: u32, bits: i32) -> i32 {
    let bits = bits as u32;
    if bits == 0 || bits >= 32 {
        return value as i32;
    }
    if (value >> (bits - 1)) & 1 != 0 {
        // MSB is set — extend sign
        (value | (!0u32 << bits)) as i32
    } else {
        value as i32
    }
}

/// Reconstructs a BeiDou 33-bit sign+magnitude value.
///
/// BeiDou B-CNAV uses sign+magnitude for 33-bit orbital parameters:
/// bit 32 is the sign bit, bits 31..0 are the unsigned magnitude.
/// Reconstructs a 33-bit two's complement value from sign bit and lower 32 bits.
///
/// The BeiDou encoder uses `unscale_long` (i64) and then extracts bit 32 as sign
/// and lower 32 bits as value. This is NOT sign+magnitude — it's the i64 two's complement
/// representation split at bit 32.
///
/// For negative values: value = lower 32 bits of two's complement i64.
/// Reconstruction: if sign is set, result = value - 2^32 (sign extend bit 32).
pub fn sign_magnitude_33(sign: u32, value: u32) -> f64 {
    if sign != 0 {
        // Sign bit set → negative. The 32-bit value is the lower bits of a negative i64.
        // Reconstruct: (value as i64) - (1i64 << 32)
        (value as f64) - 4294967296.0 // 2^32
    } else {
        value as f64
    }
}

// ===========================================================================
// 2. ParamDiff — comparison result for a single parameter
// ===========================================================================

#[derive(Debug)]
pub struct ParamDiff {
    pub name: &'static str,
    pub original: f64,
    pub decoded: f64,
    pub diff: f64,
    pub tolerance: f64,
    pub ok: bool,
}

impl ParamDiff {
    pub fn new(name: &'static str, original: f64, decoded: f64, tolerance: f64) -> Self {
        let diff = (original - decoded).abs();
        ParamDiff {
            name,
            original,
            decoded,
            diff,
            tolerance,
            ok: diff <= tolerance,
        }
    }
}

// ===========================================================================
// 3. GPS LNAV decoder
// ===========================================================================

/// Decoded GPS LNAV ephemeris from subframes 1/2/3.
#[derive(Debug, Default)]
pub struct DecodedLnavEph {
    pub iode: u8,
    pub iodc: u16,
    pub ura: i16,
    pub health: u16,
    pub flag: u16,
    pub toe: i32,
    pub toc: i32,
    pub M0: f64,
    pub delta_n: f64,
    pub ecc: f64,
    pub sqrtA: f64,
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
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub tgd: f64,
}

/// Decodes GPS LNAV ephemeris from the 3-subframe stream array.
///
/// `stream` is `[[u32; 8]; 3]` — 3 subframes of 8 words each.
/// Each word contains 24 data bits in bits 0..23 (no parity).
/// This is the exact inverse of `compose_gps_stream123` in `lnavbit.rs`.
pub fn decode_lnav_stream123(stream: &[[u32; 8]; 3]) -> DecodedLnavEph {
    let mut d = DecodedLnavEph::default();

    // === Subframe 1 (stream[0]) ===

    // Word 3 (stream[0][0]): L2 code(12,2), URA(8,4), health(2,6), IODC MSB(0,2)
    let l2_code = extract_bits(stream[0][0], 12, 2) as u16;
    d.ura = extract_bits(stream[0][0], 8, 4) as i16;
    d.health = extract_bits(stream[0][0], 2, 6) as u16;
    let iodc_msb = extract_bits(stream[0][0], 0, 2) as u16;

    // Word 4 (stream[0][1]): L2P flag(23,1)
    let l2p_flag = extract_bits(stream[0][1], 23, 1) as u16;
    d.flag = l2_code | (l2p_flag << 2);

    // Words 5-6 (stream[0][2], [0][3]): reserved — skip

    // Word 7 (stream[0][4]): TGD(0,8) scale -31
    let tgd_raw = extract_bits(stream[0][4], 0, 8);
    d.tgd = rescale_int(sign_extend(tgd_raw, 8), -31);

    // Word 8 (stream[0][5]): IODC LSB(16,8), toc>>4(0,16)
    let iodc_lsb = extract_bits(stream[0][5], 16, 8) as u16;
    d.iodc = (iodc_msb << 8) | iodc_lsb;
    let toc_shifted = extract_bits(stream[0][5], 0, 16);
    d.toc = (toc_shifted as i32) << 4; // toc = (toc>>4) << 4

    // Word 9 (stream[0][6]): af2(16,8) scale -55, af1(0,16) scale -43
    let af2_raw = extract_bits(stream[0][6], 16, 8);
    d.af2 = rescale_int(sign_extend(af2_raw, 8), -55);
    let af1_raw = extract_bits(stream[0][6], 0, 16);
    d.af1 = rescale_int(sign_extend(af1_raw, 16), -43);

    // Word 10 (stream[0][7]): af0(2,22) scale -31
    let af0_raw = extract_bits(stream[0][7], 2, 22);
    d.af0 = rescale_int(sign_extend(af0_raw, 22), -31);

    // === Subframe 2 (stream[1]) ===

    // stream[1][0]: IODE(16,8), Crs(0,16) scale -5
    d.iode = extract_bits(stream[1][0], 16, 8) as u8;
    let crs_raw = extract_bits(stream[1][0], 0, 16);
    d.crs = rescale_int(sign_extend(crs_raw, 16), -5);

    // stream[1][1]: delta_n/PI(8,16) scale -43, M0/PI MSB(0,8) upper 8 of 32
    let delta_n_raw = extract_bits(stream[1][1], 8, 16);
    d.delta_n = rescale_int(sign_extend(delta_n_raw, 16), -43) * PI;

    let m0_msb = extract_bits(stream[1][1], 0, 8);
    // stream[1][2]: M0/PI LSB(0,24) lower 24 of 32 total, scale -31
    let m0_lsb = extract_bits(stream[1][2], 0, 24);
    let m0_combined = (m0_msb << 24) | m0_lsb;
    d.M0 = rescale_int(sign_extend(m0_combined, 32), -31) * PI;

    // stream[1][3]: Cuc(8,16) scale -29, ecc MSB(0,8) upper 8 of 32
    let cuc_raw = extract_bits(stream[1][3], 8, 16);
    d.cuc = rescale_int(sign_extend(cuc_raw, 16), -29);
    let ecc_msb = extract_bits(stream[1][3], 0, 8);

    // stream[1][4]: ecc LSB(0,24) lower 24 of 32, scale -33 unsigned
    let ecc_lsb = extract_bits(stream[1][4], 0, 24);
    let ecc_combined = (ecc_msb << 24) | ecc_lsb;
    d.ecc = rescale_uint(ecc_combined, -33);

    // stream[1][5]: Cus(8,16) scale -29, sqrtA MSB(0,8) upper 8 of 32
    let cus_raw = extract_bits(stream[1][5], 8, 16);
    d.cus = rescale_int(sign_extend(cus_raw, 16), -29);
    let sqrtA_msb = extract_bits(stream[1][5], 0, 8);

    // stream[1][6]: sqrtA LSB(0,24) lower 24 of 32, scale -19 unsigned
    let sqrtA_lsb = extract_bits(stream[1][6], 0, 24);
    let sqrtA_combined = (sqrtA_msb << 24) | sqrtA_lsb;
    d.sqrtA = rescale_uint(sqrtA_combined, -19);

    // stream[1][7]: toe>>4(8,16), fit interval flag(7,1)
    let toe_shifted = extract_bits(stream[1][7], 8, 16);
    d.toe = (toe_shifted as i32) << 4;
    let fit_flag = extract_bits(stream[1][7], 7, 1) as u16;
    d.flag |= fit_flag << 3;

    // === Subframe 3 (stream[2]) ===

    // stream[2][0]: Cic(8,16) scale -29, omega0/PI MSB(0,8) upper 8 of 32
    let cic_raw = extract_bits(stream[2][0], 8, 16);
    d.cic = rescale_int(sign_extend(cic_raw, 16), -29);
    let omega0_msb = extract_bits(stream[2][0], 0, 8);

    // stream[2][1]: omega0/PI LSB(0,24) lower 24, scale -31
    let omega0_lsb = extract_bits(stream[2][1], 0, 24);
    let omega0_combined = (omega0_msb << 24) | omega0_lsb;
    d.omega0 = rescale_int(sign_extend(omega0_combined, 32), -31) * PI;

    // stream[2][2]: Cis(8,16) scale -29, i0/PI MSB(0,8) upper 8 of 32
    let cis_raw = extract_bits(stream[2][2], 8, 16);
    d.cis = rescale_int(sign_extend(cis_raw, 16), -29);
    let i0_msb = extract_bits(stream[2][2], 0, 8);

    // stream[2][3]: i0/PI LSB(0,24) lower 24, scale -31
    let i0_lsb = extract_bits(stream[2][3], 0, 24);
    let i0_combined = (i0_msb << 24) | i0_lsb;
    d.i0 = rescale_int(sign_extend(i0_combined, 32), -31) * PI;

    // stream[2][4]: Crc(8,16) scale -5, w/PI MSB(0,8) upper 8 of 32
    let crc_raw = extract_bits(stream[2][4], 8, 16);
    d.crc = rescale_int(sign_extend(crc_raw, 16), -5);
    let w_msb = extract_bits(stream[2][4], 0, 8);

    // stream[2][5]: w/PI LSB(0,24) lower 24, scale -31
    let w_lsb = extract_bits(stream[2][5], 0, 24);
    let w_combined = (w_msb << 24) | w_lsb;
    d.w = rescale_int(sign_extend(w_combined, 32), -31) * PI;

    // stream[2][6]: omega_dot/PI(0,24) 24 bits, scale -43
    let omega_dot_raw = extract_bits(stream[2][6], 0, 24);
    d.omega_dot = rescale_int(sign_extend(omega_dot_raw, 24), -43) * PI;

    // stream[2][7]: IODE(16,8), idot/PI(2,14) scale -43
    // IODE from subframe 3 — should match subframe 2 IODE
    let _iode_sf3 = extract_bits(stream[2][7], 16, 8) as u8;
    let idot_raw = extract_bits(stream[2][7], 2, 14);
    d.idot = rescale_int(sign_extend(idot_raw, 14), -43) * PI;

    d
}

impl DecodedLnavEph {
    /// Compares decoded ephemeris against the original `GpsEphemeris`.
    ///
    /// Returns a vector of `ParamDiff` entries showing per-parameter comparison.
    /// Tolerances are set to LSB/2 based on the quantization scale of each parameter.
    pub fn compare_with_original(&self, eph: &GpsEphemeris) -> Vec<ParamDiff> {
        vec![
            ParamDiff::new("M0", eph.M0, self.M0, PI * 2.0_f64.powi(-31) / 2.0),
            ParamDiff::new(
                "delta_n",
                eph.delta_n,
                self.delta_n,
                PI * 2.0_f64.powi(-43) / 2.0,
            ),
            ParamDiff::new("ecc", eph.ecc, self.ecc, 2.0_f64.powi(-33) / 2.0),
            ParamDiff::new("sqrtA", eph.sqrtA, self.sqrtA, 2.0_f64.powi(-19) / 2.0),
            ParamDiff::new(
                "omega0",
                eph.omega0,
                self.omega0,
                PI * 2.0_f64.powi(-31) / 2.0,
            ),
            ParamDiff::new("i0", eph.i0, self.i0, PI * 2.0_f64.powi(-31) / 2.0),
            ParamDiff::new("w", eph.w, self.w, PI * 2.0_f64.powi(-31) / 2.0),
            ParamDiff::new(
                "omega_dot",
                eph.omega_dot,
                self.omega_dot,
                PI * 2.0_f64.powi(-43) / 2.0,
            ),
            ParamDiff::new(
                "idot",
                eph.idot,
                self.idot,
                PI * 2.0_f64.powi(-43) / 2.0,
            ),
            ParamDiff::new("cuc", eph.cuc, self.cuc, 2.0_f64.powi(-29) / 2.0),
            ParamDiff::new("cus", eph.cus, self.cus, 2.0_f64.powi(-29) / 2.0),
            ParamDiff::new("crc", eph.crc, self.crc, 2.0_f64.powi(-5) / 2.0),
            ParamDiff::new("crs", eph.crs, self.crs, 2.0_f64.powi(-5) / 2.0),
            ParamDiff::new("cic", eph.cic, self.cic, 2.0_f64.powi(-29) / 2.0),
            ParamDiff::new("cis", eph.cis, self.cis, 2.0_f64.powi(-29) / 2.0),
            ParamDiff::new("af0", eph.af0, self.af0, 2.0_f64.powi(-31) / 2.0),
            ParamDiff::new("af1", eph.af1, self.af1, 2.0_f64.powi(-43) / 2.0),
            ParamDiff::new("af2", eph.af2, self.af2, 2.0_f64.powi(-55) / 2.0),
            ParamDiff::new("tgd", eph.tgd, self.tgd, 2.0_f64.powi(-31) / 2.0),
            ParamDiff::new("toe", eph.toe as f64, self.toe as f64, 16.0 / 2.0),
            ParamDiff::new("toc", eph.toc as f64, self.toc as f64, 16.0 / 2.0),
        ]
    }
}

// ===========================================================================
// 4. Galileo I-NAV decoder
// ===========================================================================

/// Decoded Galileo I-NAV ephemeris from the 20-word ephdata array.
#[derive(Debug, Default)]
pub struct DecodedInavEph {
    pub iod_nav: u16,
    pub toe: i32,
    pub toc: i32,
    pub M0: f64,
    pub delta_n: f64,
    pub ecc: f64,
    pub sqrtA: f64,
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
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub bgd_e5a_e1: f64,
    pub bgd_e5b_e1: f64,
    pub sisa: u8,
    pub health: u16,
    pub svid: u8,
}

/// Decodes Galileo I-NAV ephemeris from the 20-word `ephdata` array.
///
/// This is the exact inverse of `compose_eph_words` in `inavbit.rs`.
/// `ephdata` is `[u32; 20]`, each word has 32 bits. COMPOSE_BITS places values at bit positions.
pub fn decode_inav_eph_words(ephdata: &[u32]) -> DecodedInavEph {
    let mut d = DecodedInavEph::default();

    // === Word 1 (ephdata[0..4]) ===
    // ephdata[0]: 0x04000000 | IODnav(16,10) | toe/60(2,14) | M0/PI MSB(0,2)
    d.iod_nav = extract_bits(ephdata[0], 16, 10) as u16;
    let toe_div60 = extract_bits(ephdata[0], 2, 14);
    d.toe = (toe_div60 as i32) * 60;
    let m0_msb = extract_bits(ephdata[0], 0, 2);

    // ephdata[1]: M0/PI(2,30) | ecc MSB(0,2)
    let m0_mid = extract_bits(ephdata[1], 2, 30);
    let m0_combined = (m0_msb << 30) | m0_mid;
    d.M0 = rescale_int(sign_extend(m0_combined, 32), -31) * PI;

    let ecc_msb = extract_bits(ephdata[1], 0, 2);

    // ephdata[2]: ecc(2,30) | sqrtA MSB(0,2)
    let ecc_mid = extract_bits(ephdata[2], 2, 30);
    let ecc_combined = (ecc_msb << 30) | ecc_mid;
    d.ecc = rescale_uint(ecc_combined, -33);

    let sqrtA_msb = extract_bits(ephdata[2], 0, 2);

    // ephdata[3]: sqrtA(2,30)
    let sqrtA_mid = extract_bits(ephdata[3], 2, 30);
    let sqrtA_combined = (sqrtA_msb << 30) | sqrtA_mid;
    d.sqrtA = rescale_uint(sqrtA_combined, -19);

    // === Word 2 (ephdata[4..8]) ===
    // ephdata[4]: 0x08000000 | IODnav(16,10) | omega0/PI MSB(0,16)
    let omega0_msb = extract_bits(ephdata[4], 0, 16);

    // ephdata[5]: omega0/PI(16,16) | i0/PI MSB(0,16)
    let omega0_lsb = extract_bits(ephdata[5], 16, 16);
    let omega0_combined = (omega0_msb << 16) | omega0_lsb;
    d.omega0 = rescale_int(sign_extend(omega0_combined, 32), -31) * PI;

    let i0_msb = extract_bits(ephdata[5], 0, 16);

    // ephdata[6]: i0/PI(16,16) | w/PI MSB(0,16)
    let i0_lsb = extract_bits(ephdata[6], 16, 16);
    let i0_combined = (i0_msb << 16) | i0_lsb;
    d.i0 = rescale_int(sign_extend(i0_combined, 32), -31) * PI;

    let w_msb = extract_bits(ephdata[6], 0, 16);

    // ephdata[7]: w/PI(16,16) | idot/PI(2,14)
    let w_lsb = extract_bits(ephdata[7], 16, 16);
    let w_combined = (w_msb << 16) | w_lsb;
    d.w = rescale_int(sign_extend(w_combined, 32), -31) * PI;

    let idot_raw = extract_bits(ephdata[7], 2, 14);
    d.idot = rescale_int(sign_extend(idot_raw, 14), -43) * PI;

    // === Word 3 (ephdata[8..12]) ===
    // ephdata[8]: 0x0c000000 | IODnav(16,10) | omega_dot/PI MSB(0,16)
    let omega_dot_msb = extract_bits(ephdata[8], 0, 16);

    // ephdata[9]: omega_dot/PI(24,8) | delta_n/PI(8,16) | Cuc MSB(0,8)
    let omega_dot_lsb = extract_bits(ephdata[9], 24, 8);
    let omega_dot_combined = (omega_dot_msb << 8) | omega_dot_lsb;
    d.omega_dot = rescale_int(sign_extend(omega_dot_combined, 24), -43) * PI;

    let delta_n_raw = extract_bits(ephdata[9], 8, 16);
    d.delta_n = rescale_int(sign_extend(delta_n_raw, 16), -43) * PI;

    let cuc_msb = extract_bits(ephdata[9], 0, 8);

    // ephdata[10]: Cuc(24,8) | Cus(8,16) | Crc MSB(0,8)
    let cuc_lsb = extract_bits(ephdata[10], 24, 8);
    let cuc_combined = (cuc_msb << 8) | cuc_lsb;
    d.cuc = rescale_int(sign_extend(cuc_combined, 16), -29);

    let cus_raw = extract_bits(ephdata[10], 8, 16);
    d.cus = rescale_int(sign_extend(cus_raw, 16), -29);

    let crc_msb = extract_bits(ephdata[10], 0, 8);

    // ephdata[11]: Crc(24,8) | Crs(8,16) | URA(0,8)
    let crc_lsb = extract_bits(ephdata[11], 24, 8);
    let crc_combined = (crc_msb << 8) | crc_lsb;
    d.crc = rescale_int(sign_extend(crc_combined, 16), -5);

    let crs_raw = extract_bits(ephdata[11], 8, 16);
    d.crs = rescale_int(sign_extend(crs_raw, 16), -5);

    // URA at bits 0..7 — stored as SISA in the Galileo context but we have it here
    // (not mapped into DecodedInavEph from word 3; SISA is in word 5)

    // === Word 4 (ephdata[12..16]) ===
    // ephdata[12]: 0x10000000 | IODnav(16,10) | SVID(10,6) | Cic MSB(0,10)
    d.svid = extract_bits(ephdata[12], 10, 6) as u8;
    let cic_msb = extract_bits(ephdata[12], 0, 10);

    // ephdata[13]: Cic(26,6) | Cis(10,16) | toc/60 MSB(0,10)
    let cic_lsb = extract_bits(ephdata[13], 26, 6);
    let cic_combined = (cic_msb << 6) | cic_lsb;
    d.cic = rescale_int(sign_extend(cic_combined, 16), -29);

    let cis_raw = extract_bits(ephdata[13], 10, 16);
    d.cis = rescale_int(sign_extend(cis_raw, 16), -29);

    let toc_msb = extract_bits(ephdata[13], 0, 10);

    // ephdata[14]: toc/60(28,4) | af0 MSB(0,28)
    let toc_lsb = extract_bits(ephdata[14], 28, 4);
    let toc_div60 = (toc_msb << 4) | toc_lsb;
    d.toc = (toc_div60 as i32) * 60;

    let af0_msb = extract_bits(ephdata[14], 0, 28);

    // ephdata[15]: af0(29,3) | af1(8,21) | af2(2,6)
    let af0_lsb = extract_bits(ephdata[15], 29, 3);
    let af0_combined = (af0_msb << 3) | af0_lsb;
    d.af0 = rescale_int(sign_extend(af0_combined, 31), -34);

    let af1_raw = extract_bits(ephdata[15], 8, 21);
    d.af1 = rescale_int(sign_extend(af1_raw, 21), -46);

    let af2_raw = extract_bits(ephdata[15], 2, 6);
    d.af2 = rescale_int(sign_extend(af2_raw, 6), -59);

    // === Word 5 (ephdata[16..20]) ===
    // ephdata[17]: BGD E5a/E1(7,10 scale -32) | BGD E5b/E1 MSB(0,7)
    let bgd_e5a_raw = extract_bits(ephdata[17], 7, 10);
    d.bgd_e5a_e1 = rescale_int(sign_extend(bgd_e5a_raw, 10), -32);

    let bgd_e5b_msb = extract_bits(ephdata[17], 0, 7);

    // ephdata[18]: BGD E5b/E1(29,3) | SISA(15,8) | E5b HS(27,2) | E1B HS(25,2)
    //              | E5b DVS(24,1) | E1B DVS(23,1) | WN MSB(11,8)
    let bgd_e5b_lsb = extract_bits(ephdata[18], 29, 3);
    let bgd_e5b_combined = (bgd_e5b_msb << 3) | bgd_e5b_lsb;
    d.bgd_e5b_e1 = rescale_int(sign_extend(bgd_e5b_combined, 10), -32);

    // URA from word 3 (ephdata[11], bits 0,8) — store as part of decoded
    // (not used for comparison, but helpful for diagnostics)

    d.sisa = extract_bits(ephdata[18], 15, 8) as u8;

    let e5b_hs = extract_bits(ephdata[18], 27, 2) as u16;
    let e1b_hs = extract_bits(ephdata[18], 25, 2) as u16;
    let e5b_dvs = extract_bits(ephdata[18], 24, 1) as u16;
    let e1b_dvs = extract_bits(ephdata[18], 23, 1) as u16;
    // Pack health: same layout as the encoder reads it
    // Encoder: e5b_hs = health >> 7, e1b_hs = health >> 1, e5b_dvs = health >> 5, e1b_dvs = health
    d.health = e1b_dvs | (e1b_hs << 1) | (e5b_dvs << 5) | (e5b_hs << 7);

    d
}

impl DecodedInavEph {
    /// Compares decoded Galileo I-NAV ephemeris against the original `GpsEphemeris`.
    ///
    /// Tolerances are LSB/2 for each I-NAV parameter.
    /// Note: toe/toc have 60s resolution, so tolerance is 30s.
    pub fn compare_with_original(&self, eph: &GpsEphemeris) -> Vec<ParamDiff> {
        vec![
            ParamDiff::new("M0", eph.M0, self.M0, PI * 2.0_f64.powi(-31) / 2.0),
            ParamDiff::new("delta_n", eph.delta_n, self.delta_n, PI * 2.0_f64.powi(-43) / 2.0),
            ParamDiff::new("ecc", eph.ecc, self.ecc, 2.0_f64.powi(-33) / 2.0),
            ParamDiff::new("sqrtA", eph.sqrtA, self.sqrtA, 2.0_f64.powi(-19) / 2.0),
            ParamDiff::new("omega0", eph.omega0, self.omega0, PI * 2.0_f64.powi(-31) / 2.0),
            ParamDiff::new("i0", eph.i0, self.i0, PI * 2.0_f64.powi(-31) / 2.0),
            ParamDiff::new("w", eph.w, self.w, PI * 2.0_f64.powi(-31) / 2.0),
            ParamDiff::new("omega_dot", eph.omega_dot, self.omega_dot, PI * 2.0_f64.powi(-43) / 2.0),
            ParamDiff::new("idot", eph.idot, self.idot, PI * 2.0_f64.powi(-43) / 2.0),
            ParamDiff::new("cuc", eph.cuc, self.cuc, 2.0_f64.powi(-29) / 2.0),
            ParamDiff::new("cus", eph.cus, self.cus, 2.0_f64.powi(-29) / 2.0),
            ParamDiff::new("crc", eph.crc, self.crc, 2.0_f64.powi(-5) / 2.0),
            ParamDiff::new("crs", eph.crs, self.crs, 2.0_f64.powi(-5) / 2.0),
            ParamDiff::new("cic", eph.cic, self.cic, 2.0_f64.powi(-29) / 2.0),
            ParamDiff::new("cis", eph.cis, self.cis, 2.0_f64.powi(-29) / 2.0),
            ParamDiff::new("af0", eph.af0, self.af0, 2.0_f64.powi(-34) / 2.0),
            ParamDiff::new("af1", eph.af1, self.af1, 2.0_f64.powi(-46) / 2.0),
            ParamDiff::new("af2", eph.af2, self.af2, 2.0_f64.powi(-59) / 2.0),
            ParamDiff::new("bgd_e5a_e1", eph.tgd, self.bgd_e5a_e1, 2.0_f64.powi(-32) / 2.0),
            ParamDiff::new("bgd_e5b_e1", eph.tgd2, self.bgd_e5b_e1, 2.0_f64.powi(-32) / 2.0),
            ParamDiff::new("toe", eph.toe as f64, self.toe as f64, 30.0), // 60s resolution
            ParamDiff::new("toc", eph.toc as f64, self.toc as f64, 30.0), // 60s resolution
        ]
    }
}

// ===========================================================================
// 4b. Galileo F/NAV (E5a) decoder
// ===========================================================================

/// Decodes Galileo F/NAV (E5a) ephemeris from the 4 page-type × 7-word array produced by
/// `FNavBit::compose_eph_words`. F/NAV carries the same ephemeris/clock parameters and scales as
/// I/NAV (Galileo OS SIS ICD), but in a different page/word bit layout, and broadcasts only the
/// E5a BGD (no E5b BGD). Returns a `DecodedInavEph` so the I/NAV tolerances/compare apply.
pub fn decode_fnav_ephemeris(pages: &[[u32; 7]; 4]) -> DecodedInavEph {
    let bits = |page: usize, start: usize, width: usize| -> u32 {
        (start - 1..start - 1 + width).fold(0, |v, i| {
            (v << 1) | ((pages[page][i / 32] >> (31 - i % 32)) & 1)
        })
    };
    let signed = |p, s, n, scale| rescale_int(sign_extend(bits(p, s, n), n as i32), scale);
    DecodedInavEph {
        iod_nav: bits(0, 13, 10) as u16,
        toc: bits(0, 23, 14) as i32 * 60,
        af0: signed(0, 37, 31, -34),
        af1: signed(0, 68, 21, -46),
        af2: signed(0, 89, 6, -59),
        bgd_e5a_e1: signed(0, 144, 10, -32),
        M0: signed(1, 17, 32, -31) * PI,
        omega_dot: signed(1, 49, 24, -43) * PI,
        ecc: rescale_uint(bits(1, 73, 32), -33),
        sqrtA: rescale_uint(bits(1, 105, 32), -19),
        omega0: signed(1, 137, 32, -31) * PI,
        idot: signed(1, 169, 14, -43) * PI,
        i0: signed(2, 17, 32, -31) * PI,
        w: signed(2, 49, 32, -31) * PI,
        delta_n: signed(2, 81, 16, -43) * PI,
        cuc: signed(2, 97, 16, -29),
        cus: signed(2, 113, 16, -29),
        crc: signed(2, 129, 16, -5),
        crs: signed(2, 145, 16, -5),
        toe: bits(2, 161, 14) as i32 * 60,
        cic: signed(3, 17, 16, -29),
        cis: signed(3, 33, 16, -29),
        ..DecodedInavEph::default()
    }
}

// ===========================================================================
// 5. BeiDou B-CNAV decoder
// ===========================================================================

/// Decoded BeiDou B-CNAV ephemeris from ephemeris1, ephemeris2, and clock arrays.
#[derive(Debug, Default)]
pub struct DecodedBcnavEph {
    pub iode: u8,
    pub iodc: u16,
    pub toe: i32,
    pub toc: i32,
    pub sat_type: u8,
    pub delta_A: f64,
    pub axis_dot: f64,
    pub M0: f64,
    pub delta_n: f64,
    pub delta_n_dot: f64,
    pub ecc: f64,
    pub w: f64,
    pub omega0: f64,
    pub i0: f64,
    pub omega_dot: f64,
    pub idot: f64,
    pub cuc: f64,
    pub cus: f64,
    pub crc: f64,
    pub crs: f64,
    pub cic: f64,
    pub cis: f64,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
}

/// Decodes BeiDou B-CNAV ephemeris from the three data arrays.
///
/// This is the exact inverse of `set_ephemeris` in `bcnavbit.rs`.
///
/// - `eph1`: `[u32; 9]` — Ephemeris1 (delta_A, A_dot, delta_n, delta_n_dot, M0, ecc, w)
/// - `eph2`: `[u32; 10]` — Ephemeris2 (omega0, i0, omega_dot, idot, Cis/Cic/Crs/Crc/Cus/Cuc)
/// - `clock`: `[u32; 4]` — Clock parameters (toc, af0, af1, af2, IODC)
pub fn decode_bcnav_ephemeris(eph1: &[u32; 9], eph2: &[u32; 10], clock: &[u32; 4]) -> DecodedBcnavEph {
    let mut d = DecodedBcnavEph::default();

    // === Ephemeris1 (eph1[0..9]) ===

    // eph1[0]: IODE(16,8) | toe/300(5,11) | sat_type(3,2) | delta_A MSB(0,3)
    d.iode = extract_bits(eph1[0], 16, 8) as u8;
    let toe_div300 = extract_bits(eph1[0], 5, 11);
    d.toe = (toe_div300 as i32) * 300;
    d.sat_type = extract_bits(eph1[0], 3, 2) as u8;
    let delta_a_msb = extract_bits(eph1[0], 0, 3);

    // eph1[1]: delta_A(1,23) | A_dot MSB(0,1)
    let delta_a_mid = extract_bits(eph1[1], 1, 23);
    let delta_a_combined = (delta_a_msb << 23) | delta_a_mid;
    // delta_A is 26 bits total (3+23), scale -9, signed
    d.delta_A = rescale_int(sign_extend(delta_a_combined, 26), -9);

    let a_dot_msb = extract_bits(eph1[1], 0, 1);

    // eph1[2]: A_dot(0,24) — 25 total bits (1+24), scale -21, signed
    let a_dot_mid = extract_bits(eph1[2], 0, 24);
    let a_dot_combined = (a_dot_msb << 24) | a_dot_mid;
    d.axis_dot = rescale_int(sign_extend(a_dot_combined, 25), -21);

    // eph1[3]: delta_n(7,17 scale -44) | delta_n_dot MSB(0,7)
    let delta_n_raw = extract_bits(eph1[3], 7, 17);
    d.delta_n = rescale_int(sign_extend(delta_n_raw, 17), -44) * PI;

    let dn_dot_msb = extract_bits(eph1[3], 0, 7);

    // eph1[4]: delta_n_dot(8,16) | M0_sign(7,1) | M0 MSB(0,7)
    // delta_n_dot: 23 total bits (7+16), scale -57, signed
    let dn_dot_lsb = extract_bits(eph1[4], 8, 16);
    let dn_dot_combined = (dn_dot_msb << 16) | dn_dot_lsb;
    // Δn0-dot is in semicircles/s² (π/s²) per BDS-SIS-ICD-B1C Table 7-8 → ×PI for rad.
    d.delta_n_dot = rescale_int(sign_extend(dn_dot_combined, 23), -57) * PI;

    let m0_sign = extract_bits(eph1[4], 7, 1);
    let m0_bits_6_0 = extract_bits(eph1[4], 0, 7); // upper 7 bits of 32-bit value

    // eph1[5]: M0(0,24) — continue
    let m0_bits_30_7 = extract_bits(eph1[5], 0, 24); // middle 24 bits

    // eph1[6]: M0 LSB(23,1) | ecc_sign(22,1) | ecc MSB(0,22)
    let m0_bit_31 = extract_bits(eph1[6], 23, 1); // lowest 1 bit

    // Reconstruct M0 32-bit value: bits are packed MSB-first
    // From encoder: data[4] |= (m0_value >> 25, 0, 7)  — upper 7 bits
    //               data[5] = (m0_value >> 1, 0, 24)    — middle 24 bits
    //               data[6] = (m0_value, 23, 1)          — lowest 1 bit
    let m0_value = (m0_bits_6_0 << 25) | (m0_bits_30_7 << 1) | m0_bit_31;
    d.M0 = sign_magnitude_33(m0_sign, m0_value) * 2.0_f64.powi(-32) * PI;

    // ecc: 33-bit sign+magnitude, scale -34
    let ecc_sign = extract_bits(eph1[6], 22, 1);
    let ecc_bits_31_10 = extract_bits(eph1[6], 0, 22); // upper 22 bits

    // eph1[7]: ecc(14,10) | w_sign(13,1) | w MSB(0,13)
    let ecc_bits_9_0 = extract_bits(eph1[7], 14, 10); // lower 10 bits
    let ecc_value = (ecc_bits_31_10 << 10) | ecc_bits_9_0;
    d.ecc = sign_magnitude_33(ecc_sign, ecc_value) * 2.0_f64.powi(-34);

    // w: 33-bit sign+magnitude, scale -32
    let w_sign = extract_bits(eph1[7], 13, 1);
    let w_bits_31_19 = extract_bits(eph1[7], 0, 13); // upper 13 bits

    // eph1[8]: w(5,19) — lower 19 bits
    let w_bits_18_0 = extract_bits(eph1[8], 5, 19);
    let w_value = (w_bits_31_19 << 19) | w_bits_18_0;
    d.w = sign_magnitude_33(w_sign, w_value) * 2.0_f64.powi(-32) * PI;

    // === Ephemeris2 (eph2[0..10]) ===

    // eph2[0]: omega0_sign(23,1) | omega0 MSB(0,23)
    let omega0_sign = extract_bits(eph2[0], 23, 1);
    let omega0_bits_31_9 = extract_bits(eph2[0], 0, 23); // upper 23 bits

    // eph2[1]: omega0(15,9) | i0_sign(14,1) | i0 MSB(0,14)
    let omega0_bits_8_0 = extract_bits(eph2[1], 15, 9); // lower 9 bits
    let omega0_value = (omega0_bits_31_9 << 9) | omega0_bits_8_0;
    d.omega0 = sign_magnitude_33(omega0_sign, omega0_value) * 2.0_f64.powi(-32) * PI;

    let i0_sign = extract_bits(eph2[1], 14, 1);
    let i0_bits_31_18 = extract_bits(eph2[1], 0, 14); // upper 14 bits

    // eph2[2]: i0(6,18) | omega_dot MSB(0,6)
    let i0_bits_17_0 = extract_bits(eph2[2], 6, 18); // lower 18 bits
    let i0_value = (i0_bits_31_18 << 18) | i0_bits_17_0;
    d.i0 = sign_magnitude_33(i0_sign, i0_value) * 2.0_f64.powi(-32) * PI;

    let omega_dot_msb = extract_bits(eph2[2], 0, 6); // upper 6 bits of 19

    // eph2[3]: omega_dot(11,13) | idot MSB(0,11)
    let omega_dot_lsb = extract_bits(eph2[3], 11, 13); // lower 13 bits of 19
    let omega_dot_combined = (omega_dot_msb << 13) | omega_dot_lsb;
    d.omega_dot = rescale_int(sign_extend(omega_dot_combined, 19), -44) * PI;

    let idot_msb = extract_bits(eph2[3], 0, 11); // upper 11 bits of 15

    // eph2[4]: idot(20,4) | Cis(4,16) | Cic MSB(0,4)
    let idot_lsb = extract_bits(eph2[4], 20, 4); // lower 4 bits of 15
    let idot_combined = (idot_msb << 4) | idot_lsb;
    d.idot = rescale_int(sign_extend(idot_combined, 15), -44) * PI;

    let cis_raw = extract_bits(eph2[4], 4, 16);
    d.cis = rescale_int(sign_extend(cis_raw, 16), -30);

    let cic_msb = extract_bits(eph2[4], 0, 4); // upper 4 bits of 16

    // eph2[5]: Cic(12,12) | Crs MSB(0,12)
    let cic_lsb = extract_bits(eph2[5], 12, 12); // lower 12 bits of 16
    let cic_combined = (cic_msb << 12) | cic_lsb;
    d.cic = rescale_int(sign_extend(cic_combined, 16), -30);

    let crs_msb = extract_bits(eph2[5], 0, 12); // upper 12 bits of 24

    // eph2[6]: Crs(12,12) | Crc MSB(0,12)
    let crs_lsb = extract_bits(eph2[6], 12, 12); // lower 12 bits of 24
    let crs_combined = (crs_msb << 12) | crs_lsb;
    d.crs = rescale_int(sign_extend(crs_combined, 24), -8);

    let crc_msb = extract_bits(eph2[6], 0, 12); // upper 12 bits of 24

    // eph2[7]: Crc(12,12) | Cus MSB(0,12)
    let crc_lsb = extract_bits(eph2[7], 12, 12); // lower 12 bits of 24
    let crc_combined = (crc_msb << 12) | crc_lsb;
    d.crc = rescale_int(sign_extend(crc_combined, 24), -8);

    let cus_msb = extract_bits(eph2[7], 0, 12); // upper 12 bits of 21

    // eph2[8]: Cus(15,9) | Cuc MSB(0,15)
    let cus_lsb = extract_bits(eph2[8], 15, 9); // lower 9 bits of 21
    let cus_combined = (cus_msb << 9) | cus_lsb;
    d.cus = rescale_int(sign_extend(cus_combined, 21), -30);

    let cuc_msb = extract_bits(eph2[8], 0, 15); // upper 15 bits of 21

    // eph2[9]: Cuc(18,6)
    let cuc_lsb = extract_bits(eph2[9], 18, 6); // lower 6 bits of 21
    let cuc_combined = (cuc_msb << 6) | cuc_lsb;
    d.cuc = rescale_int(sign_extend(cuc_combined, 21), -30);

    // === Clock parameters (clock[0..4]) ===

    // clock[0]: toc/300(13,11) | af0 MSB(0,13)
    let toc_div300 = extract_bits(clock[0], 13, 11);
    d.toc = (toc_div300 as i32) * 300;

    let af0_msb = extract_bits(clock[0], 0, 13); // upper 13 bits of 25

    // clock[1]: af0(12,12) | af1 MSB(0,12)
    let af0_lsb = extract_bits(clock[1], 12, 12); // lower 12 bits of 25
    let af0_combined = (af0_msb << 12) | af0_lsb;
    d.af0 = rescale_int(sign_extend(af0_combined, 25), -34);

    let af1_msb = extract_bits(clock[1], 0, 12); // upper 12 bits of 22

    // clock[2]: af1(14,10) | af2(3,11)
    let af1_lsb = extract_bits(clock[2], 14, 10); // lower 10 bits of 22
    let af1_combined = (af1_msb << 10) | af1_lsb;
    d.af1 = rescale_int(sign_extend(af1_combined, 22), -50);

    let af2_raw = extract_bits(clock[2], 3, 11);
    d.af2 = rescale_int(sign_extend(af2_raw, 11), -66);

    // clock[3]: IODC(0,10)
    d.iodc = extract_bits(clock[3], 0, 10) as u16;

    d
}

impl DecodedBcnavEph {
    /// Compares decoded BeiDou B-CNAV ephemeris against the original `GpsEphemeris`.
    ///
    /// BeiDou uses different quantization scales than GPS/Galileo.
    /// The `axis` field is compared via `delta_A + reference_axis` vs `eph.axis`.
    pub fn compare_with_original(&self, eph: &GpsEphemeris) -> Vec<ParamDiff> {
        // Reconstruct axis from delta_A for comparison
        let ref_axis = if eph.flag == 3 { 27906100.0 } else { 42162200.0 };
        let decoded_axis = self.delta_A + ref_axis;

        vec![
            ParamDiff::new("M0", eph.M0, self.M0, PI * 2.0_f64.powi(-32) / 2.0),
            ParamDiff::new("delta_n", eph.delta_n, self.delta_n, PI * 2.0_f64.powi(-44) / 2.0),
            ParamDiff::new("delta_n_dot", eph.delta_n_dot, self.delta_n_dot, PI * 2.0_f64.powi(-57) / 2.0),
            ParamDiff::new("ecc", eph.ecc, self.ecc, 2.0_f64.powi(-34) / 2.0),
            ParamDiff::new("axis", eph.axis, decoded_axis, 2.0_f64.powi(-9) / 2.0),
            ParamDiff::new("axis_dot", eph.axis_dot, self.axis_dot, 2.0_f64.powi(-21) / 2.0),
            ParamDiff::new("omega0", eph.omega0, self.omega0, PI * 2.0_f64.powi(-32) / 2.0),
            ParamDiff::new("i0", eph.i0, self.i0, PI * 2.0_f64.powi(-32) / 2.0),
            ParamDiff::new("w", eph.w, self.w, PI * 2.0_f64.powi(-32) / 2.0),
            ParamDiff::new("omega_dot", eph.omega_dot, self.omega_dot, PI * 2.0_f64.powi(-44) / 2.0),
            ParamDiff::new("idot", eph.idot, self.idot, PI * 2.0_f64.powi(-44) / 2.0),
            ParamDiff::new("cuc", eph.cuc, self.cuc, 2.0_f64.powi(-30) / 2.0),
            ParamDiff::new("cus", eph.cus, self.cus, 2.0_f64.powi(-30) / 2.0),
            ParamDiff::new("crc", eph.crc, self.crc, 2.0_f64.powi(-8) / 2.0),
            ParamDiff::new("crs", eph.crs, self.crs, 2.0_f64.powi(-8) / 2.0),
            ParamDiff::new("cic", eph.cic, self.cic, 2.0_f64.powi(-30) / 2.0),
            ParamDiff::new("cis", eph.cis, self.cis, 2.0_f64.powi(-30) / 2.0),
            ParamDiff::new("af0", eph.af0, self.af0, 2.0_f64.powi(-34) / 2.0),
            ParamDiff::new("af1", eph.af1, self.af1, 2.0_f64.powi(-50) / 2.0),
            ParamDiff::new("af2", eph.af2, self.af2, 2.0_f64.powi(-66) / 2.0),
            ParamDiff::new("toe", eph.toe as f64, self.toe as f64, 150.0), // 300s resolution
            ParamDiff::new("toc", eph.toc as f64, self.toc as f64, 150.0), // 300s resolution
        ]
    }
}

// ===========================================================================
// 6. GPS LNAV Parity verifier
// ===========================================================================

/// Computes the 6-bit GPS LNAV parity for a 32-bit word.
///
/// The word is structured as:
/// - bit 31: D29* (previous word parity bit 29)
/// - bit 30: D30* (previous word parity bit 30)
/// - bits 29..6: data bits d1..d24
/// - bits 5..0: parity bits (ignored for computation)
///
/// This is a free-function version of `LNavBit::gps_get_parity`.
pub fn gps_get_parity(word: u32) -> u32 {
    let mut w = word >> 6; // remove bits 5..0
    let mut parity = 0u32;

    for i in 0..6 {
        parity ^= PARITY_TABLE[i][(w & 0xf) as usize] as u32;
        w >>= 4;
    }
    // add d29* and d30*
    if (w & 1) != 0 {
        parity ^= 0x15;
    }
    if (w & 2) != 0 {
        parity ^= 0x29;
    }

    parity
}

/// Verifies GPS LNAV parity for a 300-bit navigation subframe.
///
/// `nav_bits` contains 300 bits (10 words x 30 bits), MSB first per word.
/// Each element is 0 or 1 (as i32).
///
/// Returns a vector of `(word_index, parity_ok)` tuples for words 0..9.
///
/// For each word:
/// 1. Reconstructs the 30-bit word from `nav_bits`
/// 2. Gets D29* and D30* from the previous word (bits 28,29); for word 0 both are 0
/// 3. Assembles the 32-bit value: D29* at bit31, D30* at bit30, D1..D24 at bits 29..6, parity at bits 5..0
/// 4. If D30* is set, the data bits D1..D24 were XORed during encoding — un-XOR them before parity check
/// 5. Computes expected parity and compares with the received 6 parity bits
pub fn verify_lnav_parity(nav_bits: &[i32; 300]) -> Vec<(usize, bool)> {
    let mut results = Vec::with_capacity(10);

    for word_idx in 0..10 {
        let bit_offset = word_idx * 30;

        // Reconstruct the 30-bit word from nav_bits (MSB first)
        let mut word_30: u32 = 0;
        for i in 0..30 {
            word_30 = (word_30 << 1) | (nav_bits[bit_offset + i] as u32 & 1);
        }

        // Get D29* and D30* from previous word's last two bits
        let (d29_star, d30_star) = if word_idx == 0 {
            (0u32, 0u32)
        } else {
            let prev_offset = (word_idx - 1) * 30;
            let d29 = nav_bits[prev_offset + 28] as u32 & 1;
            let d30 = nav_bits[prev_offset + 29] as u32 & 1;
            (d29, d30)
        };

        // Assemble 32-bit word: D29*(31) | D30*(30) | D1..D24(29..6) | parity(5..0)
        let full_word = (d29_star << 31) | (d30_star << 30) | (word_30 & 0x3FFFFFFF);

        // The received parity bits (bits 5..0 of the 30-bit word)
        let received_parity = word_30 & 0x3F;

        // If D30* is set, data bits were XORed during encoding.
        // We need to un-XOR them before computing parity, BUT the parity algorithm
        // in gps_get_parity already accounts for D30* via the lookup table.
        // The encoder does: if D30* set, XOR d1..d24, then compute parity.
        // So the stored bits are already the XORed version.
        // The parity function takes the full word including D29*/D30* and computes
        // parity on the already-XORed data bits, which is what we have.
        let computed_parity = gps_get_parity(full_word);

        results.push((word_idx, computed_parity == received_parity));
    }

    results
}

// ===========================================================================
// 7. I-NAV CRC24Q verifier
// ===========================================================================

/// Computes CRC24Q over an array of 32-bit words.
///
/// This mirrors `crc24q_encode` from `crc24q.rs`.
fn crc24q_compute(bit_stream: &[u32], length_bits: usize) -> u32 {
    let byte_count = length_bits.div_ceil(32) * 4;
    let mut crc_result = 0u32;
    let mut data = if !bit_stream.is_empty() {
        bit_stream[0]
    } else {
        0
    };

    for i in 0..byte_count {
        crc_result =
            (crc_result << 8) ^ CRC24Q_TABLE[((data >> 24) ^ (crc_result >> 16)) as u8 as usize];
        data <<= 8;

        if (i & 3) == 3 && ((i >> 2) + 1) < bit_stream.len() {
            data = bit_stream[(i >> 2) + 1];
        }
    }

    crc_result & 0xFFFFFF
}

/// Verifies CRC24Q for Galileo I-NAV data.
///
/// # Parameters
/// - `data`: array of 32-bit words containing the data
/// - `data_len`: length of data in bits (not including CRC)
/// - `expected_crc`: the 24-bit CRC to verify against
///
/// # Returns
/// `true` if the computed CRC matches `expected_crc`.
pub fn verify_inav_crc24q(data: &[u32], data_len: usize, expected_crc: u32) -> bool {
    let computed = crc24q_compute(data, data_len);
    computed == (expected_crc & 0xFFFFFF)
}

// ===========================================================================
// 7. GPS CNAV decoder (MT10 + MT11 + clock/delay) — inverse of
//    CNavBit::compose_eph_words. The CNAV payload is identical on L2C and L5,
//    so this also verifies L5CNavBit once it produces the same words.
// ===========================================================================

const CNAV_A_REF: f64 = 26559710.0;
const CNAV_OMEGA_DOT_REF: f64 = -2.6e-9;

#[derive(Debug, Clone, Default)]
pub struct DecodedCnavEph {
    pub svid: u32,
    pub week: i32,
    pub toe: i32,
    pub toc: i32,
    pub axis: f64,
    pub axis_dot: f64,
    pub delta_n: f64,
    pub delta_n_dot: f64,
    pub M0: f64,
    pub ecc: f64,
    pub w: f64,
    pub omega0: f64,
    pub i0: f64,
    pub omega_dot: f64,
    pub idot: f64,
    pub cis: f64,
    pub cic: f64,
    pub crs: f64,
    pub crc: f64,
    pub cus: f64,
    pub cuc: f64,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub tgd: f64,
}

/// Reverses `CNavBit::compose_eph_words`. `eph_data` holds MT10 (index 0) and MT11
/// (index 1), `clock_data` is the MT30 clock block, `delay_data` the TGD/ISC block.
pub fn decode_cnav_ephemeris(
    eph_data: &[[u32; 9]; 2],
    clock_data: &[u32; 4],
    delay_data: &[u32; 3],
) -> DecodedCnavEph {
    let m10 = &eph_data[0];
    let m11 = &eph_data[1];

    let svid = extract_bits(m10[0], 6, 6);
    let week = extract_bits(m10[1], 1, 13) as i32;
    let toe = (extract_bits(m10[2], 3, 11) * 300) as i32;

    // delta_A (26-bit, 2^-9 m); axis = A_REF + delta_A
    let da = (extract_bits(m10[2], 0, 3) << 23) | extract_bits(m10[3], 9, 23);
    let axis = CNAV_A_REF + rescale_int(sign_extend(da, 26), -9);
    // axis_dot (25-bit, 2^-21)
    let adot = (extract_bits(m10[3], 0, 9) << 16) | extract_bits(m10[4], 16, 16);
    let axis_dot = rescale_int(sign_extend(adot, 25), -21);
    // delta_n (17-bit, 2^-44 semicircles)
    let dn = (extract_bits(m10[4], 0, 16) << 1) | extract_bits(m10[5], 31, 1);
    let delta_n = rescale_int(sign_extend(dn, 17), -44) * PI;
    // delta_n_dot (23-bit, 2^-57 semicircles)
    let dnd = extract_bits(m10[5], 8, 23);
    let delta_n_dot = rescale_int(sign_extend(dnd, 23), -57) * PI;
    // M0 (33-bit, 2^-32 semicircles)
    let m0 = sign_magnitude_33(
        extract_bits(m10[5], 7, 1),
        (extract_bits(m10[5], 0, 7) << 25) | extract_bits(m10[6], 7, 25),
    ) * 2.0_f64.powi(-32) * PI;
    // ecc (33-bit, 2^-34)
    let ecc = sign_magnitude_33(
        extract_bits(m10[6], 6, 1),
        (extract_bits(m10[6], 0, 6) << 26) | extract_bits(m10[7], 6, 26),
    ) * 2.0_f64.powi(-34);
    // w (33-bit, 2^-32 semicircles)
    let w = sign_magnitude_33(
        extract_bits(m10[7], 5, 1),
        (extract_bits(m10[7], 0, 5) << 27) | extract_bits(m10[8], 5, 27),
    ) * 2.0_f64.powi(-32) * PI;

    // MT11
    let omega0 = sign_magnitude_33(
        extract_bits(m11[1], 2, 1),
        (extract_bits(m11[1], 0, 2) << 30) | extract_bits(m11[2], 2, 30),
    ) * 2.0_f64.powi(-32) * PI;
    let i0 = sign_magnitude_33(
        extract_bits(m11[2], 1, 1),
        (extract_bits(m11[2], 0, 1) << 31) | extract_bits(m11[3], 1, 31),
    ) * 2.0_f64.powi(-32) * PI;
    // omega_dot (17-bit, 2^-44 semicircles, offset OMEGA_DOT_REF)
    let od = (extract_bits(m11[3], 0, 1) << 16) | extract_bits(m11[4], 16, 16);
    let omega_dot = (rescale_int(sign_extend(od, 17), -44) + CNAV_OMEGA_DOT_REF) * PI;
    // idot (15-bit, 2^-44 semicircles)
    let idot = rescale_int(sign_extend(extract_bits(m11[4], 1, 15), 15), -44) * PI;
    // cis (16-bit, 2^-30)
    let cis = rescale_int(
        sign_extend((extract_bits(m11[4], 0, 1) << 15) | extract_bits(m11[5], 17, 15), 16),
        -30,
    );
    // cic (16-bit, 2^-30)
    let cic = rescale_int(sign_extend(extract_bits(m11[5], 1, 16), 16), -30);
    // crs (24-bit, 2^-8)
    let crs = rescale_int(
        sign_extend((extract_bits(m11[5], 0, 1) << 23) | extract_bits(m11[6], 9, 23), 24),
        -8,
    );
    // crc (24-bit, 2^-8)
    let crc = rescale_int(
        sign_extend((extract_bits(m11[6], 0, 9) << 15) | extract_bits(m11[7], 17, 15), 24),
        -8,
    );
    // cus (21-bit, 2^-30)
    let cus = rescale_int(
        sign_extend((extract_bits(m11[7], 0, 17) << 4) | extract_bits(m11[8], 28, 4), 21),
        -30,
    );
    // cuc (21-bit, 2^-30)
    let cuc = rescale_int(sign_extend(extract_bits(m11[8], 7, 21), 21), -30);

    // Clock (MT30)
    let toc = (extract_bits(clock_data[1], 13, 11) * 300) as i32;
    let af0 = rescale_int(
        sign_extend((extract_bits(clock_data[1], 0, 13) << 13) | extract_bits(clock_data[2], 19, 13), 26),
        -35,
    );
    let af1 = rescale_int(
        sign_extend((extract_bits(clock_data[2], 0, 19) << 1) | extract_bits(clock_data[3], 31, 1), 20),
        -48,
    );
    let af2 = rescale_int(sign_extend(extract_bits(clock_data[3], 21, 10), 10), -60);

    // Delay: TGD (13-bit, 2^-35) at delay[0] bits 8..20
    let tgd = rescale_int(sign_extend(extract_bits(delay_data[0], 8, 13), 13), -35);

    DecodedCnavEph {
        svid, week, toe, toc, axis, axis_dot, delta_n, delta_n_dot,
        M0: m0, ecc, w, omega0, i0, omega_dot, idot,
        cis, cic, crs, crc, cus, cuc, af0, af1, af2, tgd,
    }
}

impl DecodedCnavEph {
    pub fn compare_with_original(&self, eph: &GpsEphemeris) -> Vec<ParamDiff> {
        let two = |p: i32| 2.0_f64.powi(p);
        vec![
            ParamDiff::new("M0", eph.M0, self.M0, PI * two(-32)),
            ParamDiff::new("ecc", eph.ecc, self.ecc, two(-34)),
            ParamDiff::new("axis", eph.axis, self.axis, two(-9)),
            ParamDiff::new("axis_dot", eph.axis_dot, self.axis_dot, two(-21)),
            ParamDiff::new("delta_n", eph.delta_n, self.delta_n, PI * two(-44)),
            ParamDiff::new("delta_n_dot", eph.delta_n_dot, self.delta_n_dot, PI * two(-57)),
            ParamDiff::new("omega0", eph.omega0, self.omega0, PI * two(-32)),
            ParamDiff::new("i0", eph.i0, self.i0, PI * two(-32)),
            ParamDiff::new("w", eph.w, self.w, PI * two(-32)),
            ParamDiff::new("omega_dot", eph.omega_dot, self.omega_dot, PI * two(-44)),
            ParamDiff::new("idot", eph.idot, self.idot, PI * two(-44)),
            ParamDiff::new("cuc", eph.cuc, self.cuc, two(-30)),
            ParamDiff::new("cus", eph.cus, self.cus, two(-30)),
            ParamDiff::new("crc", eph.crc, self.crc, two(-8)),
            ParamDiff::new("crs", eph.crs, self.crs, two(-8)),
            ParamDiff::new("cic", eph.cic, self.cic, two(-30)),
            ParamDiff::new("cis", eph.cis, self.cis, two(-30)),
            ParamDiff::new("af0", eph.af0, self.af0, two(-35)),
            ParamDiff::new("af1", eph.af1, self.af1, two(-48)),
            ParamDiff::new("af2", eph.af2, self.af2, two(-60)),
            ParamDiff::new("toe", eph.toe as f64, self.toe as f64, 150.0),
            ParamDiff::new("toc", eph.toc as f64, self.toc as f64, 150.0),
            ParamDiff::new("tgd", eph.tgd_ext[4], self.tgd, two(-35)),
        ]
    }
}

/// Decodes GPS L1C CNAV-2 ephemeris from subframe 2 (18 × 32-bit words), the inverse of
/// `CNav2Bit::compose_subframe2`. CNAV-2 carries the same parameters/scales as L2C CNAV
/// (IS-GPS-800), so it returns `DecodedCnavEph` to reuse the tolerances/compare. The 33-bit
/// angle/ecc fields use the same sign-bit + 32-bit packing as L2C (via `sign_magnitude_33`),
/// ΔA references A_REF = 26,559,710 m, and Ω̇ is offset by OMEGA_DOT_REF = -2.6e-9 sc/s.
pub fn decode_cnav2_ephemeris(sf2: &[u32; 18]) -> DecodedCnavEph {
    let mut d = DecodedCnavEph::default();

    d.toe = (extract_bits(sf2[1], 15, 11) as i32) * 300;
    d.toc = d.toe; // CNAV-2 uses a single reference time (t_oc = t_oe)

    // delta_A (26-bit, 2^-9): axis = A_REF + dA
    let da = (extract_bits(sf2[1], 0, 15) << 11) | extract_bits(sf2[2], 21, 11);
    d.axis = CNAV_A_REF + rescale_int(sign_extend(da, 26), -9);

    // axis_dot (25-bit, 2^-21)
    let adot = (extract_bits(sf2[2], 0, 21) << 4) | extract_bits(sf2[3], 28, 4);
    d.axis_dot = rescale_int(sign_extend(adot, 25), -21);

    // delta_n (17-bit, 2^-44 sc) and delta_n_dot (23-bit, 2^-57 sc)
    d.delta_n = rescale_int(sign_extend(extract_bits(sf2[3], 11, 17), 17), -44) * PI;
    let dnd = (extract_bits(sf2[3], 0, 11) << 12) | extract_bits(sf2[4], 20, 12);
    d.delta_n_dot = rescale_int(sign_extend(dnd, 23), -57) * PI;

    // M0 (33-bit, 2^-32 sc)
    let m0_u = (extract_bits(sf2[4], 0, 19) << 13) | extract_bits(sf2[5], 19, 13);
    d.M0 = sign_magnitude_33(extract_bits(sf2[4], 19, 1), m0_u) * 2.0_f64.powi(-32) * PI;

    // ecc (33-bit, 2^-34)
    let ecc_u = (extract_bits(sf2[5], 0, 18) << 14) | extract_bits(sf2[6], 18, 14);
    d.ecc = sign_magnitude_33(extract_bits(sf2[5], 18, 1), ecc_u) * 2.0_f64.powi(-34);

    // w (33-bit, 2^-32 sc)
    let w_u = (extract_bits(sf2[6], 0, 17) << 15) | extract_bits(sf2[7], 17, 15);
    d.w = sign_magnitude_33(extract_bits(sf2[6], 17, 1), w_u) * 2.0_f64.powi(-32) * PI;

    // omega0 (33-bit, 2^-32 sc)
    let om_u = (extract_bits(sf2[7], 0, 16) << 16) | extract_bits(sf2[8], 16, 16);
    d.omega0 = sign_magnitude_33(extract_bits(sf2[7], 16, 1), om_u) * 2.0_f64.powi(-32) * PI;

    // i0 (33-bit, 2^-32 sc)
    let i0_u = (extract_bits(sf2[8], 0, 15) << 17) | extract_bits(sf2[9], 15, 17);
    d.i0 = sign_magnitude_33(extract_bits(sf2[8], 15, 1), i0_u) * 2.0_f64.powi(-32) * PI;

    // omega_dot (17-bit, 2^-44 sc, offset OMEGA_DOT_REF)
    let od = (extract_bits(sf2[9], 0, 15) << 2) | extract_bits(sf2[10], 30, 2);
    d.omega_dot = (rescale_int(sign_extend(od, 17), -44) + CNAV_OMEGA_DOT_REF) * PI;

    // idot (15-bit, 2^-44 sc)
    d.idot = rescale_int(sign_extend(extract_bits(sf2[10], 15, 15), 15), -44) * PI;

    // Harmonic corrections
    let cis = (extract_bits(sf2[10], 0, 15) << 1) | extract_bits(sf2[11], 31, 1);
    d.cis = rescale_int(sign_extend(cis, 16), -30);
    d.cic = rescale_int(sign_extend(extract_bits(sf2[11], 15, 16), 16), -30);
    let crs = (extract_bits(sf2[11], 0, 15) << 9) | extract_bits(sf2[12], 23, 9);
    d.crs = rescale_int(sign_extend(crs, 24), -8);
    let crc = (extract_bits(sf2[12], 0, 23) << 1) | extract_bits(sf2[13], 31, 1);
    d.crc = rescale_int(sign_extend(crc, 24), -8);
    d.cus = rescale_int(sign_extend(extract_bits(sf2[13], 10, 21), 21), -30);
    let cuc = (extract_bits(sf2[13], 0, 10) << 11) | extract_bits(sf2[14], 21, 11);
    d.cuc = rescale_int(sign_extend(cuc, 21), -30);

    // Clock (af0 26-bit 2^-35, af1 20-bit 2^-48, af2 10-bit 2^-60) and TGD (13-bit 2^-35)
    let af0 = (extract_bits(sf2[14], 0, 10) << 16) | extract_bits(sf2[15], 16, 16);
    d.af0 = rescale_int(sign_extend(af0, 26), -35);
    let af1 = (extract_bits(sf2[15], 0, 16) << 4) | extract_bits(sf2[16], 28, 4);
    d.af1 = rescale_int(sign_extend(af1, 20), -48);
    d.af2 = rescale_int(sign_extend(extract_bits(sf2[16], 18, 10), 10), -60);
    d.tgd = rescale_int(sign_extend(extract_bits(sf2[16], 5, 13), 13), -35);

    d.week = extract_bits(sf2[17], 2, 8) as i32;

    d
}

#[cfg(test)]
mod cnav_roundtrip_tests {
    use super::*;
    use crate::cnavbit::CNavBit;
    use crate::types::GpsEphemeris;

    fn sample_eph() -> GpsEphemeris {
        let mut e = GpsEphemeris::default();
        e.svid = 7;
        e.valid = 1;
        e.week = 2369;
        e.health = 0;
        e.ura = 4;
        e.top = 388800;
        e.toe = 388800;
        e.toc = 388800;
        e.axis = 26560218.0;
        e.axis_dot = -0.0432;
        e.delta_n = 4.51e-9;
        e.delta_n_dot = 1.2e-14;
        e.M0 = 0.5123;
        e.ecc = 0.0089;
        e.w = 0.3271;
        e.omega0 = -1.234;
        e.i0 = 0.9612;
        e.omega_dot = -8.13e-9;
        e.idot = 1.05e-10;
        e.cuc = 1.3e-6;
        e.cus = 5.7e-6;
        e.crc = 215.4;
        e.crs = -48.3;
        e.cic = -1.1e-7;
        e.cis = 8.2e-8;
        e.af0 = -1.523e-4;
        e.af1 = 2.81e-12;
        e.af2 = 0.0;
        e.tgd_ext[4] = -5.12e-9;
        e
    }

    /// Round-trips the verified L2C CNAV encoder: GpsEphemeris -> CNavBit words ->
    /// decode -> GpsEphemeris. Confirms both the encoder and the new decoder are correct.
    #[test]
    fn cnav_eph_round_trip_matches_l2c_encoder() {
        let eph = sample_eph();
        let mut ed = [[0u32; 9]; 2];
        let mut cd = [0u32; 4];
        let mut dd = [0u32; 3];
        CNavBit::compose_eph_words(&eph, &mut ed, &mut cd, &mut dd);
        let dec = decode_cnav_ephemeris(&ed, &cd, &dd);
        let bad: Vec<_> = dec
            .compare_with_original(&eph)
            .into_iter()
            .filter(|d| !d.ok)
            .map(|d| format!("{}: orig={:.6e} dec={:.6e} (>{:.1e})", d.name, d.original, d.decoded, d.tolerance))
            .collect();
        assert!(bad.is_empty(), "CNAV round-trip mismatches:\n  {}", bad.join("\n  "));
    }

    /// Round-trips the GPS L1C CNAV-2 encoder: GpsEphemeris -> CNav2Bit subframe 2 -> decode ->
    /// GpsEphemeris. L1C and L2C share parameters/scales, so the same sample/compare apply.
    #[test]
    fn cnav2_eph_round_trips_via_subframe2_decoder() {
        use crate::cnav2bit::CNav2Bit;
        let eph = sample_eph(); // svid 7
        let svid: i32 = 7;
        let mut cnav2 = CNav2Bit::new();
        assert_ne!(cnav2.set_ephemeris(svid, &eph), 0, "set_ephemeris failed");
        let sf2 = &cnav2.subframe2[(svid - 1) as usize];
        let dec = decode_cnav2_ephemeris(sf2);
        let bad: Vec<_> = dec
            .compare_with_original(&eph)
            .into_iter()
            .filter(|d| !d.ok)
            .map(|d| format!("{}: orig={:.6e} dec={:.6e} (>{:.1e})", d.name, d.original, d.decoded, d.tolerance))
            .collect();
        assert!(bad.is_empty(), "CNAV-2 round-trip mismatches:\n  {}", bad.join("\n  "));
    }
}

// ===========================================================================
// 8. GLONASS G-NAV ephemeris decoder (strings 1-4) — inverse of
//    GNavBit::ComposeStringEph, using the official ICD (Edition 5.1) bit layout
//    (Table 4.6) and sign-magnitude convention (Table 4.5, remark 2).
// ===========================================================================

/// Extracts the field at ICD bit positions `icd_lo..=icd_hi` (Table 4.6, bit 85 first) from
/// one 3-word GLONASS string. Transmission index = 85 - position.
pub fn glo_get_field(string: &[u32; 3], icd_lo: u32, icd_hi: u32) -> u32 {
    let mut value = 0u32;
    for k in 0..=(icd_hi - icd_lo) {
        let j = 85 - (icd_lo + k);
        let bit = (string[(j / 32) as usize] >> (31 - (j % 32))) & 1;
        value |= bit << k;
    }
    value
}

/// Decodes a GLONASS sign-magnitude field (sign in bit `width-1`) to its physical value.
pub fn glo_sign_mag_decode(field: u32, width: u32, scale_pow2: i32) -> f64 {
    let sign = (field >> (width - 1)) & 1;
    let mag = field & ((1u32 << (width - 1)) - 1);
    let v = mag as f64 * 2.0_f64.powi(scale_pow2);
    if sign != 0 {
        -v
    } else {
        v
    }
}

#[derive(Debug, Clone)]
pub struct DecodedGnavEph {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub ax: f64,
    pub ay: f64,
    pub az: f64,
    pub gamma: f64,
    pub tn: f64,
    pub dtn: f64,
    pub tk: u32,
    pub tb_seconds: u32,
}

/// Reverses `GNavBit::ComposeStringEph`. `strings` are strings 1-4 ([[u32;3];4]).
/// Coordinates/velocities/accelerations are returned in SI (metres) — the ICD encodes km.
pub fn decode_gnav_eph(strings: &[[u32; 3]; 4]) -> DecodedGnavEph {
    let (s1, s2, s3, s4) = (&strings[0], &strings[1], &strings[2], &strings[3]);
    // coordinates: 27-bit sign-magnitude, 2^-11 km LSB -> *1000 for metres
    let x = glo_sign_mag_decode(glo_get_field(s1, 9, 35), 27, -11) * 1000.0;
    let y = glo_sign_mag_decode(glo_get_field(s2, 9, 35), 27, -11) * 1000.0;
    let z = glo_sign_mag_decode(glo_get_field(s3, 9, 35), 27, -11) * 1000.0;
    // velocities: 24-bit, 2^-20 km/s
    let vx = glo_sign_mag_decode(glo_get_field(s1, 41, 64), 24, -20) * 1000.0;
    let vy = glo_sign_mag_decode(glo_get_field(s2, 41, 64), 24, -20) * 1000.0;
    let vz = glo_sign_mag_decode(glo_get_field(s3, 41, 64), 24, -20) * 1000.0;
    // accelerations: 5-bit, 2^-30 km/s^2
    let ax = glo_sign_mag_decode(glo_get_field(s1, 36, 40), 5, -30) * 1000.0;
    let ay = glo_sign_mag_decode(glo_get_field(s2, 36, 40), 5, -30) * 1000.0;
    let az = glo_sign_mag_decode(glo_get_field(s3, 36, 40), 5, -30) * 1000.0;
    let gamma = glo_sign_mag_decode(glo_get_field(s3, 69, 79), 11, -40);
    let tn = glo_sign_mag_decode(glo_get_field(s4, 59, 80), 22, -30);
    let dtn = glo_sign_mag_decode(glo_get_field(s4, 54, 58), 5, -30);
    let tk = glo_get_field(s1, 65, 76);
    let tb_seconds = glo_get_field(s2, 70, 76) * 900;
    DecodedGnavEph { x, y, z, vx, vy, vz, ax, ay, az, gamma, tn, dtn, tk, tb_seconds }
}

impl DecodedGnavEph {
    pub fn compare_with_original(&self, eph: &crate::types::GlonassEphemeris) -> Vec<ParamDiff> {
        let two = |p: i32| 2.0_f64.powi(p);
        vec![
            // tolerances are half an LSB in metres (km LSB * 1000)
            ParamDiff::new("x", eph.x, self.x, two(-11) * 1000.0),
            ParamDiff::new("y", eph.y, self.y, two(-11) * 1000.0),
            ParamDiff::new("z", eph.z, self.z, two(-11) * 1000.0),
            ParamDiff::new("vx", eph.vx, self.vx, two(-20) * 1000.0),
            ParamDiff::new("vy", eph.vy, self.vy, two(-20) * 1000.0),
            ParamDiff::new("vz", eph.vz, self.vz, two(-20) * 1000.0),
            ParamDiff::new("ax", eph.ax, self.ax, two(-30) * 1000.0),
            ParamDiff::new("ay", eph.ay, self.ay, two(-30) * 1000.0),
            ParamDiff::new("az", eph.az, self.az, two(-30) * 1000.0),
            ParamDiff::new("gamma", eph.gamma, self.gamma, two(-40)),
            ParamDiff::new("tn", eph.tn, self.tn, two(-30)),
            ParamDiff::new("dtn", eph.dtn, self.dtn, two(-30)),
            ParamDiff::new("tk", eph.tk as f64, self.tk as f64, 0.5),
            ParamDiff::new("tb", eph.tb as f64, self.tb_seconds as f64, 450.0),
        ]
    }
}

#[cfg(test)]
mod gnav_roundtrip_tests {
    use super::*;
    use crate::gnavbit::GNavBit;
    use crate::types::GlonassEphemeris;

    fn sample_glo_eph() -> GlonassEphemeris {
        let mut e = GlonassEphemeris::new();
        e.flag = 1;
        e.x = 2.5e7;
        e.y = -1.2e7;
        e.z = 8.0e6;
        e.vx = 1503.125;
        e.vy = -3201.5;
        e.vz = 2800.0;
        e.ax = 2.8e-6;
        e.ay = -1.9e-6;
        e.az = 0.0;
        e.gamma = 1.5e-11;
        e.tn = -2.5e-4;
        e.dtn = 1.86e-9;
        e.tk = (3 << 7) | (15 << 1); // 03:15:00
        e.tb = 5400; // seconds (90 min)
        e
    }

    /// Audit H15/H16/H17 + units: round-trips the GLONASS ephemeris encoder against the
    /// official ICD (Edition 5.1) bit layout. Catches the old magnitude-only coordinates,
    /// two's-complement velocity/accel, m/km unit error and the tk/30 corruption.
    #[test]
    fn gnav_eph_round_trip_matches_icd() {
        let eph = sample_glo_eph();
        let mut gnav = GNavBit::new();
        assert_ne!(gnav.set_glonass_ephemeris(1, &eph), 0);
        let dec = decode_gnav_eph(&gnav.StringEph[0]);
        let bad: Vec<_> = dec
            .compare_with_original(&eph)
            .into_iter()
            .filter(|d| !d.ok)
            .map(|d| format!("{}: orig={:.6e} dec={:.6e} (>{:.1e})", d.name, d.original, d.decoded, d.tolerance))
            .collect();
        assert!(bad.is_empty(), "GLONASS round-trip mismatches:\n  {}", bad.join("\n  "));
    }
}

#[cfg(test)]
mod extract_bits_tests {
    use super::extract_bits;

    #[test]
    fn extract_bits_handles_full_width() {
        // length == 32 must not trigger `1u32 << 32` (UB: debug panic / release wraparound).
        assert_eq!(extract_bits(0xFFFF_FFFF, 0, 32), 0xFFFF_FFFF);
        assert_eq!(extract_bits(0xDEAD_BEEF, 0, 32), 0xDEAD_BEEF);
        // Narrow extracts still behave.
        assert_eq!(extract_bits(0b1011_0000, 4, 4), 0b1011);
        assert_eq!(extract_bits(0xFF, 0, 8), 0xFF);
    }
}

#[cfg(test)]
mod fnav_roundtrip_tests {
    use super::decode_fnav_ephemeris;
    use crate::fnavbit::FNavBit;
    use crate::types::GpsEphemeris;

    /// Galileo F/NAV (E5a) ephemeris must round-trip through the page/word encoder. This is the
    /// first content-level check of fnavbit beyond frame-assembly smoke tests — it confirms the
    /// bit packing AND the convolutional-encoder-adjacent scales after the C1 G1/G2 fix.
    #[test]
    fn fnav_eph_round_trips_via_decoder() {
        let mut eph = GpsEphemeris::default();
        eph.valid = 1;
        eph.iodc = 42;
        eph.sqrtA = 5440.6;
        eph.ecc = 1.3e-4;
        eph.M0 = -0.85;
        eph.omega0 = 1.7;
        eph.i0 = 0.96;
        eph.w = -2.1;
        eph.omega_dot = -5.3e-9;
        eph.idot = 1.1e-10;
        eph.delta_n = 2.9e-9;
        eph.cuc = -1.0e-6;
        eph.cus = 5.0e-6;
        eph.crc = 150.0;
        eph.crs = -30.0;
        eph.cic = -1.5e-8;
        eph.cis = 8.0e-8;
        eph.af0 = 1.2e-4;
        eph.af1 = -3.0e-12;
        eph.af2 = 0.0;
        eph.tgd = 2.5e-9; // BGD E5a/E1
        eph.tgd2 = 0.0; // F/NAV (E5a) carries no E5b BGD
        eph.toe = 345600; // multiple of 60 (F/NAV stores toe/60)
        eph.toc = 345600;

        let svid = 7;
        let mut fnav = FNavBit::new();
        assert_ne!(fnav.set_ephemeris(svid, &eph), 0, "set_ephemeris failed");
        let pages = &fnav.gal_eph_data[(svid - 1) as usize];
        let dec = decode_fnav_ephemeris(pages);

        let bad: Vec<_> = dec
            .compare_with_original(&eph)
            .into_iter()
            .filter(|d| !d.ok)
            .map(|d| format!("{}: orig={:.6e} dec={:.6e}", d.name, d.original, d.decoded))
            .collect();
        assert!(bad.is_empty(), "F/NAV round-trip mismatches:\n  {}", bad.join("\n  "));
    }
}

// ===========================================================================
// 9. BeiDou D1 (B1I/B2I/B3I legacy, MEO/IGSO) ephemeris decoder — inverse of
//    D1D2NavBit::compose_bds_stream123 (BDS-SIS-ICD-B1I). Each of the 27 words
//    holds 22 information bits (bits 0..21).
// ===========================================================================

#[derive(Debug, Default, Clone)]
pub struct DecodedD1Eph {
    pub toc: i32,
    pub toe: i32,
    pub sqrtA: f64,
    pub ecc: f64,
    pub M0: f64,
    pub w: f64,
    pub omega0: f64,
    pub i0: f64,
    pub omega_dot: f64,
    pub idot: f64,
    pub delta_n: f64,
    pub cuc: f64,
    pub cus: f64,
    pub crc: f64,
    pub crs: f64,
    pub cic: f64,
    pub cis: f64,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub tgd: f64,
    pub tgd2: f64,
}

/// Decodes BeiDou D1 ephemeris from the 27-word subframe-1..3 stream. TGD1/TGD2 are 10-bit in
/// units of 0.1 ns (×1e-10 s); toe/toc are 17-bit at 2^3 s; angles are semicircles × 2^scale.
pub fn decode_d1_ephemeris(s: &[u32; 27]) -> DecodedD1Eph {
    let mut d = DecodedD1Eph::default();

    // --- Subframe 1: toc, TGD1/2, af0/af1/af2 ---
    let toc_div8 = (extract_bits(s[1], 0, 9) << 8) | extract_bits(s[2], 14, 8);
    d.toc = (toc_div8 as i32) << 3;
    d.tgd = sign_extend(extract_bits(s[2], 4, 10), 10) as f64 * 1e-10;
    let tgd2 = (extract_bits(s[2], 0, 4) << 6) | extract_bits(s[3], 16, 6);
    d.tgd2 = sign_extend(tgd2, 10) as f64 * 1e-10;
    d.af2 = rescale_int(sign_extend(extract_bits(s[6], 7, 11), 11), -66);
    let af0 = (extract_bits(s[6], 0, 7) << 17) | extract_bits(s[7], 5, 17);
    d.af0 = rescale_int(sign_extend(af0, 24), -33);
    let af1 = (extract_bits(s[7], 0, 5) << 17) | extract_bits(s[8], 5, 17);
    d.af1 = rescale_int(sign_extend(af1, 22), -50);

    // --- Subframe 2: delta_n, Cuc, M0, ecc, Cus, Crc, Crs, sqrtA, toe[19:18] ---
    let dn = (extract_bits(s[9], 0, 10) << 6) | extract_bits(s[10], 16, 6);
    d.delta_n = rescale_int(sign_extend(dn, 16), -43) * PI;
    let cuc = (extract_bits(s[10], 0, 16) << 2) | extract_bits(s[11], 20, 2);
    d.cuc = rescale_int(sign_extend(cuc, 18), -31);
    let m0 = (extract_bits(s[11], 0, 20) << 12) | extract_bits(s[12], 10, 12);
    d.M0 = rescale_int(sign_extend(m0, 32), -31) * PI;
    let ecc = (extract_bits(s[12], 0, 10) << 22) | extract_bits(s[13], 0, 22);
    d.ecc = rescale_uint(ecc, -33);
    d.cus = rescale_int(sign_extend(extract_bits(s[14], 4, 18), 18), -31);
    let crc = (extract_bits(s[14], 0, 4) << 14) | extract_bits(s[15], 8, 14);
    d.crc = rescale_int(sign_extend(crc, 18), -6);
    let crs = (extract_bits(s[15], 0, 8) << 10) | extract_bits(s[16], 12, 10);
    d.crs = rescale_int(sign_extend(crs, 18), -6);
    let sqrta = (extract_bits(s[16], 0, 12) << 20) | extract_bits(s[17], 2, 20);
    d.sqrtA = rescale_uint(sqrta, -19);

    // --- toe (17 significant bits at 2^3 s) split across SF2/SF3 ---
    let toe_hi = extract_bits(s[17], 0, 2); // toe[19:18]
    let toe_mid = extract_bits(s[18], 0, 10); // toe[17:8]
    let toe_lo = extract_bits(s[19], 17, 5); // toe[7:3]
    d.toe = ((toe_hi << 18) | (toe_mid << 8) | (toe_lo << 3)) as i32;

    // --- Subframe 3: i0, Cic, omega_dot, Cis, idot, omega0, w ---
    let i0 = (extract_bits(s[19], 0, 17) << 15) | extract_bits(s[20], 7, 15);
    d.i0 = rescale_int(sign_extend(i0, 32), -31) * PI;
    let cic = (extract_bits(s[20], 0, 7) << 11) | extract_bits(s[21], 11, 11);
    d.cic = rescale_int(sign_extend(cic, 18), -31);
    let od = (extract_bits(s[21], 0, 11) << 13) | extract_bits(s[22], 9, 13);
    d.omega_dot = rescale_int(sign_extend(od, 24), -43) * PI;
    let cis = (extract_bits(s[22], 0, 9) << 9) | extract_bits(s[23], 13, 9);
    d.cis = rescale_int(sign_extend(cis, 18), -31);
    let idot = (extract_bits(s[23], 0, 13) << 1) | extract_bits(s[24], 21, 1);
    d.idot = rescale_int(sign_extend(idot, 14), -43) * PI;
    let om0 = (extract_bits(s[24], 0, 21) << 11) | extract_bits(s[25], 11, 11);
    d.omega0 = rescale_int(sign_extend(om0, 32), -31) * PI;
    let w = (extract_bits(s[25], 0, 11) << 21) | extract_bits(s[26], 1, 21);
    d.w = rescale_int(sign_extend(w, 32), -31) * PI;

    d
}

impl DecodedD1Eph {
    /// Compares decoded BeiDou D1 ephemeris against the original `GpsEphemeris` (tolerance = LSB).
    pub fn compare_with_original(&self, eph: &GpsEphemeris) -> Vec<ParamDiff> {
        let two = |e: i32| 2.0_f64.powi(e);
        vec![
            ParamDiff::new("sqrtA", eph.sqrtA, self.sqrtA, two(-19)),
            ParamDiff::new("ecc", eph.ecc, self.ecc, two(-33)),
            ParamDiff::new("M0", eph.M0, self.M0, PI * two(-31)),
            ParamDiff::new("w", eph.w, self.w, PI * two(-31)),
            ParamDiff::new("omega0", eph.omega0, self.omega0, PI * two(-31)),
            ParamDiff::new("i0", eph.i0, self.i0, PI * two(-31)),
            ParamDiff::new("omega_dot", eph.omega_dot, self.omega_dot, PI * two(-43)),
            ParamDiff::new("idot", eph.idot, self.idot, PI * two(-43)),
            ParamDiff::new("delta_n", eph.delta_n, self.delta_n, PI * two(-43)),
            ParamDiff::new("cuc", eph.cuc, self.cuc, two(-31)),
            ParamDiff::new("cus", eph.cus, self.cus, two(-31)),
            ParamDiff::new("crc", eph.crc, self.crc, two(-6)),
            ParamDiff::new("crs", eph.crs, self.crs, two(-6)),
            ParamDiff::new("cic", eph.cic, self.cic, two(-31)),
            ParamDiff::new("cis", eph.cis, self.cis, two(-31)),
            ParamDiff::new("af0", eph.af0, self.af0, two(-33)),
            ParamDiff::new("af1", eph.af1, self.af1, two(-50)),
            ParamDiff::new("af2", eph.af2, self.af2, two(-66)),
            ParamDiff::new("tgd", eph.tgd, self.tgd, 1e-10),
            ParamDiff::new("tgd2", eph.tgd2, self.tgd2, 1e-10),
            ParamDiff::new("toe", eph.toe as f64, self.toe as f64, 8.0),
            ParamDiff::new("toc", eph.toc as f64, self.toc as f64, 8.0),
        ]
    }
}

#[cfg(test)]
mod d1d2_roundtrip_tests {
    use super::decode_d1_ephemeris;
    use crate::d1d2navbit::D1D2NavBit;
    use crate::types::GpsEphemeris;

    /// BeiDou D1 (MEO/IGSO) ephemeris must round-trip through the subframe encoder. First
    /// content-level check of d1d2navbit — it caught the unscale_double inverted-exponent bug.
    #[test]
    fn d1_eph_round_trips_via_decoder() {
        let mut eph = GpsEphemeris::default();
        eph.valid = 1;
        eph.week = 800;
        eph.iodc = 3;
        eph.ura = 2;
        eph.iode = 7;
        eph.sqrtA = 5282.6; // BDS MEO A ≈ 27,906,100 m
        eph.ecc = 0.003;
        eph.M0 = 0.6;
        eph.w = -1.5;
        eph.omega0 = 2.0;
        eph.i0 = 0.95;
        eph.omega_dot = -6.5e-9;
        eph.idot = 1.5e-10;
        eph.delta_n = 4.0e-9;
        eph.cuc = -2.0e-6;
        eph.cus = 8.0e-6;
        eph.crc = 200.0;
        eph.crs = -60.0;
        eph.cic = -4.0e-8;
        eph.cis = 9.0e-8;
        eph.af0 = -1.5e-4;
        eph.af1 = 2.0e-12;
        eph.af2 = 0.0;
        eph.tgd = 3.0e-9;
        eph.tgd2 = -2.0e-9;
        eph.toe = 345600; // multiple of 8
        eph.toc = 345600;

        let svid = 10; // MEO/IGSO range (6..58) -> D1
        let mut d1 = D1D2NavBit::new();
        assert_ne!(d1.set_ephemeris(svid, &eph), 0, "set_ephemeris failed");
        let stream = &d1.bds_stream123[(svid - 6) as usize];
        let dec = decode_d1_ephemeris(stream);
        let bad: Vec<_> = dec
            .compare_with_original(&eph)
            .into_iter()
            .filter(|d| !d.ok)
            .map(|d| format!("{}: orig={:.6e} dec={:.6e}", d.name, d.original, d.decoded))
            .collect();
        assert!(bad.is_empty(), "D1 round-trip mismatches:\n  {}", bad.join("\n  "));
    }
}
