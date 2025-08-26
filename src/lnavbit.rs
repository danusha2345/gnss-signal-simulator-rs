//----------------------------------------------------------------------
// lnavbit.rs:
//   Implementation of navigation bit synthesis class for LNAV
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use std::f64::consts::PI;

// Import types from other modules
use crate::{GpsEphemeris, GpsAlmanac, IonoParam, UtcParam};
use crate::types::GnssTime;
use crate::constants::*;

// Parity table for GPS LNAV
const PARITY_TABLE: [[u8; 16]; 6] = [
    [0x00, 0x13, 0x25, 0x36, 0x0B, 0x18, 0x2E, 0x3D, 0x16, 0x05, 0x33, 0x20, 0x1D, 0x0E, 0x38, 0x2B],
    [0x00, 0x2C, 0x19, 0x35, 0x32, 0x1E, 0x2B, 0x07, 0x26, 0x0A, 0x3F, 0x13, 0x14, 0x38, 0x0D, 0x21],
    [0x00, 0x0E, 0x1F, 0x11, 0x3E, 0x30, 0x21, 0x2F, 0x3D, 0x33, 0x22, 0x2C, 0x03, 0x0D, 0x1C, 0x12],
    [0x00, 0x38, 0x31, 0x09, 0x23, 0x1B, 0x12, 0x2A, 0x07, 0x3F, 0x36, 0x0E, 0x24, 0x1C, 0x15, 0x2D],
    [0x00, 0x0D, 0x1A, 0x17, 0x37, 0x3A, 0x2D, 0x20, 0x2F, 0x22, 0x35, 0x38, 0x18, 0x15, 0x02, 0x0F],
    [0x00, 0x1C, 0x3B, 0x27, 0x34, 0x28, 0x0F, 0x13, 0x2A, 0x36, 0x11, 0x0D, 0x1E, 0x02, 0x25, 0x39],
];

const PARITY_ADJUST: [u32; 4] = [0x00, 0xa5, 0xf6, 0x53];

const PAGE_ID: [[i32; 25]; 2] = [
    [57, 25, 26, 27, 28, 57, 29, 30, 31, 32, 57, 62, 52, 53, 54, 57, 55, 56, 58, 59, 57, 60, 61, 62, 63],
    [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 51],
];

// Main LNavBit structure
#[derive(Clone)]
pub struct LNavBit {
    gps_stream123: [[[u32; 8]; 3]; 32], // 32 satellites, 3 subframes, 8 words each
    gps_stream45: [[[u32; 8]; 25]; 2],  // 2 subframes (4&5), 25 pages, 8 words each
}

impl LNavBit {
    pub fn new() -> Self {
        let mut lnav_bit = LNavBit {
            gps_stream123: [[[0xaa; 8]; 3]; 32],
            gps_stream45: [[[0x55; 8]; 25]; 2],
        };

        // Assign page id
        for i in 0..25 {
            lnav_bit.gps_stream45[0][i][0] = (lnav_bit.gps_stream45[0][i][0] & !0x3f0000) | ((PAGE_ID[0][i] as u32) << 16);
        }
        for i in 0..25 {
            lnav_bit.gps_stream45[1][i][0] = (lnav_bit.gps_stream45[1][i][0] & !0x3f0000) | ((PAGE_ID[1][i] as u32) << 16);
        }

        lnav_bit
    }

    pub fn get_frame_data(&self, mut start_time: GnssTime, svid: i32, _param: i32, nav_bits: &mut [i32; 300]) -> i32 {
        // First determine the current TOW and subframe number
        start_time.Week += start_time.MilliSeconds / 604800000;
        start_time.MilliSeconds %= 604800000;
        let tow = start_time.MilliSeconds / 6000;
        let subframe = (tow % 5) + 1;
        
        let stream = if subframe > 3 {
            // subframe 4/5, further determine page number
            let page = (tow / 5) % 25;
            &self.gps_stream45[(subframe - 4) as usize][page as usize]
        } else if svid >= 1 && svid <= 32 {
            &self.gps_stream123[(svid - 1) as usize][(subframe - 1) as usize]
        } else {
            return 1;
        };

        let tlm_word = 0x8b << 22; // set all TLM message to 0 and let ISF=0
        let mut tow = tow + 1; // TOW is the count of NEXT subframe
        if tow >= 100800 {
            tow = 0;
        }
        let how_word = (tow << 13) + (subframe << 8); // set HOW word

        // Generate bit stream of TLM, this is WORD 1
        let mut cur_word = tlm_word | self.gps_get_parity(tlm_word); // D29* and D30* are 00 for TLM
        self.assign_bits(cur_word, 30, &mut nav_bits[0..30]);

        // Generate bit stream of HOW, this is WORD 2
        cur_word <<= 30; // put D29* and D30* into bit31 and bit30
        cur_word |= how_word as u32; // fill in d1 to d24 into bit29 to bit6
        if (cur_word & 0x40000000) != 0 {
            // convert d1 to d24 into D1 to D24
            cur_word ^= 0x3fffffc0;
        }
        cur_word |= self.gps_get_parity(cur_word); // add parity check
        cur_word ^= PARITY_ADJUST[(cur_word & 3) as usize]; // adjust d23 and d24 to ensure last two bit of parity to be 00
        self.assign_bits(cur_word, 30, &mut nav_bits[30..60]);

        // WORD 3 to WORD 10
        for i in 0..8 {
            cur_word <<= 30; // put D29* and D30* into bit31 and bit30
            cur_word |= ((stream[i] & 0xffffff) << 6); // fill in d1 to d24 into bit29 to bit6
            if subframe == 1 && i == 0 {
                // WORD3 of subframe 1, put in week number from bit 29~20
                cur_word |= ((start_time.Week as u32 & 0x3ff) << 20);
            }
            if (cur_word & 0x40000000) != 0 {
                // convert d1 to d24 into D1 to D24
                cur_word ^= 0x3fffffc0;
            }
            cur_word |= self.gps_get_parity(cur_word); // add parity check
            if i == 7 {
                cur_word ^= PARITY_ADJUST[(cur_word & 3) as usize]; // adjust d23 and d24 to ensure last two bit of parity to be 00
            }
            let start_idx = 60 + i * 30;
            self.assign_bits(cur_word, 30, &mut nav_bits[start_idx..start_idx + 30]);
        }

        0
    }

    pub fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) -> i32 {
        if svid < 1 || svid > 32 || eph.valid == 0 {
            return 0;
        }
        Self::compose_gps_stream123(eph, &mut self.gps_stream123[(svid - 1) as usize]);
        svid
    }

    pub fn set_almanac(&mut self, alm: &[GpsAlmanac; 32]) -> i32 {
        for i in 0..24 {
            Self::fill_gps_almanac_page(&alm[i], &mut self.gps_stream45[1][i]);
        }
        for i in 24..28 {
            Self::fill_gps_almanac_page(&alm[i], &mut self.gps_stream45[0][i - 23]);
        }
        for i in 28..32 {
            Self::fill_gps_almanac_page(&alm[i], &mut self.gps_stream45[0][i - 22]);
        }
        let (stream4_ref, stream5_ref) = self.gps_stream45.split_at_mut(1);
        Self::fill_gps_health_page(alm, &mut stream4_ref[0][24], &mut stream5_ref[0][24]);

        0
    }

    pub fn set_iono_utc(&mut self, iono_param: &IonoParam, utc_param: &UtcParam) -> i32 {
        if iono_param.flag == 0 || (utc_param.flag & 3) != 3 {
            return 0;
        }

        let stream = &mut self.gps_stream45[0][17]; // Ionosphere and UTC parameter in page 18 (indexed at 17)

        stream[0] = Self::compose_bits(56 + 0x40, 16, 8);
        let int_value = Self::unscale_int(iono_param.a0, -30);
        stream[0] |= Self::compose_bits(int_value, 8, 8);
        let int_value = Self::unscale_int(iono_param.a1, -27);
        stream[0] |= Self::compose_bits(int_value, 0, 8);
        let int_value = Self::unscale_int(iono_param.a2, -24);
        stream[1] = Self::compose_bits(int_value, 16, 8);
        let int_value = Self::unscale_int(iono_param.a3, -24);
        stream[1] |= Self::compose_bits(int_value, 8, 8);
        let int_value = Self::unscale_int(iono_param.b0, 11);
        stream[1] |= Self::compose_bits(int_value, 0, 8);
        let int_value = Self::unscale_int(iono_param.b1, 14);
        stream[2] = Self::compose_bits(int_value, 16, 8);
        let int_value = Self::unscale_int(iono_param.b2, 16);
        stream[2] |= Self::compose_bits(int_value, 8, 8);
        let int_value = Self::unscale_int(iono_param.b3, 16);
        stream[2] |= Self::compose_bits(int_value, 0, 8);
        let int_value = Self::unscale_int(utc_param.A1, -50);
        stream[3] = Self::compose_bits(int_value, 0, 24);
        let int_value = Self::unscale_int(utc_param.A0, -30);
        stream[4] = Self::compose_bits(int_value >> 8, 0, 24);
        stream[5] = Self::compose_bits(int_value & 0xff, 16, 8);
        stream[5] |= Self::compose_bits(utc_param.tot as i32, 8, 8);
        stream[5] |= Self::compose_bits(utc_param.WN.into(), 0, 8);
        stream[6] = Self::compose_bits(utc_param.TLS.into(), 16, 8);
        stream[6] |= Self::compose_bits(utc_param.WNLSF.into(), 8, 8);
        stream[6] |= Self::compose_bits(utc_param.DN.into(), 0, 8);
        stream[7] = Self::compose_bits(utc_param.TLSF.into(), 16, 8);

        0
    }

    // Helper methods
    fn compose_bits(value: i32, position: i32, length: i32) -> u32 {
        ((value as u32) & ((1u32 << length) - 1)) << position
    }

    fn unscale_int(value: f64, scale_factor: i32) -> i32 {
        (value * 2.0_f64.powi(-scale_factor)).round() as i32
    }

    fn unscale_uint(value: f64, scale_factor: i32) -> u32 {
        (value * 2.0_f64.powi(-scale_factor)).round() as u32
    }

    fn assign_bits(&self, word: u32, bit_count: usize, nav_bits: &mut [i32]) {
        for i in 0..bit_count {
            nav_bits[i] = ((word >> (bit_count - 1 - i)) & 1) as i32;
        }
    }

    // Fill ephemeris data into subframe 1/2/3
    // Word1/2 removed, parity not included
    // 24 information bits occupies bit0~23 of each DWORD in Stream[]
    fn compose_gps_stream123(ephemeris: &GpsEphemeris, stream: &mut [[u32; 8]; 3]) -> i32 {
        // subframe 1, Stream[0]~Stream[7]
        let int_value = (ephemeris.flag & 3) as i32;
        stream[0][0] = Self::compose_bits(int_value, 12, 2);
        let int_value = (ephemeris.ura & 0xf) as i32;
        stream[0][0] |= Self::compose_bits(int_value, 8, 4);
        stream[0][0] |= Self::compose_bits(ephemeris.health as i32, 2, 6);
        let int_value = (ephemeris.iodc >> 8) as i32;
        stream[0][0] |= Self::compose_bits(int_value, 0, 2);
        let int_value = (ephemeris.flag >> 2) as i32;
        stream[0][1] = Self::compose_bits(int_value, 23, 1);
        let int_value = Self::unscale_int(ephemeris.tgd, -31);
        stream[0][4] = Self::compose_bits(int_value, 0, 8);
        stream[0][5] = Self::compose_bits(ephemeris.iodc as i32, 16, 8);
        stream[0][5] |= Self::compose_bits((ephemeris.toc as i32) >> 4, 0, 16);
        let int_value = Self::unscale_int(ephemeris.af2, -55);
        stream[0][6] = Self::compose_bits(int_value, 16, 8);
        let int_value = Self::unscale_int(ephemeris.af1, -43);
        stream[0][6] |= Self::compose_bits(int_value, 0, 16);
        let int_value = Self::unscale_int(ephemeris.af0, -31);
        stream[0][7] = Self::compose_bits(int_value, 2, 22);

        // subframe 2, Stream[8]~Stream[15]
        stream[1][0] = Self::compose_bits(ephemeris.iode as i32, 16, 8);
        let int_value = Self::unscale_int(ephemeris.crs, -5);
        stream[1][0] |= Self::compose_bits(int_value, 0, 16);
        let int_value = Self::unscale_int(ephemeris.delta_n / PI, -43);
        stream[1][1] = Self::compose_bits(int_value, 8, 16);
        let int_value = Self::unscale_int(ephemeris.M0 / PI, -31);
        stream[1][1] |= Self::compose_bits(int_value >> 24, 0, 8);
        stream[1][2] = Self::compose_bits(int_value, 0, 24);
        let int_value = Self::unscale_int(ephemeris.cuc, -29);
        stream[1][3] = Self::compose_bits(int_value, 8, 16);
        let uint_value = Self::unscale_uint(ephemeris.ecc, -33);
        stream[1][3] |= Self::compose_bits((uint_value >> 24) as i32, 0, 8);
        stream[1][4] = Self::compose_bits(uint_value as i32, 0, 24);
        let int_value = Self::unscale_int(ephemeris.cus, -29);
        stream[1][5] = Self::compose_bits(int_value, 8, 16);
        let uint_value = Self::unscale_uint(ephemeris.sqrtA, -19);
        stream[1][5] |= Self::compose_bits((uint_value >> 24) as i32, 0, 8);
        stream[1][6] = Self::compose_bits(uint_value as i32, 0, 24);
        stream[1][7] = Self::compose_bits((ephemeris.toe as i32) >> 4, 8, 16);
        stream[1][7] |= Self::compose_bits((ephemeris.flag >> 3) as i32, 7, 1);

        // subframe 3, Stream[16]~Stream[23]
        let int_value = Self::unscale_int(ephemeris.cic, -29);
        stream[2][0] = Self::compose_bits(int_value, 8, 16);
        let int_value = Self::unscale_int(ephemeris.omega0 / PI, -31);
        stream[2][0] |= Self::compose_bits(int_value >> 24, 0, 8);
        stream[2][1] = Self::compose_bits(int_value, 0, 24);
        let int_value = Self::unscale_int(ephemeris.cis, -29);
        stream[2][2] = Self::compose_bits(int_value, 8, 16);
        let int_value = Self::unscale_int(ephemeris.i0 / PI, -31);
        stream[2][2] |= Self::compose_bits(int_value >> 24, 0, 8);
        stream[2][3] = Self::compose_bits(int_value, 0, 24);
        let int_value = Self::unscale_int(ephemeris.crc, -5);
        stream[2][4] = Self::compose_bits(int_value, 8, 16);
        let int_value = Self::unscale_int(ephemeris.w / PI, -31);
        stream[2][4] |= Self::compose_bits(int_value >> 24, 0, 8);
        stream[2][5] = Self::compose_bits(int_value, 0, 24);
        let int_value = Self::unscale_int(ephemeris.omega_dot / PI, -43);
        stream[2][6] = Self::compose_bits(int_value, 0, 24);
        stream[2][7] = Self::compose_bits(ephemeris.iode as i32, 16, 8);
        let int_value = Self::unscale_int(ephemeris.idot / PI, -43);
        stream[2][7] |= Self::compose_bits(int_value, 2, 14);

        0
    }

    fn fill_gps_almanac_page(almanac: &GpsAlmanac, stream: &mut [u32; 8]) -> i32 {
        if (almanac.valid & 1) == 0 {
            return 0;
        }
        stream[0] = Self::compose_bits(almanac.svid as i32 + 0x40, 16, 8);
        let uint_value = Self::unscale_uint(almanac.ecc, -21);
        stream[0] |= Self::compose_bits(uint_value as i32, 0, 16);
        stream[1] = Self::compose_bits((almanac.toa >> 12) as i32, 16, 8);
        let int_value = Self::unscale_int(almanac.i0 / PI - 0.3, -19);
        stream[1] |= Self::compose_bits(int_value, 0, 16);
        let int_value = Self::unscale_int(almanac.omega_dot / PI, -38);
        stream[2] = Self::compose_bits(int_value, 8, 16);
        stream[2] |= Self::compose_bits(almanac.health as i32, 0, 8);
        let uint_value = Self::unscale_uint(almanac.sqrtA, -11);
        stream[3] = Self::compose_bits(uint_value as i32, 0, 24);
        let int_value = Self::unscale_int(almanac.omega0 / PI, -23);
        stream[4] = Self::compose_bits(int_value, 0, 24);
        let int_value = Self::unscale_int(almanac.w / PI, -23);
        stream[5] = Self::compose_bits(int_value, 0, 24);
        let int_value = Self::unscale_int(almanac.M0 / PI, -23);
        stream[6] = Self::compose_bits(int_value, 0, 24);
        let int_value = Self::unscale_int(almanac.af0, -20);
        stream[7] = Self::compose_bits(int_value >> 3, 16, 8);
        stream[7] |= Self::compose_bits(int_value & 0x7, 2, 3);
        let int_value = Self::unscale_int(almanac.af1, -38);
        stream[7] |= Self::compose_bits(int_value, 5, 11);

        0
    }

    fn fill_gps_health_page(almanac: &[GpsAlmanac; 32], stream4: &mut [u32; 8], stream5: &mut [u32; 8]) -> i32 {
        let mut toa = 0;
        let mut week = 0;

        // Find first valid almanac to get toa and week number
        for i in 0..32 {
            if (almanac[i].valid & 1) != 0 {
                toa = (almanac[i].toa >> 12) as i32;
                week = (almanac[i].week & 0xff) as i32;
                break;
            }
        }

        // subframe 5 page 25
        stream5[0] = Self::compose_bits(0x73, 16, 8); // DataID = 01, PageID = 51
        stream5[0] |= Self::compose_bits(toa, 8, 8);
        stream5[0] |= Self::compose_bits(week, 0, 8);
        for i in (0..24).step_by(4) {
            stream5[i / 4 + 1] = Self::compose_bits(if (almanac[i].valid & 1) != 0 { 0 } else { 0x3f }, 18, 6);
            stream5[i / 4 + 1] |= Self::compose_bits(if (almanac[i + 1].valid & 1) != 0 { 0 } else { 0x3f }, 12, 6);
            stream5[i / 4 + 1] |= Self::compose_bits(if (almanac[i + 2].valid & 1) != 0 { 0 } else { 0x3f }, 6, 6);
            stream5[i / 4 + 1] |= Self::compose_bits(if (almanac[i + 3].valid & 1) != 0 { 0 } else { 0x3f }, 0, 6);
        }

        // subframe 4 page 25
        // assign AS flag 1100 as AS on and all signal capability
        stream4[0] = Self::compose_bits(0x7f, 16, 8); // DataID = 01, PageID = 63
        stream4[0] |= 0xcccc;
        for i in 1..5 {
            stream4[i] = 0xcccccc;
        }
        stream4[5] = 0xcccc00 + if (almanac[24].valid & 1) != 0 { 0 } else { 0x3f }; // SV25 health
        stream4[6] = Self::compose_bits(if (almanac[25].valid & 1) != 0 { 0 } else { 0x3f }, 18, 6);
        stream4[6] |= Self::compose_bits(if (almanac[26].valid & 1) != 0 { 0 } else { 0x3f }, 12, 6);
        stream4[6] |= Self::compose_bits(if (almanac[27].valid & 1) != 0 { 0 } else { 0x3f }, 6, 6);
        stream4[6] |= Self::compose_bits(if (almanac[28].valid & 1) != 0 { 0 } else { 0x3f }, 0, 6);
        stream4[7] = Self::compose_bits(if (almanac[29].valid & 1) != 0 { 0 } else { 0x3f }, 18, 6);
        stream4[7] |= Self::compose_bits(if (almanac[30].valid & 1) != 0 { 0 } else { 0x3f }, 12, 6);
        stream4[7] |= Self::compose_bits(if (almanac[31].valid & 1) != 0 { 0 } else { 0x3f }, 6, 6);

        0
    }

    // d29* at bit31, d30* at bit30, current word without parity at bit29 to bit6, bit5~0 ignored
    // generate 6 parity bits
    fn gps_get_parity(&self, word: u32) -> u32 {
        let mut word = word >> 6; // remove bit5~0
        let mut parity = 0u32;

        for i in 0..6 {
            parity ^= PARITY_TABLE[i][(word & 0xf) as usize] as u32;
            word >>= 4;
        }
        // add d29* and d30*
        if (word & 1) != 0 {
            parity ^= 0x15;
        }
        if (word & 2) != 0 {
            parity ^= 0x29;
        }

        parity
    }
}