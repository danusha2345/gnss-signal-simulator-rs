//! # GPS LNAV Навигационные сообщения
//!
//! Этот модуль реализует синтез навигационных битов для GPS LNAV (Legacy Navigation).
//!
//! ## Назначение
//! LNAV - это классический формат навигационных сообщений GPS, передаваемый на частоте L1
//! (1575.42 МГц). Содержит эфемеридную информацию, альманах, ионосферные и UTC параметры.
//!
//! ## Основные функции модуля
//! - Генерация кадров LNAV с эфемеридной информацией (подкадры 1-3)
//! - Формирование альманаха в подкадрах 4-5
//! - Кодирование ионосферных и UTC параметров
//! - Вычисление контрольных битов четности для каждого слова
//! - Управление временем передачи HOW (Hand Over Word)
//!
//! LNAV использует формат из 5 подкадров по 10 слов в каждом, где каждое слово содержит
//! 30 бит (24 бита данных + 6 бит четности).

//----------------------------------------------------------------------
// lnavbit.rs:
//   Implementation of navigation bit synthesis class for LNAV
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use std::f64::consts::PI;

// Import types from other modules
use crate::types::GnssTime;
use crate::{dprintln, GpsAlmanac, GpsEphemeris, IonoParam, UtcParam};
// use crate::constants::*; // Unused import

// Parity table for GPS LNAV
const PARITY_TABLE: [[u8; 16]; 6] = [
    [
        0x00, 0x13, 0x25, 0x36, 0x0B, 0x18, 0x2E, 0x3D, 0x16, 0x05, 0x33, 0x20, 0x1D, 0x0E, 0x38,
        0x2B,
    ],
    [
        0x00, 0x2C, 0x19, 0x35, 0x32, 0x1E, 0x2B, 0x07, 0x26, 0x0A, 0x3F, 0x13, 0x14, 0x38, 0x0D,
        0x21,
    ],
    [
        0x00, 0x0E, 0x1F, 0x11, 0x3E, 0x30, 0x21, 0x2F, 0x3D, 0x33, 0x22, 0x2C, 0x03, 0x0D, 0x1C,
        0x12,
    ],
    [
        0x00, 0x38, 0x31, 0x09, 0x23, 0x1B, 0x12, 0x2A, 0x07, 0x3F, 0x36, 0x0E, 0x24, 0x1C, 0x15,
        0x2D,
    ],
    [
        0x00, 0x0D, 0x1A, 0x17, 0x37, 0x3A, 0x2D, 0x20, 0x2F, 0x22, 0x35, 0x38, 0x18, 0x15, 0x02,
        0x0F,
    ],
    [
        0x00, 0x1C, 0x3B, 0x27, 0x34, 0x28, 0x0F, 0x13, 0x2A, 0x36, 0x11, 0x0D, 0x1E, 0x02, 0x25,
        0x39,
    ],
];

const PARITY_ADJUST: [u32; 4] = [0x00, 0xa5, 0xf6, 0x53];

const PAGE_ID: [[i32; 25]; 2] = [
    [
        57, 25, 26, 27, 28, 57, 29, 30, 31, 32, 57, 62, 52, 53, 54, 57, 55, 56, 58, 59, 57, 60, 61,
        62, 63,
    ],
    [
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 51,
    ],
];

// Main LNavBit structure
#[derive(Clone)]
pub struct LNavBit {
    pub gps_stream123: [[[u32; 8]; 3]; 32], // 32 satellites, 3 subframes, 8 words each
    gps_stream45: [[[u32; 8]; 25]; 2],  // 2 subframes (4&5), 25 pages, 8 words each
}

impl Default for LNavBit {
    fn default() -> Self {
        Self::new()
    }
}

impl LNavBit {
    pub fn new() -> Self {
        let mut lnav_bit = LNavBit {
            gps_stream123: [[[0xaa; 8]; 3]; 32],
            gps_stream45: [[[0x00; 8]; 25]; 2],
        };

        // Assign page id
        for i in 0..25 {
            lnav_bit.gps_stream45[0][i][0] =
                (lnav_bit.gps_stream45[0][i][0] & !0x3f0000) | ((PAGE_ID[0][i] as u32) << 16);
        }
        for i in 0..25 {
            lnav_bit.gps_stream45[1][i][0] =
                (lnav_bit.gps_stream45[1][i][0] & !0x3f0000) | ((PAGE_ID[1][i] as u32) << 16);
        }

        lnav_bit
    }

    pub fn get_frame_data(
        &self,
        mut start_time: GnssTime,
        svid: i32,
        _param: i32,
        nav_bits: &mut [i32; 300],
    ) -> i32 {
        // First determine the current TOW and subframe number
        start_time.Week += start_time.MilliSeconds / 604800000;
        start_time.MilliSeconds %= 604800000;
        let tow = start_time.MilliSeconds / 6000;
        let subframe = (tow % 5) + 1; // Циклическое переключение subframes 1-5
                                      // DEBUG: GPS subframe отладка отключена для уменьшения вывода
                                      // println!("[GPS-SUBFRAME] SV{:02}: tow={}, subframe={}", svid, tow, subframe);

        let stream = if subframe > 3 {
            // subframe 4/5, further determine page number
            let page = (tow / 5) % 25;
            &self.gps_stream45[(subframe - 4) as usize][page as usize]
        } else if (1..=32).contains(&svid) {
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
            cur_word |= (stream[i] & 0xffffff) << 6; // fill in d1 to d24 into bit29 to bit6
            if subframe == 1 && i == 0 {
                // WORD3 of subframe 1, put in week number from bit 29~20
                cur_word |= (start_time.Week as u32 & 0x3ff) << 20;
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
        dprintln!(
            "[GPS-EPH] set_ephemeris GPS SV{:02}: valid={}, svid_check={}",
            svid,
            eph.valid,
            (1..=32).contains(&svid)
        );
        if !(1..=32).contains(&svid) || eph.valid == 0 {
            dprintln!(
                "[GPS-EPH] REJECTED GPS SV{:02}: out of range or invalid",
                svid
            );
            return 0;
        }
        dprintln!(
            "[GPS-EPH] ACCEPTED GPS SV{:02}: composing navigation data",
            svid
        );
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
        // === Subframe 1: Words 3-10 (stream[0][0]..stream[0][7]) ===
        // Per GPS ICD-200, Word 3 bits 14-23 are WN (inserted by get_frame_data)

        // Word 3 (stream[0][0]): bits 0-13 only (WN in bits 14-23 added elsewhere)
        stream[0][0] = Self::compose_bits((ephemeris.flag & 3) as i32, 12, 2);    // L2 code flag
        stream[0][0] |= Self::compose_bits((ephemeris.ura & 0xf) as i32, 8, 4);   // URA index
        stream[0][0] |= Self::compose_bits(ephemeris.health as i32, 2, 6);         // SV health
        stream[0][0] |= Self::compose_bits((ephemeris.iodc >> 8) as i32, 0, 2);    // IODC MSB (2 bits)

        // Word 4 (stream[0][1]): L2P flag + reserved
        stream[0][1] = Self::compose_bits(((ephemeris.flag >> 2) & 1) as i32, 23, 1);

        // Words 5-6 (stream[0][2], stream[0][3]): reserved
        // C++ leaves these at 0xaa (not written by ComposeGpsStream123)
        // Do NOT overwrite — keep 0xaa from initialization

        // Word 7 (stream[0][4]): TGD(8 bits, 0-7) — C++ uses = COMPOSE_BITS(IntValue, 0, 8)
        stream[0][4] = Self::compose_bits(Self::unscale_int(ephemeris.tgd, -31), 0, 8);

        // Word 8 (stream[0][5]): IODC_LSB(8 bits, 16-23) + toc(16 bits, 0-15)
        stream[0][5] = Self::compose_bits((ephemeris.iodc & 0xFF) as i32, 16, 8);
        stream[0][5] |= Self::compose_bits(ephemeris.toc >> 4, 0, 16);

        // Word 9 (stream[0][6]): af2(8 bits, 16-23) + af1(16 bits, 0-15)
        stream[0][6] = Self::compose_bits(Self::unscale_int(ephemeris.af2, -55), 16, 8);
        stream[0][6] |= Self::compose_bits(Self::unscale_int(ephemeris.af1, -43), 0, 16);

        // Word 10 (stream[0][7]): af0(22 bits, 2-23) + spare(2 bits, 0-1)
        stream[0][7] = Self::compose_bits(Self::unscale_int(ephemeris.af0, -31), 2, 22);

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
        stream[1][7] = Self::compose_bits(ephemeris.toe >> 4, 8, 16);
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

        // DEBUG: проверим заполненность субкадров
        #[allow(clippy::needless_range_loop)]
        for sf in 0..3 {
            let mut _total_bits = 0;
            for word in 0..8 {
                let word_bits = stream[sf][word].count_ones();
                _total_bits += word_bits;
            }
            // DEBUG: GPS stream отключен для уменьшения вывода
            // println!("[GPS-STREAM-DEBUG] SV{:02} subframe {}: {} битов из 192 ({:.1}%)",
            //         ephemeris.svid, sf+1, total_bits, (total_bits as f32 / 192.0) * 100.0);
        }

        0
    }

    fn fill_gps_almanac_page(almanac: &GpsAlmanac, stream: &mut [u32; 8]) -> i32 {
        if (almanac.valid & 1) == 0 {
            return 0;
        }
        stream[0] = Self::compose_bits(almanac.svid as i32 + 0x40, 16, 8);
        let uint_value = Self::unscale_uint(almanac.ecc, -21);
        stream[0] |= Self::compose_bits(uint_value as i32, 0, 16);
        stream[1] = Self::compose_bits(almanac.toa >> 12, 16, 8);
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

    fn fill_gps_health_page(
        almanac: &[GpsAlmanac; 32],
        stream4: &mut [u32; 8],
        stream5: &mut [u32; 8],
    ) -> i32 {
        let mut toa = 0;
        let mut week = 0;

        // Find first valid almanac to get toa and week number
        #[allow(clippy::needless_range_loop)]
        for i in 0..32 {
            if (almanac[i].valid & 1) != 0 {
                toa = almanac[i].toa >> 12;
                week = almanac[i].week & 0xff;
                break;
            }
        }

        // subframe 5 page 25
        stream5[0] = Self::compose_bits(0x73, 16, 8); // DataID = 01, PageID = 51
        stream5[0] |= Self::compose_bits(toa, 8, 8);
        stream5[0] |= Self::compose_bits(week, 0, 8);
        for i in (0..24).step_by(4) {
            stream5[i / 4 + 1] =
                Self::compose_bits(if (almanac[i].valid & 1) != 0 { 0 } else { 0x3f }, 18, 6);
            stream5[i / 4 + 1] |= Self::compose_bits(
                if (almanac[i + 1].valid & 1) != 0 {
                    0
                } else {
                    0x3f
                },
                12,
                6,
            );
            stream5[i / 4 + 1] |= Self::compose_bits(
                if (almanac[i + 2].valid & 1) != 0 {
                    0
                } else {
                    0x3f
                },
                6,
                6,
            );
            stream5[i / 4 + 1] |= Self::compose_bits(
                if (almanac[i + 3].valid & 1) != 0 {
                    0
                } else {
                    0x3f
                },
                0,
                6,
            );
        }

        // subframe 4 page 25
        // assign AS flag 1100 as AS on and all signal capability
        stream4[0] = Self::compose_bits(0x7f, 16, 8); // DataID = 01, PageID = 63
        stream4[0] |= 0xcccc;
        #[allow(clippy::needless_range_loop)]
        for i in 1..5 {
            stream4[i] = 0xcccccc;
        }
        stream4[5] = 0xcccc00
            + if (almanac[24].valid & 1) != 0 {
                0
            } else {
                0x3f
            }; // SV25 health
        stream4[6] = Self::compose_bits(
            if (almanac[25].valid & 1) != 0 {
                0
            } else {
                0x3f
            },
            18,
            6,
        );
        stream4[6] |= Self::compose_bits(
            if (almanac[26].valid & 1) != 0 {
                0
            } else {
                0x3f
            },
            12,
            6,
        );
        stream4[6] |= Self::compose_bits(
            if (almanac[27].valid & 1) != 0 {
                0
            } else {
                0x3f
            },
            6,
            6,
        );
        stream4[6] |= Self::compose_bits(
            if (almanac[28].valid & 1) != 0 {
                0
            } else {
                0x3f
            },
            0,
            6,
        );
        stream4[7] = Self::compose_bits(
            if (almanac[29].valid & 1) != 0 {
                0
            } else {
                0x3f
            },
            18,
            6,
        );
        stream4[7] |= Self::compose_bits(
            if (almanac[30].valid & 1) != 0 {
                0
            } else {
                0x3f
            },
            12,
            6,
        );
        stream4[7] |= Self::compose_bits(
            if (almanac[31].valid & 1) != 0 {
                0
            } else {
                0x3f
            },
            6,
            6,
        );

        // Подсчет плотности для subframes 4-5
        let mut sf4_density = 0;
        let mut sf5_density = 0;
        for word in 0..8 {
            sf4_density += stream4[word].count_ones();
            sf5_density += stream5[word].count_ones();
        }
        println!(
            "[GPS-STREAM-DEBUG] Almanac subframe 4: {} битов из 192 ({:.1}%)",
            sf4_density,
            (sf4_density as f32 / 192.0) * 100.0
        );
        println!(
            "[GPS-STREAM-DEBUG] Almanac subframe 5: {} битов из 192 ({:.1}%)",
            sf5_density,
            (sf5_density as f32 / 192.0) * 100.0
        );

        0
    }

    // d29* at bit31, d30* at bit30, current word without parity at bit29 to bit6, bit5~0 ignored
    // generate 6 parity bits
    fn gps_get_parity(&self, word: u32) -> u32 {
        let mut word = word >> 6; // remove bit5~0
        let mut parity = 0u32;

        #[allow(clippy::needless_range_loop)]
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nav_decode::*;

    /// Helper: create a realistic GPS ephemeris for testing
    fn make_test_gps_ephemeris() -> GpsEphemeris {
        GpsEphemeris {
            svid: 1,
            valid: 1,
            iode: 42,
            iodc: 42,
            ura: 2,
            health: 0,
            flag: 0x01, // L2 code
            toe: 345600,
            toc: 345600,
            top: 0,
            week: 2369,
            M0: 0.539_812_714_5,
            delta_n: 4.908_419_825e-9,
            delta_n_dot: 0.0,
            ecc: 0.012_190_744_280,
            sqrtA: 5153.648_437_5,
            axis_dot: 0.0,
            omega0: -2.163_827_543_2,
            i0: 0.952_716_246_8,
            w: 0.687_462_195_1,
            omega_dot: -8.157_320_451e-9,
            idot: 2.535_843_151e-10,
            cuc: -4.842_877_388e-6,
            cus: 7.055_699_825e-6,
            crc: 234.125,
            crs: -44.84375,
            cic: -1.862_645_149e-8,
            cis: -6.332_993_507e-8,
            af0: 3.210_194_408e-4,
            af1: -2.046_363_078e-12,
            af2: 0.0,
            tgd: -1.117_587_089e-8,
            tgd2: 0.0,
            tgd_ext: [0.0; 5],
            axis: 5153.648_437_5 * 5153.648_437_5,
            n: 0.0,
            root_ecc: (1.0 - 0.012_190_744_280_f64.powi(2)).sqrt(),
            omega_t: -2.163_827_543_2,
            omega_delta: -8.157_320_451e-9,
            Ek: 0.0,
            Ek_dot: 0.0,
            source: 0,
        }
    }

    #[test]
    fn test_lnav_ephemeris_round_trip() {
        let eph = make_test_gps_ephemeris();
        let mut lnav = LNavBit::new();
        lnav.set_ephemeris(1, &eph);

        // Access stream123 for SV1
        let stream = &lnav.gps_stream123[0];
        let decoded = decode_lnav_stream123(stream);

        let diffs = decoded.compare_with_original(&eph);
        for d in &diffs {
            assert!(
                d.ok,
                "GPS LNAV round-trip FAILED for {}: original={:.15e}, decoded={:.15e}, diff={:.6e}, tol={:.6e}",
                d.name, d.original, d.decoded, d.diff, d.tolerance
            );
        }
    }

    #[test]
    fn test_lnav_parity() {
        let eph = make_test_gps_ephemeris();
        let mut lnav = LNavBit::new();
        lnav.set_ephemeris(1, &eph);

        // Generate frame data for several TOW values
        for tow_ms in [36330000i32, 42000000, 0, 600000] {
            let start_time = GnssTime {
                Week: 2369,
                MilliSeconds: tow_ms,
                SubMilliSeconds: 0.0,
            };
            let mut nav_bits = [0i32; 300];
            lnav.get_frame_data(start_time, 1, 0, &mut nav_bits);

            let results = verify_lnav_parity(&nav_bits);
            for (word_idx, parity_ok) in &results {
                assert!(
                    *parity_ok,
                    "GPS LNAV parity FAILED for word {} at TOW_ms={}",
                    word_idx, tow_ms
                );
            }
        }
    }

    #[test]
    fn test_nav_orbit_consistency_gps() {
        let eph = make_test_gps_ephemeris();
        let mut lnav = LNavBit::new();
        lnav.set_ephemeris(1, &eph);

        let stream = &lnav.gps_stream123[0];
        let decoded = decode_lnav_stream123(stream);

        // Build GpsEphemeris from decoded values
        let mut dec_eph = GpsEphemeris::default();
        dec_eph.valid = 1;
        dec_eph.svid = 1;
        dec_eph.M0 = decoded.M0;
        dec_eph.delta_n = decoded.delta_n;
        dec_eph.ecc = decoded.ecc;
        dec_eph.sqrtA = decoded.sqrtA;
        dec_eph.omega0 = decoded.omega0;
        dec_eph.i0 = decoded.i0;
        dec_eph.w = decoded.w;
        dec_eph.omega_dot = decoded.omega_dot;
        dec_eph.idot = decoded.idot;
        dec_eph.cuc = decoded.cuc;
        dec_eph.cus = decoded.cus;
        dec_eph.crc = decoded.crc;
        dec_eph.crs = decoded.crs;
        dec_eph.cic = decoded.cic;
        dec_eph.cis = decoded.cis;
        dec_eph.toe = decoded.toe;
        dec_eph.toc = decoded.toc;

        // Compute derived fields
        use crate::constants::EARTH_GM;
        dec_eph.axis = dec_eph.sqrtA * dec_eph.sqrtA;
        dec_eph.root_ecc = (1.0 - dec_eph.ecc * dec_eph.ecc).sqrt();
        dec_eph.n = (EARTH_GM / (dec_eph.axis.powi(3))).sqrt() + dec_eph.delta_n;
        dec_eph.omega_t = dec_eph.omega0;
        dec_eph.omega_delta = dec_eph.omega_dot;

        let mut orig_eph = eph;
        orig_eph.axis = orig_eph.sqrtA * orig_eph.sqrtA;
        orig_eph.root_ecc = (1.0 - orig_eph.ecc * orig_eph.ecc).sqrt();
        orig_eph.n = (EARTH_GM / (orig_eph.axis.powi(3))).sqrt() + orig_eph.delta_n;
        orig_eph.omega_t = orig_eph.omega0;
        orig_eph.omega_delta = orig_eph.omega_dot;

        // Propagate at toe + 1800s
        use crate::coordinate::gps_sat_pos_speed_eph;
        use crate::types::{GnssSystem, KinematicInfo};

        let t = eph.toe as f64 + 1800.0;
        let mut orig_pos = KinematicInfo::default();
        let mut dec_pos = KinematicInfo::default();

        gps_sat_pos_speed_eph(GnssSystem::GpsSystem, t, &mut orig_eph, &mut orig_pos, None);
        gps_sat_pos_speed_eph(GnssSystem::GpsSystem, t, &mut dec_eph, &mut dec_pos, None);

        let dx = orig_pos.x - dec_pos.x;
        let dy = orig_pos.y - dec_pos.y;
        let dz = orig_pos.z - dec_pos.z;
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();

        assert!(
            dist < 2.0,
            "GPS orbit consistency: position difference {:.3} m exceeds 2.0 m threshold",
            dist
        );
    }

    #[test]
    fn test_lnav_tow_encoding() {
        let eph = make_test_gps_ephemeris();
        let mut lnav = LNavBit::new();
        lnav.set_ephemeris(1, &eph);

        // Test several TOW values
        let test_cases = [
            (0, 1),             // TOW=0 ms -> tow_count=0, next=1
            (6000, 2),          // TOW=6000 ms -> tow_count=1, next=2
            (30000, 6),         // TOW=30000 ms -> tow_count=5, next=6
            (600000, 101),      // TOW=600s -> tow_count=100, next=101
        ];

        for (tow_ms, expected_next_tow) in &test_cases {
            let start_time = GnssTime {
                Week: 2369,
                MilliSeconds: *tow_ms,
                SubMilliSeconds: 0.0,
            };
            let mut nav_bits = [0i32; 300];
            lnav.get_frame_data(start_time, 1, 0, &mut nav_bits);

            // Extract HOW word (bits 30-59)
            // Bits 30-46 (17 bits) = TOW count (truncated count of next subframe)
            let mut how_tow: u32 = 0;
            for i in 0..17 {
                how_tow = (how_tow << 1) | (nav_bits[30 + i] as u32);
            }

            assert_eq!(
                how_tow, *expected_next_tow as u32,
                "TOW encoding mismatch: tow_ms={}, expected next_tow={}, got={}",
                tow_ms, expected_next_tow, how_tow
            );

            // Extract subframe number (bits 49-51, 3 bits)
            let mut sf_num: u32 = 0;
            for i in 0..3 {
                sf_num = (sf_num << 1) | (nav_bits[49 + i] as u32);
            }
            let expected_sf = (((*tow_ms / 6000) % 5) + 1) as u32;
            assert_eq!(
                sf_num, expected_sf,
                "Subframe number mismatch: tow_ms={}, expected sf={}, got={}",
                tow_ms, expected_sf, sf_num
            );
        }
    }
}
