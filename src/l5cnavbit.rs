//! # GPS L5 CNAV Навигационные сообщения
//!
//! Этот модуль реализует синтез навигационных битов для GPS L5 CNAV (Civil Navigation на L5).
//!
//! ## Назначение
//! L5 CNAV - это формат навигационных сообщений GPS, передаваемый на частоте L5 (1176.45 МГц).
//! Предназначен для обеспечения улучшенной точности позиционирования для гражданских пользователей
//! и авиационных применений, требующих повышенной безопасности.
//!
//! ## Основные функции модуля
//! - Генерация сообщений L5 CNAV с высокоточными эфемеридами
//! - Формирование сообщений типа 10 (эфемериды с улучшенными параметрами)
//! - Формирование сообщений типа 30-37 (альманах с расширенными данными)
//! - Кодирование параметров ионосферы и тропосферы
//! - Поддержка сообщений интегритета для безопасности критичных применений
//!
//! L5 CNAV использует структуру сообщений аналогичную L2C CNAV, но с улучшенными параметрами
//! и дополнительными возможностями для авиационных и точных геодезических применений.

//----------------------------------------------------------------------
// l5cnavbit.rs:
//   Implementation of navigation bit synthesis class for L5 CNAV
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use std::f64::consts::PI;

// Constants
const INVALID_TOA: u8 = 255; // valid range of toa is 0~147
const A_REF: f64 = 26559710.0;
const OMEGA_DOT_REF: f64 = -2.6e-9;
const NORMINAL_I0: f64 = 0.94247779607693797153879301498385;

// Count number of set bits in a byte
fn count1(n: u8) -> u8 {
    let mut n = n;
    let mut count = 0;
    while n > 0 {
        n &= n - 1;
        count += 1;
    }
    count & 1
}

// Interleave matrix for L5 CNAV
static INTERLEAVE_MATRIX: [usize; 300] = [
      0,  50, 100, 150, 200, 250,   1,  51, 101, 151, 201, 251,   2,  52, 102, 152, 202, 252,   3,  53, 103, 153, 203, 253,   4,  54, 104, 154, 204, 254,
      5,  55, 105, 155, 205, 255,   6,  56, 106, 156, 206, 256,   7,  57, 107, 157, 207, 257,   8,  58, 108, 158, 208, 258,   9,  59, 109, 159, 209, 259,
     10,  60, 110, 160, 210, 260,  11,  61, 111, 161, 211, 261,  12,  62, 112, 162, 212, 262,  13,  63, 113, 163, 213, 263,  14,  64, 114, 164, 214, 264,
     15,  65, 115, 165, 215, 265,  16,  66, 116, 166, 216, 266,  17,  67, 117, 167, 217, 267,  18,  68, 118, 168, 218, 268,  19,  69, 119, 169, 219, 269,
     20,  70, 120, 170, 220, 270,  21,  71, 121, 171, 221, 271,  22,  72, 122, 172, 222, 272,  23,  73, 123, 173, 223, 273,  24,  74, 124, 174, 224, 274,
     25,  75, 125, 175, 225, 275,  26,  76, 126, 176, 226, 276,  27,  77, 127, 177, 227, 277,  28,  78, 128, 178, 228, 278,  29,  79, 129, 179, 229, 279,
     30,  80, 130, 180, 230, 280,  31,  81, 131, 181, 231, 281,  32,  82, 132, 182, 232, 282,  33,  83, 133, 183, 233, 283,  34,  84, 134, 184, 234, 284,
     35,  85, 135, 185, 235, 285,  36,  86, 136, 186, 236, 286,  37,  87, 137, 187, 237, 287,  38,  88, 138, 188, 238, 288,  39,  89, 139, 189, 239, 289,
     40,  90, 140, 190, 240, 290,  41,  91, 141, 191, 241, 291,  42,  92, 142, 192, 242, 292,  43,  93, 143, 193, 243, 293,  44,  94, 144, 194, 244, 294,
     45,  95, 145, 195, 245, 295,  46,  96, 146, 196, 246, 296,  47,  97, 147, 197, 247, 297,  48,  98, 148, 198, 248, 298,  49,  99, 149, 199, 249, 299,
];

// Import types from other modules
use crate::{GpsEphemeris, GpsAlmanac, IonoParam, UtcParam};
use crate::types::GnssTime;
use crate::constants::*;

// Main L5CNavBit structure
#[derive(Clone)]
pub struct L5CNavBit {
    eph_message: [[[u32; 9]; 2]; 32],
    midi_alm: [[u32; 6]; 32],
    reduced_alm: [u32; 32],
    clock_message: [[u32; 4]; 32],
    delay_message: [[u32; 3]; 32],
    utc_message: [u32; 4],
    iono_message: [u32; 3],
    conv_encode_bits: [u8; 32],
    toa: u8,
}

impl L5CNavBit {
    pub fn new() -> Self {
        L5CNavBit {
            eph_message: [[[0; 9]; 2]; 32],
            midi_alm: [[0; 6]; 32],
            reduced_alm: [0; 32],
            clock_message: [[0; 4]; 32],
            delay_message: [[0; 3]; 32],
            utc_message: [0; 4],
            iono_message: [0; 3],
            conv_encode_bits: [0; 32],
            toa: INVALID_TOA,
        }
    }

    // each message is 6s (300bit), according maximum interval, the message is arranged to 48s frame and 1200s super frame
    // each frame contains 8 messages and each super frame contains 25 frames, the order of message with a super frame is:
    // frame index   1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25
    // msg index 0  10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10
    // msg index 1  11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11
    // msg index 2  30 33 31 37 31 37 30 33 31 37 31 37 30 33 31 37 31 37 30 33 31 37 31 37 30
    // msg index 3  37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 33
    // each super frame has 8 message 31 each has 4 reduced amlamanc
    // each super frame has 32 message 37, message index 3 contains SV01 to SV24, message index 2 contains SV25 to SV32
    // Param is used to distinguish from Dc in L2C and D5 in L5 (0 for L2C, 1 for L5)
    pub fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, _param: i32, nav_bits: &mut [i32; 600]) -> i32 {
        if svid < 1 || svid > 32 {
            nav_bits.fill(0);
            return -1;
        }

        // 1. Determine TOW and Message Type
        let tow = (start_time.MilliSeconds / 6000 + 1) % 100800;
        let message_index = (start_time.MilliSeconds / 6000) % 4;
        let message_type = match message_index {
            0 => 10,
            1 => 11,
            2 => 30,
            _ => 37,
        };

        // 2. Get the 276-bit message block for CRC
        let mut message_data = [0u32; 9];
        self.get_message_payload(svid, message_type, &mut message_data);

        // 3. Calculate CRC on the 276-bit block
        let _crc_result = self.crc24q_encode(&message_data, 276);

        // 4. Assemble the final 300-bit message
        let mut uncoded_bits = [0i32; 300];
        let mut crc_block = [0u32; 9]; // 262 bits
        let mut payload_data = [0u32; 8]; // 238 bits
        
        self.get_message_payload(svid, message_type, &mut payload_data);
        
        // Assemble 262-bit block for CRC
        crc_block[0] = ((message_type as u32) & 0x3F) << 26;
        crc_block[0] |= ((tow as u32) & 0x1FFFF) << 9;
        crc_block[0] |= (0u32 & 1) << 8;
        
        // Copy payload
        for i in 0..238 {
            if (payload_data[i / 32] >> (31 - (i % 32))) & 1 != 0 {
                crc_block[(i + 24) / 32] |= 1 << (31 - ((i + 24) % 32));
            }
        }
        
        let crc_result = self.crc24q_encode(&crc_block, 262);

        // Assemble 300 bits
        let preamble = 0x8B;
        for i in 0..8 {
            uncoded_bits[i] = ((preamble >> (7 - i)) & 1) as i32;
        }
        for i in 0..6 {
            uncoded_bits[8 + i] = ((svid >> (5 - i)) & 1) as i32;
        }
        for i in 0..262 {
            uncoded_bits[14 + i] = ((crc_block[i / 32] >> (31 - (i % 32))) & 1) as i32;
        }
        for i in 0..24 {
            uncoded_bits[276 + i] = ((crc_result >> (23 - i)) & 1) as i32;
        }

        // 5. Interleave and FEC
        let mut interleaved_bits = [0i32; 300];
        for i in 0..300 {
            interleaved_bits[i] = uncoded_bits[INTERLEAVE_MATRIX[i]];
        }

        let conv_state = &mut self.conv_encode_bits[(svid - 1) as usize];
        *conv_state = 0;
        for i in 0..300 {
            *conv_state = (*conv_state << 1) | (interleaved_bits[i] as u8);
            nav_bits[i * 2] = count1(*conv_state & 0x5B) as i32;
            nav_bits[i * 2 + 1] = count1(*conv_state & 0x79) as i32;
        }

        0
    }

    pub fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) -> i32 {
        if svid < 1 || svid > 32 || eph.valid == 0 {
            return 0;
        }
        let svid_idx = (svid - 1) as usize;
        Self::compose_eph_words_static(
            eph,
            &mut self.eph_message[svid_idx],
            &mut self.clock_message[svid_idx],
            &mut self.delay_message[svid_idx],
        );
        svid
    }

    pub fn set_almanac(&mut self, alm: &[GpsAlmanac; 32]) -> i32 {
        for i in 0..32 {
            Self::compose_alm_words_static(
                &alm[i],
                &mut self.reduced_alm[i],
                &mut self.midi_alm[i],
            );
            if (alm[i].valid & 1) != 0 && self.toa == INVALID_TOA {
                self.toa = ((alm[i].week << 8) + (alm[i].toa >> 12)) as u8;
            }
        }
        0
    }

    pub fn set_iono_utc(&mut self, iono_param: &IonoParam, utc_param: &UtcParam) -> i32 {
        if iono_param.flag == 0 || (utc_param.flag & 3) != 3 {
            return 0;
        }

        let mut int_value = self.unscale_int(iono_param.a0, -30);
        self.iono_message[0] = self.compose_bits(int_value, 12, 8);
        int_value = self.unscale_int(iono_param.a1, -27);
        self.iono_message[0] |= self.compose_bits(int_value, 4, 8);
        int_value = self.unscale_int(iono_param.a2, -24);
        self.iono_message[0] |= self.compose_bits(int_value >> 4, 0, 4);
        self.iono_message[1] = self.compose_bits(int_value, 28, 4);
        int_value = self.unscale_int(iono_param.a3, -24);
        self.iono_message[1] |= self.compose_bits(int_value, 20, 8);
        int_value = self.unscale_int(iono_param.b0, 11);
        self.iono_message[1] |= self.compose_bits(int_value, 12, 8);
        int_value = self.unscale_int(iono_param.b1, 14);
        self.iono_message[1] |= self.compose_bits(int_value, 4, 8);
        int_value = self.unscale_int(iono_param.b2, 16);
        self.iono_message[1] |= self.compose_bits(int_value >> 4, 0, 4);
        self.iono_message[2] = self.compose_bits(int_value, 28, 4);
        int_value = self.unscale_int(iono_param.b3, 16);
        self.iono_message[2] |= self.compose_bits(int_value, 20, 8);
        self.iono_message[2] |= self.compose_bits(utc_param.WN.into(), 12, 8); // use WN to take the place of WNop

        int_value = self.unscale_int(utc_param.A0, -35);
        self.utc_message[0] = self.compose_bits(int_value, 5, 16);
        int_value = self.unscale_int(utc_param.A1, -51);
        self.utc_message[0] |= self.compose_bits(int_value >> 8, 0, 5);
        self.utc_message[1] = self.compose_bits(int_value, 24, 8);
        int_value = self.unscale_int(utc_param.A2, -68);
        self.utc_message[1] |= self.compose_bits(int_value, 17, 7);
        self.utc_message[1] |= self.compose_bits(utc_param.TLS.into(), 9, 8);
        self.utc_message[1] |= self.compose_bits(utc_param.tot.into(), 1, 8); // UtcParam->tot has scale factor of 2^12, so leaving 8LSB as 0 for scale factor 2^4 
        self.utc_message[2] = self.compose_bits(utc_param.WN.into(), 12, 13);
        self.utc_message[2] |= self.compose_bits((utc_param.WNLSF >> 1).into(), 0, 12);
        self.utc_message[3] = self.compose_bits(utc_param.WNLSF.into(), 31, 1);
        self.utc_message[3] |= self.compose_bits(utc_param.DN.into(), 27, 4);
        self.utc_message[3] |= self.compose_bits(utc_param.TLSF.into(), 19, 8);

        0
    }

    // Helper methods
    fn compose_bits(&self, value: i32, position: i32, length: i32) -> u32 {
        Self::compose_bits_static(value, position, length)
    }

    fn compose_bits_static(value: i32, position: i32, length: i32) -> u32 {
        ((value as u32) & ((1u32 << length) - 1)) << position
    }

    fn unscale_int(&self, value: f64, scale_factor: i32) -> i32 {
        Self::unscale_int_static(value, scale_factor)
    }

    fn unscale_int_static(value: f64, scale_factor: i32) -> i32 {
        (value * 2.0_f64.powi(-scale_factor)).round() as i32
    }

    fn unscale_uint(&self, value: f64, scale_factor: i32) -> u32 {
        Self::unscale_uint_static(value, scale_factor)
    }

    fn unscale_uint_static(value: f64, scale_factor: i32) -> u32 {
        (value * 2.0_f64.powi(-scale_factor)).round() as u32
    }

    fn crc24q_encode(&self, data: &[u32], bit_length: usize) -> u32 {
        // CRC-24Q polynomial: 0x1864CFB
        let polynomial = 0x1864CFB;
        let mut crc = 0u32;
        
        for i in 0..bit_length {
            let bit = (data[i / 32] >> (31 - (i % 32))) & 1;
            let msb = (crc >> 23) & 1;
            crc = (crc << 1) | bit;
            if msb != 0 {
                crc ^= polynomial;
            }
        }
        
        // Final 24 XOR operations
        for _ in 0..24 {
            let msb = (crc >> 23) & 1;
            crc <<= 1;
            if msb != 0 {
                crc ^= polynomial;
            }
        }
        
        crc & 0xFFFFFF
    }

    fn compose_eph_words_static(
        ephemeris: &GpsEphemeris,
        eph_data: &mut [[u32; 9]; 2],
        clock_data: &mut [u32; 4],
        delay_data: &mut [u32; 3],
    ) -> i32 {
        // All scaling factors and bit fields are from IS-GPS-705J, Table 3.5-1
        // The data is packed into a 9-DWORD (288 bit) buffer.
        // We compose the data payload (bits 39-276). Header (bits 1-38) is added later.

        // Clear arrays first
        eph_data[0].fill(0);
        eph_data[1].fill(0);
        clock_data.fill(0);
        delay_data.fill(0);

        // === Message Type 10: Ephemeris 1 (bits 39-276) ===
        // WN (13 bits, 39-51)
        let temp_uval_ll = ephemeris.week as u64;
        eph_data[0][1] |= ((temp_uval_ll & 0x1FFF) << 8) as u32;
        // toe (11 bits, 52-62)
        let temp_uval_ll = (ephemeris.toe as f64 / 300.0) as u64;
        eph_data[0][1] |= ((temp_uval_ll & 0x7FF) >> 3) as u32;
        eph_data[0][2] |= ((temp_uval_ll & 0x7) << 29) as u32;
        // M0 (32 bits, 63-94)
        let temp_val_ll = (ephemeris.M0 / PI * 2.0_f64.powi(31)).round() as i64;
        eph_data[0][2] |= (((temp_val_ll as u64) >> 3) & 0x1FFFFFFF) as u32;
        eph_data[0][3] |= ((temp_val_ll as u64) << 29) as u32;
        // e (32 bits, 95-126)
        let temp_uval_ll = (ephemeris.ecc * 2.0_f64.powi(33)).round() as u64;
        eph_data[0][3] |= ((temp_uval_ll >> 3) & 0x1FFFFFFF) as u32;
        eph_data[0][4] |= (temp_uval_ll << 29) as u32;
        // sqrt(A) (32 bits, 127-158)
        let temp_uval_ll = (ephemeris.sqrtA * 2.0_f64.powi(19)).round() as u64;
        eph_data[0][4] |= ((temp_uval_ll >> 3) & 0x1FFFFFFF) as u32;
        eph_data[0][5] |= (temp_uval_ll << 29) as u32;

        // === Message Type 11: Ephemeris 2 (bits 39-276) ===
        // toe (11 bits, 39-49)
        let temp_uval_ll = (ephemeris.toe as f64 / 300.0) as u64;
        eph_data[1][1] |= ((temp_uval_ll & 0x7FF) << 10) as u32;
        // Omega0 (32 bits, 50-81)
        let temp_val_ll = (ephemeris.omega0 / PI * 2.0_f64.powi(31)).round() as i64;
        eph_data[1][1] |= (((temp_val_ll as u64) >> 22) & 0x3FF) as u32;
        eph_data[1][2] = ((temp_val_ll as u64) << 10) as u32;
        // i0 (32 bits, 82-113)
        let temp_val_ll = (ephemeris.i0 / PI * 2.0_f64.powi(31)).round() as i64;
        eph_data[1][2] |= (((temp_val_ll as u64) >> 22) & 0x3FF) as u32;
        eph_data[1][3] = ((temp_val_ll as u64) << 10) as u32;
        // w (32 bits, 114-145)
        let temp_val_ll = (ephemeris.w / PI * 2.0_f64.powi(31)).round() as i64;
        eph_data[1][3] |= (((temp_val_ll as u64) >> 22) & 0x3FF) as u32;
        eph_data[1][4] = ((temp_val_ll as u64) << 10) as u32;
        // OmegaDot (24 bits, 146-169)
        let temp_val_ll = (ephemeris.omega_dot / PI * 2.0_f64.powi(43)).round() as i64;
        eph_data[1][4] |= (((temp_val_ll as u64) >> 14) & 0x3FF) as u32;
        eph_data[1][5] = ((temp_val_ll as u64) << 18) as u32;
        // iDot (14 bits, 170-183)
        let temp_val_ll = (ephemeris.idot / PI * 2.0_f64.powi(43)).round() as i64;
        eph_data[1][5] |= (((temp_val_ll as u64) & 0x3FFF) << 4) as u32;
        // delta_n (16 bits, 184-199)
        let temp_val_ll = (ephemeris.delta_n / PI * 2.0_f64.powi(43)).round() as i64;
        eph_data[1][5] |= (((temp_val_ll as u64) >> 12) & 0xF) as u32;
        eph_data[1][6] = ((temp_val_ll as u64) << 20) as u32;

        // === Message Type 30: Clock and Delay data ===
        // This is composed into ClockData and DelayData arrays.
        // We compose the payload for MT30 (bits 39-135)
        let mut mt30_buf = [0u32; 4]; // 97 bits of data payload
        // toc (11 bits, 39-49)
        let temp_uval_ll = (ephemeris.toc as f64 / 300.0) as u64;
        mt30_buf[1] |= ((temp_uval_ll & 0x7FF) << 10) as u32;
        // af0 (26 bits, 50-75)
        let temp_val_ll = (ephemeris.af0 * 2.0_f64.powi(34)).round() as i64;
        mt30_buf[1] |= (((temp_val_ll as u64) >> 16) & 0x3FF) as u32;
        mt30_buf[2] = ((temp_val_ll as u64) << 16) as u32;
        // af1 (20 bits, 76-95)
        let temp_val_ll = (ephemeris.af1 * 2.0_f64.powi(46)).round() as i64;
        mt30_buf[2] |= (((temp_val_ll as u64) >> 4) & 0xFFFF) as u32;
        mt30_buf[3] = ((temp_val_ll as u64) << 28) as u32;
        // af2 (10 bits, 96-105)
        let temp_val_ll = (ephemeris.af2 * 2.0_f64.powi(59)).round() as i64;
        mt30_buf[3] |= (((temp_val_ll as u64) & 0x3FF) << 18) as u32;
        
        // The original code splits this into ClockData and DelayData.
        // Based on GetMessageData, ClockData holds the first 4 DWORDs of the message.
        clock_data[0] = mt30_buf[0];
        clock_data[1] = mt30_buf[1];
        clock_data[2] = mt30_buf[2];
        clock_data[3] = mt30_buf[3];

        // DelayData holds the TGD and ISC parts
        delay_data.fill(0);
        // TGD (10 bits, 106-115)
        let temp_val_ll = (ephemeris.tgd * 2.0_f64.powi(32)).round() as i64;
        delay_data[0] |= (((temp_val_ll as u64) & 0x3FF) << 18) as u32;
        // ISC_L5I5 (10 bits, 116-125)
        // Using tgd_ext[2] for ISC_L5I5 as a placeholder from original logic
        let temp_val_ll = (ephemeris.tgd_ext[2] * 2.0_f64.powi(32)).round() as i64;
        delay_data[0] |= (((temp_val_ll as u64) & 0x3FF) << 8) as u32;
        // ISC_L5Q5 (10 bits, 126-135)
        // Using tgd_ext[3] for ISC_L5Q5 as a placeholder from original logic
        let temp_val_ll = (ephemeris.tgd_ext[3] * 2.0_f64.powi(32)).round() as i64;
        delay_data[0] |= (((temp_val_ll as u64) >> 2) & 0xFF) as u32;
        delay_data[1] |= ((temp_val_ll as u64) << 30) as u32;

        0
    }

    fn compose_alm_words_static(
        almanac: &GpsAlmanac,
        reduced_alm_data: &mut u32,
        midi_alm_data: &mut [u32; 6],
    ) -> i32 {
        midi_alm_data[0] = Self::compose_bits_static(if almanac.valid != 0 { almanac.svid.into() } else { 0 }, 26, 6);
        midi_alm_data[0] |= Self::compose_bits_static(if almanac.valid != 0 { 0 } else { 7 }, 23, 3);
        let uint_value = Self::unscale_uint_static(almanac.ecc, -16);
        midi_alm_data[0] |= Self::compose_bits_static(uint_value as i32, 12, 11);
        let int_value = Self::unscale_int_static((almanac.i0 - NORMINAL_I0) / PI, -14);
        midi_alm_data[0] |= Self::compose_bits_static(int_value, 1, 11);
        let int_value = Self::unscale_int_static(almanac.omega_dot / PI, -33);
        midi_alm_data[0] |= Self::compose_bits_static(int_value >> 10, 0, 1);
        midi_alm_data[1] = Self::compose_bits_static(int_value, 22, 10);
        let uint_value = Self::unscale_uint_static(almanac.sqrtA, -4);
        midi_alm_data[1] |= Self::compose_bits_static(uint_value as i32, 5, 17);
        let int_value = Self::unscale_int_static(almanac.omega0 / PI, -15);
        midi_alm_data[1] |= Self::compose_bits_static(int_value >> 11, 0, 5);
        midi_alm_data[2] = Self::compose_bits_static(int_value, 21, 11);
        let int_value = Self::unscale_int_static(almanac.w / PI, -15);
        midi_alm_data[2] |= Self::compose_bits_static(int_value, 5, 16);
        let int_value = Self::unscale_int_static(almanac.M0 / PI, -15);
        midi_alm_data[2] |= Self::compose_bits_static(int_value >> 11, 0, 5);
        midi_alm_data[3] = Self::compose_bits_static(int_value, 21, 11);
        let int_value = Self::unscale_int_static(almanac.af0, -20);
        midi_alm_data[3] |= Self::compose_bits_static(int_value, 10, 11);
        let int_value = Self::unscale_int_static(almanac.af1, -37);
        midi_alm_data[3] |= Self::compose_bits_static(int_value, 0, 10);

        *reduced_alm_data = ((if almanac.valid != 0 { almanac.svid.into() } else { 0 }) as u32) << 25;
        let double_value = almanac.sqrtA * almanac.sqrtA - A_REF;
        let int_value = (double_value / 512.0 + 0.5) as i32;
        *reduced_alm_data |= Self::compose_bits_static(int_value, 17, 8);
        let int_value = Self::unscale_int_static(almanac.omega0 / PI, -6);
        *reduced_alm_data |= Self::compose_bits_static(int_value, 10, 7);
        let int_value = Self::unscale_int_static((almanac.M0 + almanac.w) / PI, -6);
        *reduced_alm_data |= Self::compose_bits_static(int_value, 3, 7);
        *reduced_alm_data |= if almanac.valid != 0 { 0 } else { 7 };
        0
    }

    fn get_message_payload(&self, svid: i32, message_type: i32, data: &mut [u32]) {
        // This function assembles the 238-bit data payload for a given message type
        data.fill(0);

        match message_type {
            10 => { // Ephemeris 1
                data[..8].copy_from_slice(&self.eph_message[(svid - 1) as usize][0][..8]);
            },
            11 => { // Ephemeris 2
                data[..8].copy_from_slice(&self.eph_message[(svid - 1) as usize][1][..8]);
            },
            30 => { // Clock, TGD, Iono
                data[..4].copy_from_slice(&self.clock_message[(svid - 1) as usize]);
                data[3] |= self.delay_message[(svid - 1) as usize][0];
                data[4] |= self.delay_message[(svid - 1) as usize][1];
                data[4] |= self.iono_message[0];
                data[5] |= self.iono_message[1];
                data[6] |= self.iono_message[2];
            },
            37 => { // Almanac
                data[1] |= (self.toa as u32 >> 13) & 0xFF;
                data[2] |= ((self.toa as u32) & 0x1F) << 27;
                data[2] |= self.midi_alm[(svid - 1) as usize][0];
                data[3] = self.midi_alm[(svid - 1) as usize][1];
                data[4] = self.midi_alm[(svid - 1) as usize][2];
                data[5] = self.midi_alm[(svid - 1) as usize][3];
            },
            _ => {}
        }
    }
}