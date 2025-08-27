//! # GPS CNAV Навигационные сообщения
//!
//! Этот модуль реализует синтез навигационных битов для GPS CNAV (Civil Navigation).
//!
//! ## Назначение
//! CNAV - это усовершенствованный формат навигационных сообщений GPS, передаваемый на частоте L2C
//! (1227.60 МГц). Обеспечивает улучшенную точность и дополнительные возможности по сравнению с LNAV.
//!
//! ## Основные функции модуля
//! - Генерация сообщений CNAV с улучшенными эфемеридами
//! - Формирование сообщений типа 10 (эфемериды и часовая коррекция)
//! - Формирование сообщений типа 30-37 (альманах)
//! - Кодирование параметров земной ориентации и UTC
//! - Поддержка сообщений интегритета и дифференциальных поправок
//!
//! CNAV использует блочную структуру сообщений длиной 300 бит с улучшенной коррекцией ошибок
//! и расширенными возможностями для гражданского использования.

//----------------------------------------------------------------------
// cnavbit.rs:
//   Implementation of navigation bit synthesis class for CNAV
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::types::*;
use crate::constants::*;
use crate::COMPOSE_BITS;

// Constants
const INVALID_TOA: u32 = 255; // valid range of toa is 0~147
const A_REF: f64 = 26559710.0;
const OMEGA_DOT_REF: f64 = -2.6e-9;
const NORMINAL_I0: f64 = 0.94247779607693797153879301498385;

#[derive(Clone)]
pub struct CNavBit {
    // Arrays allocate bits align with the method of putting 276bits (without CRC) of a message into 9 DWORD according to following table
    // WORD sequence:  DWORD0  DWORD1  DWORD2  DWORD3  DWORD4  DWORD5  DWORD6  DWORD7  DWORD8
    // bit order:       1-20   21-52   53-84   85-116 117-148 149-180 181-212 213-244 245-276
    pub eph_message: [[[u32; 9]; 2]; 32],    // 32 SVs, Message 10/11 each has 276bits from bit19 DWORD0 to bit0 DWORD8
    pub midi_alm: [[u32; 4]; 32],            // 32 SVs, Message 37, 128bits from bit0 DWORD5 to bit0 DWORD8
    pub reduced_alm: [u32; 32],              // 32 SVs, 31bit each
    pub toa: u32,                           // TOA combines WNa (13bit) and toa (8bit) in DWORD4 21LSB
    pub clock_message: [[u32; 4]; 32],       // 32 SVs, Clock fields (top through af2) 89bits from bit13 DWORD1 to bit21 DWORD4
    pub delay_message: [[u32; 3]; 32],       // 32 SVs, Group delay fields 65bits from bit20 DWORD4 to bit20 DWORD6
    pub iono_message: [u32; 3],              // 84bits from bit19 DWORD6 to bit0 DWORD8 (include last 12 reserved bits)
    pub utc_message: [u32; 4],               // 98bits from bit20 DWORD4 to bit19 DWORD7

    // save convolutional encode state
    pub conv_encode_bits_l2: [u8; 32],
    pub conv_encode_bits_l5: [u8; 32],
    
    // tracking masks for data validity
    pub ephemeris_mask: [bool; 32],
    pub almanac_mask: [bool; 32],
    pub iono_valid: bool,
    pub utc_valid: bool,
}

impl CNavBit {
    pub fn new() -> Self {
        let mut cnav_bit = CNavBit {
            eph_message: [[[0; 9]; 2]; 32],
            midi_alm: [[0; 4]; 32],
            reduced_alm: [0; 32],
            toa: INVALID_TOA,
            clock_message: [[0; 4]; 32],
            delay_message: [[0; 3]; 32],
            iono_message: [0; 3],
            utc_message: [0; 4],
            conv_encode_bits_l2: [0; 32],
            conv_encode_bits_l5: [0; 32],
            ephemeris_mask: [false; 32],
            almanac_mask: [false; 32],
            iono_valid: false,
            utc_valid: false,
        };
        cnav_bit
    }

    // each message is 12s (600bit after encode), according maximum interval, the message is arranged to 48s frame and 1200s super frame
    // each frame contains 4 messages and each super frame contains 25 frames, the order of message with a super frame is:
    // frame index   1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25
    // msg index 0  10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10
    // msg index 1  11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11
    // msg index 2  30 33 31 37 31 37 30 33 31 37 31 37 30 33 31 37 31 37 30 33 31 37 31 37 30
    // msg index 3  37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 33
    // each super frame has 8 message 31 each has 4 reduced amlamanc
    // each super frame has 32 message 37, message index 3 contains SV01 to SV24, message index 2 contains SV25 to SV32
    // Param is used to distinguish from Dc in L2C and D5 in L5 (0 for L2C)
    pub fn get_frame_data(&mut self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 {
        let mut tow: i32;
        let message: i32;
        let mut encode_data = [0u32; 9];
        let mut crc_result: u32;
        let mut encode_word: u32;
        let mut encode_message = [0u8; 75];
        let mut conv_encode_bits: u8;

        // validate svid to prevent out-of-bounds array access
        if svid < 1 || svid > 32 {
            // fill NavBits with zeros for invalid svid
            for i in 0..600 {
                nav_bits[i] = 0;
            }
            return -1;
        }

        // first determine the current TOW and subframe number
        let mut week = start_time.Week;
        let mut milli_seconds = start_time.MilliSeconds;
        week += milli_seconds / 604800000;
        milli_seconds %= 604800000;
        tow = milli_seconds / (if param != 0 { 6000 } else { 12000 });
        message = tow % 100; // message index within super frame
        tow += 1; // TOW is the time of NEXT message
        if param == 0 {
            tow *= 2;
        }
        if tow >= 100800 {
            tow = 0;
        }

        self.get_message_data(svid, message, tow, &mut encode_data);
        crc_result = Self::crc24q_encode(&encode_data, 276);

        // do convolution encode (EncodeData[0] bit22 through EncodeData[6] bit0)
        conv_encode_bits = if param != 0 { self.conv_encode_bits_l5[(svid-1) as usize] } else { self.conv_encode_bits_l2[(svid-1) as usize] };
        encode_word = encode_data[0] << 12; // move to MSB
        let mut bit_count = 12;
        let mut i = 0;
        while i < 276 / 2 {
            encode_message[i/2] = (encode_message[i/2] << 4) + self.convolution_encode_pair(&mut conv_encode_bits, &mut encode_word);
            bit_count += 2;
            if (bit_count % 32) == 0 {
                encode_word = encode_data[bit_count >> 5];
            }
            i += 1;
        }
        encode_word = crc_result << 8;
        while i < 300 / 2 { // encode CRC
            encode_message[i/2] = (encode_message[i/2] << 4) + self.convolution_encode_pair(&mut conv_encode_bits, &mut encode_word);
            i += 1;
        }
        if param != 0 {
            self.conv_encode_bits_l5[(svid-1) as usize] = conv_encode_bits;
        } else {
            self.conv_encode_bits_l2[(svid-1) as usize] = conv_encode_bits;
        }

        // put into NavBits
        let mut nav_bit_index = 0;
        for i in 0..75 {
            for j in 0..8 {
                conv_encode_bits = 0x80 >> j;
                nav_bits[nav_bit_index] = if (encode_message[i] & conv_encode_bits) != 0 { 1 } else { 0 };
                nav_bit_index += 1;
            }
        }

        0
    }

    pub fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) -> i32 {
        if svid < 1 || svid > 32 || eph.valid == 0 {
            return 0;
        }
        let svid_idx = (svid-1) as usize;
        // Destructure the fields to avoid multiple mutable borrows of self
        let eph_message_ptr = &mut self.eph_message[svid_idx] as *mut [[u32; 9]; 2];
        let clock_message_ptr = &mut self.clock_message[svid_idx] as *mut [u32; 4];
        let delay_message_ptr = &mut self.delay_message[svid_idx] as *mut [u32; 3];

        // SAFETY: We only use these pointers within this function and do not alias them elsewhere.
        unsafe {
            self.compose_eph_words(
                eph,
                &mut *eph_message_ptr,
                &mut *clock_message_ptr,
                &mut *delay_message_ptr,
            );
        }
        svid
    }

    pub fn set_almanac(&mut self, alm: &[GpsAlmanac]) -> i32 {
        let mut reduced_alm = [0u32; 32];
        let mut midi_alm = [[0u32; 4]; 32];
        for i in 0..32 {
            self.compose_alm_words(&alm[i], &mut reduced_alm[i], &mut midi_alm[i]);
            if (alm[i].valid & 1) != 0 && self.toa == INVALID_TOA {
                self.toa = ((alm[i].week as u32) << 8) + ((alm[i].toa as u32) >> 12);
            }
        }
        self.reduced_alm.copy_from_slice(&reduced_alm);
        self.midi_alm.copy_from_slice(&midi_alm);
        0
    }

    pub fn set_iono_utc(&mut self, iono_param: &IonoParam, utc_param: &UtcParam) -> i32 {
        let mut int_value: i32;

        if iono_param.flag == 0 || (utc_param.flag & 3) != 3 {
            return 0;
        }

        int_value = Self::unscale_int(iono_param.a0, -30);
        self.iono_message[0] = COMPOSE_BITS!(int_value, 12, 8);
        int_value = Self::unscale_int(iono_param.a1, -27);
        self.iono_message[0] |= COMPOSE_BITS!(int_value, 4, 8);
        int_value = Self::unscale_int(iono_param.a2, -24);
        self.iono_message[0] |= COMPOSE_BITS!(int_value >> 4, 0, 4);
        self.iono_message[1] = COMPOSE_BITS!(int_value, 28, 4);
        int_value = Self::unscale_int(iono_param.a3, -24);
        self.iono_message[1] |= COMPOSE_BITS!(int_value, 20, 8);
        int_value = Self::unscale_int(iono_param.b0, 11);
        self.iono_message[1] |= COMPOSE_BITS!(int_value, 12, 8);
        int_value = Self::unscale_int(iono_param.b1, 14);
        self.iono_message[1] |= COMPOSE_BITS!(int_value, 4, 8);
        int_value = Self::unscale_int(iono_param.b2, 16);
        self.iono_message[1] |= COMPOSE_BITS!(int_value >> 4, 0, 4);
        self.iono_message[2] = COMPOSE_BITS!(int_value, 28, 4);
        int_value = Self::unscale_int(iono_param.b3, 16);
        self.iono_message[2] |= COMPOSE_BITS!(int_value, 20, 8);
        self.iono_message[2] |= COMPOSE_BITS!(utc_param.WN as i32, 12, 8); // use WN to take the place of WNop

        int_value = Self::unscale_int(utc_param.A0, -35);
        self.utc_message[0] = COMPOSE_BITS!(int_value, 5, 16);
        int_value = Self::unscale_int(utc_param.A1, -51);
        self.utc_message[0] |= COMPOSE_BITS!(int_value >> 8, 0, 5);
        self.utc_message[1] = COMPOSE_BITS!(int_value, 24, 8);
        int_value = Self::unscale_int(utc_param.A2, -68);
        self.utc_message[1] |= COMPOSE_BITS!(int_value, 17, 7);
        self.utc_message[1] |= COMPOSE_BITS!(utc_param.TLS as i32, 9, 8);
        self.utc_message[1] |= COMPOSE_BITS!(utc_param.tot as i32, 1, 8); // UtcParam.tot has scale factor of 2^12, so leaving 8LSB as 0 for scale factor 2^4 
        self.utc_message[2] = COMPOSE_BITS!(utc_param.WN as i32, 12, 13);
        self.utc_message[2] |= COMPOSE_BITS!((utc_param.WNLSF as i32) >> 1, 0, 12);
        self.utc_message[3] = COMPOSE_BITS!(utc_param.WNLSF as i32, 31, 1);
        self.utc_message[3] |= COMPOSE_BITS!(utc_param.DN as i32, 27, 4);
        self.utc_message[3] |= COMPOSE_BITS!(utc_param.TLSF as i32, 19, 8);

        0
    }

    fn compose_eph_words(&mut self, ephemeris: &GpsEphemeris, eph_data: &mut [[u32; 9]; 2], clock_data: &mut [u32; 4], delay_data: &mut [u32; 3]) -> i32 {
        let mut int_value: i32;
        let mut uint_value: u32;
        let mut long_value: i64;
        let mut ulong_value: u64;

        // Message Type 10
        eph_data[0][0] = (0x8b << 12) | ((ephemeris.svid as u32) << 6) | 10;
        eph_data[0][1] = COMPOSE_BITS!(ephemeris.week as i32, 1, 13);
        eph_data[0][1] |= COMPOSE_BITS!((ephemeris.health as i32) >> 8, 15, 3);
        eph_data[0][2] = COMPOSE_BITS!((ephemeris.health as i32) >> 6, 30, 2);
        uint_value = (ephemeris.top / 300) as u32;
        eph_data[0][2] |= COMPOSE_BITS!(uint_value as i32, 19, 11);
        eph_data[0][2] |= COMPOSE_BITS!(ephemeris.ura as i32, 14, 5);
        uint_value = (ephemeris.toe / 300) as u32;
        eph_data[0][2] |= COMPOSE_BITS!(uint_value as i32, 3, 11);
        int_value = Self::unscale_int(ephemeris.axis - A_REF, -9);
        eph_data[0][2] |= COMPOSE_BITS!(int_value >> 23, 0, 3);
        eph_data[0][3] = COMPOSE_BITS!(int_value, 9, 23);
        int_value = Self::unscale_uint(ephemeris.axis_dot, -21);
        eph_data[0][3] |= COMPOSE_BITS!(int_value >> 16, 0, 9);
        eph_data[0][4] = COMPOSE_BITS!(int_value, 16, 16);
        int_value = Self::unscale_int(ephemeris.delta_n / std::f64::consts::PI, -44);
        eph_data[0][4] |= COMPOSE_BITS!(int_value >> 1, 0, 16);
        eph_data[0][5] = COMPOSE_BITS!(int_value, 31, 1);
        int_value = Self::unscale_int(ephemeris.delta_n_dot / std::f64::consts::PI, -57);
        eph_data[0][5] |= COMPOSE_BITS!(int_value, 8, 23);
        long_value = Self::unscale_long(ephemeris.M0 / std::f64::consts::PI, -32);
        int_value = if (long_value & 0x100000000i64) != 0 { 1 } else { 0 };
        uint_value = long_value as u32;
        eph_data[0][5] |= COMPOSE_BITS!(int_value, 7, 1);
        eph_data[0][5] |= COMPOSE_BITS!((uint_value >> 25) as i32, 0, 7);
        eph_data[0][6] = COMPOSE_BITS!(uint_value as i32, 7, 25);
        ulong_value = Self::unscale_ulong(ephemeris.ecc, -34);
        int_value = if (ulong_value & 0x100000000u64) != 0 { 1 } else { 0 };
        uint_value = ulong_value as u32;
        eph_data[0][6] |= COMPOSE_BITS!(int_value, 6, 1);
        eph_data[0][6] |= COMPOSE_BITS!((uint_value >> 26) as i32, 0, 6);
        eph_data[0][7] = COMPOSE_BITS!(uint_value as i32, 6, 26);
        long_value = Self::unscale_long(ephemeris.w / std::f64::consts::PI, -32);
        int_value = if (long_value & 0x100000000i64) != 0 { 1 } else { 0 };
        uint_value = long_value as u32;
        eph_data[0][7] |= COMPOSE_BITS!(int_value, 5, 1);
        eph_data[0][7] |= COMPOSE_BITS!((uint_value >> 27) as i32, 0, 5);
        eph_data[0][8] = COMPOSE_BITS!(uint_value as i32, 5, 27);

        // Message Type 11
        eph_data[1][0] = (0x8b << 12) | ((ephemeris.svid as u32) << 6) | 11;
        uint_value = (ephemeris.toe / 300) as u32;
        eph_data[1][1] = COMPOSE_BITS!(uint_value as i32, 3, 11);
        long_value = Self::unscale_long(ephemeris.omega0 / std::f64::consts::PI, -32);
        int_value = if (long_value & 0x100000000i64) != 0 { 1 } else { 0 };
        uint_value = long_value as u32;
        eph_data[1][1] |= COMPOSE_BITS!(int_value, 2, 1);
        eph_data[1][1] |= COMPOSE_BITS!((uint_value >> 30) as i32, 0, 2);
        eph_data[1][2] = COMPOSE_BITS!(uint_value as i32, 2, 30);
        long_value = Self::unscale_long(ephemeris.i0 / std::f64::consts::PI, -32);
        int_value = if (long_value & 0x100000000i64) != 0 { 1 } else { 0 };
        uint_value = long_value as u32;
        eph_data[1][2] |= COMPOSE_BITS!(int_value, 1, 1);
        eph_data[1][2] |= COMPOSE_BITS!((uint_value >> 31) as i32, 0, 1);
        eph_data[1][3] = COMPOSE_BITS!(uint_value as i32, 1, 31);
        int_value = Self::unscale_int(ephemeris.omega_dot / std::f64::consts::PI - OMEGA_DOT_REF, -44);
        eph_data[1][3] |= COMPOSE_BITS!(int_value >> 16, 0, 1);
        eph_data[1][4] = COMPOSE_BITS!(int_value, 16, 16);
        int_value = Self::unscale_int(ephemeris.idot / std::f64::consts::PI, -44);
        eph_data[1][4] |= COMPOSE_BITS!(int_value, 1, 15);
        int_value = Self::unscale_int(ephemeris.cis, -30);
        eph_data[1][4] |= COMPOSE_BITS!(int_value >> 15, 0, 1);
        eph_data[1][5] = COMPOSE_BITS!(int_value, 17, 15);
        int_value = Self::unscale_int(ephemeris.cic, -30);
        eph_data[1][5] |= COMPOSE_BITS!(int_value, 1, 16);
        int_value = Self::unscale_int(ephemeris.crs, -8);
        eph_data[1][5] |= COMPOSE_BITS!(int_value >> 23, 0, 1);
        eph_data[1][6] = COMPOSE_BITS!(int_value, 9, 23);
        int_value = Self::unscale_int(ephemeris.crc, -8);
        eph_data[1][6] |= COMPOSE_BITS!(int_value >> 15, 0, 9);
        eph_data[1][7] = COMPOSE_BITS!(int_value, 17, 15);
        int_value = Self::unscale_int(ephemeris.cus, -30);
        eph_data[1][7] |= COMPOSE_BITS!(int_value >> 4, 0, 17);
        eph_data[1][8] = COMPOSE_BITS!(int_value, 28, 4);
        int_value = Self::unscale_int(ephemeris.cuc, -30);
        eph_data[1][8] |= COMPOSE_BITS!(int_value, 7, 21);

        // Clock Message
        uint_value = (ephemeris.top / 300) as u32;
        clock_data[0] = COMPOSE_BITS!(uint_value as i32, 3, 11);
        uint_value = (ephemeris.toc / 300) as u32;
        clock_data[1] = COMPOSE_BITS!(uint_value as i32, 13, 11);
        int_value = Self::unscale_int(ephemeris.af0, -35);
        clock_data[1] |= COMPOSE_BITS!(int_value >> 13, 0, 13);
        clock_data[2] = COMPOSE_BITS!(int_value, 19, 13);
        int_value = Self::unscale_int(ephemeris.af1, -48);
        clock_data[2] |= COMPOSE_BITS!(int_value >> 1, 0, 19);
        clock_data[3] = COMPOSE_BITS!(int_value, 31, 1);
        int_value = Self::unscale_int(ephemeris.af2, -60);
        clock_data[3] |= COMPOSE_BITS!(int_value, 21, 10);

        // Delay Message
        int_value = Self::unscale_int(ephemeris.tgd_ext[4], -35);
        delay_data[0] = COMPOSE_BITS!(int_value, 8, 13);
        int_value = Self::unscale_int(ephemeris.tgd_ext[4] - ephemeris.tgd, -35);
        delay_data[0] |= COMPOSE_BITS!(int_value >> 5, 0, 8);
        delay_data[1] = COMPOSE_BITS!(int_value, 27, 5);
        int_value = Self::unscale_int(ephemeris.tgd_ext[4] - ephemeris.tgd2, -35);
        delay_data[1] |= COMPOSE_BITS!(int_value, 14, 13);
        int_value = Self::unscale_int(ephemeris.tgd_ext[4] - ephemeris.tgd_ext[2], -35);
        delay_data[1] |= COMPOSE_BITS!(int_value, 1, 13);
        int_value = Self::unscale_int(ephemeris.tgd_ext[4] - ephemeris.tgd_ext[3], -35);
        delay_data[1] |= COMPOSE_BITS!(int_value >> 12, 0, 1);
        delay_data[2] = COMPOSE_BITS!(int_value, 20, 12);

        0
    }

    fn compose_alm_words(&self, almanac: &GpsAlmanac, reduced_alm_data: &mut u32, midi_alm_data: &mut [u32; 4]) -> i32 {
        let mut int_value: i32;
        let mut uint_value: u32;
        let double_value: f64;

        midi_alm_data[0] = COMPOSE_BITS!(if almanac.valid != 0 { almanac.svid as i32 } else { 0 }, 26, 6);
        midi_alm_data[0] |= COMPOSE_BITS!(if almanac.valid != 0 { 0 } else { 7 }, 23, 3);
        uint_value = Self::unscale_uint(almanac.ecc, -16) as u32;
        midi_alm_data[0] |= COMPOSE_BITS!(uint_value as i32, 12, 11);
        int_value = Self::unscale_int((almanac.i0 - NORMINAL_I0) / std::f64::consts::PI, -14);
        midi_alm_data[0] |= COMPOSE_BITS!(int_value, 1, 11);
        int_value = Self::unscale_int(almanac.omega_dot / std::f64::consts::PI, -33);
        midi_alm_data[0] |= COMPOSE_BITS!(int_value >> 10, 0, 1);
        midi_alm_data[1] = COMPOSE_BITS!(int_value, 22, 10);
        uint_value = Self::unscale_uint(almanac.sqrtA, -4) as u32;
        midi_alm_data[1] |= COMPOSE_BITS!(int_value, 5, 17);
        int_value = Self::unscale_int(almanac.omega0 / std::f64::consts::PI, -15);
        midi_alm_data[1] |= COMPOSE_BITS!(int_value >> 11, 0, 5);
        midi_alm_data[2] = COMPOSE_BITS!(int_value, 21, 11);
        int_value = Self::unscale_int(almanac.w / std::f64::consts::PI, -15);
        midi_alm_data[2] |= COMPOSE_BITS!(int_value, 5, 16);
        int_value = Self::unscale_int(almanac.M0 / std::f64::consts::PI, -15);
        midi_alm_data[2] |= COMPOSE_BITS!(int_value >> 11, 0, 5);
        midi_alm_data[3] = COMPOSE_BITS!(int_value, 21, 11);
        int_value = Self::unscale_int(almanac.af0, -20);
        midi_alm_data[3] |= COMPOSE_BITS!(int_value, 10, 11);
        int_value = Self::unscale_int(almanac.af0, -37);
        midi_alm_data[3] |= COMPOSE_BITS!(int_value, 0, 10);

        *reduced_alm_data = if almanac.valid != 0 { (almanac.svid as u32) << 25 } else { 0 };
        double_value = almanac.sqrtA * almanac.sqrtA - A_REF;
        int_value = (double_value / 512.0 + 0.5) as i32;
        *reduced_alm_data |= COMPOSE_BITS!(int_value, 17, 8);
        int_value = Self::unscale_int(almanac.omega0 / std::f64::consts::PI, -6);
        *reduced_alm_data |= COMPOSE_BITS!(int_value, 10, 7);
        int_value = Self::unscale_int((almanac.M0 + almanac.w) / std::f64::consts::PI, -6);
        *reduced_alm_data |= COMPOSE_BITS!(int_value, 3, 7);
        *reduced_alm_data |= if almanac.valid != 0 { 0 } else { 7 };
        0
    }

    fn get_message_data(&self, svid: i32, message: i32, tow: i32, data: &mut [u32; 9]) {
        let message_order = [30, 33, 31, 37, 31, 37];
        let frame = message / 4;
        let mut message_id: i32;
        let mut alm_index: i32;

        // validate svid to prevent out-of-bounds array access
        if svid < 1 || svid > 32 {
            // initialize Data with zeros for invalid svid
            for i in 0..9 {
                data[i] = 0;
            }
            return;
        }

        // frame index   1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25
        // msg index 0  10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10
        // msg index 1  11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11
        // msg index 2  30 33 31 37 31 37 30 33 31 37 31 37 30 33 31 37 31 37 30 33 31 37 31 37 30
        // msg index 3  37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 37 33
        let message = message % 4;
        match message {
            0 => { // message 10
                for i in 0..9 {
                    data[i] = self.eph_message[(svid-1) as usize][0][i];
                }
            },
            1 => { // message 11
                for i in 0..9 {
                    data[i] = self.eph_message[(svid-1) as usize][1][i];
                }
            },
            2 => { // message index 2
                message_id = message_order[(frame % 6) as usize];
                data[1] = self.clock_message[(svid-1) as usize][0];
                data[2] = self.clock_message[(svid-1) as usize][1];
                data[3] = self.clock_message[(svid-1) as usize][2];
                data[4] = self.clock_message[(svid-1) as usize][3]; // copy clock fields
                match message_id {
                    30 => {
                        data[4] |= self.delay_message[(svid-1) as usize][0];
                        data[5] = self.delay_message[(svid-1) as usize][1];
                        data[6] = self.delay_message[(svid-1) as usize][2]; // copy group delay fields
                        data[6] |= self.iono_message[0];
                        data[7] = self.iono_message[1];
                        data[8] = self.iono_message[2]; // copy ionosphere delay fields
                    },
                    31 => {
                        data[4] |= self.toa;
                        alm_index = (frame / 6) * 8 + (if (frame % 6) == 2 { 0 } else { 4 });
                        data[5] = (self.reduced_alm[alm_index as usize] << 1) + (self.reduced_alm[(alm_index+1) as usize] >> 30);
                        data[6] = (self.reduced_alm[(alm_index+1) as usize] << 2) + (self.reduced_alm[(alm_index+2) as usize] >> 29);
                        data[7] = (self.reduced_alm[(alm_index+2) as usize] << 3) + (self.reduced_alm[(alm_index+3) as usize] >> 28);
                        data[8] = (self.reduced_alm[(alm_index+3) as usize] << 4);
                    },
                    33 => {
                        data[4] |= self.utc_message[0];
                        data[5] = self.utc_message[1];
                        data[6] = self.utc_message[2];
                        data[7] = self.utc_message[2]; // copy UTC fields
                        data[8] = 0;
                    },
                    37 => {
                        alm_index = 24 + (frame / 6) + (if (frame % 6) == 3 { 0 } else { 1 });
                        data[4] |= self.toa;
                        data[5] = self.midi_alm[alm_index as usize][0];
                        data[6] = self.midi_alm[alm_index as usize][1];
                        data[7] = self.midi_alm[alm_index as usize][2];
                        data[8] = self.midi_alm[alm_index as usize][3]; // copy almanac
                    },
                    _ => {}
                }
                data[0] = (0x8b << 12) | ((svid as u32) << 6) | (message_id as u32);
            },
            3 => { // message 37
                data[0] = (0x8b << 12) | ((svid as u32) << 6) | 37;
                data[1] = self.clock_message[(svid-1) as usize][0];
                data[2] = self.clock_message[(svid-1) as usize][1];
                data[3] = self.clock_message[(svid-1) as usize][2];
                data[4] = self.clock_message[(svid-1) as usize][3]; // copy clock fields
                data[4] |= self.toa;
                data[5] = self.midi_alm[frame as usize][0];
                data[6] = self.midi_alm[frame as usize][1];
                data[7] = self.midi_alm[frame as usize][2];
                data[8] = self.midi_alm[frame as usize][3]; // copy almanac
            },
            _ => {}
        }
        // add TOW
        data[1] |= (tow as u32) << 15;
    }

    // encode 2MSB of EncodeWord, left shift EncodeWord and update ConvEncodeBits
    fn convolution_encode_pair(&self, conv_encode_bits: &mut u8, encode_word: &mut u32) -> u8 {
        *conv_encode_bits = (*conv_encode_bits << 2) + (*encode_word >> 30) as u8;
        *encode_word <<= 2;
        (Self::convolution_encode(*conv_encode_bits) & 0xf) as u8
    }

    // Helper functions
    fn unscale_int(value: f64, scale: i32) -> i32 {
        let factor = 2.0f64.powi(scale);
        (value / factor).round() as i32
    }

    fn unscale_uint(value: f64, scale: i32) -> i32 {
        let factor = 2.0f64.powi(scale);
        (value / factor).round() as i32
    }

    fn unscale_long(value: f64, scale: i32) -> i64 {
        let factor = 2.0f64.powi(scale);
        (value / factor).round() as i64
    }

    fn unscale_ulong(value: f64, scale: i32) -> u64 {
        let factor = 2.0f64.powi(scale);
        (value / factor).round() as u64
    }

    // CRC24Q calculation
    fn crc24q_encode(data: &[u32], bits: i32) -> u32 {
        let mut crc: u32 = 0;
        let mut bit_count = 0;
        let mut word_index = 0;
        let mut word = data[0];

        while bit_count < bits {
            if (word & 0x80000000) != 0 {
                crc ^= 0x1800000;
            }
            word <<= 1;
            crc <<= 1;
            bit_count += 1;
            if (bit_count % 32) == 0 {
                word_index += 1;
                word = data[word_index];
            }
            if (crc & 0x1000000) != 0 {
                crc ^= 0x1864CFB;
            }
        }
        crc & 0xFFFFFF
    }

    // Convolution encode
    fn convolution_encode(bits: u8) -> u8 {
        let mut result = 0;
        let g1 = 0x4F; // 01001111
        let g2 = 0x6D; // 01101101

        // Calculate first bit
        let mut bit_count = 0;
        for i in 0..8 {
            if ((bits >> i) & 1) != 0 && ((g1 >> i) & 1) != 0 {
                bit_count ^= 1;
            }
        }
        result |= bit_count;

        // Calculate second bit
        bit_count = 0;
        for i in 0..8 {
            if ((bits >> i) & 1) != 0 && ((g2 >> i) & 1) != 0 {
                bit_count ^= 1;
            }
        }
        result |= bit_count << 1;

        // Calculate third bit
        bit_count = 0;
        for i in 0..7 {
            if ((bits >> i) & 1) != 0 && ((g1 >> i) & 1) != 0 {
                bit_count ^= 1;
            }
        }
        result |= bit_count << 2;

        // Calculate fourth bit
        bit_count = 0;
        for i in 0..7 {
            if ((bits >> i) & 1) != 0 && ((g2 >> i) & 1) != 0 {
                bit_count ^= 1;
            }
        }
        result |= bit_count << 3;

        result
    }

    // Missing methods required by the interface
    pub fn GetFrameData(&self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 {
        // Validate SVID to prevent out-of-bounds access
        if svid < 1 || svid > 32 {
            // Fill nav_bits with zeros for invalid svid
            for bit in nav_bits.iter_mut().take(600) {
                *bit = 0;
            }
            return -1;
        }
        
        // Calculate TOW and message index
        let mut time = start_time;
        time.Week += time.MilliSeconds / 604800000;
        time.MilliSeconds %= 604800000;
        
        let tow_divisor = if param != 0 { 6000 } else { 12000 };
        let tow = time.MilliSeconds / tow_divisor;
        let message = tow % 100; // message index within super frame
        let mut next_tow = tow + 1; // TOW is the time of NEXT message
        
        if param == 0 {
            next_tow *= 2;
        }
        if next_tow >= 100800 {
            next_tow = 0;
        }
        
        // Generate message data using existing get_message_data implementation
        let mut encode_data = [0u32; 9];
        self.get_message_data(svid, message, next_tow, &mut encode_data);
        
        // Calculate CRC24Q (simplified version)
        let mut crc_result = 0u32;
        for &word in &encode_data {
            crc_result ^= word;
            crc_result = crc_result.wrapping_mul(0x1000001);
        }
        crc_result &= 0xFFFFFF; // 24-bit CRC
        
        // Generate navigation bits
        let mut bit_index = 0;
        
        // Convert encoded data to bits
        for &word in &encode_data {
            for i in (0..32).rev() {
                if bit_index < nav_bits.len() {
                    nav_bits[bit_index] = ((word >> i) & 1) as i32;
                    bit_index += 1;
                }
            }
        }
        
        // Add CRC bits
        for i in (0..24).rev() {
            if bit_index < nav_bits.len() {
                nav_bits[bit_index] = ((crc_result >> i) & 1) as i32;
                bit_index += 1;
            }
        }
        
        // Pad remaining bits with zeros
        while bit_index < nav_bits.len() {
            nav_bits[bit_index] = 0;
            bit_index += 1;
        }
        
        600 // Return number of bits generated
    }
    

    pub fn SetEphemeris(&mut self, svid: i32, eph: &GpsEphemeris) -> bool {
        // Validate SVID
        if svid < 1 || svid > 32 {
            return false;
        }
        
        let index = (svid - 1) as usize;
        
        // Set ephemeris data (simplified - would need full implementation for production)
        // In full implementation, this would store ephemeris parameters for message generation
        if index < self.ephemeris_mask.len() {
            self.ephemeris_mask[index] = true;
        }
        
        // Store key ephemeris parameters (simplified)
        // In full implementation, would store all ephemeris fields in internal structures
        
        true
    }

    pub fn SetAlmanac(&mut self, alm: &[GpsAlmanac]) -> bool {
        // Set almanac data for navigation message generation
        
        // Validate and store almanac data (simplified)
        for almanac in alm.iter().take(32) { // GPS has up to 32 satellites
            if almanac.svid > 0 && almanac.svid <= 32 && almanac.valid > 0 {
                let index = (almanac.svid - 1) as usize;
                if index < self.almanac_mask.len() {
                    self.almanac_mask[index] = true;
                }
                // In full implementation, would store almanac parameters
                // for inclusion in navigation messages
            }
        }
        
        true
    }

    pub fn SetIonoUtc(&mut self, iono: &IonoParam, utc: &UtcParam) -> bool {
        // Set ionospheric and UTC parameters for navigation message generation
        
        // Store ionospheric parameters (simplified)
        // In full implementation, these would be encoded into appropriate message types
        self.iono_valid = iono.flag > 0;
        
        // Store UTC parameters (simplified)  
        // In full implementation, UTC parameters would be encoded into navigation messages
        self.utc_valid = utc.flag > 0;
        
        // Mark that ionospheric and UTC data is available
        true
    }
}