//! Galileo F/NAV (E5a): 214 бит данных + CRC24Q(24) + 6 tail-бит.
//! После FEC 1/2 и интерливинга 8×61 добавляется 12-битная синхропосылка:
//! всего 500 символов за 10 секунд (50 symbols/s).
// Copyright (C) 2020-2029 by Jun Mo, All rights reserved.

use crate::types::*;
use std::f64::consts::PI;

const DATA_BITS: usize = 214;
const SQRT_A0: f64 = 5_440.588_203_494_177;
const NOMINAL_I0: f64 = 0.977_384_381_116_824_6;
type Page = [u32; 7];

/// Поле с нумерацией от 1 (MSB), как в таблицах ICD; отрицательные — two's complement.
fn put(page: &mut Page, start: usize, width: usize, value: i64) {
    debug_assert!(start > 0 && start + width - 1 <= DATA_BITS);
    for i in 0..width {
        let bit = start - 1 + i;
        let mask = 1_u32 << (31 - bit % 32);
        page[bit / 32] = (page[bit / 32] & !mask)
            | ((((value as u64 >> (width - 1 - i)) & 1) as u32) << (31 - bit % 32));
    }
}
fn quantize(value: f64, exponent: i32) -> i64 {
    (value / 2_f64.powi(exponent)).round() as i64
}
fn get_bit(page: &Page, index: usize) -> u8 {
    ((page[index / 32] >> (31 - index % 32)) & 1) as u8
}

#[derive(Clone)]
pub struct FNavBit {
    /// Канонические MSB-first payload-страницы 1..4; младшие 10 бит последнего слова не передаются.
    pub gal_eph_data: [[Page; 4]; 36],
    pub gal_alm_data: [[Page; 2]; 12],
    iono: Option<IonoNequick>,
    utc: Option<UtcParam>,
}
impl Default for FNavBit {
    fn default() -> Self {
        Self::new()
    }
}

impl FNavBit {
    const SYNC_PATTERN: [i32; 12] = [1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0];
    pub fn new() -> Self {
        let mut nav = Self {
            gal_eph_data: [[[0; 7]; 4]; 36],
            gal_alm_data: [[[0; 7]; 2]; 12],
            iono: None,
            utc: None,
        };
        nav.set_almanac(&[]);
        nav
    }

    pub fn get_frame_data(
        &self,
        time: GnssTime,
        svid: i32,
        _param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        if !(1..=36).contains(&svid) || nav_bits.len() < 500 {
            return -1;
        }
        let week = time.Week + time.MilliSeconds.div_euclid(604800000);
        // TOW относится к началу страницы, а не к произвольному моменту первого обращения.
        let tow = time.MilliSeconds.rem_euclid(604800000) / 10000 * 10;
        let page = self.get_page_data(svid as usize, week, tow);
        let mut input = [0u8; 244];
        let mut crc = 0u32;
        for (i, bit) in input[..DATA_BITS].iter_mut().enumerate() {
            *bit = get_bit(&page, i);
            let feedback = (crc >> 23) ^ *bit as u32;
            crc = (crc << 1) & 0xffffff;
            if feedback & 1 != 0 {
                crc ^= 0x864cfb;
            }
        }
        for i in 0..24 {
            input[DATA_BITS + i] = ((crc >> (23 - i)) & 1) as u8;
        }
        // input[238..244] — нулевые tail-биты для возврата кодера в состояние 0.
        let mut symbols = [0i32; 488];
        let mut state = 0u8;
        for (i, &bit) in input.iter().enumerate() {
            state = ((state << 1) | bit) & 0x7f;
            symbols[2 * i] = ((state & 0x4f).count_ones() & 1) as i32;
            symbols[2 * i + 1] = (((state & 0x6d).count_ones() & 1) ^ 1) as i32;
        }
        nav_bits[..12].copy_from_slice(&Self::SYNC_PATTERN);
        // Запись столбцами (8 строк × 61 столбец), чтение строками.
        for i in 0..488 {
            nav_bits[12 + i] = symbols[(i % 61) * 8 + i / 61];
        }
        0
    }

    pub fn set_ephemeris(&mut self, svid: i32, e: &GpsEphemeris) -> i32 {
        if !(1..=36).contains(&svid) || e.valid == 0 {
            return 0;
        }
        let mut pages = [[0u32; 7]; 4];
        for (i, p) in pages.iter_mut().enumerate() {
            put(p, 1, 6, i as i64 + 1);
        }
        let p = &mut pages[0];
        put(p, 7, 6, svid as i64);
        put(p, 13, 10, e.iodc as i64);
        put(p, 23, 14, (e.toc / 60) as i64);
        put(p, 37, 31, quantize(e.af0, -34));
        put(p, 68, 21, quantize(e.af1, -46));
        put(p, 89, 6, quantize(e.af2, -59));
        put(p, 95, 8, e.ura as i64);
        put(p, 144, 10, quantize(e.tgd, -32));
        // RINEX: E5a HS в битах 4..5, DVS в бите 3.
        put(p, 154, 2, (e.health >> 4) as i64);
        put(p, 188, 1, (e.health >> 3) as i64);
        let p = &mut pages[1];
        put(p, 7, 10, e.iodc as i64);
        put(p, 17, 32, quantize(e.M0 / PI, -31));
        put(p, 49, 24, quantize(e.omega_dot / PI, -43));
        put(p, 73, 32, quantize(e.ecc, -33));
        put(p, 105, 32, quantize(e.sqrtA, -19));
        put(p, 137, 32, quantize(e.omega0 / PI, -31));
        put(p, 169, 14, quantize(e.idot / PI, -43));
        let p = &mut pages[2];
        put(p, 7, 10, e.iodc as i64);
        put(p, 17, 32, quantize(e.i0 / PI, -31));
        put(p, 49, 32, quantize(e.w / PI, -31));
        put(p, 81, 16, quantize(e.delta_n / PI, -43));
        put(p, 97, 16, quantize(e.cuc, -29));
        put(p, 113, 16, quantize(e.cus, -29));
        put(p, 129, 16, quantize(e.crc, -5));
        put(p, 145, 16, quantize(e.crs, -5));
        put(p, 161, 14, (e.toe / 60) as i64);
        let p = &mut pages[3];
        put(p, 7, 10, e.iodc as i64);
        put(p, 17, 16, quantize(e.cic, -29));
        put(p, 33, 16, quantize(e.cis, -29));
        self.gal_eph_data[svid as usize - 1] = pages;
        svid
    }

    pub fn set_iono(&mut self, iono: &IonoNequick) {
        self.iono = Some(*iono);
    }
    /// tot — часы, WN/WNLSF — GST. Общий GPS Klobuchar сюда не принимается.
    pub fn set_utc(&mut self, utc: &UtcParam) {
        self.utc = Some(*utc);
    }
    pub fn set_iono_utc(&mut self, iono: &IonoNequick, utc: &UtcParam) -> i32 {
        self.set_iono(iono);
        self.set_utc(utc);
        0
    }

    pub fn set_almanac(&mut self, alm: &[GpsAlmanac]) -> i32 {
        self.gal_alm_data = [[[0; 7]; 2]; 12];
        for group in 0..12 {
            let sats: [Option<&GpsAlmanac>; 3] = std::array::from_fn(|i| {
                alm.get(group * 3 + i)
                    .filter(|a| a.valid & 1 != 0 && (1..=36).contains(&a.svid))
            });
            let (left, right) = self.gal_alm_data[group].split_at_mut(1);
            let p5 = &mut left[0];
            let p6 = &mut right[0];
            put(p5, 1, 6, 5);
            put(p6, 1, 6, 6);
            put(p5, 7, 4, 4);
            put(p6, 7, 4, 4);
            if let Some(a) = sats.iter().flatten().next() {
                put(p5, 11, 2, a.week as i64);
                put(p5, 13, 10, (a.toa / 600) as i64);
            }
            if let Some(a) = sats[0] {
                put(p5, 23, 6, a.svid as i64);
                put(p5, 29, 13, quantize(a.sqrtA - SQRT_A0, -9));
                put(p5, 42, 11, quantize(a.ecc, -16));
                put(p5, 53, 16, quantize(a.w / PI, -15));
                put(p5, 69, 11, quantize((a.i0 - NOMINAL_I0) / PI, -14));
                put(p5, 80, 16, quantize(a.omega0 / PI, -15));
                put(p5, 96, 11, quantize(a.omega_dot / PI, -33));
                put(p5, 107, 16, quantize(a.M0 / PI, -15));
                put(p5, 123, 16, quantize(a.af0, -19));
                put(p5, 139, 13, quantize(a.af1, -38));
                put(p5, 152, 2, (a.health >> 4) as i64);
            }
            if let Some(a) = sats[1] {
                put(p5, 154, 6, a.svid as i64);
                put(p5, 160, 13, quantize(a.sqrtA - SQRT_A0, -9));
                put(p5, 173, 11, quantize(a.ecc, -16));
                put(p5, 184, 16, quantize(a.w / PI, -15));
                put(p5, 200, 11, quantize((a.i0 - NOMINAL_I0) / PI, -14));
                let omega = quantize(a.omega0 / PI, -15);
                put(p5, 211, 4, omega >> 12);
                put(p6, 11, 12, omega);
                put(p6, 23, 11, quantize(a.omega_dot / PI, -33));
                put(p6, 34, 16, quantize(a.M0 / PI, -15));
                put(p6, 50, 16, quantize(a.af0, -19));
                put(p6, 66, 13, quantize(a.af1, -38));
                put(p6, 79, 2, (a.health >> 4) as i64);
            }
            if let Some(a) = sats[2] {
                put(p6, 81, 6, a.svid as i64);
                put(p6, 87, 13, quantize(a.sqrtA - SQRT_A0, -9));
                put(p6, 100, 11, quantize(a.ecc, -16));
                put(p6, 111, 16, quantize(a.w / PI, -15));
                put(p6, 127, 11, quantize((a.i0 - NOMINAL_I0) / PI, -14));
                put(p6, 138, 16, quantize(a.omega0 / PI, -15));
                put(p6, 154, 11, quantize(a.omega_dot / PI, -33));
                put(p6, 165, 16, quantize(a.M0 / PI, -15));
                put(p6, 181, 16, quantize(a.af0, -19));
                put(p6, 197, 13, quantize(a.af1, -38));
                put(p6, 210, 2, (a.health >> 4) as i64);
            }
        }
        0
    }

    fn get_page_data(&self, svid: usize, week: i32, tow: i32) -> Page {
        let index = (tow % 50 / 10) as usize;
        if index == 4 {
            return self.gal_alm_data[(tow % 1200 / 100) as usize][(tow / 50 % 2) as usize];
        }
        let mut p = self.gal_eph_data[svid - 1][index];
        if p[0] >> 26 == 0 {
            put(&mut p, 1, 6, 63);
            return p;
        }
        let gst_week = (week - 1024) as i64;
        match index {
            0 => {
                if let Some(i) = self.iono {
                    put(&mut p, 103, 11, quantize(i.ai0, -2));
                    put(&mut p, 114, 11, quantize(i.ai1, -8));
                    put(&mut p, 125, 14, quantize(i.ai2, -15));
                    put(&mut p, 139, 5, (i.flag & 31) as i64);
                }
                put(&mut p, 156, 12, gst_week);
                put(&mut p, 168, 20, tow as i64);
            }
            1 => {
                put(&mut p, 183, 12, gst_week);
                put(&mut p, 195, 20, tow as i64);
            }
            2 => {
                put(&mut p, 175, 12, gst_week);
                put(&mut p, 187, 20, tow as i64);
            }
            3 => {
                if let Some(u) = self.utc {
                    put(&mut p, 49, 32, quantize(u.A0, -30));
                    put(&mut p, 81, 24, quantize(u.A1, -50));
                    put(&mut p, 105, 8, u.TLS as i64);
                    put(&mut p, 113, 8, u.tot as i64);
                    put(&mut p, 121, 8, u.WN as i64);
                    put(&mut p, 129, 8, u.WNLSF as i64);
                    put(&mut p, 137, 3, u.DN as i64);
                    put(&mut p, 140, 8, u.TLSF as i64);
                    // GST == GPST в модели генератора: A0G/A1G нулевые, общий epoch.
                    put(&mut p, 148, 8, u.tot as i64);
                    put(&mut p, 184, 6, u.WN as i64);
                }
                put(&mut p, 190, 20, tow as i64);
            }
            _ => unreachable!(),
        }
        p
    }
}
