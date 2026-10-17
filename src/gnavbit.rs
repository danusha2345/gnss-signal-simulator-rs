//! # ГЛОНАСС G/NAV Навигационные сообщения
//!
//! Этот модуль реализует синтез навигационных битов для ГЛОНАСС G/NAV (GLONASS Navigation).
//!
//! ## Назначение
//! G/NAV - это формат навигационных сообщений российской системы ГЛОНАСС, передаваемый на частотах
//! L1 (около 1602 МГц) и L2 (около 1246 МГц) с частотным разделением каналов (FDMA).
//! Каждый спутник ГЛОНАСС использует свою уникальную частоту в пределах выделенной полосы.
//!
//! ## Основные функции модуля
//! - Генерация строк навигационной информации с эфемеридами (строки 1-4)
//! - Формирование альманаха для всех спутников системы (строки 6-15)
//! - Кодирование параметров времени и связи шкал времени
//! - Формирование служебной информации о состоянии спутников
//! - Поддержка дифференциального режима и коррекций
//!
//! G/NAV использует суперкадр из 25 кадров, каждый кадр содержит 5 строк по 100 символов,
//! обеспечивая полный цикл передачи навигационной информации за 2.5 минуты.

//----------------------------------------------------------------------
// gnavbit.rs:
//   Implementation of navigation bit synthesis class for GLONASS G/NAV
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#![allow(non_snake_case)]

use crate::gps_time_to_utc;
use crate::types::*;
use crate::COMPOSE_BITS;
// use crate::constants::*; // Unused import

#[derive(Clone)]
pub struct GNavBit {
    pub StringEph: [[[u32; 3]; 4]; 24], // 24 satellites, 4 strings, 3 words each
    pub StringAlm: [[[u32; 3]; 11]; 24], // 24 satellites, 11 almanac strings, 3 words each
    pub last_bit: [i32; 24],            // Last bit for each satellite
}

impl GNavBit {
    pub fn new() -> Self {
        GNavBit {
            StringEph: [[[0; 3]; 4]; 24],
            StringAlm: [[[0; 3]; 11]; 24],
            last_bit: [0; 24],
        }
    }

    pub fn get_frame_data(
        &self,
        start_time: GnssTime,
        svid: i32,
        _param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        if !(1..=24).contains(&svid) || nav_bits.len() < 85 {
            return -1;
        }
        let utc = gps_time_to_utc(start_time, true);
        let time = crate::gnsstime::utc_to_glonass_time_corrected(utc);
        let string_idx = (time.MilliSeconds % 30000 / 2000) as usize;
        let mut data = if string_idx < 4 {
            self.StringEph[svid as usize - 1][string_idx]
        } else {
            self.StringAlm[svid as usize - 1][string_idx - 4]
        };
        Self::glo_set_field(&mut data, 81, 84, string_idx as u32 + 1);
        Self::glo_set_field(&mut data, 85, 85, 0);
        if string_idx == 0 {
            let seconds = time.MilliSeconds / 1000;
            let tk = ((seconds / 3600) << 7) | ((seconds / 60 % 60) << 1) | ((seconds % 60) / 30);
            Self::glo_set_field(&mut data, 65, 76, tk as u32);
        }
        if string_idx == 4 {
            // Модель синхронизирована с UTC: только дробные tau_c/tau_GPS равны 0.
            data = [0; 3];
            Self::glo_set_field(&mut data, 81, 84, 5);
            Self::glo_set_field(
                &mut data,
                70,
                80,
                ((time.Day - 1).rem_euclid(1461) + 1) as u32,
            );
            Self::glo_set_field(&mut data, 32, 36, (time.LeapYear + 1) as u32);
        }
        Self::glo_set_field(&mut data, 1, 8, 0);
        let checksum = self.CheckSum(&data);
        Self::glo_set_field(&mut data, 1, 8, checksum);
        for (i, out) in nav_bits[..85].iter_mut().enumerate() {
            *out = ((data[i / 32] >> (31 - i % 32)) & 1) as i32;
        }
        // Здесь ровно 85 исходных битов, включая idle первым и 8 check bits последними.
        // Относительное и bi-binary кодирование выполняет SatelliteSignal.
        0
    }

    // Generate GLONASS time marker (shortened PRS)
    // According to GLONASS ICD, time marker is a 30-bit sequence
    pub fn get_time_marker(time_marker_bits: &mut [i32]) {
        // GLONASS time marker according to GLONASS ICD v5.1:
        // 111110001101110101000010010110b = 0x3E375096
        // This is a shortened pseudo-random sequence for receiver synchronization
        const TIME_MARK: u32 = 0x3E375096;
        const MARK_LENGTH: usize = 30;

        for i in 0..MARK_LENGTH.min(time_marker_bits.len()) {
            time_marker_bits[i] = ((TIME_MARK >> (29 - i)) & 1) as i32;
        }
    }

    pub fn set_ephemeris(&mut self, _svid: i32, _eph: &GpsEphemeris) -> i32 {
        0
    }

    pub fn set_glonass_ephemeris(&mut self, svid: i32, eph: &GlonassEphemeris) -> i32 {
        if !(1..=24).contains(&svid) || eph.flag == 0 {
            return 0;
        }
        Self::ComposeStringEph(eph, &mut self.StringEph[(svid - 1) as usize]);
        svid
    }

    pub fn set_almanac(&mut self, alm: &[GpsAlmanac]) -> i32 {
        // Convert GPS almanac to GLONASS almanac format
        let glo_alm = self.convert_gps_to_glonass_almanac(alm);

        // Find first valid almanac to get NA
        let mut na = 0u32;
        for i in 0..24 {
            if i < glo_alm.len() && glo_alm[i].flag == 1 {
                na = glo_alm[i].day as u32;
                break;
            }
        }

        // For each satellite, fill its part of the almanac
        for sat in 0..24 {
            // String 5 contains common time parameters for all satellites
            self.StringAlm[sat][0][0] = (5 << 16) | COMPOSE_BITS!(na, 5, 11);
            self.StringAlm[sat][0][1] = 0;
            self.StringAlm[sat][0][2] = 0;

            // Each satellite N transmits almanac for 5 satellites:
            // N, N+4, N+8, N+12, N+16 (modulo 24)
            for slot in 0..5 {
                let target_sat = (sat + slot * 4) % 24;
                let string = slot * 2 + 6; // Strings 6,8,10,12,14 for almanac

                if target_sat < glo_alm.len() {
                    // Fill even and odd strings for the given satellite
                    {
                        let (even, odd) = {
                            let alm_slice = &mut self.StringAlm[sat];
                            let (left, right) = alm_slice.split_at_mut(string - 4);
                            (&mut left[string - 5], &mut right[0])
                        };
                        Self::ComposeStringAlm(&glo_alm[target_sat], target_sat + 1, even, odd);
                    }

                    // Add string number and satellite number
                    self.StringAlm[sat][string - 5][0] |=
                        ((string << 16) | ((target_sat + 1) << 8)) as u32;
                    self.StringAlm[sat][string - 4][0] |= ((string + 1) << 16) as u32;
                }
            }
        }

        0
    }

    pub fn set_iono_utc(
        &mut self,
        _iono_param: Option<&IonoParam>,
        utc_param: Option<&UtcParam>,
    ) -> i32 {
        let mut na = 0u32;
        let tau_c = 0i32;
        let mut n4 = 0u32;
        let mut tau_gps = 0i32;
        let mut ln5 = 0u32;

        if let Some(utc) = utc_param {
            if (utc.flag & 0x3) != 0 {
                ln5 = 1;
                if utc.WN >= 834 {
                    let weeks_since_glonass_epoch = utc.WN - 834;
                    n4 = (weeks_since_glonass_epoch / 209) as u32;
                    let days_in_current_period = (weeks_since_glonass_epoch % 209) * 7;
                    na = (days_in_current_period % 1461) as u32;
                }

                if (utc.flag & 0x2) != 0 {
                    let time_diff = 19.0 - utc.TLS as f64 - 10800.0;
                    tau_gps = (time_diff * (1i64 << 30) as f64) as i32;
                }
            }
        }

        // String 5 with UTC parameters is transmitted by all satellites
        for sat in 0..24 {
            self.StringAlm[sat][0][0] = (5 << 16) as u32;
            self.StringAlm[sat][0][0] |= COMPOSE_BITS!(na, 5, 11);
            self.StringAlm[sat][0][0] |= COMPOSE_BITS!(tau_c >> 28, 0, 4);
            self.StringAlm[sat][0][1] = COMPOSE_BITS!(tau_c >> 4, 8, 24);
            self.StringAlm[sat][0][1] |= COMPOSE_BITS!(tau_c, 4, 4);
            self.StringAlm[sat][0][1] |= COMPOSE_BITS!(0, 3, 1);
            self.StringAlm[sat][0][1] |= COMPOSE_BITS!(n4, 0, 3);
            self.StringAlm[sat][0][2] = COMPOSE_BITS!(n4, 30, 2);
            self.StringAlm[sat][0][2] |= COMPOSE_BITS!(tau_gps, 8, 22);
            self.StringAlm[sat][0][2] |= COMPOSE_BITS!(ln5, 7, 1);
        }

        0
    }

    /// Places `value` into a GLONASS string at ICD bit positions `icd_lo..=icd_hi` (the
    /// positions of Table 4.6, where bit 85 is transmitted first). value bit 0 goes to
    /// `icd_lo`, value bit (width-1) to `icd_hi`. Transmission index = 85 - position, so the
    /// 85 string bits occupy data_bits[0..85] in order (bit 85 first) — the convention the
    /// `get_frame_data` extractor reads. The previous code used a 95-position offset, which
    /// shifted every field by 10 bits and put coordinates into the velocity field's slot.
    fn glo_set_field(string: &mut [u32; 3], icd_lo: u32, icd_hi: u32, value: u32) {
        for k in 0..=(icd_hi - icd_lo) {
            {
                let j = 85 - (icd_lo + k); // transmission bit index (ICD 85 -> 0)
                let mask = 1u32 << (31 - (j % 32));
                string[(j / 32) as usize] =
                    (string[(j / 32) as usize] & !mask) | (((value >> k) & 1) << (31 - j % 32));
            }
        }
    }

    /// GLONASS sign-magnitude field (ICD Table 4.5 remark 2: the high bit is the sign,
    /// '0' = '+', '1' = '-'). `value` is in the field's units; `scale_pow2` is the LSB as a
    /// power of two; `width` is the total field width (1 sign + magnitude).
    fn glo_sign_mag(value: f64, scale_pow2: i32, width: u32) -> u32 {
        let mag =
            (value.abs() / 2.0_f64.powi(scale_pow2)).round() as u32 & ((1u32 << (width - 1)) - 1);
        mag | if value < 0.0 { 1u32 << (width - 1) } else { 0 }
    }

    fn ComposeStringEph(ephemeris: &GlonassEphemeris, string: &mut [[u32; 3]; 4]) -> i32 {
        for s in string.iter_mut() {
            *s = [0; 3];
        }

        // String number 'm' (ICD positions 81-84) is present in every string.
        for (i, s) in string.iter_mut().enumerate() {
            Self::glo_set_field(s, 81, 84, (i as u32) + 1);
        }

        // GLONASS coordinates/velocities/accelerations are in km; eph holds SI metres.
        // --- String 1: P1, tk, x, vx, ax ---
        Self::glo_set_field(&mut string[0], 77, 78, (ephemeris.P & 0x3) as u32); // P1
        Self::glo_set_field(&mut string[0], 65, 76, ephemeris.tk as u32); // tk = (h<<7)|(m<<1)|s30
        Self::glo_set_field(
            &mut string[0],
            9,
            35,
            Self::glo_sign_mag(ephemeris.x / 1000.0, -11, 27),
        );
        Self::glo_set_field(
            &mut string[0],
            41,
            64,
            Self::glo_sign_mag(ephemeris.vx / 1000.0, -20, 24),
        );
        Self::glo_set_field(
            &mut string[0],
            36,
            40,
            Self::glo_sign_mag(ephemeris.ax / 1000.0, -30, 5),
        );

        // --- String 2: Bn, P2, tb, y, vy, ay ---
        Self::glo_set_field(&mut string[1], 78, 80, ephemeris.Bn as u32);
        Self::glo_set_field(&mut string[1], 77, 77, ((ephemeris.P >> 2) & 0x1) as u32); // P2
        Self::glo_set_field(&mut string[1], 70, 76, (ephemeris.tb / 900) as u32); // tb: sec -> 15-min units
        Self::glo_set_field(
            &mut string[1],
            9,
            35,
            Self::glo_sign_mag(ephemeris.y / 1000.0, -11, 27),
        );
        Self::glo_set_field(
            &mut string[1],
            41,
            64,
            Self::glo_sign_mag(ephemeris.vy / 1000.0, -20, 24),
        );
        Self::glo_set_field(
            &mut string[1],
            36,
            40,
            Self::glo_sign_mag(ephemeris.ay / 1000.0, -30, 5),
        );

        // --- String 3: P3, gamma, P, ln, z, vz, az ---
        Self::glo_set_field(&mut string[2], 80, 80, ((ephemeris.P >> 3) & 0x1) as u32); // P3
        Self::glo_set_field(
            &mut string[2],
            69,
            79,
            Self::glo_sign_mag(ephemeris.gamma, -40, 11),
        );
        Self::glo_set_field(&mut string[2], 66, 67, ((ephemeris.P >> 4) & 0x3) as u32); // P
        Self::glo_set_field(&mut string[2], 65, 65, 0); // ln
        Self::glo_set_field(
            &mut string[2],
            9,
            35,
            Self::glo_sign_mag(ephemeris.z / 1000.0, -11, 27),
        );
        Self::glo_set_field(
            &mut string[2],
            41,
            64,
            Self::glo_sign_mag(ephemeris.vz / 1000.0, -20, 24),
        );
        Self::glo_set_field(
            &mut string[2],
            36,
            40,
            Self::glo_sign_mag(ephemeris.az / 1000.0, -30, 5),
        );

        // --- String 4: tau_n, dtau_n, En, P4, FT, NT, n, M ---
        Self::glo_set_field(
            &mut string[3],
            59,
            80,
            Self::glo_sign_mag(ephemeris.tn, -30, 22),
        );
        Self::glo_set_field(
            &mut string[3],
            54,
            58,
            Self::glo_sign_mag(ephemeris.dtn, -30, 5),
        );
        Self::glo_set_field(&mut string[3], 49, 53, ephemeris.En as u32);
        Self::glo_set_field(&mut string[3], 34, 34, ((ephemeris.P >> 6) & 0x1) as u32); // P4
        Self::glo_set_field(&mut string[3], 30, 33, ephemeris.Ft as u32);
        Self::glo_set_field(
            &mut string[3],
            16,
            26,
            ((ephemeris.day as u32).saturating_sub(1) % 1461) + 1,
        ); // NT
        Self::glo_set_field(&mut string[3], 11, 15, ephemeris.n as u32);
        Self::glo_set_field(&mut string[3], 9, 10, ephemeris.M as u32);

        0
    }

    fn ComposeStringAlm(
        almanac: &GlonassAlmanac,
        _slot: usize,
        stream_even: &mut [u32; 3],
        stream_odd: &mut [u32; 3],
    ) -> i32 {
        if almanac.flag == 0 {
            stream_even[0] = 0;
            stream_even[1] = 0;
            stream_even[2] = 0;
            stream_odd[0] = 0;
            stream_odd[1] = 0;
            stream_odd[2] = 0;
            return 0;
        }

        stream_even[0] = COMPOSE_BITS!(if almanac.flag != 0 { 0 } else { 1 }, 15, 1);
        stream_even[0] |= COMPOSE_BITS!(1, 13, 2);
        stream_even[0] |= COMPOSE_BITS!(almanac.freq & 0x1F, 8, 5);

        let mut uint_value = Self::UnscaleInt(almanac.clock_error.abs(), -18) as u32;
        uint_value |= if almanac.clock_error < 0.0 { 1 << 9 } else { 0 };
        stream_even[0] |= COMPOSE_BITS!(uint_value >> 2, 0, 8);

        stream_even[1] = COMPOSE_BITS!(uint_value, 30, 2);

        let mut uint_value = Self::UnscaleUint(almanac.lambda.abs(), -20);
        uint_value |= if almanac.lambda < 0.0 { 1 << 20 } else { 0 };
        stream_even[1] |= COMPOSE_BITS!(uint_value, 9, 21);

        let mut uint_value = Self::UnscaleUint(almanac.di.abs(), -20);
        uint_value |= if almanac.di < 0.0 { 1 << 17 } else { 0 };
        stream_even[1] |= COMPOSE_BITS!(uint_value >> 9, 0, 9);

        stream_even[2] = COMPOSE_BITS!(uint_value, 23, 9);

        let uint_value = Self::UnscaleUint(almanac.ecc, -20);
        stream_even[2] |= COMPOSE_BITS!(uint_value, 8, 15);

        let mut uint_value = Self::UnscaleUint(almanac.w.abs(), -15);
        uint_value |= if almanac.w < 0.0 { 1 << 15 } else { 0 };
        stream_odd[0] = COMPOSE_BITS!(uint_value, 0, 16);

        let uint_value = Self::UnscaleUint(almanac.t, -5);
        stream_odd[1] = COMPOSE_BITS!(uint_value, 11, 21);

        let mut uint_value = Self::UnscaleUint(almanac.dt.abs(), -9);
        uint_value |= if almanac.dt < 0.0 { 1 << 21 } else { 0 };
        stream_odd[1] |= COMPOSE_BITS!(uint_value >> 11, 0, 11);

        stream_odd[2] = COMPOSE_BITS!(uint_value, 21, 11);

        let mut uint_value = Self::UnscaleUint(almanac.dt_dot.abs(), -14);
        uint_value |= if almanac.dt_dot < 0.0 { 1 << 6 } else { 0 };
        stream_odd[2] |= COMPOSE_BITS!(uint_value, 14, 7);
        stream_odd[2] |= COMPOSE_BITS!(almanac.freq & 0x1F, 9, 5);
        stream_odd[2] |= COMPOSE_BITS!((if almanac.flag != 0 { 0 } else { 1 }) as u32, 8, 1);

        0
    }

    // Calculate Hamming code for GLONASS string (85 data bits)
    // The checksum is calculated over 85 data bits. The result is 8 parity bits.
    // The bit numbering is from MSB (1) to LSB (85).
    fn CheckSum(&self, data: &[u32; 3]) -> u32 {
        // ICD 4.7 parity masks, MSB-first (также test_glostr в RTKLIB).
        const MASKS: [[u8; 11]; 7] = [
            [
                0x55, 0x55, 0x5A, 0xAA, 0xAA, 0xAA, 0xB5, 0x55, 0x6A, 0xD8, 0x08,
            ],
            [
                0x66, 0x66, 0x6C, 0xCC, 0xCC, 0xCC, 0xD9, 0x99, 0xB3, 0x68, 0x10,
            ],
            [
                0x87, 0x87, 0x8F, 0x0F, 0x0F, 0x0F, 0x1E, 0x1E, 0x3C, 0x70, 0x20,
            ],
            [
                0x07, 0xF8, 0x0F, 0xF0, 0x0F, 0xF0, 0x1F, 0xE0, 0x3F, 0x80, 0x40,
            ],
            [
                0xF8, 0x00, 0x0F, 0xFF, 0xF0, 0x00, 0x1F, 0xFF, 0xC0, 0x00, 0x80,
            ],
            [
                0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x01, 0x00,
            ],
            [
                0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
            ],
        ];
        let mut bytes = [0u8; 11];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = (data[i / 4] >> (24 - 8 * (i % 4))) as u8;
        }
        bytes[9] &= 0xf8;
        bytes[10] = 0;
        let mut parity = 0u32;
        for (i, mask) in MASKS.iter().enumerate() {
            let p = bytes
                .iter()
                .zip(mask)
                .fold(0, |acc, (&b, &m)| acc ^ (b & m).count_ones())
                & 1;
            parity |= p << i;
        }
        let overall = (bytes.iter().map(|b| b.count_ones()).sum::<u32>() + parity.count_ones()) & 1;
        parity | (overall << 7)
    }

    // Helper functions
    fn UnscaleInt(value: f64, scale: i32) -> i32 {
        let scaled = value * (2.0_f64).powi(scale);
        if scaled >= 0.0 {
            (scaled + 0.5) as i32
        } else {
            (scaled - 0.5) as i32
        }
    }

    fn UnscaleUint(value: f64, scale: i32) -> u32 {
        let scaled = value * (2.0_f64).powi(scale);
        (scaled + 0.5) as u32
    }

    // Conversion functions (placeholder implementations)
    #[allow(dead_code)]
    fn convert_gps_to_glonass_ephemeris(&self, gps_eph: &GpsEphemeris) -> GlonassEphemeris {
        // Минимально достаточная конверсия для формирования непустых строк
        let mut glo = GlonassEphemeris::new();
        glo.valid = 1;
        glo.flag = 1;
        glo.slot = ((gps_eph.svid - 1) % 24 + 1) as u8;
        glo.freq = ((gps_eph.svid - 1) % 14) as i8 - 7; // частотный канал в допустимом диапазоне
        glo.tb = 900; // опорное время 15 минут
        glo.tk = 1200; // секунда недели кратно 30с
        glo.day = 100; // произвольный DOY
        glo.gamma = 1e-9;
        glo.tn = 0.0;
        glo.dtn = 0.0;
        // Положение/скорость/ускорение — ненулевые безопасные значения
        glo.x = 1.0e6;
        glo.y = -1.2e6;
        glo.z = 2.0e6;
        glo.vx = 0.0;
        glo.vy = 0.0;
        glo.vz = 0.0;
        glo.ax = 0.0;
        glo.ay = 0.0;
        glo.az = 0.0;
        glo
    }

    fn convert_gps_to_glonass_almanac(&self, gps_alm: &[GpsAlmanac]) -> Vec<GlonassAlmanac> {
        let mut out = Vec::with_capacity(24);
        for i in 0..24 {
            let mut g = GlonassAlmanac {
                flag: 1,
                freq: (i as i8 % 14) - 7,
                leap_year: 0,
                day: 100 + i as i16,
                t: 0.0,
                lambda: 0.1 * (i as f64),
                di: 0.0,
                ecc: 0.0,
                w: 0.0,
                dt: 0.0,
                dt_dot: 0.0,
                clock_error: 0.0,
            };
            // если есть соответствующий GPS альманах — можно добавить здоровье
            if i < gps_alm.len() && gps_alm[i].valid == 0 {
                g.flag = 0;
            }
            out.push(g);
        }
        out
    }
}

impl Default for GNavBit {
    fn default() -> Self {
        Self::new()
    }
}
