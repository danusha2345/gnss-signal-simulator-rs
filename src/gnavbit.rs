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

use crate::gnsstime::utc_to_glonass_time;
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
        let mut data = [0u32; 3];
        let mut data_bits = [0i32; 85];

        // Calculate frame and string index from GLONASS time
        let utc_time = gps_time_to_utc(start_time, true);
        let glonass_time = utc_to_glonass_time(utc_time);
        let _frame_idx = (glonass_time.MilliSeconds / 30000) % 5;
        let string_idx = (glonass_time.MilliSeconds % 30000) / 2000;

        // Get the appropriate data string (ephemeris or almanac)
        if string_idx < 4 {
            // Ephemeris strings (1-4)
            data.copy_from_slice(&self.StringEph[(svid - 1) as usize][string_idx as usize]);
        } else {
            // Almanac strings (5-15)
            data.copy_from_slice(&self.StringAlm[(svid - 1) as usize][(string_idx - 4) as usize]);
        }

        // Add checksum to the data bits
        data[2] |= self.CheckSum(&data);

        // Extract 85 bits from the 3-word data array
        for i in 0..85 {
            data_bits[i] = ((data[i / 32] >> (31 - (i % 32))) & 1) as i32;
        }

        // According to GLONASS ICD, the 85th bit is "idle" and always equals 0
        data_bits[84] = 0;

        // According to GLONASS ICD, navigation data is transmitted directly without relative encoding
        // Relative encoding was removed to comply with the standard
        for i in 0..85 {
            nav_bits[i] = data_bits[i];
        }

        // Add 8 Hamming code bits (already included in data through CheckSum)
        // Hamming code is located in bits 78-85 of the third word data[2]
        for i in 0..8 {
            if 85 + i < nav_bits.len() {
                nav_bits[85 + i] = ((data[2] >> (7 - i)) & 1) as i32;
            }
        }

        // Add 7 padding bits (always zeros)
        for i in 0..7 {
            if 93 + i < nav_bits.len() {
                nav_bits[93 + i] = 0;
            }
        }

        // Total: 85 information + 8 check + 7 padding = 100 bits
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

    fn ComposeStringEph(ephemeris: &GlonassEphemeris, string: &mut [[u32; 3]; 4]) -> i32 {
        // Initialize all strings to zero
        for i in 0..4 {
            for j in 0..3 {
                string[i][j] = 0;
            }
        }

        // String 1
        string[0][0] = COMPOSE_BITS!(1, 17, 4); // m (positions 81-84)
        string[0][0] |= COMPOSE_BITS!(ephemeris.Bn, 16, 1); // Bn (position 80)
        string[0][0] |= COMPOSE_BITS!(ephemeris.P & 0x3, 14, 2); // P1 (positions 77-78)
        string[0][0] |= COMPOSE_BITS!(ephemeris.tk / 30, 2, 12);

        let uint_value = (ephemeris.x.abs() / 2.0_f64.powi(-11)).round() as u32;
        string[0][0] |= COMPOSE_BITS!(uint_value >> 25, 0, 2);
        string[0][1] = COMPOSE_BITS!(uint_value, 7, 25);

        let int_value = (ephemeris.vx / 2.0_f64.powi(-20)).round() as i32;
        string[0][1] |= COMPOSE_BITS!(int_value >> 18, 0, 7);
        string[0][2] = COMPOSE_BITS!(int_value, 14, 18);

        let int_value = (ephemeris.ax / 2.0_f64.powi(-30)).round() as i32;
        string[0][2] |= COMPOSE_BITS!(int_value, 9, 5);

        // String 2
        string[1][0] = COMPOSE_BITS!(2, 17, 4); // m (positions 81-84)
        string[1][0] |= COMPOSE_BITS!(ephemeris.Bn >> 1, 14, 3); // Bn (positions 78-80, upper 3 bits)
        string[1][0] |= COMPOSE_BITS!((ephemeris.P >> 2) & 0x1, 13, 1); // P2 (position 77)
        string[1][0] |= COMPOSE_BITS!(ephemeris.tb / 900, 7, 7);

        let uint_value = (ephemeris.y.abs() / 2.0_f64.powi(-11)).round() as u32;
        string[1][0] |= COMPOSE_BITS!(uint_value >> 21, 0, 7);
        string[1][1] = COMPOSE_BITS!(uint_value, 12, 20);

        let int_value = (ephemeris.vy / 2.0_f64.powi(-20)).round() as i32;
        string[1][1] |= COMPOSE_BITS!(int_value, 0, 12);
        string[1][2] = COMPOSE_BITS!(int_value >> 12, 20, 12);

        let int_value = (ephemeris.ay / 2.0_f64.powi(-30)).round() as i32;
        string[1][2] |= COMPOSE_BITS!(int_value, 15, 5);

        // String 3
        string[2][0] = COMPOSE_BITS!(3, 17, 4); // m (positions 81-84)
        string[2][0] |= COMPOSE_BITS!((ephemeris.P >> 3) & 0x1, 16, 1); // P3 (position 80)

        let int_value = (ephemeris.gamma / 2.0_f64.powi(-40)).round() as i32;
        string[2][0] |= COMPOSE_BITS!(int_value, 5, 11); // γn(tb) (positions 69-79)
        string[2][0] |= COMPOSE_BITS!(0, 4, 1); // ln (position 65) - currently 0
        string[2][0] |= COMPOSE_BITS!((ephemeris.P >> 4) & 0x3, 2, 2); // P (positions 66-67)

        let uint_value = (ephemeris.z.abs() / 2.0_f64.powi(-11)).round() as u32;
        string[2][0] |= COMPOSE_BITS!(uint_value >> 23, 0, 5);
        string[2][1] = COMPOSE_BITS!(uint_value, 10, 22);

        let int_value = (ephemeris.vz / 2.0_f64.powi(-20)).round() as i32;
        string[2][1] |= COMPOSE_BITS!(int_value, 0, 10);
        string[2][2] = COMPOSE_BITS!(int_value >> 10, 22, 14);

        let int_value = (ephemeris.az / 2.0_f64.powi(-30)).round() as i32;
        string[2][2] |= COMPOSE_BITS!(int_value, 17, 5);

        // String 4
        string[3][0] = COMPOSE_BITS!(4, 17, 4);

        let int_value = (ephemeris.tn / 2.0_f64.powi(-30)).round() as i32;
        string[3][0] |= COMPOSE_BITS!(int_value >> 5, 0, 17);
        string[3][1] = COMPOSE_BITS!(int_value, 27, 5);

        let int_value = (ephemeris.dtn / 2.0_f64.powi(-30)).round() as i32;
        string[3][1] |= COMPOSE_BITS!(int_value, 22, 5);
        string[3][1] |= COMPOSE_BITS!(ephemeris.En, 17, 5);
        string[3][1] |= COMPOSE_BITS!((ephemeris.P >> 6) & 0x1, 1, 1); // P4 (position 34)
        string[3][2] = COMPOSE_BITS!(ephemeris.Ft, 28, 4);
        string[3][2] |= COMPOSE_BITS!(ephemeris.day, 17, 11);
        string[3][2] |= COMPOSE_BITS!(ephemeris.n, 12, 5);
        string[3][2] |= COMPOSE_BITS!(ephemeris.M, 10, 2);

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
        let mut data_bits = [0i32; 85];
        let mut parity = [0u8; 8];

        // Extract 85 data bits into a simple array
        Self::AssignBits(data[0] as i32, 21, &mut data_bits[0..21]);
        Self::AssignBits(data[1] as i32, 32, &mut data_bits[21..53]);
        Self::AssignBits(data[2] as i32, 32, &mut data_bits[53..85]);

        // Calculate parity bits p1-p7 based on ICD
        for i in 0..85 {
            if ((i + 1) % 2) != 0 {
                parity[0] ^= data_bits[i] as u8;
            } // p1
            if (i.div_ceil(2) % 2) != 0 {
                parity[1] ^= data_bits[i] as u8;
            } // p2
            if (((i + 1) / 4) % 2) != 0 {
                parity[2] ^= data_bits[i] as u8;
            } // p3
            if (((i + 1) / 8) % 2) != 0 {
                parity[3] ^= data_bits[i] as u8;
            } // p4
            if (((i + 1) / 16) % 2) != 0 {
                parity[4] ^= data_bits[i] as u8;
            } // p5
            if (((i + 1) / 32) % 2) != 0 {
                parity[5] ^= data_bits[i] as u8;
            } // p6
            if (((i + 1) / 64) % 2) != 0 {
                parity[6] ^= data_bits[i] as u8;
            } // p7
        }

        // p8 is the overall parity of data bits + p1-p7
        parity[7] =
            parity[0] ^ parity[1] ^ parity[2] ^ parity[3] ^ parity[4] ^ parity[5] ^ parity[6];
        for i in 0..85 {
            parity[7] ^= data_bits[i] as u8;
        }

        // Assemble the 8-bit checksum
        let mut checksum = 0u32;
        for i in 0..8 {
            checksum |= (parity[i] as u32) << (7 - i);
        }

        checksum
    }

    // Helper functions
    fn AssignBits(value: i32, bits: usize, output: &mut [i32]) {
        for i in 0..bits.min(output.len()) {
            output[i] = (value >> (bits - 1 - i)) & 1;
        }
    }

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
