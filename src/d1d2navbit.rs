//! # BeiDou D1/D2 Навигационные сообщения
//!
//! Этот модуль реализует синтез навигационных битов для BeiDou D1/D2 (BDS-2 Navigation Data).
//!
//! ## Назначение
//! D1/D2 - это формат навигационных сообщений китайской системы BeiDou второго поколения (BDS-2),
//! передаваемый на частотах B1I (1561.098 МГц) и B2I (1207.14 МГц). D1 используется спутниками
//! MEO (Medium Earth Orbit), а D2 - спутниками GEO (Geostationary) и IGSO (Inclined GSO).
//!
//! ## Основные функции модуля
//! - Генерация подкадров D1 с эфемеридной информацией для MEO спутников
//! - Формирование страниц D2 с альманахом и параметрами системы для GEO/IGSO спутников
//! - Кодирование параметров ионосферы и связи времени BDT-UTC
//! - Формирование сообщений о состоянии спутников и здоровье системы
//! - Поддержка региональных дифференциальных поправок
//!
//! D1 использует структуру суперкадров с подкадрами 1-5, аналогичную GPS, в то время как
//! D2 использует страничную структуру для передачи расширенной региональной информации.

//----------------------------------------------------------------------
// d1d2navbit.rs:
//   Implementation of navigation bit synthesis class for BDS2 D1/D2
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::types::*;
use crate::COMPOSE_BITS;

#[derive(Clone)]
pub struct D1D2NavBit {
    pub bds_stream123: [[u32; 27]; 53], // 53 satellites, 3 subframes * 9 words each
    pub bds_stream_alm: [[u32; 9]; 63], // 63 almanac pages, 9 words each
    pub bds_stream_info: [[u32; 9]; 4], // 4 info pages, 9 words each
    pub bds_stream_health: [[u32; 9]; 3], // 3 health pages, 9 words each
    pub bds_stream_d2: [[u32; 40]; 10], // 10 satellites, 10 pages * 4 words each
    pub iono_param_save: IonoParam,
}

impl D1D2NavBit {
    // H matrix:
    // 1 1 1 1 0 1 0 1 1 0 0 1 0 0 0
    // 0 1 1 1 1 0 1 0 1 1 0 0 1 0 0
    // 0 0 1 1 1 1 0 1 0 1 1 0 0 1 0
    // 1 1 1 0 1 0 1 1 0 0 1 0 0 0 1
    const BCH_POLY: [u32; 4] = [0x7ac0, 0x3d60, 0x1eb0, 0x7590];

    pub fn new() -> Self {
        let mut nav_bit = D1D2NavBit {
            bds_stream123: [[0; 27]; 53],
            bds_stream_alm: [[0; 9]; 63],
            bds_stream_info: [[0; 9]; 4],
            bds_stream_health: [[0; 9]; 3],
            bds_stream_d2: [[0; 40]; 10],
            iono_param_save: IonoParam::default(),
        };

        // Fill page number and AlEpID/AmID
        for i in 0..30 {
            // subframe 4 page 1~24 and subframe 5 page1~6
            nav_bit.bds_stream_alm[i][0] = if i < 24 {
                ((i + 1) << 2) as u32
            } else {
                ((i - 23) << 2) as u32
            };
            nav_bit.bds_stream_alm[i][8] = 3; // AlEpID = 11
        }

        for i in 30..43 {
            // subframe 5 page 11~23 for svid 31~43
            nav_bit.bds_stream_alm[i][0] = ((i - 19) << 2) as u32;
            nav_bit.bds_stream_alm[i][8] = 1; // AmID = 01
        }

        for i in 43..56 {
            // subframe 5 page 11~23 for svid 44~56
            nav_bit.bds_stream_alm[i][0] = ((i - 32) << 2) as u32;
            nav_bit.bds_stream_alm[i][8] = 2; // AmID = 01
        }

        for i in 56..63 {
            // subframe 5 page 11~17 for svid 57~63
            nav_bit.bds_stream_alm[i][0] = ((i - 45) << 2) as u32;
            nav_bit.bds_stream_alm[i][8] = 3; // AmID = 01
        }

        for i in 0..4 {
            // subframe 5 page 7~10
            nav_bit.bds_stream_info[0][0] = ((i + 7) << 2) as u32;
        }

        for i in 0..3 {
            // subframe 5 page 24
            nav_bit.bds_stream_health[i][0] = (24 << 2) as u32;
            nav_bit.bds_stream_health[i][6] = ((i + 1) << 15) as u32; // AmID
        }

        nav_bit
    }

    pub fn get_frame_data(
        &self,
        start_time: GnssTime,
        svid: i32,
        _param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        let mut stream = [0u32; 10];
        let d1_data: bool;
        let sow: i32;
        let subframe: i32;
        let page_ext: i32;

        if (1..=5).contains(&svid) || (59..=63).contains(&svid) {
            d1_data = false;
        } else if (6..=58).contains(&svid) {
            d1_data = true;
        } else {
            return 1;
        }

        // First determine the current TOW and subframe number
        let _week = start_time.Week + start_time.MilliSeconds / 604800000;
        let milliseconds = start_time.MilliSeconds % 604800000;

        if d1_data {
            // D1 bit stream
            sow = milliseconds / 1000;
            subframe = ((sow / 6) % 5) + 1;

            if subframe > 3 {
                // subframe 4/5, further determine page number
                let mut page = (sow / 30) % 72;
                page_ext = page / 24;
                page %= 24;

                if subframe == 4 {
                    stream[1..10].copy_from_slice(&self.bds_stream_alm[page as usize][0..9]);
                } else if page < 6 {
                    // subframe 5 page 1~6
                    stream[1..10].copy_from_slice(&self.bds_stream_alm[(page + 24) as usize][0..9]);
                } else if page < 10 {
                    // subframe 5 page 7~10
                    stream[1..10].copy_from_slice(&self.bds_stream_info[(page - 6) as usize][0..9]);
                } else if page < 23 {
                    // subframe 5 page 11~23
                    if page_ext == 0 {
                        // AmID = 1
                        stream[1..10]
                            .copy_from_slice(&self.bds_stream_alm[(page + 20) as usize][0..9]);
                    } else if page_ext == 1 {
                        // AmID = 2
                        stream[1..10]
                            .copy_from_slice(&self.bds_stream_alm[(page + 33) as usize][0..9]);
                    } else if page < 17 {
                        // AmID = 3
                        stream[1..10]
                            .copy_from_slice(&self.bds_stream_alm[(page + 46) as usize][0..9]);
                    } else {
                        // subframe 5 undefined page
                        for i in 1..10 {
                            stream[i] = 0;
                        }
                        stream[1] = ((page + 1) << 24) as u32;
                    }
                } else {
                    // subframe 5 page 24
                    stream[1..10].copy_from_slice(&self.bds_stream_health[page_ext as usize][0..9]);
                }
            } else {
                for i in 0..9 {
                    stream[i + 1] =
                        self.bds_stream123[(svid - 6) as usize][(subframe - 1) as usize * 9 + i];
                }
            }
        } else {
            // D2 bit Stream
            sow = milliseconds / 3000 * 3;
            subframe = ((milliseconds / 600) % 5) + 1;

            if subframe == 1 {
                // subframe 1, further determine page number
                let page = (milliseconds / 3000) % 10;
                let sat_idx = if svid <= 5 { svid - 1 } else { svid - 54 };

                for i in 0..4 {
                    stream[i + 1] = self.bds_stream_d2[sat_idx as usize][page as usize * 4 + i];
                }
            }
        }

        // Add preamble and SOW
        stream[0] = (0x712 << 19) | ((subframe << 12) as u32) | (((sow >> 8) & 0xff0) as u32);
        stream[1] |= ((sow & 0xfff) << 10) as u32;

        stream[0] = Self::get_bch(stream[0]);
        Self::assign_bits(stream[0] as i32, 30, &mut nav_bits[0..30]);

        for i in 1..10 {
            let mut cur_word = Self::get_bch((stream[i] >> 7) & 0x7ff0) << 16;
            cur_word |= Self::get_bch((stream[i] << 4) & 0x7ff0);
            cur_word = Self::interleave(cur_word);
            Self::assign_bits(cur_word as i32, 30, &mut nav_bits[i * 30..(i + 1) * 30]);
        }

        0
    }

    pub fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) -> i32 {
        if eph.valid == 0 {
            return 0;
        }

        if svid < 1 {
            return 0;
        } else if svid < 6 {
            // GEO 1~5
            let iono_param = self.iono_param_save;
            let stream_idx = (svid - 1) as usize;
            let stream_ptr = &mut self.bds_stream_d2[stream_idx];
            Self::compose_bds_stream_d2(eph, &iono_param, stream_ptr);
        } else if svid < 59 {
            // MEO/IGSO 6~58
            let iono_param = self.iono_param_save;
            let idx = (svid - 6) as usize;
            let stream = &mut self.bds_stream123[idx];
            Self::compose_bds_stream123(eph, &iono_param, stream);
        } else if svid < 64 {
            // GEO 59~63
            let iono_param = self.iono_param_save;
            let stream_idx = (svid - 54) as usize;
            let stream = &mut self.bds_stream_d2[stream_idx];
            Self::compose_bds_stream_d2(eph, &iono_param, stream);
        } else {
            return 0;
        }

        svid
    }

    pub fn set_almanac(&mut self, alm: &[GpsAlmanac]) -> i32 {
        let mut toa = 0i32;
        let mut week = 0i32;

        // Fill in almanac page
        for i in 0..63 {
            if i < alm.len() {
                let stream = &mut self.bds_stream_alm[i];
                Self::fill_bds_almanac_page(&alm[i], stream);
                if (alm[i].valid & 1) != 0 {
                    toa = alm[i].toa >> 12;
                    week = alm[i].week & 0xff;
                }
            }
        }

        // Reset info streams
        for i in 0..4 {
            for j in 0..9 {
                self.bds_stream_info[i][j] = 0;
            }
        }

        self.bds_stream_info[0][0] = 7 << 2;
        self.bds_stream_info[1][0] = 8 << 2;

        // Reset health streams
        for i in 0..3 {
            for j in 0..9 {
                self.bds_stream_health[i][j] = 0;
            }
            self.bds_stream_health[i][0] = (24 << 2) as u32;
            self.bds_stream_health[i][6] = ((i + 1) << 15) as u32; // AmID
        }

        // Fill in health pages
        if alm.len() >= 19 {
            Self::fill_bds_health_page(&alm[0..19], 19, &mut self.bds_stream_info[0]);
        }

        if alm.len() >= 30 {
            Self::fill_bds_health_page(&alm[19..30], 11, &mut self.bds_stream_info[1]);
            self.bds_stream_info[1][6] |= COMPOSE_BITS!(toa, 19, 3);
            self.bds_stream_info[1][5] |= COMPOSE_BITS!((toa >> 3), 0, 5);
            self.bds_stream_info[1][5] |= COMPOSE_BITS!(week, 5, 8);
        }

        if alm.len() >= 43 {
            Self::fill_bds_health_page(&alm[30..43], 13, &mut self.bds_stream_health[0]);
        }

        if alm.len() >= 56 {
            Self::fill_bds_health_page(&alm[43..56], 13, &mut self.bds_stream_health[1]);
        }

        if alm.len() >= 63 {
            Self::fill_bds_health_page(&alm[56..63], 7, &mut self.bds_stream_health[2]);
        }

        0
    }

    pub fn set_iono_utc(&mut self, iono_param: &IonoParam, utc_param: &UtcParam) -> i32 {
        if iono_param.flag == 0 || (utc_param.flag & 3) != 3 {
            return 0;
        }

        self.iono_param_save = *iono_param; // save ionosphere parameters to compose subframe1

        let stream = &mut self.bds_stream_info[3]; // UTC parameter in page 10 (indexed at 3)

        stream[0] = COMPOSE_BITS!((utc_param.TLS >> 6), 0, 2);
        stream[0] |= COMPOSE_BITS!(10, 2, 7); // page number 10
        stream[1] = COMPOSE_BITS!(utc_param.WNLSF, 0, 8);
        stream[1] |= COMPOSE_BITS!(utc_param.TLSF, 8, 8);
        stream[1] |= COMPOSE_BITS!((utc_param.TLS & 0x3f), 16, 6);

        let value = Self::unscale_double(utc_param.A0, -30);
        let int_value = Self::roundi(value);
        stream[2] = COMPOSE_BITS!((int_value >> 10), 0, 22);
        stream[3] = COMPOSE_BITS!((int_value & 0x3ff), 12, 10);

        let value = Self::unscale_double(utc_param.A1, -50);
        let int_value = Self::roundi(value);
        stream[3] |= COMPOSE_BITS!((int_value >> 12), 0, 12);
        stream[4] = COMPOSE_BITS!((int_value & 0xfff), 10, 12);
        stream[4] |= COMPOSE_BITS!(utc_param.DN, 2, 8);

        0
    }

    fn compose_bds_stream123(
        ephemeris: &GpsEphemeris,
        iono_param: &IonoParam,
        stream: &mut [u32; 27],
    ) -> i32 {
        // subframe 1, Stream[0]~Stream[8]
        stream[0] = COMPOSE_BITS!(ephemeris.health, 9, 1);
        stream[0] |= COMPOSE_BITS!(ephemeris.iodc, 4, 5);
        stream[0] |= COMPOSE_BITS!(ephemeris.ura, 0, 4);
        stream[1] = COMPOSE_BITS!(ephemeris.week, 9, 13);

        let int_value = ephemeris.toc >> 3;
        stream[1] |= COMPOSE_BITS!(int_value >> 8, 0, 9);
        stream[2] = COMPOSE_BITS!(int_value, 14, 8);

        let int_value = Self::roundi(ephemeris.tgd * 1e10);
        stream[2] |= COMPOSE_BITS!(int_value, 4, 10);

        let int_value = Self::roundi(ephemeris.tgd2 * 1e10);
        stream[2] |= COMPOSE_BITS!(int_value >> 6, 0, 4);
        stream[3] = COMPOSE_BITS!(int_value, 16, 6);

        let value = Self::unscale_double(iono_param.a0, -30);
        let int_value = Self::roundi(value);
        stream[3] |= COMPOSE_BITS!(int_value, 8, 8);

        let value = Self::unscale_double(iono_param.a1, -27);
        let int_value = Self::roundi(value);
        stream[3] |= COMPOSE_BITS!(int_value, 0, 8);

        let value = Self::unscale_double(iono_param.a2, -24);
        let int_value = Self::roundi(value);
        stream[4] = COMPOSE_BITS!(int_value, 14, 8);

        let value = Self::unscale_double(iono_param.a3, -24);
        let int_value = Self::roundi(value);
        stream[4] |= COMPOSE_BITS!(int_value, 6, 8);

        let value = Self::unscale_double(iono_param.b0, 11);
        let int_value = Self::roundi(value);
        stream[4] |= COMPOSE_BITS!(int_value >> 2, 0, 6);
        stream[5] = COMPOSE_BITS!(int_value, 20, 2);

        let value = Self::unscale_double(iono_param.b1, 14);
        let int_value = Self::roundi(value);
        stream[5] |= COMPOSE_BITS!(int_value, 12, 8);

        let value = Self::unscale_double(iono_param.b2, 16);
        let int_value = Self::roundi(value);
        stream[5] |= COMPOSE_BITS!(int_value, 4, 8);

        let value = Self::unscale_double(iono_param.b3, 16);
        let int_value = Self::roundi(value);
        stream[5] |= COMPOSE_BITS!(int_value >> 4, 0, 4);
        stream[6] = COMPOSE_BITS!(int_value, 18, 4);

        let value = Self::unscale_double(ephemeris.af2, -66);
        let int_value = Self::roundi(value);
        stream[6] |= COMPOSE_BITS!(int_value, 7, 11);

        let value = Self::unscale_double(ephemeris.af0, -33);
        let int_value = Self::roundi(value);
        stream[6] |= COMPOSE_BITS!(int_value >> 17, 0, 7);
        stream[7] = COMPOSE_BITS!(int_value, 5, 17);

        let value = Self::unscale_double(ephemeris.af1, -50);
        let int_value = Self::roundi(value);
        stream[7] |= COMPOSE_BITS!(int_value >> 17, 0, 5);
        stream[8] = COMPOSE_BITS!(int_value, 5, 17);
        stream[8] |= COMPOSE_BITS!(ephemeris.iode, 0, 5);

        // subframe 2, Stream[9]~Stream[17]
        let value = Self::unscale_double(ephemeris.delta_n / std::f64::consts::PI, -43);
        let int_value = Self::roundi(value);
        stream[9] = COMPOSE_BITS!(int_value >> 6, 0, 10);
        stream[10] = COMPOSE_BITS!(int_value, 16, 6);

        let value = Self::unscale_double(ephemeris.cuc, -31);
        let int_value = Self::roundi(value);
        stream[10] |= COMPOSE_BITS!(int_value >> 2, 0, 16);
        stream[11] = COMPOSE_BITS!(int_value, 20, 2);

        let value = Self::unscale_double(ephemeris.M0 / std::f64::consts::PI, -31);
        let int_value = Self::roundi(value);
        stream[11] |= COMPOSE_BITS!(int_value >> 12, 0, 20);
        stream[12] = COMPOSE_BITS!(int_value, 10, 12);

        let value = Self::unscale_double(ephemeris.ecc, -33);
        let uint_value = Self::roundu(value);
        stream[12] |= COMPOSE_BITS!(uint_value >> 22, 0, 10);
        stream[13] = COMPOSE_BITS!(uint_value, 0, 22);

        let value = Self::unscale_double(ephemeris.cus, -31);
        let int_value = Self::roundi(value);
        stream[14] = COMPOSE_BITS!(int_value, 4, 18);

        let value = Self::unscale_double(ephemeris.crc, -6);
        let int_value = Self::roundi(value);
        stream[14] |= COMPOSE_BITS!(int_value >> 14, 0, 4);
        stream[15] = COMPOSE_BITS!(int_value, 8, 14);

        let value = Self::unscale_double(ephemeris.crs, -6);
        let int_value = Self::roundi(value);
        stream[15] |= COMPOSE_BITS!(int_value >> 10, 0, 8);
        stream[16] = COMPOSE_BITS!(int_value, 12, 10);

        let value = Self::unscale_double(ephemeris.sqrtA, -19);
        let uint_value = Self::roundu(value);
        stream[16] |= COMPOSE_BITS!(uint_value >> 20, 0, 12);
        stream[17] = COMPOSE_BITS!(uint_value, 2, 20);
        stream[17] |= COMPOSE_BITS!(ephemeris.toe >> 18, 0, 2);

        // subframe 3, Stream[18]~Stream[26]
        stream[18] = COMPOSE_BITS!(ephemeris.toe >> 8, 0, 10);
        stream[19] = COMPOSE_BITS!(ephemeris.toe >> 3, 17, 5);

        let value = Self::unscale_double(ephemeris.i0 / std::f64::consts::PI, -31);
        let int_value = Self::roundi(value);
        stream[19] |= COMPOSE_BITS!(int_value >> 15, 0, 17);
        stream[20] = COMPOSE_BITS!(int_value, 7, 15);

        let value = Self::unscale_double(ephemeris.cic, -31);
        let int_value = Self::roundi(value);
        stream[20] |= COMPOSE_BITS!(int_value >> 11, 0, 7);
        stream[21] = COMPOSE_BITS!(int_value, 11, 11);

        let value = Self::unscale_double(ephemeris.omega_dot / std::f64::consts::PI, -43);
        let int_value = Self::roundi(value);
        stream[21] |= COMPOSE_BITS!(int_value >> 13, 0, 11);
        stream[22] = COMPOSE_BITS!(int_value, 9, 13);

        let value = Self::unscale_double(ephemeris.cis, -31);
        let int_value = Self::roundi(value);
        stream[22] |= COMPOSE_BITS!(int_value >> 9, 0, 9);
        stream[23] = COMPOSE_BITS!(int_value, 13, 9);

        let value = Self::unscale_double(ephemeris.idot / std::f64::consts::PI, -43);
        let int_value = Self::roundi(value);
        stream[23] |= COMPOSE_BITS!(int_value >> 1, 0, 13);
        stream[24] = COMPOSE_BITS!(int_value, 21, 1);

        let value = Self::unscale_double(ephemeris.omega0 / std::f64::consts::PI, -31);
        let int_value = Self::roundi(value);
        stream[24] |= COMPOSE_BITS!(int_value >> 11, 0, 21);
        stream[25] = COMPOSE_BITS!(int_value, 11, 11);

        let value = Self::unscale_double(ephemeris.w / std::f64::consts::PI, -31);
        let int_value = Self::roundi(value);
        stream[25] |= COMPOSE_BITS!(int_value >> 21, 0, 11);
        stream[26] = COMPOSE_BITS!(int_value, 1, 21);

        0
    }

    fn compose_bds_stream_d2(
        ephemeris: &GpsEphemeris,
        iono_param: &IonoParam,
        stream: &mut [u32; 40],
    ) -> i32 {
        // page 1
        stream[0] = COMPOSE_BITS!(1, 6, 4);
        stream[0] |= COMPOSE_BITS!(ephemeris.health, 5, 1);
        stream[0] |= COMPOSE_BITS!(ephemeris.iodc, 0, 5);
        stream[1] = COMPOSE_BITS!(ephemeris.ura, 18, 4);
        stream[1] |= COMPOSE_BITS!(ephemeris.week, 5, 13);

        let int_value = ephemeris.toc >> 3;
        stream[1] |= COMPOSE_BITS!(int_value >> 12, 0, 5);
        stream[2] = COMPOSE_BITS!(int_value, 10, 12);

        let int_value = Self::roundi(ephemeris.tgd * 1e10);
        stream[2] |= COMPOSE_BITS!(int_value, 0, 10);

        let int_value = Self::roundi(ephemeris.tgd2 * 1e10);
        stream[3] = COMPOSE_BITS!(int_value, 12, 10);

        // page 2
        stream[4] = COMPOSE_BITS!(2, 6, 4);

        let value = Self::unscale_double(iono_param.a0, -30);
        let int_value = Self::roundi(value);
        stream[4] |= COMPOSE_BITS!(int_value >> 2, 0, 6);
        stream[4 + 1] = COMPOSE_BITS!(int_value, 20, 2);

        let value = Self::unscale_double(iono_param.a1, -27);
        let int_value = Self::roundi(value);
        stream[4 + 1] |= COMPOSE_BITS!(int_value, 12, 8);

        let value = Self::unscale_double(iono_param.a2, -24);
        let int_value = Self::roundi(value);
        stream[4 + 1] |= COMPOSE_BITS!(int_value, 4, 8);

        let value = Self::unscale_double(iono_param.a3, -24);
        let int_value = Self::roundi(value);
        stream[4 + 1] |= COMPOSE_BITS!(int_value >> 4, 0, 4);
        stream[4 + 2] = COMPOSE_BITS!(int_value, 18, 4);

        let value = Self::unscale_double(iono_param.b0, 11);
        let int_value = Self::roundi(value);
        stream[4 + 2] |= COMPOSE_BITS!(int_value, 10, 8);

        let value = Self::unscale_double(iono_param.b1, 14);
        let int_value = Self::roundi(value);
        stream[4 + 2] |= COMPOSE_BITS!(int_value, 2, 8);

        let value = Self::unscale_double(iono_param.b2, 16);
        let int_value = Self::roundi(value);
        stream[4 + 2] |= COMPOSE_BITS!(int_value >> 6, 0, 2);
        stream[4 + 3] = COMPOSE_BITS!(int_value, 16, 6);

        let value = Self::unscale_double(iono_param.b3, 16);
        let int_value = Self::roundi(value);
        stream[4 + 3] |= COMPOSE_BITS!(int_value, 8, 6);

        // page 3
        stream[2 * 4] = COMPOSE_BITS!(3, 6, 4);

        let value = Self::unscale_double(ephemeris.af0, -33);
        let int_value = Self::roundi(value);
        stream[2 * 4 + 2] = COMPOSE_BITS!(int_value >> 12, 0, 12);
        stream[2 * 4 + 3] = COMPOSE_BITS!(int_value, 10, 12);

        let value = Self::unscale_double(ephemeris.af1, -50);
        let int_value = Self::roundi(value);
        stream[2 * 4 + 3] |= COMPOSE_BITS!(int_value >> 18, 6, 4);

        // page 4
        stream[3 * 4] = COMPOSE_BITS!(4, 6, 4);
        stream[3 * 4] |= COMPOSE_BITS!(int_value >> 12, 0, 6);
        stream[3 * 4 + 1] = COMPOSE_BITS!(int_value, 10, 12);

        let value = Self::unscale_double(ephemeris.af2, -66);
        let int_value = Self::roundi(value);
        stream[3 * 4 + 1] |= COMPOSE_BITS!(int_value >> 1, 0, 11);
        stream[3 * 4 + 2] = COMPOSE_BITS!(int_value, 21, 1);
        stream[3 * 4 + 2] |= COMPOSE_BITS!(ephemeris.iode, 16, 5);

        let value = Self::unscale_double(ephemeris.delta_n / std::f64::consts::PI, -43);
        let int_value = Self::roundi(value);
        stream[3 * 4 + 2] |= COMPOSE_BITS!(int_value, 0, 16);

        let value = Self::unscale_double(ephemeris.cuc, -31);
        let int_value = Self::roundi(value);
        stream[3 * 4 + 3] = COMPOSE_BITS!(int_value >> 4, 8, 14);

        // page 5
        stream[4 * 4] = COMPOSE_BITS!(5, 6, 4);
        stream[4 * 4] |= COMPOSE_BITS!(int_value, 2, 4);

        let value = Self::unscale_double(ephemeris.M0 / std::f64::consts::PI, -31);
        let int_value = Self::roundi(value);
        stream[4 * 4] |= COMPOSE_BITS!(int_value >> 30, 0, 2);
        stream[4 * 4 + 1] = COMPOSE_BITS!(int_value >> 8, 0, 22);
        stream[4 * 4 + 2] = COMPOSE_BITS!(int_value, 14, 8);

        let value = Self::unscale_double(ephemeris.cus, -31);
        let int_value = Self::roundi(value);
        stream[4 * 4 + 2] |= COMPOSE_BITS!(int_value >> 4, 0, 14);
        stream[4 * 4 + 3] = COMPOSE_BITS!(int_value, 18, 4);

        let value = Self::unscale_double(ephemeris.ecc, -33);
        let uint_value = Self::roundu(value);
        stream[4 * 4 + 3] |= COMPOSE_BITS!(uint_value >> 22, 8, 10);

        // page 6
        stream[5 * 4] = COMPOSE_BITS!(6, 6, 4);
        stream[5 * 4] |= COMPOSE_BITS!(uint_value >> 16, 0, 6);
        stream[5 * 4 + 1] = COMPOSE_BITS!(uint_value, 6, 16);

        let value = Self::unscale_double(ephemeris.sqrtA, -19);
        let uint_value = Self::roundu(value);
        stream[5 * 4 + 1] |= COMPOSE_BITS!(uint_value >> 26, 0, 6);
        stream[5 * 4 + 2] = COMPOSE_BITS!(uint_value >> 4, 0, 22);
        stream[5 * 4 + 3] = COMPOSE_BITS!(uint_value, 18, 4);

        let value = Self::unscale_double(ephemeris.cic, -31);
        let int_value = Self::roundi(value);
        stream[5 * 4 + 3] |= COMPOSE_BITS!(int_value >> 8, 8, 10);

        // page 7
        stream[6 * 4] = COMPOSE_BITS!(7, 6, 4);
        stream[6 * 4] |= COMPOSE_BITS!(int_value >> 2, 0, 6);
        stream[6 * 4 + 1] = COMPOSE_BITS!(int_value, 20, 2);

        let value = Self::unscale_double(ephemeris.cis, -31);
        let int_value = Self::roundi(value);
        stream[6 * 4 + 1] |= COMPOSE_BITS!(int_value, 2, 18);
        stream[6 * 4 + 1] |= COMPOSE_BITS!(ephemeris.toe >> 18, 0, 2);
        stream[6 * 4 + 2] = COMPOSE_BITS!(ephemeris.toe >> 3, 7, 15);

        let value = Self::unscale_double(ephemeris.i0 / std::f64::consts::PI, -31);
        let int_value = Self::roundi(value);
        stream[6 * 4 + 2] |= COMPOSE_BITS!(int_value >> 25, 0, 7);
        stream[6 * 4 + 3] = COMPOSE_BITS!(int_value >> 11, 8, 14);

        // page 8
        stream[7 * 4] = COMPOSE_BITS!(8, 6, 4);
        stream[7 * 4] |= COMPOSE_BITS!(int_value >> 5, 0, 6);
        stream[7 * 4 + 1] = COMPOSE_BITS!(int_value, 17, 5);

        let value = Self::unscale_double(ephemeris.crc, -6);
        let int_value = Self::roundi(value);
        stream[7 * 4 + 1] |= COMPOSE_BITS!(int_value >> 1, 0, 17);
        stream[7 * 4 + 2] = COMPOSE_BITS!(int_value, 21, 1);

        let value = Self::unscale_double(ephemeris.crs, -6);
        let int_value = Self::roundi(value);
        stream[7 * 4 + 2] |= COMPOSE_BITS!(int_value, 3, 18);

        let value = Self::unscale_double(ephemeris.omega_dot / std::f64::consts::PI, -43);
        let int_value = Self::roundi(value);
        stream[7 * 4 + 2] |= COMPOSE_BITS!(int_value >> 21, 0, 3);
        stream[7 * 4 + 3] = COMPOSE_BITS!(int_value >> 5, 6, 16);

        // page 9
        stream[8 * 4] = COMPOSE_BITS!(9, 6, 4);
        stream[8 * 4] |= COMPOSE_BITS!(int_value, 1, 5);

        let value = Self::unscale_double(ephemeris.omega0 / std::f64::consts::PI, -31);
        let int_value = Self::roundi(value);
        stream[8 * 4] |= COMPOSE_BITS!(int_value >> 31, 0, 1);
        stream[8 * 4 + 1] = COMPOSE_BITS!(int_value >> 9, 0, 22);
        stream[8 * 4 + 2] = COMPOSE_BITS!(int_value, 13, 9);

        let value = Self::unscale_double(ephemeris.w / std::f64::consts::PI, -31);
        let int_value = Self::roundi(value);
        stream[8 * 4 + 2] |= COMPOSE_BITS!(int_value >> 19, 0, 13);
        stream[8 * 4 + 3] = COMPOSE_BITS!(int_value >> 5, 8, 14);

        // page 10
        stream[9 * 4] = COMPOSE_BITS!(10, 6, 4);
        stream[9 * 4] |= COMPOSE_BITS!(int_value, 1, 5);

        let value = Self::unscale_double(ephemeris.idot / std::f64::consts::PI, -43);
        let int_value = Self::roundi(value);
        stream[9 * 4] |= COMPOSE_BITS!(int_value >> 13, 0, 1);
        stream[9 * 4 + 1] = COMPOSE_BITS!(int_value, 9, 13);

        0
    }

    fn fill_bds_almanac_page(almanac: &GpsAlmanac, stream: &mut [u32; 9]) -> i32 {
        if almanac.valid == 0 {
            return 0;
        }

        let value = Self::unscale_double(almanac.sqrtA, -11);
        let uint_value = Self::roundu(value);
        stream[0] |= COMPOSE_BITS!(uint_value >> 22, 0, 2);
        stream[1] = COMPOSE_BITS!(uint_value, 0, 22);

        let value = Self::unscale_double(almanac.af1, -38);
        let int_value = Self::roundi(value);
        stream[2] = COMPOSE_BITS!(int_value, 11, 11);

        let value = Self::unscale_double(almanac.af0, -20);
        let int_value = Self::roundi(value);
        stream[2] |= COMPOSE_BITS!(int_value, 0, 11);

        let value = Self::unscale_double(almanac.omega0 / std::f64::consts::PI, -23);
        let int_value = Self::roundi(value);
        stream[3] = COMPOSE_BITS!(int_value >> 2, 0, 22);
        stream[4] = COMPOSE_BITS!(int_value, 20, 2);

        let value = Self::unscale_double(almanac.ecc, -21);
        let uint_value = Self::roundu(value);
        stream[4] |= COMPOSE_BITS!(uint_value, 3, 17);

        let value = if almanac.i0 > 0.5 {
            Self::unscale_double(almanac.i0 / std::f64::consts::PI - 0.3, -19)
        } else {
            Self::unscale_double(almanac.i0 / std::f64::consts::PI, -19)
        };
        let int_value = Self::roundi(value);
        stream[4] |= COMPOSE_BITS!(int_value >> 13, 0, 3);
        stream[5] = COMPOSE_BITS!(int_value, 9, 13);
        stream[5] |= COMPOSE_BITS!(almanac.toa >> 12, 1, 8);

        let value = Self::unscale_double(almanac.omega_dot / std::f64::consts::PI, -38);
        let int_value = Self::roundi(value);
        stream[5] |= COMPOSE_BITS!(int_value >> 16, 0, 1);
        stream[6] = COMPOSE_BITS!(int_value, 6, 16);

        let value = Self::unscale_double(almanac.w / std::f64::consts::PI, -23);
        let int_value = Self::roundi(value);
        stream[6] |= COMPOSE_BITS!(int_value >> 18, 0, 6);
        stream[7] = COMPOSE_BITS!(int_value, 4, 18);

        let value = Self::unscale_double(almanac.M0 / std::f64::consts::PI, -23);
        let int_value = Self::roundi(value);
        stream[7] |= COMPOSE_BITS!(int_value >> 20, 0, 4);
        stream[8] = COMPOSE_BITS!(int_value, 2, 20);

        0
    }

    fn fill_bds_health_page(almanac: &[GpsAlmanac], length: usize, stream: &mut [u32; 9]) -> i32 {
        // Упрощённая безопасная раскладка 9‑битовых health индикаторов последовательно по 24‑битным словам
        let mut bit_cursor: usize = 0; // от 0 до 9*24-1
        let count = length.min(almanac.len());
        for i in 0..count {
            let health: u32 = if almanac[i].valid == 1 { 0 } else { 0x1FF };
            for k in (0..9).rev() {
                let bit = ((health >> k) & 1) as u32;
                let word = bit_cursor / 24;
                let pos = bit_cursor % 24; // 0..23 (MSB..LSB)
                if word < stream.len() {
                    // ставим бит в позицию (23-pos)
                    stream[word] |= bit << (23 - pos);
                }
                bit_cursor += 1;
                if bit_cursor >= stream.len() * 24 {
                    break;
                }
            }
            if bit_cursor >= stream.len() * 24 {
                break;
            }
        }
        count as i32
    }

    // Calculate BCH and XOR to lowest 4bit
    // to check BCH, after calling this function, the lowest 4bit of return value should be 0
    // to calculate BCH, set lowest 4bit as 0 and return value contains the full word
    fn get_bch(word: u32) -> u32 {
        let mut word = word;

        // calculate parity value
        for i in 0..4 {
            let mut xor_value = word & Self::BCH_POLY[i];
            // calculate bit 1 number
            xor_value = (xor_value & 0x5555) + ((xor_value & 0xaaaa) >> 1);
            xor_value = (xor_value & 0x3333) + ((xor_value & 0xcccc) >> 2);
            xor_value = (xor_value & 0x0f0f) + ((xor_value & 0xf0f0) >> 4);
            xor_value = (xor_value & 0x000f) + ((xor_value & 0x0f00) >> 8);
            if (xor_value & 1) != 0 {
                word ^= 1 << (3 - i);
            }
        }
        word
    }

    fn interleave(data: u32) -> u32 {
        let mut data = data;
        data = (data & 0xff0000ff) | ((data & 0x0000ff00) << 8) | ((data & 0x00ff0000) >> 8);
        data = (data & 0xf00ff00f) | ((data & 0x00f000f0) << 4) | ((data & 0x0f000f00) >> 4);
        data = (data & 0xc3c3c3c3) | ((data & 0x0c0c0c0c) << 2) | ((data & 0x30303030) >> 2);
        data = (data & 0x19999999) | ((data & 0x02222222) << 1) | ((data & 0x44444444) >> 1);
        data
    }

    fn assign_bits(value: i32, bits: usize, output: &mut [i32]) {
        for i in 0..bits.min(output.len()) {
            output[i] = (value >> (bits - 1 - i)) & 1;
        }
    }

    // Helper functions
    fn unscale_double(value: f64, scale: i32) -> f64 {
        value * (2.0_f64).powi(scale)
    }

    fn roundi(value: f64) -> i32 {
        if value >= 0.0 {
            (value + 0.5) as i32
        } else {
            (value - 0.5) as i32
        }
    }

    fn roundu(value: f64) -> u32 {
        (value + 0.5) as u32
    }
}

impl Default for D1D2NavBit {
    fn default() -> Self {
        Self::new()
    }
}
