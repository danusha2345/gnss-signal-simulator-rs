//! # GPS CNAV-2 Navigation Messages
//!
//! GPS L1C CNAV-2 frame synthesis. The bit packing and LDPC path are ported
//! from the upstream SignalSim `CNav2Bit` implementation and use the IS-GPS-800
//! CNAV-2 generator tables in `cnav2_ldpc_tables`.

//----------------------------------------------------------------------
// cnav2bit.rs:
//   Implementation of navigation bit synthesis class for CNAV2
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::cnav2_ldpc_tables::{BCH_TOI_TABLE, L1C_MATRIX_GEN2, L1C_MATRIX_GEN3};
use crate::navbit::NavBit;
use crate::types::*;

const L1C_SUBFRAME2_SYMBOL_LENGTH: usize = 600;
const L1C_SUBFRAME3_SYMBOL_LENGTH: usize = 274;
const PAGE_INDEX_IONO_UTC: usize = 0;
const PI: f64 = std::f64::consts::PI;

#[derive(Clone)]
pub struct CNav2Bit {
    // 576-bit subframe 2 information, 32-bit words, MSB first.
    subframe2: [[u32; 18]; 32],
    // 250-bit subframe 3 pages with zero-filled PRN prefix before transmit PRN insertion.
    subframe3: [[u32; 8]; 64],
    // ISC_L1C/A, ISC_L2C, ISC_L5I5, ISC_L5Q5 packed as in SignalSim CNav2Bit.
    isc: [[u32; 2]; 32],
    eph_valid: [bool; 32],
}

impl Default for CNav2Bit {
    fn default() -> Self {
        Self::new()
    }
}

impl CNav2Bit {
    pub fn new() -> Self {
        CNav2Bit {
            subframe2: [[0; 18]; 32],
            subframe3: [[0; 8]; 64],
            isc: [[0; 2]; 32],
            eph_valid: [false; 32],
        }
    }

    pub fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        _param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        if !(1..=32).contains(&svid) {
            nav_bits.iter_mut().take(1800).for_each(|b| *b = 0);
            return -1;
        }

        nav_bits.iter_mut().take(1800).for_each(|b| *b = 0);

        let svid_idx = (svid - 1) as usize;
        let page = start_time.MilliSeconds / 18_000;
        let itow = page / 400;
        let toi = (page + 1).rem_euclid(400);

        let mut stream = [0u32; 19];
        stream[..18].copy_from_slice(&self.subframe2[svid_idx]);
        stream[0] |= Self::compose_bits(start_time.Week as u32, 19, 13);
        stream[0] |= Self::compose_bits(itow as u32, 11, 8);
        stream[18] = NavBit::crc24q_encode(&stream, 576) << 8;

        let mut bits = [0i32; 1748];
        Self::ldpc_encode(
            &stream,
            &mut bits[..1200],
            L1C_SUBFRAME2_SYMBOL_LENGTH,
            19,
            &L1C_MATRIX_GEN2,
        );

        let subframe3_stream = self.get_subframe3_data(svid_idx, PAGE_INDEX_IONO_UTC);
        Self::ldpc_encode(
            &subframe3_stream,
            &mut bits[1200..],
            L1C_SUBFRAME3_SYMBOL_LENGTH,
            9,
            &L1C_MATRIX_GEN3,
        );

        let mut out = 52usize;
        for i in 0..46 {
            for j in 0..38 {
                if out < nav_bits.len() {
                    nav_bits[out] = bits[j * 46 + i];
                }
                out += 1;
            }
        }

        Self::assign_toi_bch(toi as usize, nav_bits);
        0
    }

    pub fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) -> i32 {
        if !(1..=32).contains(&svid) || eph.valid == 0 {
            return 0;
        }

        let aligned;
        let eph = if eph.toe % 300 != 0 {
            aligned = NavBit::align_toe_300s(eph);
            &aligned
        } else {
            eph
        };

        let svid_idx = (svid - 1) as usize;
        Self::compose_subframe2(eph, &mut self.subframe2[svid_idx], &mut self.isc[svid_idx]);
        self.eph_valid[svid_idx] = true;
        svid
    }

    pub fn set_almanac(&mut self, _alm: &[GpsAlmanac]) -> i32 {
        0
    }

    pub fn set_iono_utc(&mut self, iono_param: &IonoParam, utc_param: &UtcParam) -> i32 {
        let subframe = &mut self.subframe3[PAGE_INDEX_IONO_UTC];
        subframe.fill(0);
        subframe[0] = 1 << 12;

        if iono_param.flag == 0 || (utc_param.flag & 3) != 3 {
            return 0;
        }

        let mut int_value = NavBit::unscale_int(utc_param.A0, -35);
        subframe[0] |= Self::compose_bits((int_value >> 4) as u32, 0, 12);
        subframe[1] = Self::compose_bits(int_value as u32, 28, 4);
        int_value = NavBit::unscale_int(utc_param.A1, -51);
        subframe[1] |= Self::compose_bits(int_value as u32, 15, 13);
        int_value = NavBit::unscale_int(utc_param.A2, -68);
        subframe[1] |= Self::compose_bits(int_value as u32, 8, 7);
        subframe[1] |= Self::compose_bits(utc_param.TLS as u32, 0, 8);
        subframe[2] = Self::compose_bits(utc_param.tot as u32, 24, 8);
        subframe[2] |= Self::compose_bits(utc_param.WN as u32, 3, 13);
        subframe[2] |= Self::compose_bits((utc_param.WNLSF >> 10) as u32, 0, 3);
        subframe[3] = Self::compose_bits(utc_param.WNLSF as u32, 22, 10);
        subframe[3] |= Self::compose_bits(utc_param.DN as u32, 18, 4);
        subframe[3] |= Self::compose_bits(utc_param.TLSF as u32, 10, 8);

        int_value = NavBit::unscale_int(iono_param.a0, -30);
        subframe[3] |= Self::compose_bits(int_value as u32, 2, 8);
        int_value = NavBit::unscale_int(iono_param.a1, -27);
        subframe[3] |= Self::compose_bits((int_value >> 6) as u32, 0, 2);
        subframe[4] = Self::compose_bits(int_value as u32, 26, 6);
        int_value = NavBit::unscale_int(iono_param.a2, -24);
        subframe[4] |= Self::compose_bits(int_value as u32, 18, 8);
        int_value = NavBit::unscale_int(iono_param.a3, -24);
        subframe[4] |= Self::compose_bits(int_value as u32, 10, 8);
        int_value = NavBit::unscale_int(iono_param.b0, 11);
        subframe[4] |= Self::compose_bits(int_value as u32, 2, 8);
        int_value = NavBit::unscale_int(iono_param.b1, 14);
        subframe[4] |= Self::compose_bits((int_value >> 6) as u32, 0, 2);
        subframe[5] = Self::compose_bits(int_value as u32, 26, 6);
        int_value = NavBit::unscale_int(iono_param.b2, 16);
        subframe[5] |= Self::compose_bits(int_value as u32, 18, 8);
        int_value = NavBit::unscale_int(iono_param.b3, 16);
        subframe[5] |= Self::compose_bits(int_value as u32, 10, 8);
        0
    }

    fn compose_subframe2(eph: &GpsEphemeris, subframe2: &mut [u32; 18], isc_data: &mut [u32; 2]) {
        subframe2.fill(0);
        isc_data.fill(0);

        let axis = if eph.axis != 0.0 {
            eph.axis
        } else {
            eph.sqrtA * eph.sqrtA
        };
        let top = if eph.top != 0 { eph.top } else { eph.toe };

        subframe2[0] = Self::compose_bits((top / 300) as u32, 0, 11);
        subframe2[1] = if (eph.health & 0x100) != 0 {
            0x8000_0000
        } else {
            0
        };
        subframe2[1] |= Self::compose_bits(eph.ura as u32, 26, 5);
        subframe2[1] |= Self::compose_bits((eph.toe / 300) as u32, 15, 11);
        let mut int_value = NavBit::unscale_int(axis - 26_559_710.0, -9);
        subframe2[1] |= Self::compose_bits((int_value >> 11) as u32, 0, 15);
        subframe2[2] = Self::compose_bits(int_value as u32, 21, 11);
        int_value = NavBit::unscale_int(eph.axis_dot, -21);
        subframe2[2] |= Self::compose_bits((int_value >> 4) as u32, 0, 21);
        subframe2[3] = Self::compose_bits(int_value as u32, 28, 4);
        int_value = NavBit::unscale_int(eph.delta_n / PI, -44);
        subframe2[3] |= Self::compose_bits(int_value as u32, 11, 17);
        int_value = NavBit::unscale_int(eph.delta_n_dot / PI, -57);
        subframe2[3] |= Self::compose_bits((int_value >> 12) as u32, 0, 11);
        subframe2[4] = Self::compose_bits(int_value as u32, 20, 12);

        let mut long_value = NavBit::unscale_long(eph.M0 / PI, -32);
        let mut high_bit = if ((long_value as u64) & 0x1_0000_0000) != 0 {
            1
        } else {
            0
        };
        let mut uint_value = long_value as u32;
        subframe2[4] |= Self::compose_bits(high_bit, 19, 1);
        subframe2[4] |= Self::compose_bits(uint_value >> 13, 0, 19);
        subframe2[5] = Self::compose_bits(uint_value, 19, 13);

        let ulong_value = NavBit::unscale_ulong(eph.ecc, -34);
        high_bit = if (ulong_value & 0x1_0000_0000) != 0 {
            1
        } else {
            0
        };
        uint_value = ulong_value as u32;
        subframe2[5] |= Self::compose_bits(high_bit, 18, 1);
        subframe2[5] |= Self::compose_bits(uint_value >> 14, 0, 18);
        subframe2[6] = Self::compose_bits(uint_value, 18, 14);

        long_value = NavBit::unscale_long(eph.w / PI, -32);
        high_bit = if ((long_value as u64) & 0x1_0000_0000) != 0 {
            1
        } else {
            0
        };
        uint_value = long_value as u32;
        subframe2[6] |= Self::compose_bits(high_bit, 17, 1);
        subframe2[6] |= Self::compose_bits(uint_value >> 15, 0, 17);
        subframe2[7] = Self::compose_bits(uint_value, 17, 15);

        long_value = NavBit::unscale_long(eph.omega0 / PI, -32);
        high_bit = if ((long_value as u64) & 0x1_0000_0000) != 0 {
            1
        } else {
            0
        };
        uint_value = long_value as u32;
        subframe2[7] |= Self::compose_bits(high_bit, 16, 1);
        subframe2[7] |= Self::compose_bits(uint_value >> 16, 0, 16);
        subframe2[8] = Self::compose_bits(uint_value, 16, 16);

        long_value = NavBit::unscale_long(eph.i0 / PI, -32);
        high_bit = if ((long_value as u64) & 0x1_0000_0000) != 0 {
            1
        } else {
            0
        };
        uint_value = long_value as u32;
        subframe2[8] |= Self::compose_bits(high_bit, 15, 1);
        subframe2[8] |= Self::compose_bits(uint_value >> 17, 0, 15);
        subframe2[9] = Self::compose_bits(uint_value, 15, 17);

        int_value = NavBit::unscale_int(eph.omega_dot / PI + 2.6e-9, -44);
        subframe2[9] |= Self::compose_bits((int_value >> 2) as u32, 0, 15);
        subframe2[10] = Self::compose_bits(int_value as u32, 30, 2);
        int_value = NavBit::unscale_int(eph.idot / PI, -44);
        subframe2[10] |= Self::compose_bits(int_value as u32, 15, 15);
        int_value = NavBit::unscale_int(eph.cis, -30);
        subframe2[10] |= Self::compose_bits((int_value >> 1) as u32, 0, 15);
        subframe2[11] = Self::compose_bits(int_value as u32, 31, 1);
        int_value = NavBit::unscale_int(eph.cic, -30);
        subframe2[11] |= Self::compose_bits(int_value as u32, 15, 16);
        int_value = NavBit::unscale_int(eph.crs, -8);
        subframe2[11] |= Self::compose_bits((int_value >> 9) as u32, 0, 15);
        subframe2[12] = Self::compose_bits(int_value as u32, 23, 9);
        int_value = NavBit::unscale_int(eph.crc, -8);
        subframe2[12] |= Self::compose_bits((int_value >> 1) as u32, 0, 23);
        subframe2[13] = Self::compose_bits(int_value as u32, 31, 1);
        int_value = NavBit::unscale_int(eph.cus, -30);
        subframe2[13] |= Self::compose_bits(int_value as u32, 10, 21);
        int_value = NavBit::unscale_int(eph.cuc, -30);
        subframe2[13] |= Self::compose_bits((int_value >> 11) as u32, 0, 10);
        subframe2[14] = Self::compose_bits(int_value as u32, 21, 11);

        int_value = NavBit::unscale_int(eph.af0, -35);
        subframe2[14] |= Self::compose_bits((int_value >> 16) as u32, 0, 10);
        subframe2[15] = Self::compose_bits(int_value as u32, 16, 16);
        int_value = NavBit::unscale_int(eph.af1, -48);
        subframe2[15] |= Self::compose_bits((int_value >> 4) as u32, 0, 16);
        subframe2[16] = Self::compose_bits(int_value as u32, 28, 4);
        int_value = NavBit::unscale_int(eph.af2, -60);
        subframe2[16] |= Self::compose_bits(int_value as u32, 18, 10);
        int_value = NavBit::unscale_int(eph.tgd_ext[4], -35);
        subframe2[16] |= Self::compose_bits(int_value as u32, 5, 13);
        int_value = NavBit::unscale_int(eph.tgd_ext[4] - eph.tgd_ext[1], -35);
        subframe2[16] |= Self::compose_bits((int_value >> 8) as u32, 0, 5);
        subframe2[17] = Self::compose_bits(int_value as u32, 24, 8);
        int_value = NavBit::unscale_int(eph.tgd_ext[4] - eph.tgd_ext[0], -35);
        subframe2[17] |= Self::compose_bits(int_value as u32, 11, 13);
        subframe2[17] |= Self::compose_bits(eph.week as u32, 2, 8);

        int_value = NavBit::unscale_int(eph.tgd_ext[4] - eph.tgd, -35);
        isc_data[0] = Self::compose_bits(int_value as u32, 13, 13);
        int_value = NavBit::unscale_int(eph.tgd_ext[4] - eph.tgd2, -35);
        isc_data[0] |= Self::compose_bits(int_value as u32, 0, 13);
        int_value = NavBit::unscale_int(eph.tgd_ext[4] - eph.tgd_ext[2], -35);
        isc_data[1] = Self::compose_bits(int_value as u32, 13, 13);
        int_value = NavBit::unscale_int(eph.tgd_ext[4] - eph.tgd_ext[3], -35);
        isc_data[1] |= Self::compose_bits(int_value as u32, 0, 13);
    }

    fn get_subframe3_data(&self, svid_idx: usize, page_index: usize) -> [u32; 9] {
        let mut stream = [0u32; 9];
        stream[..8].copy_from_slice(&self.subframe3[page_index]);
        stream[0] &= 0x3ffff;
        stream[0] |= ((svid_idx + 1) as u32) << 18;

        if page_index == PAGE_INDEX_IONO_UTC {
            stream[5] |= Self::compose_bits(self.isc[svid_idx][0] >> 16, 0, 10);
            stream[6] = Self::compose_bits(self.isc[svid_idx][0], 16, 16);
            stream[6] |= Self::compose_bits(self.isc[svid_idx][1] >> 10, 0, 16);
            stream[7] = Self::compose_bits(self.isc[svid_idx][1], 22, 10);
        }

        stream[8] = NavBit::crc24q_encode(&stream, 250) << 8;
        for i in 0..8 {
            stream[i] <<= 6;
            stream[i] |= stream[i + 1] >> 26;
        }
        stream[8] <<= 6;
        stream
    }

    fn ldpc_encode(
        stream: &[u32],
        bits: &mut [i32],
        symbol_length: usize,
        table_size: usize,
        matrix_gen: &[u32],
    ) {
        debug_assert!(bits.len() >= symbol_length * 2);
        debug_assert!(stream.len() >= table_size);
        debug_assert!(matrix_gen.len() >= symbol_length * table_size);

        for i in 0..(symbol_length / 32) {
            Self::assign_bits(stream[i], 32, &mut bits[i * 32..]);
        }
        let rem_bits = symbol_length & 0x1f;
        if rem_bits != 0 {
            let word = symbol_length / 32;
            Self::assign_bits(
                stream[word] >> (32 - rem_bits),
                rem_bits,
                &mut bits[word * 32..],
            );
        }

        let mut matrix_idx = 0usize;
        for i in 0..symbol_length {
            let mut parity_bit = 0;
            for word in stream.iter().take(table_size) {
                parity_bit ^= Self::xor_bits(*word & matrix_gen[matrix_idx]);
                matrix_idx += 1;
            }
            bits[symbol_length + i] = parity_bit;
        }
    }

    fn assign_toi_bch(toi: usize, nav_bits: &mut [i32]) {
        let msb = if (toi & 0x100) != 0 { 1 } else { 0 };
        if !nav_bits.is_empty() {
            nav_bits[0] = msb;
        }

        let mask19 = if msb != 0 { 0x7ffff } else { 0 };
        let mask32 = if msb != 0 { 0xffff_ffff } else { 0 };
        let code = BCH_TOI_TABLE[toi & 0xff];
        if nav_bits.len() > 1 {
            Self::assign_bits(((code >> 32) as u32) ^ mask19, 19, &mut nav_bits[1..]);
        }
        if nav_bits.len() > 20 {
            Self::assign_bits((code as u32) ^ mask32, 32, &mut nav_bits[20..]);
        }
    }

    fn assign_bits(data: u32, bit_number: usize, bit_stream: &mut [i32]) {
        let mut data = data << (32 - bit_number);
        for slot in bit_stream.iter_mut().take(bit_number) {
            *slot = if (data & 0x8000_0000) != 0 { 1 } else { 0 };
            data <<= 1;
        }
    }

    fn xor_bits(mut data: u32) -> i32 {
        data ^= data >> 1;
        data ^= data >> 2;
        data ^= data >> 4;
        data ^= data >> 8;
        data ^= data >> 16;
        (data & 1) as i32
    }

    #[inline]
    fn compose_bits(value: u32, start: u32, width: u32) -> u32 {
        let mask = if width >= 32 {
            u32::MAX
        } else {
            (1u32 << width) - 1
        };
        (value & mask) << start
    }
}
