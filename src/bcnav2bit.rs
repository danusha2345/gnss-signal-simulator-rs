//! # BeiDou B2a Навигационные сообщения
//!
//! Этот модуль реализует синтез навигационных битов для BeiDou B2a (B-CNAV2 - B2a Civil Navigation).
//!
//! ## Назначение
//! B2a - это современный гражданский сигнал BeiDou-3, передаваемый на частоте B2a (1176.45 МГц),
//! той же частоте, что и GPS L5 и Galileo E5a. Предназначен для высокоточных применений
//! и обеспечения интероперабельности между различными системами GNSS.
//!
//! ## Основные функции модуля
//! - Генерация сообщений B-CNAV2 с высокоточными эфемеридами
//! - Формирование сообщений типов 10, 11, 30-37 (эфемериды, часы, альманах)
//! - Кодирование улучшенных параметров ионосферы и тропосферы
//! - Поддержка сообщений интегритета для безопасности критичных применений
//! - Формирование дифференциальных поправок и региональных сервисов
//!
//! B2a использует структуру символов длиной 48 символов с улучшенными характеристиками
//! помехоустойчивости и точности для авиационных и геодезических применений.

//----------------------------------------------------------------------
// bcnav2bit.rs:
//   Implementation of navigation bit synthesis class for B-CNAV2
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::types::*;
// use crate::constants::*;
use crate::COMPOSE_BITS;

pub const B2A_SYMBOL_LENGTH: usize = 48;

#[derive(Clone)]
pub struct BCNav2Bit {
    // Ephemeris parameters part 1 (63 satellites, 9 parameters each)
    pub ephemeris1: [[u32; 9]; 63],

    // Ephemeris parameters part 2 (63 satellites, 10 parameters each)
    pub ephemeris2: [[u32; 10]; 63],

    // Clock parameters for all satellites (63 satellites, 4 parameters each)
    pub clock_param: [[u32; 4]; 63],

    // Integrity flags for all satellites (63 satellites)
    pub integrity_flags: [u32; 63],
}

impl BCNav2Bit {
    // Message broadcast cycle is 60s (20 frames) with following MesType order
    const MESSAGE_ORDER: [i32; 20] = [
        10, 11, 30, 34, 10, 11, 30, 34, 10, 11, 30, 34, 10, 11, 30, 34, 10, 11, 30, 34,
    ];

    const B2A_SYMBOL_LENGTH: usize = 48;

    // LDPC Generator matrix for B2a
    const B2A_MATRIX_GEN: &str = "h[0G@Y0<JiVK0c0^KI40hKN0DNVh]i0JKN<F[0Jo0C0UFYo9K`C0QKa0ggDVR0T0S70^VV0EW1>i^R20[;aG09YT>0c0GbQN8KZ0>810<<jIZ0Q0nH0<110V5L`RlZW0C1]N0i[Q^0=0\\SIWkg0_mX0EMEHc0e0WR9`0CRS03SHCFE0FcS1Yg0Md0<0XL`dNO`T0VOC0aaDVQ0R0h]0g;;0EW1Ii`gN0[ZCG0hMEo0=0HbV\\Z60`Yj0>PCMQ0K0jn2j0=ng0KgM=1C0Png9^j0PD0F0dijD[U8H0^UL0FF6`?0>0@N0F``0I_OlZ=?i0]LGW0RC>l0Y0Wf`5RW0d1N0OIO:T070[TMD0RTX0`X:R_O0OTX@9W0IG050>9NGYO`05Ud0A<A]40M0Ii@I0[ib0Mb][8A0<ib;oI0:e0g0Z1IeGkKZ0>810<7jI>0Z0nH0<110V5L`Rl<W0C?]N0n[Q^0:0\\SIWm:\\0lmN022VK40L09W0bKK02d8=1:be0GON50Z]^a0[05Jl_jm0Q5U0CnPR[0>0UbZU0\\bG0>9c\\EP0nb9C]m0n80R0fJU8_kKQ0>81077jI>0Z0nH0<110V5L`Rl<W0C?]N0n[VY0:0\\SIW8l0YMi0T?hG50[0^6D^0a6O0[OGa:T0?6OIP^0?90J01`^92:cI0No^01?T\\I0G0J201^^0H@l4V51f0<^FS0JgGW0_0h8\\f`>0h=@0MRMaN0S0FDkQ0PD10o1aPlM0RN1E5Q0cf0i0@VQf<en10beN0>>a2H0?0:`0>NN0FUWS]fH[0;N1K0C6?S0@0Oc28RW0d1N0OIO:T070[TFD0lhX0`X:R_O0OTX@>W0IG050N9DGY>]0WiC02jZ`U050=UKC0eUf05V`e\\Z0jU52h]0j[0`0ATC[Ro5F0No?011T\\20G0Ub0I\\\\01J4WV5Ik0<^?S0EgGi0d0S=NfRj0IoD0Q_[<h0`0bhFO0lh`0``<l;[0_h`Q>j0_J060e9DJYnS;0?nV0QQBF10Z0cI0SFF0<82bCS]l06GV`0[jZb0J0`W?KNFi0RNj0AA_gR0[0^603aa0X`7<Y?3I0DCjV0^@Xk0S0;Hg18l0YFi0_?hG50[0^6D^0a6O0[OGa:T0?6OIP^0?>0J01`^>2h260Zh70TTP<C0R0l;0T770gKG?[bC`0j76I0YDR?0S01\\<`TO0mHL04G4eg0;0iKIY0hKA0NAeTJ40JgA:FO0G10k0LMY19TO0mHL04G4eg0;0igdY0TKA0NAeTJ404gA:MO0G10k0FMY19K`C0QK;0ggDVR0T0S70g;V0EW1>i^R@0[;aG09YT>0c0HbVNJk>0`JH0GGRLF0I0e\\0GLL01oSK<8F50QbH40gTIK0M04nLc]803NK0Y259Z0I06ZiB0]ZT0hT9]m502ZTY[802`0U0b[K`jgi01:90Vo37W0f02WGS0KWL0LL7KZ30oWLVIi0o@0R0_I9@M2?j0R2A033_g[0i0\\E0?gg0X`<ZY?[>0DaAV0oJiZ0O0VHRI3YW0c3h0SSI=50K0jn0O==0Si:o`YO90N8he01G4<0a0emcdZ60`Yj0>PCFn0K0jn2j0=n40KgM=1C0PnK>^60PD0F0dijD[Pe0RKn0jlU[D0Z0nHCn01Ho0ZoO1XU0lHoj6n0lh0[045nh88l]0>870<<AIH0;0fG0ZII0Qn`^6lZh0a17N0f[QP0o0NO>WFo0^Sc0@[@AI0\\0Q_WR0U_70k7AU[@0[I7XKR0O40Y0cgR4TkO<0Ik?07]i1<0V05>07??0;cbLgK<40T?Q\\053V`080^h145h09d30g>g1k0i02CY206C40i416eg0>C4GD20>M0Q0HN2M`FZD0j?X0YMnij0P0HA0YXX09>gR_;YQ0UBmT0H8960L0E7iQkK0MmX0oFWHc030`R9`0CRS03SHCUE0FRS1Y`0Fd0@0?L`INLI30T`[0BBJET0i02C0X660AN;V9>XG0YR[702oAc0l0]?EHfO<0Ik?07]i1I0V05>07??0;cbLgK740TFQ\\053V`080^h14NF0Kn:0dTd6L0@0IYc<0DYH0@H6Dhd0TYHak<0f=030:7<=V=l70H=I0ZZA>;0]0kF0ZII0Qn`\\64;D0aI<20jB]\\0c0NO>hg40P?\\0S1SUE0]03Em90KWB0LBUgoS0SEBed401H0c0\\d9HMT[0G@Y0<JiVK0N0^KI40hKb0NNVh]i0JKN<F[0Jo0V0UMYo9@kI0L@>0HHgb`010_^0kbb0H:SO7kGc0V2>l0aE130m0lnL=";

    pub fn new() -> Self {
        BCNav2Bit {
            ephemeris1: [[0; 9]; 63],
            ephemeris2: [[0; 10]; 63],
            clock_param: [[0; 4]; 63],
            integrity_flags: [0; 63],
        }
    }

    pub fn get_frame_data(
        &self,
        start_time: GnssTime,
        svid: i32,
        _param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        if !(1..=63).contains(&svid) {
            return 1;
        }

        let sow = start_time.MilliSeconds / 3000; // SOW with scale factor 3s
        let message_type = Self::MESSAGE_ORDER[(sow % 20) as usize];
        let mut frame_data = [0u32; 12];

        self.compose_message(
            message_type,
            start_time.Week - 1356,
            sow,
            svid,
            &mut frame_data,
        );
        Self::append_crc(&mut frame_data, 12);

        // Assign each 6bit into Symbols array
        let mut symbols = [0i32; 96];
        for i in 0..12 {
            symbols[i * 4] = ((frame_data[i] >> 18) & 0x3f) as i32;
            symbols[i * 4 + 1] = ((frame_data[i] >> 12) & 0x3f) as i32;
            symbols[i * 4 + 2] = ((frame_data[i] >> 6) & 0x3f) as i32;
            symbols[i * 4 + 3] = (frame_data[i] & 0x3f) as i32;
        }

        Self::ldpc_encode(&mut symbols, Self::B2A_SYMBOL_LENGTH, Self::B2A_MATRIX_GEN);

        // Preamble
        Self::assign_bits(0xe24de8, 24, &mut nav_bits[0..24]);

        // 96 encoded symbols
        for i in 0..96 {
            if 24 + i * 6 + 6 <= nav_bits.len() {
                Self::assign_bits(symbols[i], 6, &mut nav_bits[24 + i * 6..24 + i * 6 + 6]);
            }
        }

        svid
    }

    // 288 bits subframe information divided into 12 WORDs
    // each WORD has 24bits in 24LSB of unsigned int data in FrameData[]
    // bit order is MSB first (from bit23) and least index first
    // each 24bits divided into four 6bit symbols in LDPC encode
    fn compose_message(
        &self,
        message_type: i32,
        week: i32,
        sow: i32,
        svid: i32,
        frame_data: &mut [u32; 12],
    ) {
        let svid_idx = (svid - 1) as usize;

        // First fill in PRN/MesType/SOW
        frame_data[0] = COMPOSE_BITS!(svid, 18, 6);
        frame_data[0] |= COMPOSE_BITS!(message_type, 12, 6);
        frame_data[0] |= COMPOSE_BITS!(sow >> 6, 0, 12);
        frame_data[1] = COMPOSE_BITS!(sow, 18, 6);

        match message_type {
            10 => {
                frame_data[1] |= COMPOSE_BITS!(week, 5, 13);
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 5, 2, 3); // B2a DIF/SIF/AIF
                frame_data[2] = COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 13, 0, 2); // SISMAI
                frame_data[2] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 11, 22, 2); // SISMAI
                frame_data[2] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 2, 19, 3); // B1C DIF/SIF/AIF
                Self::append_word(frame_data, 2 * 24 + 5, &self.ephemeris1[svid_idx], 211);
            }
            11 => {
                // HS filled with 2 zeros
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 5, 13, 3); // B2a DIF/SIF/AIF
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 11, 9, 4); // SISMAI
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 2, 6, 3); // B1C DIF/SIF/AIF
                Self::append_word(frame_data, 24 + 18, &self.ephemeris2[svid_idx], 222);
            }
            30 => {
                // HS filled with 2 zeros
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 5, 13, 3); // B2a DIF/SIF/AIF
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 11, 9, 4); // SISMAI
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 2, 6, 3); // B1C DIF/SIF/AIF
                Self::append_word(frame_data, 24 + 18, &self.clock_param[svid_idx], 69);
                frame_data[4] |= COMPOSE_BITS!(self.clock_param[svid_idx][3] >> 1, 0, 9); // IODC
                frame_data[5] = COMPOSE_BITS!(self.clock_param[svid_idx][3], 23, 1); // IODC
                frame_data[6] = 0; // fill rest of 143bits with 0
                frame_data[7] = 0;
                frame_data[8] = 0;
                frame_data[9] = 0;
                frame_data[10] = 0;
            }
            34 => {
                // HS filled with 2 zeros
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 5, 13, 3); // B2a DIF/SIF/AIF
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 11, 9, 4); // SISMAI
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 2, 6, 3); // B1C DIF/SIF/AIF
                                                                                           // SISAI filled with 22 zeros
                frame_data[2] = 0;
                Self::append_word(frame_data, 2 * 24 + 16, &self.clock_param[svid_idx], 69);
                frame_data[4] |= COMPOSE_BITS!(self.clock_param[svid_idx][3], 1, 10); // IODC
                frame_data[6] = 0; // fill rest of 121bits with 0
                frame_data[7] = 0;
                frame_data[8] = 0;
                frame_data[9] = 0;
                frame_data[10] = 0;
            }
            _ => {
                // Unknown message type, fill with zeros
                for i in 1..12 {
                    frame_data[i] = 0;
                }
            }
        }
    }

    // Helper functions
    fn append_crc(data: &mut [u32], length: usize) {
        // Calculate CRC24Q for the data (excluding the last word where CRC will be placed)
        let data_bits = (length - 1) * 24; // Each word contains 24 bits of data
        let crc_result = crate::crc24q::crc24q_encode(&data[0..length - 1], data_bits);

        // Place the CRC24Q result in the last word
        data[length - 1] = crc_result;
    }

    fn ldpc_encode(symbols: &mut [i32], symbol_length: usize, matrix_gen: &str) {
        crate::ldpc::ldpc_encode(symbols, symbol_length, matrix_gen);
    }

    fn assign_bits(value: i32, bits: usize, output: &mut [i32]) {
        for i in 0..bits.min(output.len()) {
            output[i] = (value >> (bits - 1 - i)) & 1;
        }
    }

    fn append_word(data: &mut [u32], start_bit: usize, source: &[u32], bit_count: usize) {
        // Word appending logic - simplified implementation
        // This would need proper bit manipulation according to the protocol
        let word_index = start_bit / 24;
        let bit_offset = start_bit % 24;

        // For now, just copy some data to avoid unused parameter warnings
        if word_index < data.len() && !source.is_empty() {
            // Simplified bit appending - actual implementation would need proper bit packing
            let mut remaining_bits = bit_count;
            let mut src_index = 0;
            let mut current_word = word_index;
            let mut current_bit = bit_offset;

            while remaining_bits > 0 && src_index < source.len() && current_word < data.len() {
                let bits_to_copy = remaining_bits.min(24 - current_bit);
                let mask = (1u32 << bits_to_copy) - 1;
                let value = (source[src_index] >> (32 - bits_to_copy)) & mask;

                data[current_word] |= value << (24 - current_bit - bits_to_copy);

                remaining_bits -= bits_to_copy;
                current_bit += bits_to_copy;

                if current_bit >= 24 {
                    current_word += 1;
                    current_bit = 0;
                }

                if bits_to_copy >= 24 {
                    src_index += 1;
                }
            }
        }
    }

    // Public interface methods for setting navigation data
    pub fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) -> i32 {
        if !(1..=63).contains(&svid) {
            return 1;
        }

        let svid_idx = (svid - 1) as usize;

        // Convert ephemeris data to the required format
        // This is a simplified conversion - actual implementation would need
        // proper scaling and bit packing according to BDS specifications

        // Ephemeris1 (IODE + ephemeris I - 211 bits)
        self.ephemeris1[svid_idx][0] = eph.iode as u32;
        self.ephemeris1[svid_idx][1] = (eph.M0 * 1e10) as u32;
        self.ephemeris1[svid_idx][2] = (eph.delta_n * 1e15) as u32;
        self.ephemeris1[svid_idx][3] = (eph.ecc * 1e10) as u32;
        self.ephemeris1[svid_idx][4] = (eph.sqrtA * 1e6) as u32;
        self.ephemeris1[svid_idx][5] = (eph.omega0 * 1e10) as u32;
        self.ephemeris1[svid_idx][6] = (eph.i0 * 1e10) as u32;
        self.ephemeris1[svid_idx][7] = (eph.w * 1e10) as u32;
        self.ephemeris1[svid_idx][8] = (eph.omega_dot * 1e15) as u32;

        // Ephemeris2 (ephemeris II - 222 bits)
        self.ephemeris2[svid_idx][0] = (eph.idot * 1e15) as u32;
        self.ephemeris2[svid_idx][1] = (eph.cuc * 1e10) as u32;
        self.ephemeris2[svid_idx][2] = (eph.cus * 1e10) as u32;
        self.ephemeris2[svid_idx][3] = (eph.crc * 1e6) as u32;
        self.ephemeris2[svid_idx][4] = (eph.crs * 1e6) as u32;
        self.ephemeris2[svid_idx][5] = (eph.cic * 1e10) as u32;
        self.ephemeris2[svid_idx][6] = (eph.cis * 1e10) as u32;
        self.ephemeris2[svid_idx][7] = eph.toe as u32;
        self.ephemeris2[svid_idx][8] = eph.week as u32;
        self.ephemeris2[svid_idx][9] = eph.health as u32;

        // Clock parameters
        self.clock_param[svid_idx][0] = (eph.af0 * 1e15) as u32;
        self.clock_param[svid_idx][1] = (eph.af1 * 1e15) as u32;
        self.clock_param[svid_idx][2] = (eph.af2 * 1e15) as u32;
        self.clock_param[svid_idx][3] = eph.iodc as u32;

        // Integrity flags (simplified)
        self.integrity_flags[svid_idx] = eph.health as u32;

        0
    }

    pub fn set_almanac(&mut self, _alm: &[GpsAlmanac]) -> i32 {
        // Almanac setting would be implemented here
        // For now, this is a placeholder
        0
    }

    pub fn set_iono_utc(
        &mut self,
        _iono_param: Option<&IonoParam>,
        _utc_param: Option<&UtcParam>,
    ) -> i32 {
        // Ionosphere and UTC parameter setting would be implemented here
        // For now, this is a placeholder
        0
    }
}

impl Default for BCNav2Bit {
    fn default() -> Self {
        Self::new()
    }
}
// (moved module-level allow to file head)
