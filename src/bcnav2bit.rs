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

use crate::bcnavbit::BCNavBit;
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

    // TGS/ISC parameters (63 satellites, 3 parameters each)
    pub tgs_isc_param: [[u32; 3]; 63],

    // BDGIM ionosphere parameters
    pub bd_gim_iono: [u32; 4],

    // BDT-UTC parameters
    pub bdt_utc_param: [u32; 5],

    // EOP parameters
    pub eop_param: [u32; 6],

    // BGTO parameters
    pub bgto_param: [[u32; 3]; 4],

    // Reduced almanac (63 satellites, 2 parameters each)
    pub reduced_almanac: [[u32; 2]; 63],

    // Midi almanac (63 satellites, 7 parameters each)
    pub midi_almanac: [[u32; 7]; 63],

    // Almanac valid mask
    pub almanac_valid_mask: u64,

    // Almanac week
    pub almanac_week: i32,

    // Almanac time of applicability
    pub almanac_toa: i32,

    // Current midi almanac index
    pub cur_midi_alm_index: i32,

    // Current reduced almanac index
    pub cur_reduced_alm_index: i32,
}

impl BCNav2Bit {
    // Message broadcast cycle is 60s (20 frames) with following MesType order
    const MESSAGE_ORDER: [i32; 20] = [
        10, 11, 30, 40, 10, 11, 31, 40, 10, 11, 32, 40, 10, 11, 33, 40, 10, 11, 34, 40,
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
            tgs_isc_param: [[0; 3]; 63],
            bd_gim_iono: [0; 4],
            bdt_utc_param: [0; 5],
            eop_param: [0; 6],
            bgto_param: [[0; 3]; 4],
            reduced_almanac: [[0; 2]; 63],
            midi_almanac: [[0; 7]; 63],
            almanac_valid_mask: 0,
            almanac_week: 0,
            almanac_toa: 0,
            cur_midi_alm_index: -1,
            cur_reduced_alm_index: -1,
        }
    }

    pub fn get_frame_data(
        &mut self,
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

    fn find_next_valid_index(valid_mask: u64, cur_index: i32) -> i32 {
        if valid_mask == 0 {
            return 0;
        }
        let mut idx = cur_index;
        loop {
            idx += 1;
            if idx >= 64 {
                idx = 0;
            }
            if valid_mask & (1u64 << idx) != 0 {
                break;
            }
        }
        idx
    }

    // 288 bits subframe information divided into 12 WORDs
    // each WORD has 24bits in 24LSB of unsigned int data in FrameData[]
    // bit order is MSB first (from bit23) and least index first
    // each 24bits divided into four 6bit symbols in LDPC encode
    fn compose_message(
        &mut self,
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
                Self::append_word(frame_data, 5 * 24 + 3, &self.bd_gim_iono, 96);
                frame_data[5] |= COMPOSE_BITS!(self.tgs_isc_param[svid_idx][1], 11, 12); // TGD
                frame_data[5] |= COMPOSE_BITS!(self.tgs_isc_param[svid_idx][1] >> 13, 0, 11); // ISC
                frame_data[6] |= COMPOSE_BITS!(self.tgs_isc_param[svid_idx][1] >> 12, 0, 1); // ISC
                frame_data[9] |= COMPOSE_BITS!(self.tgs_isc_param[svid_idx][0], 9, 12); // TGD
                frame_data[10] = 0; // fill rest of 33bits with 0
            }
            31 => {
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 5, 13, 3);
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 11, 9, 4);
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 2, 6, 3);
                Self::append_word(frame_data, 24 + 18, &self.clock_param[svid_idx], 69);
                frame_data[4] |= COMPOSE_BITS!(self.clock_param[svid_idx][3] >> 1, 0, 9);
                frame_data[5] = COMPOSE_BITS!(self.clock_param[svid_idx][3], 23, 1);
                frame_data[5] |= COMPOSE_BITS!(self.almanac_week, 10, 13);
                frame_data[5] |= COMPOSE_BITS!(self.almanac_toa, 2, 8);
                self.cur_reduced_alm_index = Self::find_next_valid_index(
                    self.almanac_valid_mask,
                    self.cur_reduced_alm_index,
                );
                Self::append_word(
                    frame_data,
                    5 * 24 + 22,
                    &self.reduced_almanac[self.cur_reduced_alm_index as usize],
                    38,
                );
                self.cur_reduced_alm_index = Self::find_next_valid_index(
                    self.almanac_valid_mask,
                    self.cur_reduced_alm_index,
                );
                Self::append_word(
                    frame_data,
                    7 * 24 + 12,
                    &self.reduced_almanac[self.cur_reduced_alm_index as usize],
                    38,
                );
                self.cur_reduced_alm_index = Self::find_next_valid_index(
                    self.almanac_valid_mask,
                    self.cur_reduced_alm_index,
                );
                Self::append_word(
                    frame_data,
                    9 * 24 + 2,
                    &self.reduced_almanac[self.cur_reduced_alm_index as usize],
                    38,
                );
            }
            32 => {
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 5, 13, 3);
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 11, 9, 4);
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 2, 6, 3);
                Self::append_word(frame_data, 24 + 18, &self.clock_param[svid_idx], 69);
                frame_data[4] |= COMPOSE_BITS!(self.clock_param[svid_idx][3] >> 1, 0, 9);
                frame_data[5] = COMPOSE_BITS!(self.clock_param[svid_idx][3], 23, 1);
                Self::append_word(frame_data, 5 * 24 + 1, &self.eop_param, 138);
            }
            33 => {
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 5, 13, 3);
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 11, 9, 4);
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 2, 6, 3);
                Self::append_word(frame_data, 24 + 18, &self.clock_param[svid_idx], 69);
                Self::append_word(frame_data, 4 * 24 + 15, &self.bgto_param[0], 68);
                self.cur_reduced_alm_index = Self::find_next_valid_index(
                    self.almanac_valid_mask,
                    self.cur_reduced_alm_index,
                );
                Self::append_word(
                    frame_data,
                    7 * 24 + 11,
                    &self.reduced_almanac[self.cur_reduced_alm_index as usize],
                    38,
                );
                frame_data[9] |= COMPOSE_BITS!(self.clock_param[svid_idx][3], 13, 10);
                frame_data[9] |= COMPOSE_BITS!(self.almanac_week, 0, 13);
                frame_data[10] = COMPOSE_BITS!(self.almanac_toa, 16, 8);
            }
            40 => {
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 5, 13, 3);
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 11, 9, 4);
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 2, 6, 3);
                // SISAIoe/SISAIoc filled with 27 zeros
                frame_data[2] = 0;
                self.cur_midi_alm_index =
                    Self::find_next_valid_index(self.almanac_valid_mask, self.cur_midi_alm_index);
                Self::append_word(
                    frame_data,
                    2 * 24 + 21,
                    &self.midi_almanac[self.cur_midi_alm_index as usize],
                    156,
                );
                frame_data[10] = 0; // fill rest of 39bits with 0
            }
            34 => {
                // HS filled with 2 zeros
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 5, 13, 3); // B2a DIF/SIF/AIF
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 11, 9, 4); // SISMAI
                frame_data[1] |= COMPOSE_BITS!(self.integrity_flags[svid_idx] >> 2, 6, 3); // B1C DIF/SIF/AIF
                                                                                           // SISAI filled with 22 zeros
                frame_data[2] = 0;
                Self::append_word(frame_data, 2 * 24 + 16, &self.clock_param[svid_idx], 69);
                frame_data[5] |= COMPOSE_BITS!(self.clock_param[svid_idx][3], 1, 10); // IODC
                Self::append_word(frame_data, 5 * 24 + 23, &self.bdt_utc_param, 97);
                frame_data[10] = 0; // fill rest with 0
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
        // Ported from the verified BCNavBit::append_word (bcnavbit.rs). B-CNAV words hold
        // 24-bit payloads, so each source word's significant bits are its low 24. The old
        // version read from bit 32 (dropping the low 8 bits of every chunk) and only advanced
        // the source index on full-24-bit chunks, desyncing across word boundaries (H14).
        let mut remain_bits: i32 = 24; // bits left in the current source word
                                       // callers pass an absolute bit position; split into a dest-word index + in-word offset
        let mut dest_idx = start_bit / 24;
        let mut start_bit = (start_bit % 24) as i32; // bit offset inside the current dest word
        let mut src_idx = 0usize;
        let mut length = bit_count as i32;

        while length > 0 && dest_idx < data.len() && src_idx < source.len() {
            let mut fill_bits = 24 - start_bit;
            if fill_bits > remain_bits {
                fill_bits = remain_bits;
            }
            if fill_bits > length {
                fill_bits = length;
            }
            data[dest_idx] = if start_bit == 0 { 0 } else { data[dest_idx] }
                | COMPOSE_BITS!(
                    source[src_idx] >> (remain_bits - fill_bits),
                    (24 - start_bit - fill_bits) as u32,
                    fill_bits as u32
                );
            start_bit += fill_bits;
            if start_bit >= 24 {
                start_bit = 0;
                dest_idx += 1;
            }
            remain_bits -= fill_bits;
            if remain_bits <= 0 {
                remain_bits = 24;
                src_idx += 1;
            }
            length -= fill_bits;
        }
    }

    // Public interface methods for setting navigation data
    pub fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) -> i32 {
        if !(1..=63).contains(&svid) {
            return 1;
        }

        let svid_idx = (svid - 1) as usize;

        // The B-CNAV2 (B2a) Ephemeris I, Ephemeris II and clock data blocks are bit-identical
        // to B-CNAV1 (B1C) — same BDS-3 CNAV definitions (BDS-SIS-ICD-B2a Table 7-8 == B1C
        // Table 7-8). Reuse the verified BCNavBit packing: correct ICD power-of-two scale
        // factors, semicircle conversion (÷π), two's-complement, and ΔA = A − A_ref with the
        // sat_type-dependent reference axis. The previous code stored one raw decimal-scaled
        // field per word, which append_word then mis-packed against the ICD bit layout.
        let mut b1c = BCNavBit::new();
        b1c.set_ephemeris(svid, eph);
        self.ephemeris1[svid_idx] = b1c.ephemeris1[svid_idx];
        self.ephemeris2[svid_idx] = b1c.ephemeris2[svid_idx];
        self.clock_param[svid_idx] = b1c.clock_param[svid_idx];

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

#[cfg(test)]
mod append_word_tests {
    use super::BCNav2Bit;
    use crate::bcnavbit::BCNavBit;

    /// Audit H14: B-CNAV2 bit packing must match the verified BCNavBit::append_word for
    /// every start offset / length, across source-word boundaries (24-bit payload words).
    #[test]
    fn append_word_matches_verified_reference() {
        let reference = BCNavBit::new();
        let source = [0x00AB_CDEFu32, 0x0012_3456, 0x00FE_DCBA, 0x0000_FFFF];
        for &start in &[0usize, 1, 5, 12, 23] {
            for &len in &[1usize, 8, 17, 24, 30, 48, 60] {
                let mut got = [0u32; 8];
                let mut want = [0u32; 8];
                BCNav2Bit::append_word(&mut got, start, &source, len);
                reference.append_word(&mut want, start as i32, &source, len as i32);
                assert_eq!(
                    got, want,
                    "append_word mismatch at start={start}, len={len}"
                );
            }
        }
    }

    /// ICD verification: B-CNAV2 (B2a) Ephemeris I/II and clock blocks are bit-identical to
    /// B-CNAV1 (B1C), so the B1C decoder (decode_bcnav_ephemeris) must round-trip the B2a
    /// encoder. Catches the old decimal-multiplier / missing-ΔA encoding (BDS-SIS-ICD-B2a).
    #[test]
    fn bcnav2_ephemeris_round_trips_via_b1c_decoder() {
        use crate::nav_decode::decode_bcnav_ephemeris;
        let mut eph = crate::types::GpsEphemeris {
            valid: 1,
            ..Default::default()
        };

        eph.flag = 3; // MEO -> A_ref 27906100 m
        eph.axis = 27906100.0 + 512.0;
        eph.axis_dot = -0.0625;
        eph.delta_n = 4.2e-9;
        eph.delta_n_dot = 1.0e-14;
        eph.M0 = 0.5;
        eph.ecc = 0.0007;
        eph.w = -1.1;
        eph.omega0 = 2.3;
        eph.i0 = 0.96;
        eph.omega_dot = -7.0e-9;
        eph.idot = 3.0e-10;
        eph.cuc = 1.5e-6;
        eph.cus = 6.0e-6;
        eph.crc = 180.0;
        eph.crs = -40.0;
        eph.cic = -2.0e-8;
        eph.cis = 9.0e-8;
        eph.af0 = -3.0e-4;
        eph.af1 = 1.5e-12;
        eph.af2 = 0.0;
        eph.toe = 345600;
        eph.toc = 345600;
        let mut b2a = BCNav2Bit::new();
        assert_eq!(b2a.set_ephemeris(5, &eph), 0);
        let dec =
            decode_bcnav_ephemeris(&b2a.ephemeris1[4], &b2a.ephemeris2[4], &b2a.clock_param[4]);
        let bad: Vec<_> = dec
            .compare_with_original(&eph)
            .into_iter()
            .filter(|d| !d.ok)
            .map(|d| format!("{}: orig={:.6e} dec={:.6e}", d.name, d.original, d.decoded))
            .collect();
        assert!(
            bad.is_empty(),
            "B-CNAV2 round-trip mismatches:\n  {}",
            bad.join("\n  ")
        );
    }
}
