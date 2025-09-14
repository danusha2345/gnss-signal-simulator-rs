//! # Модуль генерации PRN кодов
//!
//! Этот модуль отвечает за генерацию псевдослучайных последовательностей (PRN codes)
//! для различных ГНСС систем. Основные функции:
//! - Генерация C/A кодов GPS
//! - Создание кодов Gold для различных спутников
//! - Генерация P(Y) кодов для защищенных сигналов
//! - Поддержка кодов BeiDou, Galileo и ГЛОНАСС
//! - Реализация различных типов модуляции (BOC, CBOC, TMBOC, QMBOC)
//! - Управление атрибутами PRN кодов и их параметрами
//!
//! PRN коды являются основой для разделения сигналов различных спутников
//! и обеспечения множественного доступа в ГНСС системах.

//----------------------------------------------------------------------
// prngenerate.rs:
//   Implementation of PRN code generation class
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::constants::*;
use crate::memory_code::{E1_MEMORY_CODE, E6_MEMORY_CODE};
use crate::types::GnssSystem;

/// PRN attribute flags
pub const PRN_ATTRIBUTE_BOC: u32 = 1; // BOC modulation
pub const PRN_ATTRIBUTE_TMD: u32 = 2; // Time multiplexed data
pub const PRN_ATTRIBUTE_QMBOC: u32 = 4; // QMBOC modulation (BeiDou B1C)
pub const PRN_ATTRIBUTE_CBOC: u32 = 8; // CBOC modulation (Galileo E1)
pub const PRN_ATTRIBUTE_TMBOC: u32 = 16; // TMBOC modulation (GPS L1C)

/// PRN attributes structure
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct PrnAttribute {
    pub chip_rate: i32,    // chips per millisecond
    pub data_period: i32,  // PRN period for data channel in millisecond
    pub pilot_period: i32, // PRN period for pilot channel in millisecond
    pub attribute: u32,    // modulation attributes
}

/// LFSR sequence generator
pub struct LsfrSequence {
    init_state: u32,
    current_state: u32,
    polynomial: u32,
    output_mask: u32,
}

impl LsfrSequence {
    pub fn new(init_state: u32, polynomial: u32, length: i32) -> Self {
        let output_mask = 1u32 << (length - 1);
        LsfrSequence {
            init_state,
            current_state: init_state,
            polynomial,
            output_mask,
        }
    }

    pub fn initial(&mut self) {
        self.current_state = self.init_state;
    }

    pub fn get_output(&mut self) -> i32 {
        let output = if (self.current_state & self.output_mask) != 0 {
            1
        } else {
            0
        };
        let mut feedback = self.current_state & self.polynomial;

        // Count number of 1s in feedback (parity calculation)
        feedback = (feedback & 0x55555555) + ((feedback >> 1) & 0x55555555);
        feedback = (feedback & 0x33333333) + ((feedback >> 2) & 0x33333333);
        feedback = (feedback & 0x0f0f0f0f) + ((feedback >> 4) & 0x0f0f0f0f);
        feedback = (feedback & 0x00ff00ff) + ((feedback >> 8) & 0x00ff00ff);
        feedback = (feedback & 0x0000ffff) + ((feedback >> 16) & 0x0000ffff);

        self.current_state = (self.current_state << 1) | (feedback & 1);

        output
    }
}

/// PRN code generator
pub struct PrnGenerate {
    pub data_prn: Option<Vec<i32>>,
    pub pilot_prn: Option<Vec<i32>>,
    pub attribute: Option<PrnAttribute>,
}

impl PrnGenerate {
    pub fn new(system: GnssSystem, signal_index: i32, svid: i32) -> Self {
        let mut generator = PrnGenerate {
            data_prn: None,
            pilot_prn: None,
            attribute: Some(PRN_ATTRIBUTES[0]), // Default, will be updated
        };

        // Validate SVID range and generate PRN codes based on system and signal
        match system {
            GnssSystem::GpsSystem => {
                if (1..=32).contains(&svid) {
                    generator.generate_gps_prn(signal_index, svid);
                }
            }
            GnssSystem::BdsSystem => {
                if (1..=63).contains(&svid) {
                    generator.generate_bds_prn(signal_index, svid);
                }
            }
            GnssSystem::GalileoSystem => {
                if (1..=50).contains(&svid) {
                    generator.generate_galileo_prn(signal_index, svid);
                }
            }
            GnssSystem::GlonassSystem => {
                generator.generate_glonass_prn(signal_index, svid);
            }
            _ => {
                generator.data_prn = None;
                generator.pilot_prn = None;
                generator.attribute = Some(PRN_ATTRIBUTES[0]);
            }
        }

        generator
    }

    fn generate_gps_prn(&mut self, signal_index: i32, svid: i32) {
        const L1CA: i32 = SIGNAL_INDEX_L1CA as i32;
        const L1C: i32 = SIGNAL_INDEX_L1C as i32;
        const L2C: i32 = SIGNAL_INDEX_L2C as i32;
        const L2P: i32 = SIGNAL_INDEX_L2P as i32;
        const L5: i32 = SIGNAL_INDEX_L5 as i32;

        match signal_index {
            L1CA => {
                self.data_prn = Some(self.get_gold_code(
                    L1CA_PRN_INIT[(svid - 1) as usize],
                    0x3a6,
                    0x3ff,
                    0x204,
                    1023,
                    10,
                    1023,
                ));
                self.pilot_prn = None;
                self.attribute = Some(PRN_ATTRIBUTES[0]);
            }
            L1C => {
                self.data_prn = Some(self.get_l1c_weil(
                    L1C_DATA_INSERT_INDEX[(svid - 1) as usize],
                    L1C_DATA_PHASE_DIFF[(svid - 1) as usize],
                ));
                self.pilot_prn = Some(self.get_l1c_weil(
                    L1C_PILOT_INSERT_INDEX[(svid - 1) as usize],
                    L1C_PILOT_PHASE_DIFF[(svid - 1) as usize],
                ));
                self.attribute = Some(PRN_ATTRIBUTES[1]);
            }
            L2C => {
                self.data_prn = Some(self.get_gold_code(
                    L2CM_PRN_INIT[(svid - 1) as usize],
                    0x0494953c,
                    0x0,
                    0x0,
                    10230,
                    27,
                    10230,
                ));
                self.pilot_prn = Some(self.get_gold_code(
                    L2CL_PRN_INIT[(svid - 1) as usize],
                    0x0494953c,
                    0x0,
                    0x0,
                    10230 * 75,
                    27,
                    0,
                ));
                self.attribute = Some(PRN_ATTRIBUTES[2]);
            }
            L2P => {
                self.data_prn = Some(self.get_simplified_p_code(svid));
                self.pilot_prn = None;
                self.attribute = Some(PRN_ATTRIBUTES[3]);
            }
            L5 => {
                self.data_prn = Some(self.get_gold_code(
                    L5I_PRN_INIT[(svid - 1) as usize],
                    0x18ed,
                    0x1fff,
                    0x1b00,
                    10230,
                    13,
                    8190,
                ));
                self.pilot_prn = Some(self.get_gold_code(
                    L5Q_PRN_INIT[(svid - 1) as usize],
                    0x18ed,
                    0x1fff,
                    0x1b00,
                    10230,
                    13,
                    8190,
                ));
                self.attribute = Some(PRN_ATTRIBUTES[4]);
            }
            _ => {
                self.data_prn = None;
                self.pilot_prn = None;
                self.attribute = Some(PRN_ATTRIBUTES[0]);
            }
        }
    }

    fn generate_bds_prn(&mut self, signal_index: i32, svid: i32) {
        const B1I: i32 = SIGNAL_INDEX_B1I as i32;
        const B2I: i32 = SIGNAL_INDEX_B2I as i32;
        const B3I: i32 = SIGNAL_INDEX_B3I as i32;
        const B1C: i32 = SIGNAL_INDEX_B1C as i32;
        const B2A: i32 = SIGNAL_INDEX_B2A as i32;
        const B2B: i32 = SIGNAL_INDEX_B2B as i32;

        match signal_index {
            B1I => {
                self.data_prn = Some(self.get_gold_code(
                    B1I_PRN_INIT[(svid - 1) as usize],
                    0x59f,
                    0x2aa,
                    0x7c1,
                    2046,
                    11,
                    2046,
                ));
                self.pilot_prn = None;
                self.attribute = Some(PRN_ATTRIBUTES[5]);
            }
            B2I => {
                self.data_prn = Some(self.get_gold_code(
                    B1I_PRN_INIT[(svid - 1) as usize],
                    0x59f,
                    0x2aa,
                    0x7c1,
                    2046,
                    11,
                    2046,
                ));
                self.pilot_prn = None;
                self.attribute = Some(PRN_ATTRIBUTES[5]);
            }
            B3I => {
                self.data_prn = Some(self.get_gold_code(
                    B3I_PRN_INIT[(svid - 1) as usize],
                    0x1b71,
                    0x1fff,
                    0x100d,
                    10230,
                    13,
                    8190,
                ));
                self.pilot_prn = None;
                self.attribute = Some(PRN_ATTRIBUTES[6]);
            }
            B1C => {
                self.data_prn = Some(self.get_b1c_weil(
                    B1C_DATA_TRUNCATION[(svid - 1) as usize],
                    B1C_DATA_PHASE_DIFF[(svid - 1) as usize],
                ));
                self.pilot_prn = Some(self.get_b1c_weil(
                    B1C_PILOT_TRUNCATION[(svid - 1) as usize],
                    B1C_PILOT_PHASE_DIFF[(svid - 1) as usize],
                ));
                self.attribute = Some(PRN_ATTRIBUTES[11]);
            }
            B2A => {
                self.data_prn = Some(self.get_gold_code(
                    B2A_D_PRN_INIT[(svid - 1) as usize],
                    0x1d14,
                    0x1fff,
                    0x1411,
                    10230,
                    13,
                    8190,
                ));
                self.pilot_prn = Some(self.get_gold_code(
                    B2A_P_PRN_INIT[(svid - 1) as usize],
                    0x18d1,
                    0x1fff,
                    0x1064,
                    10230,
                    13,
                    8190,
                ));
                self.attribute = Some(PRN_ATTRIBUTES[4]);
            }
            B2B => {
                let mask = if !(6..=58).contains(&svid) { 0 } else { 0x1fff };
                self.data_prn = Some(self.get_gold_code(
                    B2B_PRN_INIT[(svid - 1) as usize],
                    0x192c,
                    mask,
                    0x1301,
                    10230,
                    13,
                    8190,
                ));
                self.pilot_prn = None;
                self.attribute = Some(PRN_ATTRIBUTES[7]);
            }
            _ => {
                self.data_prn = None;
                self.pilot_prn = None;
                self.attribute = Some(PRN_ATTRIBUTES[0]);
            }
        }
    }

    fn generate_galileo_prn(&mut self, signal_index: i32, svid: i32) {
        const E1: i32 = SIGNAL_INDEX_E1 as i32;
        const E5A: i32 = SIGNAL_INDEX_E5A as i32;
        const E5B: i32 = SIGNAL_INDEX_E5B as i32;
        const E6: i32 = SIGNAL_INDEX_E6 as i32;

        match signal_index {
            E1 => {
                self.data_prn =
                    Some(self.get_memory_sequence(&E1_MEMORY_CODE[(svid - 1) as usize * 128..], 4));
                self.pilot_prn = Some(
                    self.get_memory_sequence(&E1_MEMORY_CODE[(svid + 49) as usize * 128..], 4),
                );
                self.attribute = Some(PRN_ATTRIBUTES[8]);
            }
            E5A => {
                self.data_prn = Some(self.get_gold_code(
                    E5A_I_PRN_INIT[(svid - 1) as usize],
                    0x28d8,
                    0x3fff,
                    0x20a1,
                    10230,
                    14,
                    10230,
                ));
                self.pilot_prn = Some(self.get_gold_code(
                    E5A_Q_PRN_INIT[(svid - 1) as usize],
                    0x28d8,
                    0x3fff,
                    0x20a1,
                    10230,
                    14,
                    10230,
                ));
                self.attribute = Some(PRN_ATTRIBUTES[4]);
            }
            E5B => {
                self.data_prn = Some(self.get_gold_code(
                    E5B_I_PRN_INIT[(svid - 1) as usize],
                    0x2992,
                    0x3fff,
                    0x3408,
                    10230,
                    14,
                    10230,
                ));
                self.pilot_prn = Some(self.get_gold_code(
                    E5B_Q_PRN_INIT[(svid - 1) as usize],
                    0x2331,
                    0x3fff,
                    0x3408,
                    10230,
                    14,
                    10230,
                ));
                self.attribute = Some(PRN_ATTRIBUTES[4]);
            }
            E6 => {
                self.data_prn =
                    Some(self.get_memory_sequence(&E6_MEMORY_CODE[(svid - 1) as usize * 160..], 5));
                self.pilot_prn = Some(
                    self.get_memory_sequence(&E6_MEMORY_CODE[(svid + 49) as usize * 160..], 5),
                );
                self.attribute = Some(PRN_ATTRIBUTES[9]);
            }
            _ => {
                self.data_prn = None;
                self.pilot_prn = None;
                self.attribute = Some(PRN_ATTRIBUTES[0]);
            }
        }
    }

    fn generate_glonass_prn(&mut self, signal_index: i32, _svid: i32) {
        const G1: i32 = SIGNAL_INDEX_G1 as i32;
        const G2: i32 = SIGNAL_INDEX_G2 as i32;
        const G3: i32 = SIGNAL_INDEX_G3 as i32;

        match signal_index {
            G1 | G2 => {
                self.data_prn = Some(self.get_gold_code(0x1fc, 0x110, 0x0, 0x0, 511, 9, 511));
                self.pilot_prn = None;
                self.attribute = Some(PRN_ATTRIBUTES[10]);
            }
            G3 => {
                // G3/L3OC uses Gold code with length 10230
                self.data_prn =
                    Some(self.get_gold_code(0x3ff, 0x224, 0x3ff, 0x387, 10230, 10, 10230));
                self.pilot_prn =
                    Some(self.get_gold_code(0x3ff, 0x224, 0x3ff, 0x387, 10230, 10, 10230));
                self.attribute = Some(PRN_ATTRIBUTES[12]);
            }
            _ => {
                self.data_prn = None;
                self.pilot_prn = None;
                self.attribute = Some(PRN_ATTRIBUTES[0]);
            }
        }
    }

    // Helper methods for code generation
    #[allow(clippy::too_many_arguments)]
    fn get_gold_code(
        &self,
        g1_init: u32,
        g1_poly: u32,
        g2_init: u32,
        g2_poly: u32,
        length: i32,
        depth: i32,
        reset_pos: i32,
    ) -> Vec<i32> {
        let mut prn_sequence = Vec::with_capacity(length as usize);
        let mut g1 = LsfrSequence::new(g1_init, g1_poly, depth);
        let mut g2 = LsfrSequence::new(g2_init, g2_poly, depth);

        for i in 0..length {
            if i == reset_pos {
                g2.initial();
            }
            prn_sequence.push(g1.get_output() ^ g2.get_output());
        }

        prn_sequence
    }

    fn legendre_sequence(&self, data: &mut [i32], length: usize) {
        data.iter_mut().take(length).for_each(|v| *v = 0);
        (1..length).for_each(|i| data[(i * i) % length] = 1);
    }

    fn get_l1c_weil(&self, insert_index: i32, phase_diff: i32) -> Vec<i32> {
        let mut legendre_code = vec![0; 10223];
        let insert_sequence = [0, 1, 1, 0, 1, 0, 0];
        let mut prn_sequence = Vec::with_capacity(10230);
        let mut index1 = 0;
        let mut index2 = phase_diff;

        self.legendre_sequence(&mut legendre_code, 10223);

        for i in 0..10230 {
            if index1 >= 10223 {
                index1 -= 10223;
            }
            if index2 >= 10223 {
                index2 -= 10223;
            }

            if i >= insert_index - 1 && i < insert_index + 6 {
                prn_sequence.push(insert_sequence[(i - insert_index + 1) as usize]);
            } else {
                prn_sequence.push(legendre_code[index1 as usize] ^ legendre_code[index2 as usize]);
                index1 += 1;
                index2 += 1;
            }
        }
        prn_sequence
    }

    fn get_b1c_weil(&self, truncation_point: i32, phase_diff: i32) -> Vec<i32> {
        let mut legendre_code = vec![0; 10243];
        let mut prn_sequence = Vec::with_capacity(10230);
        let mut index1 = truncation_point - 1;
        let mut index2 = truncation_point + phase_diff - 1;

        self.legendre_sequence(&mut legendre_code, 10243);

        for _i in 0..10230 {
            if index1 >= 10243 {
                index1 -= 10243;
            }
            if index2 >= 10243 {
                index2 -= 10243;
            }
            prn_sequence.push(legendre_code[index1 as usize] ^ legendre_code[index2 as usize]);
            index1 += 1;
            index2 += 1;
        }
        prn_sequence
    }

    fn get_memory_sequence(&self, binary_sequence: &[u32], sector_length: i32) -> Vec<i32> {
        let mut prn_sequence = Vec::with_capacity((1023 * sector_length) as usize);

        for i in 0..sector_length {
            for j in 0..1023 {
                let bit_index = (j >> 5) as usize;
                let bit_offset = 31 - (j & 0x1f);
                let word_index = (i as usize * 32) + bit_index;
                if word_index < binary_sequence.len() {
                    prn_sequence.push(if (binary_sequence[word_index] & (1 << bit_offset)) != 0 {
                        1
                    } else {
                        0
                    });
                } else {
                    prn_sequence.push(0);
                }
            }
        }

        prn_sequence
    }

    // Simplified P code generator for L2P
    fn get_simplified_p_code(&self, svid: i32) -> Vec<i32> {
        // Generate a 20460 chip sequence (2 periods of 10230 chips)
        let mut prn_sequence = Vec::with_capacity(10230 * 2);
        let mut lfsr1 = L2P_PRN_INIT[(svid - 1) as usize];
        let mut lfsr2 = L2P_PRN_INIT[(svid - 1) as usize] ^ 0x55555555;
        let mut lfsr3 = L2P_PRN_INIT[(svid - 1) as usize] ^ 0xAAAAAAAA;
        let mut lfsr4 = L2P_PRN_INIT[(svid - 1) as usize] ^ 0xFF00FF00;

        // Generate P code using 4 combined LFSRs for better randomness
        for _i in 0..(10230 * 2) {
            // Galois LFSR with polynomial x^32 + x^22 + x^2 + x^1 + 1
            let bit1 = lfsr1 & 1;
            lfsr1 = (lfsr1 >> 1) ^ (if bit1 != 0 { 0x80200003 } else { 0 });

            // Second LFSR with different polynomial
            let bit2 = lfsr2 & 1;
            lfsr2 = (lfsr2 >> 1) ^ (if bit2 != 0 { 0x80000057 } else { 0 });

            // Third LFSR
            let bit3 = lfsr3 & 1;
            lfsr3 = (lfsr3 >> 1) ^ (if bit3 != 0 { 0x8000001B } else { 0 });

            // Fourth LFSR
            let bit4 = lfsr4 & 1;
            lfsr4 = (lfsr4 >> 1) ^ (if bit4 != 0 { 0x80000062 } else { 0 });

            // Combine outputs
            prn_sequence.push((bit1 ^ bit2 ^ bit3 ^ bit4) as i32);
        }

        prn_sequence
    }

    pub fn get_data_prn(&self) -> Option<&Vec<i32>> {
        self.data_prn.as_ref()
    }

    pub fn get_pilot_prn(&self) -> Option<&Vec<i32>> {
        self.pilot_prn.as_ref()
    }

    pub fn get_attribute(&self) -> Option<&PrnAttribute> {
        self.attribute.as_ref()
    }

    /// Простая функция получения PRN бита по индексу (для потоковой обработки)
    pub fn get_prn_bit(&self, chip_index: i32) -> bool {
        if let Some(ref data_prn) = self.data_prn {
            let idx = (chip_index as usize) % data_prn.len();
            data_prn[idx] != 0
        } else {
            true // По умолчанию
        }
    }
}

// PRN attributes for different signals (matching C++ PrnAttributes array)
static PRN_ATTRIBUTES: [PrnAttribute; 13] = [
    // index 0 for L1CA
    PrnAttribute {
        chip_rate: 1023,
        data_period: 1,
        pilot_period: 1,
        attribute: 0,
    },
    // index 1 for L1C (BOC(1,1) + QMBOC for pilot)
    PrnAttribute {
        chip_rate: 2046,
        data_period: 10,
        pilot_period: 10,
        attribute: PRN_ATTRIBUTE_BOC | PRN_ATTRIBUTE_QMBOC,
    },
    // index 2 for L2C
    PrnAttribute {
        chip_rate: 1023,
        data_period: 20,
        pilot_period: 1500,
        attribute: PRN_ATTRIBUTE_TMD,
    },
    // index 3 for L2P
    PrnAttribute {
        chip_rate: 10230,
        data_period: 2,
        pilot_period: 2,
        attribute: 0,
    },
    // index 4 for L5/B2a/E5a/E5b
    PrnAttribute {
        chip_rate: 10230,
        data_period: 1,
        pilot_period: 1,
        attribute: 0,
    },
    // index 5 for B1I/B2I
    PrnAttribute {
        chip_rate: 2046,
        data_period: 1,
        pilot_period: 1,
        attribute: 0,
    },
    // index 6 for B3I
    PrnAttribute {
        chip_rate: 10230,
        data_period: 1,
        pilot_period: 1,
        attribute: 0,
    },
    // index 7 for B2b
    PrnAttribute {
        chip_rate: 10230,
        data_period: 1,
        pilot_period: 1,
        attribute: 0,
    },
    // index 8 for E1 (CBOC(6,1,1/11))
    PrnAttribute {
        chip_rate: 2046,
        data_period: 4,
        pilot_period: 4,
        attribute: PRN_ATTRIBUTE_BOC | PRN_ATTRIBUTE_CBOC,
    },
    // index 9 for E6
    PrnAttribute {
        chip_rate: 5115,
        data_period: 1,
        pilot_period: 1,
        attribute: 0,
    },
    // index 10 for G1/G2
    PrnAttribute {
        chip_rate: 511,
        data_period: 1,
        pilot_period: 1,
        attribute: 0,
    },
    // index 11 for B1C (BOC(1,1) + QMBOC for pilot)
    PrnAttribute {
        chip_rate: 2046,
        data_period: 10,
        pilot_period: 10,
        attribute: PRN_ATTRIBUTE_BOC | PRN_ATTRIBUTE_QMBOC,
    },
    // index 12 for G3/L3OC (BPSK(10))
    PrnAttribute {
        chip_rate: 10230,
        data_period: 1,
        pilot_period: 1,
        attribute: PRN_ATTRIBUTE_BOC,
    },
];

// GPS L1 C/A PRN initial states
const L1CA_PRN_INIT: [u32; 32] = [
    0x0df, 0x06f, 0x037, 0x01b, 0x1a4, 0x0d2, 0x1a6, 0x0d3, 0x069, 0x0bb, 0x05d, 0x017, 0x00b,
    0x005, 0x002, 0x001, 0x191, 0x0c8, 0x064, 0x032, 0x019, 0x00c, 0x1cc, 0x039, 0x01c, 0x00e,
    0x007, 0x003, 0x1a8, 0x0d4, 0x06a, 0x035,
];

// GPS L5 PRN initial states
const L5I_PRN_INIT: [u32; 32] = [
    0x04ea, 0x1583, 0x0202, 0x0c8d, 0x1d77, 0x0be6, 0x1f25, 0x04bd, 0x1a9f, 0x0f7e, 0x0b90, 0x13e7,
    0x0738, 0x1c82, 0x0b56, 0x1278, 0x1e32, 0x0f0f, 0x1f13, 0x16d6, 0x0204, 0x1ef7, 0x0fe1, 0x05a3,
    0x16cb, 0x0d35, 0x0f6a, 0x0d5e, 0x10fa, 0x1da1, 0x0f28, 0x13a0,
];

const L5Q_PRN_INIT: [u32; 32] = [
    0x0669, 0x0de2, 0x188f, 0x0adc, 0x09bc, 0x12aa, 0x103f, 0x02d6, 0x185d, 0x0c24, 0x1408, 0x146a,
    0x14b2, 0x1f85, 0x1e3d, 0x1f4b, 0x0267, 0x04ed, 0x1b4c, 0x11c3, 0x0136, 0x0e34, 0x17d1, 0x19f6,
    0x1b22, 0x07aa, 0x0be1, 0x085f, 0x048a, 0x13c1, 0x14fa, 0x0a89,
];

// GPS L2C PRN initial states
const L2CM_PRN_INIT: [u32; 32] = [
    0x15ef0f5, 0x50f811e, 0x10e553d, 0x16b0258, 0x416f3bc, 0x65bc21e, 0x0f5be58, 0x496777f,
    0x4a5a8e2, 0x36e44d6, 0x5e84705, 0x345ea19, 0x6965b5b, 0x447fb02, 0x0043a6e, 0x35e5896,
    0x3059ddd, 0x5c16d2a, 0x10c80db, 0x1c754b4, 0x650324e, 0x7fb4e14, 0x74e048f, 0x0663507,
    0x1f887f9, 0x487c247, 0x5fd6d8c, 0x20818d1, 0x1ece400, 0x7aeb923, 0x656b597, 0x602e157,
];

const L2CL_PRN_INIT: [u32; 32] = [
    0x29be220, 0x2012ed7, 0x3d7d64b, 0x12b1c4a, 0x5e3a308, 0x31c0719, 0x3b5179f, 0x74429a6,
    0x1d5fc3b, 0x3bf943a, 0x587c624, 0x0be84ce, 0x57d8717, 0x6a8376f, 0x5a13f5d, 0x4a5f5df,
    0x046b92b, 0x7a7c2ae, 0x45886a6, 0x5a9a643, 0x68872f2, 0x3e759f6, 0x6b6fdbd, 0x31b717b,
    0x048fcb0, 0x1cbc9e3, 0x6b38d5b, 0x6f5b8fa, 0x121a76e, 0x5f23c35, 0x326fd21, 0x3cb4e3c,
];

// Simplified P code initial states for L2P - unique per satellite
const L2P_PRN_INIT: [u32; 32] = [
    0x1234567, 0x2345678, 0x3456789, 0x456789a, 0x56789ab, 0x6789abc, 0x789abcd, 0x89abcde,
    0x9abcdef, 0xabcdef0, 0xbcdef01, 0xcdef012, 0xdef0123, 0xef01234, 0xf012345, 0x0123456,
    0x1357924, 0x2468035, 0x3579146, 0x468a257, 0x579b368, 0x68ac479, 0x79bd58a, 0x8ace69b,
    0x9bdf7ac, 0xacf08bd, 0xbd019ce, 0xce12adf, 0xdf23be0, 0xe034cf1, 0xf145d02, 0x0256e13,
];

// GPS L1C parameters
const L1C_DATA_INSERT_INDEX: [i32; 63] = [
    412, 161, 1, 303, 207, 4971, 4496, 5, 4557, 485, 253, 4676, 1, 66, 4485, 282, 193, 5211, 729,
    4848, 982, 5955, 9805, 670, 464, 29, 429, 394, 616, 9457, 4429, 4771, 365, 9705, 9489, 4193,
    9947, 824, 864, 347, 677, 6544, 6312, 9804, 278, 9461, 444, 4839, 4144, 9875, 197, 1156, 4674,
    10035, 4504, 5, 9937, 430, 5, 355, 909, 1622, 6284,
];

const L1C_DATA_PHASE_DIFF: [i32; 63] = [
    5111, 5109, 5108, 5106, 5103, 5101, 5100, 5098, 5095, 5094, 5093, 5091, 5090, 5081, 5080, 5069,
    5068, 5054, 5044, 5027, 5026, 5014, 5004, 4980, 4915, 4909, 4893, 4885, 4832, 4824, 4591, 3706,
    5092, 4986, 4965, 4920, 4917, 4858, 4847, 4790, 4770, 4318, 4126, 3961, 3790, 4911, 4881, 4827,
    4795, 4789, 4725, 4675, 4539, 4535, 4458, 4197, 4096, 3484, 3481, 3393, 3175, 2360, 1852,
];

const L1C_PILOT_INSERT_INDEX: [i32; 63] = [
    181, 359, 72, 1110, 1480, 5034, 4622, 1, 4547, 826, 6284, 4195, 368, 1, 4796, 523, 151, 713,
    9850, 5734, 34, 6142, 190, 644, 467, 5384, 801, 594, 4450, 9437, 4307, 5906, 378, 9448, 9432,
    5849, 5547, 9546, 9132, 403, 3766, 3, 684, 9711, 333, 6124, 10216, 4251, 9893, 9884, 4627,
    4449, 9798, 985, 4272, 126, 10024, 434, 1029, 561, 289, 638, 4353,
];

const L1C_PILOT_PHASE_DIFF: [i32; 63] = [
    5097, 5110, 5079, 4403, 4121, 5043, 5042, 5104, 4940, 5035, 4372, 5064, 5084, 5048, 4950, 5019,
    5076, 3736, 4993, 5060, 5061, 5096, 4983, 4783, 4991, 4815, 4443, 4769, 4879, 4894, 4985, 5056,
    4921, 5036, 4812, 4838, 4855, 4904, 4753, 4483, 4942, 4813, 4957, 4618, 4669, 4969, 5031, 5038,
    4740, 4073, 4843, 4979, 4867, 4964, 5025, 4579, 4390, 4763, 4612, 4784, 3716, 4703, 4851,
];

// BDS B1I PRN initial states
const B1I_PRN_INIT: [u32; 63] = [
    0x187, 0x639, 0x1e6, 0x609, 0x605, 0x1f8, 0x606, 0x1f9, 0x704, 0x7be, 0x061, 0x78e, 0x782,
    0x07f, 0x781, 0x07e, 0x7df, 0x030, 0x03c, 0x7c1, 0x03f, 0x7c0, 0x7ef, 0x7e3, 0x01e, 0x7e0,
    0x01f, 0x00c, 0x7f1, 0x00f, 0x7f0, 0x7fd, 0x003, 0x7fc, 0x7fe, 0x001, 0x7ff, 0x457, 0x4ed,
    0x4dd, 0x4d1, 0x4d2, 0x32d, 0x48c, 0x492, 0x4bc, 0x4b0, 0x4b3, 0x34c, 0x4a2, 0x4ae, 0x4ad,
    0x352, 0x5d0, 0x5b1, 0x5af, 0x50b, 0x515, 0x53b, 0x537, 0x534, 0x2cb, 0x525,
];

// BDS B3I PRN initial states
const B3I_PRN_INIT: [u32; 63] = [
    0x1ff5, 0x1a8f, 0x0a3d, 0x1bff, 0x1f13, 0x04c9, 0x097f, 0x17f7, 0x0805, 0x1b04, 0x01d7, 0x0f34,
    0x1526, 0x0c8e, 0x1231, 0x07c7, 0x1464, 0x06e0, 0x1d51, 0x0f68, 0x1684, 0x0a34, 0x1e68, 0x08cc,
    0x025c, 0x1292, 0x196d, 0x08f5, 0x15e8, 0x1ffe, 0x1e36, 0x1235, 0x1aa9, 0x14b3, 0x174b, 0x05df,
    0x1cd4, 0x0117, 0x013b, 0x0e6b, 0x0581, 0x137a, 0x07b6, 0x11cb, 0x089c, 0x146a, 0x0cf9, 0x025f,
    0x1250, 0x06a1, 0x064f, 0x1e32, 0x0300, 0x0401, 0x0cac, 0x0c4d, 0x03ce, 0x0a74, 0x0df3, 0x1449,
    0x008e, 0x084c, 0x0e44,
];

// BDS B2a PRN initial states
const B2A_D_PRN_INIT: [u32; 63] = [
    0x1481, 0x0581, 0x16a1, 0x1e51, 0x1551, 0x0eb1, 0x0ef1, 0x1bf1, 0x1299, 0x0b79, 0x1585, 0x0445,
    0x1545, 0x1b45, 0x0745, 0x18a5, 0x1de5, 0x1015, 0x0f95, 0x1ab5, 0x11b5, 0x194d, 0x08cd, 0x032d,
    0x0dad, 0x09ed, 0x1fed, 0x091d, 0x079d, 0x10bd, 0x027d, 0x057d, 0x1afd, 0x19fd, 0x1143, 0x0523,
    0x1da3, 0x1113, 0x1313, 0x1ab3, 0x11b3, 0x0973, 0x154b, 0x05cb, 0x1a6b, 0x1d5b, 0x0587, 0x1827,
    0x1a27, 0x18a7, 0x02a7, 0x1b97, 0x1d37, 0x024f, 0x052f, 0x132f, 0x0b6f, 0x03ef, 0x1fef, 0x15bf,
    0x0804, 0x15fb, 0x0978,
];

const B2A_P_PRN_INIT: [u32; 63] = [
    0x1481, 0x0581, 0x16a1, 0x1e51, 0x1551, 0x0eb1, 0x0ef1, 0x1bf1, 0x1299, 0x0b79, 0x1585, 0x0445,
    0x1545, 0x1b45, 0x0745, 0x18a5, 0x1de5, 0x1015, 0x0f95, 0x1ab5, 0x11b5, 0x194d, 0x08cd, 0x032d,
    0x0dad, 0x09ed, 0x1fed, 0x091d, 0x079d, 0x10bd, 0x027d, 0x057d, 0x1afd, 0x19fd, 0x1143, 0x0523,
    0x1da3, 0x1113, 0x1313, 0x1ab3, 0x11b3, 0x0973, 0x154b, 0x05cb, 0x1a6b, 0x1d5b, 0x0587, 0x1827,
    0x1a27, 0x18a7, 0x02a7, 0x1b97, 0x1d37, 0x024f, 0x052f, 0x132f, 0x0b6f, 0x03ef, 0x1fef, 0x15bf,
    0x0c25, 0x03f4, 0x1558,
];

// BDS B2b PRN initial states
const B2B_PRN_INIT: [u32; 63] = [
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0eb1, 0x0ef1, 0x1bf1, 0x1299, 0x0b79, 0x1585, 0x0445,
    0x1545, 0x1b45, 0x0745, 0x18a5, 0x1de5, 0x1015, 0x0f95, 0x1ab5, 0x11b5, 0x194d, 0x08cd, 0x032d,
    0x0dad, 0x09ed, 0x1fed, 0x091d, 0x079d, 0x10bd, 0x027d, 0x057d, 0x1afd, 0x19fd, 0x1143, 0x0523,
    0x1da3, 0x1113, 0x1313, 0x1ab3, 0x11b3, 0x0973, 0x154b, 0x05cb, 0x1a6b, 0x1d5b, 0x0587, 0x1827,
    0x1a27, 0x18a7, 0x02a7, 0x1b97, 0x1d37, 0x024f, 0x052f, 0x132f, 0x0b6f, 0x03ef, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000,
];

// BDS B1C parameters
const B1C_DATA_TRUNCATION: [i32; 63] = [
    699, 694, 7318, 2127, 715, 6682, 7850, 5495, 1162, 7682, 6792, 9973, 6596, 2092, 19, 10151,
    6297, 5766, 2359, 7136, 1706, 2128, 6827, 693, 9729, 1620, 6805, 534, 712, 1929, 5355, 6139,
    6339, 1470, 6867, 7851, 1162, 7659, 1156, 2672, 6043, 2862, 180, 2663, 6940, 1645, 1582, 951,
    6878, 7701, 1823, 2391, 2606, 822, 6403, 239, 442, 6769, 2560, 2502, 5072, 7268, 341,
];

const B1C_DATA_PHASE_DIFF: [i32; 63] = [
    2678, 4802, 958, 859, 3843, 2232, 124, 4352, 1816, 1126, 1860, 4800, 2267, 424, 4192, 4333,
    2656, 4148, 243, 1330, 1593, 1470, 882, 3202, 5095, 2546, 1733, 4795, 4577, 1627, 3638, 2553,
    3646, 1087, 1843, 216, 2245, 726, 1966, 670, 4130, 53, 4830, 182, 2181, 2006, 1080, 2288, 2027,
    271, 915, 497, 139, 3693, 2054, 4342, 3342, 2592, 1007, 310, 4203, 455, 4318,
];

const B1C_PILOT_TRUNCATION: [i32; 63] = [
    7575, 2369, 5688, 539, 2270, 7306, 6457, 6254, 5644, 7119, 1402, 5557, 5764, 1073, 7001, 5910,
    10060, 2710, 1546, 6887, 1883, 5613, 5062, 1038, 10170, 6484, 1718, 2535, 1158, 526, 7331,
    5844, 6423, 6968, 1280, 1838, 1989, 6468, 2091, 1581, 1453, 6252, 7122, 7711, 7216, 2113, 1095,
    1628, 1713, 6102, 6123, 6070, 1115, 8047, 6795, 2575, 53, 1729, 6388, 682, 5565, 7160, 2277,
];

const B1C_PILOT_PHASE_DIFF: [i32; 63] = [
    796, 156, 4198, 3941, 1374, 1338, 1833, 2521, 3175, 168, 2715, 4408, 3160, 2796, 459, 3594,
    4813, 586, 1428, 2371, 2285, 3377, 4965, 3779, 4547, 1646, 1430, 607, 2118, 4709, 1149, 3283,
    2473, 1006, 3670, 1817, 771, 2173, 740, 1433, 2458, 3459, 2155, 1205, 413, 874, 2463, 1106,
    1590, 3873, 4026, 4272, 3556, 128, 1200, 130, 4494, 1871, 3073, 4386, 4098, 1923, 1176,
];

// Galileo E5a PRN initial states
const E5A_I_PRN_INIT: [u32; 50] = [
    0x30c5, 0x189c, 0x2e8b, 0x217f, 0x26ca, 0x3733, 0x1b8c, 0x155f, 0x0357, 0x309e, 0x2ee4, 0x0eba,
    0x3cff, 0x1e26, 0x0d1c, 0x1b05, 0x28aa, 0x1399, 0x29fe, 0x0198, 0x1370, 0x1eba, 0x2f25, 0x33c2,
    0x160a, 0x1901, 0x39d7, 0x2597, 0x3193, 0x2eae, 0x0350, 0x1889, 0x3335, 0x2474, 0x374e, 0x05df,
    0x22ce, 0x3b15, 0x3b9b, 0x29ad, 0x182c, 0x2e17, 0x0d84, 0x332d, 0x3935, 0x2abb, 0x21f3, 0x33d1,
    0x1eca, 0x16bf,
];

const E5A_Q_PRN_INIT: [u32; 50] = [
    0x2baa, 0x0a62, 0x29d3, 0x33e9, 0x2ef6, 0x29b0, 0x37ad, 0x2f28, 0x0f96, 0x03c5, 0x15cf, 0x3452,
    0x1c3d, 0x1da4, 0x3f6e, 0x053f, 0x04b5, 0x0d18, 0x2a26, 0x15dd, 0x08b2, 0x1298, 0x001f, 0x0c5f,
    0x08ca, 0x2186, 0x1272, 0x24aa, 0x315b, 0x298c, 0x0ff7, 0x35c5, 0x0a2a, 0x2f6b, 0x07c9, 0x0421,
    0x39fd, 0x0abc, 0x3eee, 0x1c85, 0x3cb8, 0x0d80, 0x2dfb, 0x1efd, 0x3ab7, 0x3cad, 0x1424, 0x2d22,
    0x2391, 0x2b09,
];

// Galileo E5b PRN initial states
const E5B_I_PRN_INIT: [u32; 50] = [
    0x0e90, 0x2c27, 0x00aa, 0x1e76, 0x1871, 0x0560, 0x035f, 0x2c13, 0x03d5, 0x219f, 0x04f4, 0x2fd9,
    0x31a0, 0x387c, 0x0d34, 0x0fbe, 0x3499, 0x10eb, 0x01ed, 0x2c3f, 0x13a4, 0x135f, 0x3a4d, 0x212a,
    0x39a5, 0x2bb4, 0x2303, 0x34ab, 0x04df, 0x31ff, 0x2e52, 0x24ff, 0x3c7d, 0x363d, 0x3669, 0x165c,
    0x0f1b, 0x108e, 0x3b36, 0x055b, 0x0ae9, 0x3051, 0x1808, 0x357e, 0x30d6, 0x3f1b, 0x2c12, 0x3bf8,
    0x0db8, 0x140f,
];

const E5B_Q_PRN_INIT: [u32; 50] = [
    0x06d9, 0x0c63, 0x2ad2, 0x26f9, 0x010b, 0x3c9d, 0x1fe8, 0x09e5, 0x1605, 0x3e60, 0x306d, 0x209f,
    0x0731, 0x33b2, 0x2e66, 0x0b67, 0x052e, 0x300b, 0x00d2, 0x11f1, 0x2df7, 0x3c04, 0x31cb, 0x0fb2,
    0x2388, 0x205c, 0x12b2, 0x11c6, 0x3863, 0x1229, 0x2b30, 0x1fb5, 0x34ec, 0x2298, 0x2066, 0x12f2,
    0x3ea6, 0x1ce4, 0x1a1c, 0x2b39, 0x2ba6, 0x246f, 0x08de, 0x1cee, 0x083d, 0x0596, 0x13c6, 0x3e09,
    0x2e21, 0x3214,
];

// Galileo memory codes are now loaded from external files
