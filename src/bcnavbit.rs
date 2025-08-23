//----------------------------------------------------------------------
// bcnavbit.rs:
//   Implementation of navigation bit synthesis class for BDS3 navigation bit
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::types::*;
//use crate::constants::*;
use crate::COMPOSE_BITS;

pub struct BCNavBit {
    // Ephemeris parameters part 1 (63 satellites, 9 parameters each)
    pub Ephemeris1: [[u32; 9]; 63],
    
    // Ephemeris parameters part 2 (63 satellites, 10 parameters each)
    pub Ephemeris2: [[u32; 10]; 63],
    
    // Clock parameters (63 satellites, 4 parameters each)
    pub ClockParam: [[u32; 4]; 63],
    
    // Integrity flags (63 satellites)
    pub IntegrityFlags: [u32; 63],
    
    // TGS/ISC parameters (63 satellites, 3 parameters each)
    pub TgsIscParam: [[u32; 3]; 63],
    
    // Reduced almanac (63 satellites, 2 parameters each)
    pub ReducedAlmanac: [[u32; 2]; 63],
    
    // Midi almanac (63 satellites, 7 parameters each)
    pub MidiAlmanac: [[u32; 7]; 63],
    
    // Ionosphere parameters (BDGIM format)
    pub BdGimIono: [u32; 4],
    
    // BDT-UTC parameters
    pub BdtUtcParam: [u32; 5],
    
    // EOP parameters
    pub EopParam: [u32; 6],
    
    // BGTO parameters
    pub BgtoParam: [[u32; 3]; 7],
    
    // Almanac week
    pub AlmanacWeek: u32,
    
    // Almanac time of applicability
    pub AlmanacToa: u32,
}

impl BCNavBit {
    // CRC24Q table
    const CRC24Q: [u32; 256] = [
        0x00000000, 0x864CFB00, 0x8AD50D00, 0x0C99F600, 0x93E6E100, 0x15AA1A00, 0x1933EC00, 0x9F7F1700, 
        0xA1813900, 0x27CDC200, 0x2B543400, 0xAD18CF00, 0x3267D800, 0xB42B2300, 0xB8B2D500, 0x3EFE2E00, 
        0xC54E8900, 0x43027200, 0x4F9B8400, 0xC9D77F00, 0x56A86800, 0xD0E49300, 0xDC7D6500, 0x5A319E00, 
        0x64CFB000, 0xE2834B00, 0xEE1ABD00, 0x68564600, 0xF7295100, 0x7165AA00, 0x7DFC5C00, 0xFBB0A700, 
        0x0CD1E900, 0x8A9D1200, 0x8604E400, 0x00481F00, 0x9F370800, 0x197BF300, 0x15E20500, 0x93AEFE00, 
        0xAD50D000, 0x2B1C2B00, 0x2785DD00, 0xA1C92600, 0x3EB63100, 0xB8FACA00, 0xB4633C00, 0x322FC700, 
        0xC99F6000, 0x4FD39B00, 0x434A6D00, 0xC5069600, 0x5A798100, 0xDC357A00, 0xD0AC8C00, 0x56E07700, 
        0x681E5900, 0xEE52A200, 0xE2CB5400, 0x6487AF00, 0xFBF8B800, 0x7DB44300, 0x712DB500, 0xF7614E00, 
        0x19A3D200, 0x9FEF2900, 0x9376DF00, 0x153A2400, 0x8A453300, 0x0C09C800, 0x00903E00, 0x86DCC500, 
        0xB822EB00, 0x3E6E1000, 0x32F7E600, 0xB4BB1D00, 0x2BC40A00, 0xAD88F100, 0xA1110700, 0x275DFC00, 
        0xDCED5B00, 0x5AA1A000, 0x56385600, 0xD074AD00, 0x4F0BBA00, 0xC9474100, 0xC5DEB700, 0x43924C00, 
        0x7D6C6200, 0xFB209900, 0xF7B96F00, 0x71F59400, 0xEE8A8300, 0x68C67800, 0x645F8E00, 0xE2137500, 
        0x15723B00, 0x933EC000, 0x9FA73600, 0x19EBCD00, 0x8694DA00, 0x00D82100, 0x0C41D700, 0x8A0D2C00, 
        0xB4F30200, 0x32BFF900, 0x3E260F00, 0xB86AF400, 0x2715E300, 0xA1591800, 0xADC0EE00, 0x2B8C1500, 
        0xD03CB200, 0x56704900, 0x5AE9BF00, 0xDCA54400, 0x43DA5300, 0xC596A800, 0xC90F5E00, 0x4F43A500, 
        0x71BD8B00, 0xF7F17000, 0xFB688600, 0x7D247D00, 0xE25B6A00, 0x64179100, 0x688E6700, 0xEEC29C00, 
        0x3347A400, 0xB50B5F00, 0xB992A900, 0x3FDE5200, 0xA0A14500, 0x26EDBE00, 0x2A744800, 0xAC38B300, 
        0x92C69D00, 0x148A6600, 0x18139000, 0x9E5F6B00, 0x01207C00, 0x876C8700, 0x8BF57100, 0x0DB98A00, 
        0xF6092D00, 0x7045D600, 0x7CDC2000, 0xFA90DB00, 0x65EFCC00, 0xE3A33700, 0xEF3AC100, 0x69763A00, 
        0x57881400, 0xD1C4EF00, 0xDD5D1900, 0x5B11E200, 0xC46EF500, 0x42220E00, 0x4EBBF800, 0xC8F70300, 
        0x3F964D00, 0xB9DAB600, 0xB5434000, 0x330FBB00, 0xAC70AC00, 0x2A3C5700, 0x26A5A100, 0xA0E95A00, 
        0x9E177400, 0x185B8F00, 0x14C27900, 0x928E8200, 0x0DF19500, 0x8BBD6E00, 0x87249800, 0x01686300, 
        0xFAD8C400, 0x7C943F00, 0x700DC900, 0xF6413200, 0x693E2500, 0xEF72DE00, 0xE3EB2800, 0x65A7D300, 
        0x5B59FD00, 0xDD150600, 0xD18CF000, 0x57C00B00, 0xC8BF1C00, 0x4EF3E700, 0x426A1100, 0xC426EA00, 
        0x2AE47600, 0xACA88D00, 0xA0317B00, 0x267D8000, 0xB9029700, 0x3F4E6C00, 0x33D79A00, 0xB59B6100, 
        0x8B654F00, 0x0D29B400, 0x01B04200, 0x87FCB900, 0x1883AE00, 0x9ECF5500, 0x9256A300, 0x141A5800, 
        0xEFAAFF00, 0x69E60400, 0x657FF200, 0xE3330900, 0x7C4C1E00, 0xFA00E500, 0xF6991300, 0x70D5E800, 
        0x4E2BC600, 0xC8673D00, 0xC4FECB00, 0x42B23000, 0xDDCD2700, 0x5B81DC00, 0x57182A00, 0xD154D100, 
        0x26359F00, 0xA0796400, 0xACE09200, 0x2AAC6900, 0xB5D37E00, 0x339F8500, 0x3F067300, 0xB94A8800, 
        0x87B4A600, 0x01F85D00, 0x0D61AB00, 0x8B2D5000, 0x14524700, 0x921EBC00, 0x9E874A00, 0x18CBB100, 
        0xE37B1600, 0x6537ED00, 0x69AE1B00, 0xEFE2E000, 0x709DF700, 0xF6D10C00, 0xFA48FA00, 0x7C040100, 
        0x42FA2F00, 0xC4B6D400, 0xC82F2200, 0x4E63D900, 0xD11CCE00, 0x57503500, 0x5BC9C300, 0xDD853800,
    ];

    // E2V table
    const E2V_TABLE: [u32; 128] = [
        1,  2,  4,  8, 16, 32,  3,  6, 12, 24, 48, 35,  5, 10, 20, 40,
        19, 38, 15, 30, 60, 59, 53, 41, 17, 34, 7, 14, 28, 56, 51, 37, 
        9, 18, 36, 11, 22, 44, 27, 54, 47, 29, 58, 55, 45, 25, 50, 39, 
        13, 26, 52, 43, 21, 42, 23, 46, 31, 62, 63, 61, 57, 49, 33, 1,
        2,  4,  8, 16, 32,  3,  6, 12, 24, 48, 35,  5, 10, 20, 40, 19,
        38, 15, 30, 60, 59, 53, 41, 17, 34, 7, 14, 28, 56, 51, 37, 9,
        18, 36, 11, 22, 44, 27, 54, 47, 29, 58, 55, 45, 25, 50, 39, 13,
        26, 52, 43, 21, 42, 23, 46, 31, 62, 63, 61, 57, 49, 33, 1, 2,
    ];

    // V2E table
    const V2E_TABLE: [u32; 64] = [
        0,  1,  2,  7,  3, 13,  8, 27,  4, 33, 14, 36,  9, 49, 28, 19,
        5, 25, 34, 17, 15, 53, 37, 55, 10, 46, 50, 39, 29, 42, 20, 57,
        6, 63, 26, 12, 35, 32, 18, 48, 16, 24, 54, 52, 38, 45, 56, 41,
        11, 62, 47, 31, 51, 23, 40, 44, 30, 61, 43, 22, 21, 60, 58, 59,
    ];

    pub fn new() -> Self {
        BCNavBit {
            Ephemeris1: [[0; 9]; 63],
            Ephemeris2: [[0; 10]; 63],
            ClockParam: [[0; 4]; 63],
            IntegrityFlags: [0; 63],
            TgsIscParam: [[0; 3]; 63],
            ReducedAlmanac: [[0; 2]; 63],
            MidiAlmanac: [[0; 7]; 63],
            BdGimIono: [0; 4],
            BdtUtcParam: [0; 5],
            EopParam: [0; 6],
            BgtoParam: [[0; 3]; 7],
            AlmanacWeek: 0,
            AlmanacToa: 0,
        }
    }



    pub fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) -> i32 {
        if svid < 1 || svid > 63 || eph.valid == 0 {
            return 0;
        }
        if (eph.toe % 300) != 0 {
            // BCNAV ephemeris requires toe be multiple of 300
            return 0;
        }

        let svid_idx = (svid - 1) as usize;

        // Вычисляем ВСЕ значения заранее, чтобы избежать конфликта заимствований
        let toe_value = (eph.toe / 300) as u32;
        let sat_type = eph.flag as u32;
        let axis_scaled = Self::unscale_int(
            eph.axis
                - (if sat_type == 3 {
                    27906100.0
                } else {
                    42162200.0
                }),
            -9,
        );
        let axis_dot_scaled = Self::unscale_int(eph.axis_dot, -21);
        let delta_n_scaled = Self::unscale_int(eph.delta_n / std::f64::consts::PI, -44);
        let delta_n_dot_scaled = Self::unscale_int(eph.delta_n_dot, -57);
        let m0_scaled = Self::unscale_long(eph.M0 / std::f64::consts::PI, -32);
        let ecc_scaled = Self::unscale_ulong(eph.ecc, -34);
        let w_scaled = Self::unscale_long(eph.w / std::f64::consts::PI, -32);

        // Теперь работаем с данными без дополнительных заимствований self
        {
            let data: &mut [u32; 9] = &mut self.Ephemeris1[svid_idx];

            // fill in Ephemeris1
            data[0] = COMPOSE_BITS!(eph.iode as u32, 16, 8);
            data[0] |= COMPOSE_BITS!(toe_value, 5, 11);
            data[0] |= COMPOSE_BITS!(sat_type, 3, 2);
            data[0] |= COMPOSE_BITS!((axis_scaled >> 23) as u32, 0, 3);
            data[1] = COMPOSE_BITS!(axis_scaled as u32, 1, 23);
            data[1] |= COMPOSE_BITS!((axis_dot_scaled >> 24) as u32, 0, 1);
            data[2] = COMPOSE_BITS!(axis_dot_scaled as u32, 0, 24);
            data[3] = COMPOSE_BITS!(delta_n_scaled as u32, 7, 17);
            data[3] |= COMPOSE_BITS!((delta_n_dot_scaled >> 16) as u32, 0, 7);
            data[4] = COMPOSE_BITS!(delta_n_dot_scaled as u32, 8, 16);

            let m0_sign = if (m0_scaled & 0x100000000i64) != 0 {
                1
            } else {
                0
            };
            let m0_value = m0_scaled as u32;
            data[4] |= COMPOSE_BITS!(m0_sign as u32, 7, 1);
            data[4] |= COMPOSE_BITS!(m0_value >> 25, 0, 7);
            data[5] = COMPOSE_BITS!(m0_value >> 1, 0, 24);
            data[6] = COMPOSE_BITS!(m0_value, 23, 1);

            let ecc_sign = if (ecc_scaled & 0x100000000u64) != 0 {
                1
            } else {
                0
            };
            let ecc_value = ecc_scaled as u32;
            data[6] |= COMPOSE_BITS!(ecc_sign as u32, 22, 1);
            data[6] |= COMPOSE_BITS!(ecc_value >> 10, 0, 22);
            data[7] = COMPOSE_BITS!(ecc_value, 14, 10);

            let w_sign = if (w_scaled & 0x100000000i64) != 0 {
                1
            } else {
                0
            };
            let w_value = w_scaled as u32;
            data[7] |= COMPOSE_BITS!(w_sign as u32, 13, 1);
            data[7] |= COMPOSE_BITS!(w_value >> 19, 0, 13);
            data[8] = COMPOSE_BITS!(w_value, 5, 19);
        }

        // Вычисляем значения для Ephemeris2
        let omega0_scaled = Self::unscale_long(eph.omega0 / std::f64::consts::PI, -32);
        let i0_scaled = Self::unscale_long(eph.i0 / std::f64::consts::PI, -32);
        let omega_dot_scaled = Self::unscale_int(eph.omega_dot / std::f64::consts::PI, -44);
        let idot_scaled = Self::unscale_int(eph.idot / std::f64::consts::PI, -44);
        let cis_scaled = Self::unscale_int(eph.cis, -30);
        let cic_scaled = Self::unscale_int(eph.cic, -30);
        let crs_scaled = Self::unscale_int(eph.crs, -8);
        let crc_scaled = Self::unscale_int(eph.crc, -8);
        let cus_scaled = Self::unscale_int(eph.cus, -30);
        let cuc_scaled = Self::unscale_int(eph.cuc, -30);

        // Вычисляем значения для ClockParam
        let toc_value = (eph.toc / 300) as u32;
        let af0_scaled = Self::unscale_int(eph.af0, -34);
        let af1_scaled = Self::unscale_int(eph.af1, -50);
        let af2_scaled = Self::unscale_int(eph.af2, -66);

        // Вычисляем значения для TGD и ISC
        let isc_b1c_scaled = Self::unscale_int(eph.tgd_ext[0] - eph.tgd_ext[1], -34);
        let tgd_b1c_scaled = Self::unscale_int(eph.tgd_ext[1], -34);
        let isc_b2a_scaled = Self::unscale_int(eph.tgd_ext[2] - eph.tgd_ext[3], -34);
        let tgd_b2a_scaled = Self::unscale_int(eph.tgd_ext[1], -34);
        let tgd_b2b_scaled = Self::unscale_int(eph.tgd_ext[4], -34);

        // fill in Ephemeris2
        {
            let data: &mut [u32; 10] = &mut self.Ephemeris2[svid_idx];
            let omega0_sign = if (omega0_scaled & 0x100000000i64) != 0 {
                1
            } else {
                0
            };
            let omega0_value = omega0_scaled as u32;
            data[0] = COMPOSE_BITS!(omega0_sign as u32, 23, 1);
            data[0] |= COMPOSE_BITS!(omega0_value >> 9, 0, 23);
            data[1] = COMPOSE_BITS!(omega0_value, 15, 9);

            let i0_sign = if (i0_scaled & 0x100000000i64) != 0 {
                1
            } else {
                0
            };
            let i0_value = i0_scaled as u32;
            data[1] |= COMPOSE_BITS!(i0_sign as u32, 14, 1);
            data[1] |= COMPOSE_BITS!(i0_value >> 18, 0, 14);
            data[2] = COMPOSE_BITS!(i0_value, 6, 18);
            data[2] |= COMPOSE_BITS!((omega_dot_scaled >> 13) as u32, 0, 6);
            data[3] = COMPOSE_BITS!(omega_dot_scaled as u32, 11, 13);
            data[3] |= COMPOSE_BITS!((idot_scaled >> 4) as u32, 0, 11);
            data[4] = COMPOSE_BITS!(idot_scaled as u32, 20, 4);
            data[4] |= COMPOSE_BITS!(cis_scaled as u32, 4, 16);
            data[4] |= COMPOSE_BITS!((cic_scaled >> 12) as u32, 0, 4);
            data[5] = COMPOSE_BITS!(cic_scaled as u32, 12, 12);
            data[5] |= COMPOSE_BITS!((crs_scaled >> 12) as u32, 0, 12);
            data[6] = COMPOSE_BITS!(crs_scaled as u32, 12, 12);
            data[6] |= COMPOSE_BITS!((crc_scaled >> 12) as u32, 0, 12);
            data[7] = COMPOSE_BITS!(crc_scaled as u32, 12, 12);
            data[7] |= COMPOSE_BITS!((cus_scaled >> 9) as u32, 0, 12);
            data[8] = COMPOSE_BITS!(cus_scaled as u32, 15, 9);
            data[8] |= COMPOSE_BITS!((cuc_scaled >> 6) as u32, 0, 15);
            data[9] = COMPOSE_BITS!(cuc_scaled as u32, 18, 6);
        }

        // fill in ClockParam
        {
            let data: &mut [u32; 4] = &mut self.ClockParam[svid_idx];
            data[0] = COMPOSE_BITS!(toc_value, 13, 11);
            data[0] |= COMPOSE_BITS!((af0_scaled >> 12) as u32, 0, 13);
            data[1] = COMPOSE_BITS!(af0_scaled as u32, 12, 12);
            data[1] |= COMPOSE_BITS!((af1_scaled >> 10) as u32, 0, 12);
            data[2] = COMPOSE_BITS!(af1_scaled as u32, 14, 10);
            data[2] |= COMPOSE_BITS!(af2_scaled as u32, 3, 11);
            data[3] |= COMPOSE_BITS!(eph.iodc as u32, 0, 10);
        }

        // fill in IntegrityFlags
        self.IntegrityFlags[svid_idx] = (eph.flag >> 2) as u32;

        // fill in TGD and ISC
        {
            let data: &mut [u32; 3] = &mut self.TgsIscParam[svid_idx];
            data[0] = COMPOSE_BITS!(isc_b1c_scaled as u32, 12, 12);
            data[0] |= COMPOSE_BITS!(tgd_b1c_scaled as u32, 0, 12);
            data[1] = COMPOSE_BITS!(isc_b2a_scaled as u32, 12, 12);
            data[1] |= COMPOSE_BITS!(tgd_b2a_scaled as u32, 0, 12);
            data[2] = COMPOSE_BITS!(tgd_b2b_scaled as u32, 0, 12);
        }

        svid
    }

    pub fn set_almanac(&mut self, alm: &[GpsAlmanac]) -> i32 {
        // fill in almanac page
        for i in 0..63 {
            // Извлекаем данные перед вызовом метода, чтобы избежать конфликта заимствований
            let alm_item = &alm[i];

            // Теперь можем вызвать статический метод без конфликта заимствований
            Self::fill_bds_almanac_page(
                alm_item,
                &mut self.MidiAlmanac[i],
                &mut self.ReducedAlmanac[i],
            );

            if (alm[i].valid & 1) != 0 {
                self.AlmanacToa = (alm[i].toa >> 12) as u32;
                self.AlmanacWeek = alm[i].week as u32;
            }
        }

        0
    }

    pub fn set_iono_utc(&mut self, _iono_param: &IonoParam, _utc_param: &UtcParam) -> i32 {
        // Default implementation - does nothing
        // Derived classes (like BCNav1Bit) can override this
        0
    }

    // put Length bits in 24bit WORD Src into 24bit WORD Dest starting from StartBit with MSB to LSB order
    pub fn append_word(&self, dest: &mut [u32], start_bit: i32, src: &[u32], length: i32) -> i32 {
        let mut remain_bits = 24;
        let mut fill_bits;
        let mut start_bit = start_bit;
        let mut dest_idx = 0;
        let mut src_idx = 0;

        let mut length = length;
        while length > 0 {
            fill_bits = 24 - start_bit;
            fill_bits = if fill_bits <= remain_bits {
                fill_bits
            } else {
                remain_bits
            };
            fill_bits = if fill_bits <= length {
                fill_bits
            } else {
                length
            };
            dest[dest_idx] = if start_bit == 0 { 0 } else { dest[dest_idx] }
                | COMPOSE_BITS!(
                    src[src_idx] >> (remain_bits - fill_bits),
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

        start_bit
    }

    // put bit in Data from MSB ot LSB into BitStream, bit order from bit(BitNumber-1) to bit(0) of Data
    pub fn assign_bits(&self, data: u32, bit_number: i32, bit_stream: &mut [i32]) -> i32 {
        let mut data = data << (32 - bit_number);
        for i in 0..bit_number {
            bit_stream[i as usize] = if (data & 0x80000000) != 0 { 1 } else { 0 };
            data <<= 1;
        }

        bit_number
    }

    // Append CRC to the end of data stream, Length is the size of DataStream (24bit data in each DWORD) including CRC bits
    pub fn append_crc(&self, data_stream: &mut [u32], length: i32) -> i32 {
        let mut crc_result: u32 = 0;

        for i in 0..(length - 1) as usize {
            let mut data = data_stream[i] << 8; // move data to MSB
            let index = ((data >> 24) ^ (crc_result >> 16)) as u8 as usize;
            crc_result = (crc_result << 8) ^ Self::CRC24Q[index];
            data <<= 8;
            let index = ((data >> 24) ^ (crc_result >> 16)) as u8 as usize;
            crc_result = (crc_result << 8) ^ Self::CRC24Q[index];
            data <<= 8;
            let index = ((data >> 24) ^ (crc_result >> 16)) as u8 as usize;
            crc_result = (crc_result << 8) ^ Self::CRC24Q[index];
        }
        data_stream[length as usize - 1] = crc_result & 0xffffff;

        0
    }

    pub fn ldpc_encode(
        &self,
        symbol_stream: &mut [i32],
        symbol_length: i32,
        matrix_gen: &str,
    ) -> i32 {
        // Check for NULL pointer
        if matrix_gen.is_empty() || symbol_length <= 0 {
            return -1;
        }

        let matrix_chars: Vec<char> = matrix_gen.chars().collect();
        let mut matrix_idx = 0;

        for i in 0..symbol_length {
            let mut sum = 0;
            for j in 0..symbol_length {
                if matrix_idx >= matrix_chars.len() {
                    return -1;
                }
                let matrix_val = matrix_chars[matrix_idx] as i32 - '0' as i32;
                sum ^= self.gf6_int_mul(matrix_val, symbol_stream[j as usize]);
                matrix_idx += 1;
            }
            symbol_stream[(symbol_length + i) as usize] = sum;
        }

        0
    }

    pub fn gf6_int_mul(&self, a: i32, b: i32) -> i32 {
        if a != 0 && b != 0 {
           Self::E2V_TABLE[(Self::V2E_TABLE[a as usize] + Self::V2E_TABLE[b as usize]) as usize] as i32
        } else {
            0
        }
    }

    fn fill_bds_almanac_page(
        almanac: &GpsAlmanac,
        midi_alm: &mut [u32],
        reduced_alm: &mut [u32],
    ) -> i32 {
        if (almanac.valid & 1) == 0 {
            // almanac not valid
            midi_alm[0] = 0; // set PRN field to 0
            reduced_alm[0] = 0;
            return 0;
        }

        // fill midi almanac
        midi_alm[0] = COMPOSE_BITS!(almanac.svid as u32, 18, 6); // PRN
        midi_alm[0] |= COMPOSE_BITS!(almanac.flag as u32, 16, 2); // SatType
        midi_alm[0] |= COMPOSE_BITS!(almanac.week as u32, 3, 13); // WN
        let mut uint_value = (almanac.toa >> 12) as u32; // toa
        midi_alm[0] |= COMPOSE_BITS!(uint_value >> 5, 0, 3);
        midi_alm[1] = COMPOSE_BITS!(uint_value, 19, 5);
        uint_value = Self::unscale_uint(almanac.ecc, -16); // ecc
        midi_alm[1] |= COMPOSE_BITS!(uint_value, 8, 11);
        let mut int_value = Self::unscale_int(
            almanac.i0 / std::f64::consts::PI - (if almanac.flag == 1 { 0.0 } else { 0.3 }),
            -14,
        ); // delta_i
        midi_alm[1] |= COMPOSE_BITS!((int_value >> 3) as u32, 0, 8);
        midi_alm[2] = COMPOSE_BITS!(int_value as u32, 21, 3);
        uint_value = Self::unscale_uint(almanac.sqrtA, -4); // sqrtA
        midi_alm[2] |= COMPOSE_BITS!(uint_value, 4, 17);
        int_value = Self::unscale_int(almanac.omega0 / std::f64::consts::PI, -15); // omega0
        midi_alm[2] |= COMPOSE_BITS!((int_value >> 12) as u32, 0, 4);
        midi_alm[3] = COMPOSE_BITS!(int_value as u32, 12, 12);
        int_value = Self::unscale_int(almanac.omega_dot / std::f64::consts::PI, -33); // omega_dot
        midi_alm[3] |= COMPOSE_BITS!(int_value as u32, 1, 11);
        int_value = Self::unscale_int(almanac.w / std::f64::consts::PI, -15); // w
        midi_alm[3] |= COMPOSE_BITS!((int_value >> 15) as u32, 0, 1);
        midi_alm[4] = COMPOSE_BITS!(int_value as u32, 9, 15);
        int_value = Self::unscale_int(almanac.M0 / std::f64::consts::PI, -15); // M0
        midi_alm[4] |= COMPOSE_BITS!((int_value >> 7) as u32, 0, 9);
        midi_alm[5] = COMPOSE_BITS!(int_value as u32, 17, 7);
        int_value = Self::unscale_int(almanac.af0, -20); // af0
        midi_alm[5] |= COMPOSE_BITS!(int_value as u32, 6, 11);
        int_value = Self::unscale_int(almanac.af1, -37); // af1
        midi_alm[5] |= COMPOSE_BITS!((int_value >> 4) as u32, 0, 6);
        midi_alm[6] = COMPOSE_BITS!(int_value as u32, 20, 4);
        midi_alm[6] |= COMPOSE_BITS!(almanac.health as u32, 12, 8); // use B1I health as clock health

        // fill reduced almanac
        reduced_alm[0] = COMPOSE_BITS!(almanac.svid as u32, 18, 6); // PRN
        reduced_alm[0] |= COMPOSE_BITS!(almanac.flag as u32, 16, 2); // SatType
        int_value = Self::unscale_int(
            almanac.sqrtA * almanac.sqrtA
                - (if almanac.flag == 3 {
                    27906100.0
                } else {
                    42162200.0
                }),
            9,
        ); // deltaA
        reduced_alm[0] |= COMPOSE_BITS!(int_value as u32, 8, 8);
        int_value = Self::unscale_int(almanac.omega0 / std::f64::consts::PI, -6); // omega0
        reduced_alm[0] |= COMPOSE_BITS!(int_value as u32, 1, 7);
        int_value = Self::unscale_int((almanac.M0 + almanac.w) / std::f64::consts::PI, -6); // Phi0
        reduced_alm[0] |= COMPOSE_BITS!((int_value >> 6) as u32, 0, 1);
        reduced_alm[1] = COMPOSE_BITS!(int_value as u32, 18, 6);
        reduced_alm[1] |= COMPOSE_BITS!(almanac.health as u32, 10, 8); // use B1I health as clock health

        0
    }

    // Helper methods for scaling values - made static since they don't use self
    fn unscale_int(value: f64, scale: i32) -> i32 {
        let factor = 2.0f64.powi(scale);
        (value / factor).round() as i32
    }

    fn unscale_uint(value: f64, scale: i32) -> u32 {
        let factor = 2.0f64.powi(scale);
        (value / factor).round() as u32
    }

    fn unscale_long(value: f64, scale: i32) -> i64 {
        let factor = 2.0f64.powi(scale);
        (value / factor).round() as i64
    }

    fn unscale_ulong(value: f64, scale: i32) -> u64 {
        let factor = 2.0f64.powi(scale);
        (value / factor).round() as u64
    }
}
