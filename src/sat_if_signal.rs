//----------------------------------------------------------------------
// sat_if_signal.rs:
//   Implementation of satellite IF signal generation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::complex_number::ComplexNumber;
use crate::prngenerate::*;
use crate::types::{SatelliteParam, GnssSystem, GnssTime};
use crate::satellite_param::{get_travel_time, get_carrier_phase, get_transmit_time};
use crate::satellite_signal::SatelliteSignal;
use crate::constants::*;
use crate::fastmath::FastMath;
use crate::ifdatagen::NavBitTrait;

pub struct SatIfSignal {
    sample_number: i32,
    if_freq: i32,
    system: GnssSystem,
    signal_index: i32,
    svid: i32,
    pub sample_array: Vec<ComplexNumber>,
    prn_sequence: PrnGenerate,
    satellite_signal: SatelliteSignal,
    sat_param: Option<SatelliteParam>,
    data_length: i32,
    pilot_length: i32,
    glonass_half_cycle: bool,
    start_carrier_phase: f64,
    end_carrier_phase: f64,
    signal_time: GnssTime,
    start_transmit_time: GnssTime,
    end_transmit_time: GnssTime,
    data_signal: ComplexNumber,
    pilot_signal: ComplexNumber,
    half_cycle_flag: i32,
}

impl SatIfSignal {
    pub fn new(ms_sample_number: i32, sat_if_freq: i32, sat_system: GnssSystem, sat_signal_index: i32, sat_id: u8) -> Self {
        let prn = PrnGenerate::new(sat_system, sat_signal_index, sat_id as i32);
        let (data_len, pilot_len) = if let Some(attr) = &prn.attribute {
            let mut dl = attr.data_period * attr.chip_rate;
            let mut pl = attr.pilot_period * attr.chip_rate;

            if sat_system == GnssSystem::GpsSystem && sat_signal_index == SIGNAL_INDEX_L2C as i32 {
                dl = 10230;
                pl = 10230 * 75;
            }
            if sat_system == GnssSystem::GpsSystem && sat_signal_index == SIGNAL_INDEX_L2P as i32 {
                dl = 10230 * 2;
                pl = 1;
            }
            (if dl <= 0 { 1 } else { dl }, if pl <= 0 { 1 } else { pl })
        } else {
            (1, 1)
        };

        SatIfSignal {
            sample_number: ms_sample_number,
            if_freq: sat_if_freq,
            system: sat_system,
            signal_index: sat_signal_index,
            svid: sat_id as i32,
            sample_array: vec![ComplexNumber::new(); ms_sample_number as usize],
            prn_sequence: prn,
            satellite_signal: SatelliteSignal::new(),
            sat_param: None,
            data_length: data_len,
            pilot_length: pilot_len,
            glonass_half_cycle: (sat_if_freq % 1000) != 0,
            start_carrier_phase: 0.0,
            end_carrier_phase: 0.0,
            signal_time: GnssTime::default(),
            start_transmit_time: GnssTime::default(),
            end_transmit_time: GnssTime::default(),
            data_signal: ComplexNumber::new(),
            pilot_signal: ComplexNumber::new(),
            half_cycle_flag: 0,
        }
    }

    pub fn init_state(&mut self, cur_time: GnssTime, p_sat_param: &SatelliteParam, p_nav_data: Option<Box<dyn NavBitTrait>>) {
        self.sat_param = Some(*p_sat_param);
        if !self.satellite_signal.set_signal_attribute(self.system, self.signal_index, p_nav_data, self.svid) {
            // In Rust, we might handle this by setting self.satellite_signal.nav_data to None,
            // but set_signal_attribute already takes ownership, so we'd need to adjust its signature
            // or logic if we need to "null out" the nav data here.
            // For now, we assume set_signal_attribute handles the mismatch appropriately.
        }
        self.start_carrier_phase = get_carrier_phase(p_sat_param, self.signal_index as usize);
        self.signal_time = get_transmit_time(&cur_time, get_travel_time(p_sat_param, self.signal_index as usize));
        self.start_transmit_time = self.signal_time;
        self.satellite_signal.get_satellite_signal(self.signal_time, &mut self.data_signal, &mut self.pilot_signal);
        self.half_cycle_flag = 0;
    }

    pub fn get_if_sample(&mut self, cur_time: GnssTime) {
        let p_sat_param = if let Some(param) = &self.sat_param { param } else { return };

        self.signal_time = self.start_transmit_time;
        self.satellite_signal.get_satellite_signal(self.signal_time, &mut self.data_signal, &mut self.pilot_signal);
        self.end_carrier_phase = get_carrier_phase(p_sat_param, self.signal_index as usize);
        self.end_transmit_time = get_transmit_time(&cur_time, get_travel_time(p_sat_param, self.signal_index as usize));

        let mut phase_step = (self.start_carrier_phase - self.end_carrier_phase) / (self.sample_number as f64);
        phase_step += (self.if_freq as f64) / 1000.0 / (self.sample_number as f64);
        let mut cur_phase = self.start_carrier_phase - self.start_carrier_phase.floor();
        cur_phase = 1.0 - cur_phase;
        self.start_carrier_phase = self.end_carrier_phase;

        if self.glonass_half_cycle {
            cur_phase += if self.half_cycle_flag != 0 { 0.5 } else { 0.0 };
            self.half_cycle_flag = 1 - self.half_cycle_flag;
        }

        let mut transmit_ms_diff = self.end_transmit_time.MilliSeconds - self.start_transmit_time.MilliSeconds;
        if transmit_ms_diff < 0 {
            transmit_ms_diff += 86400000;
        }
        
        let code_attribute = self.prn_sequence.attribute.as_ref().unwrap();
        let code_diff = (transmit_ms_diff as f64 + self.end_transmit_time.SubMilliSeconds - self.start_transmit_time.SubMilliSeconds) * code_attribute.chip_rate as f64;
        let code_step = code_diff / (self.sample_number as f64);
        let mut cur_chip = (self.start_transmit_time.MilliSeconds as f64 % code_attribute.pilot_period as f64 + self.start_transmit_time.SubMilliSeconds) * code_attribute.chip_rate as f64;
        self.start_transmit_time = self.end_transmit_time;

        let amp = 10.0_f64.powf((p_sat_param.CN0 as f64 - 3000.0) / 1000.0) / (self.sample_number as f64).sqrt();

        for i in 0..self.sample_number as usize {
            let prn_value = self.get_prn_value(&mut cur_chip, code_step);
            let rotate_value = self.get_rotate_value(&mut cur_phase, phase_step);
            self.sample_array[i] = prn_value * rotate_value * amp;
        }
    }

    fn get_prn_value(&mut self, cur_chip: &mut f64, code_step: f64) -> ComplexNumber {
        let attribute = if let Some(attr) = &self.prn_sequence.attribute {
            attr
        } else {
            *cur_chip += code_step;
            return ComplexNumber::new();
        };

        let chip_count = *cur_chip as i32;
        let mut prn_value;

        let is_boc = (attribute.attribute & PRN_ATTRIBUTE_BOC) != 0;
        let is_qmboc = (attribute.attribute & PRN_ATTRIBUTE_QMBOC) != 0;
        let is_tmboc = (attribute.attribute & PRN_ATTRIBUTE_TMBOC) != 0;
        let is_cboc = (attribute.attribute & PRN_ATTRIBUTE_CBOC) != 0;
        let is_tdm = (attribute.attribute & PRN_ATTRIBUTE_TMD) != 0;

        if self.system == GnssSystem::GlonassSystem && (self.signal_index == SIGNAL_INDEX_G1 as i32 || self.signal_index == SIGNAL_INDEX_G2 as i32) {
            let data_chip = chip_count % self.data_length;
            let prn_bit = if let Some(data_prn) = &self.prn_sequence.data_prn {
                if data_chip >= 0 && (data_chip as usize) < data_prn.len() && data_prn[data_chip as usize] != 0 { 1 } else { 0 }
            } else { 0 };
            
            let nav_bit = if self.data_signal.real < 0.0 { 1 } else { 0 };
            
            let chip_time_ms = *cur_chip / 511.0;
            let total_ms = self.signal_time.MilliSeconds + chip_time_ms as i32;
            let meander = if (total_ms / 10) % 2 == 0 { 1 } else { 0 };
            
            let modulated_bit = prn_bit ^ nav_bit ^ meander;
            
            prn_value = ComplexNumber { real: if modulated_bit != 0 { -1.0 } else { 1.0 }, imag: 0.0 };
            
            *cur_chip += code_step;
            return prn_value;
        }

        if is_tdm {
            let current_ms = (*cur_chip / attribute.chip_rate as f64) as i32 % 2;
            if current_ms == 0 { // Even millisecond - L2CM (data)
                let data_chip = chip_count % self.data_length;
                prn_value = if let Some(data_prn) = &self.prn_sequence.data_prn {
                    if data_chip >= 0 && (data_chip as usize) < data_prn.len() {
                        self.data_signal * if data_prn[data_chip as usize] != 0 { -1.0 } else { 1.0 }
                    } else { ComplexNumber::new() }
                } else { ComplexNumber::new() };
            } else { // Odd millisecond - L2CL (pilot)
                let pilot_chip = chip_count % self.pilot_length;
                 prn_value = if let Some(pilot_prn) = &self.prn_sequence.pilot_prn {
                    if pilot_chip >= 0 && (pilot_chip as usize) < pilot_prn.len() {
                        self.pilot_signal * if pilot_prn[pilot_chip as usize] != 0 { -1.0 } else { 1.0 }
                    } else { ComplexNumber::new() }
                } else { ComplexNumber::new() };
            }
            *cur_chip += code_step;
            return prn_value;
        }

        let mut data_chip = chip_count % self.data_length;
        if is_boc { data_chip >>= 1; }

        prn_value = if let Some(data_prn) = &self.prn_sequence.data_prn {
            if data_chip >= 0 && (data_chip as usize) < data_prn.len() {
                let mut val = self.data_signal * if data_prn[data_chip as usize] != 0 { -1.0 } else { 1.0 };
                if is_boc && (chip_count & 1) != 0 {
                    val *= -1.0;
                }
                val
            } else { ComplexNumber::new() }
        } else { ComplexNumber::new() };

        if let Some(pilot_prn) = &self.prn_sequence.pilot_prn {
            if self.pilot_length > 0 {
                let mut pilot_chip = chip_count % self.pilot_length;
                if is_boc { pilot_chip >>= 1; }

                if pilot_chip >= 0 && (pilot_chip as usize) < pilot_prn.len() {
                    let mut pilot_val = self.pilot_signal * if pilot_prn[pilot_chip as usize] != 0 { -1.0 } else { 1.0 };
                    
                    if (is_tmboc || is_qmboc) && is_boc {
                        let symbol_pos = (self.signal_time.MilliSeconds % 330) / 10;
                        if symbol_pos == 1 || symbol_pos == 5 || symbol_pos == 7 || symbol_pos == 30 {
                            let sub_chip_pos = chip_count % 12;
                            if sub_chip_pos >= 6 { pilot_val *= -1.0; }
                        } else if (chip_count & 1) != 0 { pilot_val *= -1.0; }
                        prn_value += pilot_val;
                    } else if is_cboc && is_boc {
                        let chip_in_code = chip_count % 4092;
                        if (chip_in_code % 11) == 0 {
                            let boc6_phase = chip_count % 12;
                            if boc6_phase >= 6 { pilot_val *= -1.0; }
                        } else if (chip_count & 1) != 0 { pilot_val *= -1.0; }
                        prn_value += pilot_val;
                    } else {
                        if is_boc && (chip_count & 1) != 0 {
                            pilot_val *= -1.0;
                        }
                        prn_value += pilot_val;
                    }
                }
            }
        }

        let old_data_chip = data_chip;
        *cur_chip += code_step;
        if self.data_length > 0 && ((*cur_chip as i32) % self.data_length) < old_data_chip {
            self.signal_time.MilliSeconds += attribute.data_period;
            self.satellite_signal.get_satellite_signal(self.signal_time, &mut self.data_signal, &mut self.pilot_signal);
        }

        prn_value
    }

    fn get_rotate_value(&self, cur_phase: &mut f64, phase_step: f64) -> ComplexNumber {
        let rotate = FastMath::fast_rotate(*cur_phase * PI2);
        *cur_phase += phase_step;
        rotate
    }
}
