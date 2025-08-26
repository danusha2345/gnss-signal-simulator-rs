//----------------------------------------------------------------------
// satellite_signal.rs:
//   Implementation of functions to calculate satellite signal with data/pilot modulation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use std::mem;
use crate::complex_number::ComplexNumber;
use crate::gnsstime;
use crate::gnsstime::GnssTimeConverter;
use crate::pilotbit::get_pilot_bits;
use crate::constants::*;
use crate::types::{GnssTime, GnssSystem};
use crate::NavDataType;
use crate::ifdatagen::NavBitTrait;

const AMPLITUDE_1_2: f64 = 0.7071067811865475244;
const AMPLITUDE_1_4: f64 = 0.5;
const AMPLITUDE_29_44: f64 = 0.811844140885988713377;
const AMPLITUDE_3_4: f64 = 0.8660254037844386468;
const AMPLITUDE_5_11: f64 = 0.6741998624632421;

pub struct SignalAttribute {
    pub code_length: i32,
    pub nh_length: i32,
    pub nh_code: u32,
    pub frame_length: i32,
}

const SIGNAL_ATTRIBUTES: [SignalAttribute; 15] = [
    // CodeLength NHLength  NHCode FrameLength
    SignalAttribute { code_length: 1, nh_length: 20, nh_code: 0x0, frame_length: 6000 },      // index  0 for LNAV
    SignalAttribute { code_length: 10, nh_length: 1, nh_code: 0x0, frame_length: 18000 },     // index  1 for CNAV2
    SignalAttribute { code_length: 20, nh_length: 1, nh_code: 0x0, frame_length: 12000 },     // index  2 for CNAV L2C
    SignalAttribute { code_length: 1, nh_length: 10, nh_code: 0x2b0, frame_length: 6000 },    // index  3 for CNAV L5
    SignalAttribute { code_length: 10, nh_length: 1, nh_code: 0x0, frame_length: 18000 },     // index  4 for BCNAV1
    SignalAttribute { code_length: 1, nh_length: 20, nh_code: 0x72b20, frame_length: 6000 },  // index  5 for D1
    SignalAttribute { code_length: 1, nh_length: 2, nh_code: 0x0, frame_length: 600 },       // index  6 for D2
    SignalAttribute { code_length: 1, nh_length: 5, nh_code: 0x8, frame_length: 3000 },      // index  7 for BCNAV2
    SignalAttribute { code_length: 1, nh_length: 1, nh_code: 0x0, frame_length: 1000 },      // index  8 for BCNAV3
    SignalAttribute { code_length: 4, nh_length: 1, nh_code: 0x0, frame_length: 2000 },      // index  9 for I/NAV E1
    SignalAttribute { code_length: 1, nh_length: 20, nh_code: 0x97421, frame_length: 10000 }, // index 10 for FNAV
    SignalAttribute { code_length: 1, nh_length: 4, nh_code: 0x7, frame_length: 2000 },      // index 11 for I/NAV E5b
    SignalAttribute { code_length: 1, nh_length: 1, nh_code: 0x0, frame_length: 1000 },      // index 12 for CNAV E6
    SignalAttribute { code_length: 1, nh_length: 10, nh_code: 0x0, frame_length: 2000 },     // index 13 for GNAV
    SignalAttribute { code_length: 1, nh_length: 250, nh_code: 0x0, frame_length: 10000 },   // index 14 for G3/L3OC (250 secondary code)
];

pub struct SatelliteSignal {
    sat_system: GnssSystem,
    sat_signal: i32,
    svid: i32,
    current_frame: i32,
    nav_data: Option<Box<dyn NavBitTrait>>,
    is_in_time_marker: bool,
    time_marker_bits: [u8; 30],
    data_bits: [u8; 4096], // Adjusted size for safety
    secondary_code: u32,
    secondary_length: i32,
    attribute: &'static SignalAttribute,
}

impl SatelliteSignal {
    pub fn new() -> Self {
        SatelliteSignal {
            sat_system: GnssSystem::GpsSystem,
            sat_signal: 0,
            svid: -1,
            current_frame: -1,
            nav_data: None,
            is_in_time_marker: false,
            time_marker_bits: [0; 30],
            data_bits: [0; 4096],
            secondary_code: 0,
            secondary_length: 0,
            attribute: &SIGNAL_ATTRIBUTES[0],
        }
    }

    pub fn set_signal_attribute(&mut self, system: GnssSystem, signal_index: i32, p_nav_data: Option<Box<dyn NavBitTrait>>, svid: i32) -> bool {
        // Define constants for pattern matching
        const L1CA: i32 = crate::SIGNAL_INDEX_L1CA as i32;
        const L1C: i32 = crate::SIGNAL_INDEX_L1C as i32;
        const L2C: i32 = crate::SIGNAL_INDEX_L2C as i32;
        const L5: i32 = crate::SIGNAL_INDEX_L5 as i32;
        const L2P: i32 = crate::SIGNAL_INDEX_L2P as i32;
        const B1C: i32 = crate::SIGNAL_INDEX_B1C as i32;
        const B1I: i32 = crate::SIGNAL_INDEX_B1I as i32;
        const B2I: i32 = crate::SIGNAL_INDEX_B2I as i32;
        const B3I: i32 = crate::SIGNAL_INDEX_B3I as i32;
        const B2A: i32 = crate::SIGNAL_INDEX_B2A as i32;
        const B2B: i32 = crate::SIGNAL_INDEX_B2B as i32;
        const E1: i32 = crate::SIGNAL_INDEX_E1 as i32;
        const E5A: i32 = crate::SIGNAL_INDEX_E5A as i32;
        const E5B: i32 = crate::SIGNAL_INDEX_E5B as i32;
        const E6: i32 = crate::SIGNAL_INDEX_E6 as i32;
        const G1: i32 = crate::SIGNAL_INDEX_G1 as i32;
        const G2: i32 = crate::SIGNAL_INDEX_G2 as i32;
        const G3: i32 = crate::SIGNAL_INDEX_G3 as i32;

        self.sat_system = system;
        self.sat_signal = signal_index;
        self.nav_data = p_nav_data;
        self.svid = svid;
        self.current_frame = -1;
        self.data_bits.fill(0);

        let (attribute_index, nav_type_check) = match self.sat_system {
            GnssSystem::GpsSystem => match self.sat_signal {
                L1CA => (0, NavDataType::LNav),
                L1C => (1, NavDataType::CNav2),
                L2C => (2, NavDataType::CNav),
                L5 => (3, NavDataType::L5CNav),
                L2P => (0, NavDataType::LNav),
                _ => return false,
            },
            GnssSystem::BdsSystem => match self.sat_signal {
                B1C => (4, NavDataType::BCNav1),
                B1I | B2I | B3I => {
                    if (6..=58).contains(&self.svid) { (5, NavDataType::D1D2Nav) } else { (6, NavDataType::D1D2Nav) }
                },
                B2A => (7, NavDataType::BCNav2),
                B2B => (8, NavDataType::BCNav3),
                _ => return false,
            },
            GnssSystem::GalileoSystem => match self.sat_signal {
                E1 => (9, NavDataType::INav),
                E5A => (10, NavDataType::FNav),
                E5B => (11, NavDataType::INav),
                E6 => (12, NavDataType::Unknown), // No specific NavBit for E6 in C++
                _ => return false,
            },
            GnssSystem::GlonassSystem => match self.sat_signal {
                G1 | G2 => (13, NavDataType::GNav),
                G3 => (14, NavDataType::GNav),
                _ => return false,
            },
            _ => return false,
        };

        self.attribute = &SIGNAL_ATTRIBUTES[attribute_index];

        if let Some(nav_data) = &self.nav_data {
            nav_data.get_type() == nav_type_check
        } else {
            true
        }
    }

    pub fn get_satellite_signal(&mut self, transmit_time: GnssTime, data_signal: &mut ComplexNumber, pilot_signal: &mut ComplexNumber) -> bool {
        // Define constants for pattern matching (same as in set_signal_attribute)
        const L1CA: i32 = crate::SIGNAL_INDEX_L1CA as i32;
        const L1C: i32 = crate::SIGNAL_INDEX_L1C as i32;
        const L2C: i32 = crate::SIGNAL_INDEX_L2C as i32;
        const L5: i32 = crate::SIGNAL_INDEX_L5 as i32;
        const L2P: i32 = crate::SIGNAL_INDEX_L2P as i32;
        const B1C: i32 = crate::SIGNAL_INDEX_B1C as i32;
        const B1I: i32 = crate::SIGNAL_INDEX_B1I as i32;
        const B2I: i32 = crate::SIGNAL_INDEX_B2I as i32;
        const B3I: i32 = crate::SIGNAL_INDEX_B3I as i32;
        const B2A: i32 = crate::SIGNAL_INDEX_B2A as i32;
        const B2B: i32 = crate::SIGNAL_INDEX_B2B as i32;
        const E1: i32 = crate::SIGNAL_INDEX_E1 as i32;
        const E5A: i32 = crate::SIGNAL_INDEX_E5A as i32;
        const E5B: i32 = crate::SIGNAL_INDEX_E5B as i32;
        const E6: i32 = crate::SIGNAL_INDEX_E6 as i32;
        const G1: i32 = crate::SIGNAL_INDEX_G1 as i32;
        const G2: i32 = crate::SIGNAL_INDEX_G2 as i32;
        const G3: i32 = crate::SIGNAL_INDEX_G3 as i32;
        if self.svid < 0 {
            return false;
        }

        let mut transmit_time_adj = transmit_time;

        match self.sat_system {
            GnssSystem::BdsSystem => { // subtract leap second difference
                transmit_time_adj.MilliSeconds -= 14000;
            },
            GnssSystem::GlonassSystem => { // subtract leap second, add 3 hours
                let seconds = transmit_time.Week as u32 * 604800 + transmit_time.MilliSeconds as u32 / 1000;
                let (leap_second, _) = GnssTimeConverter::get_leap_second(seconds);
                transmit_time_adj.MilliSeconds = (transmit_time.MilliSeconds + 10800000 - leap_second * 1000) % 86400000;
            },
            _ => {},
        }

        if transmit_time_adj.MilliSeconds < 0 { // protection on negative millisecond
            transmit_time_adj.MilliSeconds += 604800000;
        }

        let bit_length = self.attribute.code_length * self.attribute.nh_length;
        let frame_number;
        let mut bit_number = 0;
        let bit_pos;
        let mut data_bit;
        let mut pilot_bit = 0;

        if self.sat_system == GnssSystem::GlonassSystem && (self.sat_signal == crate::SIGNAL_INDEX_G1 as i32 || self.sat_signal == crate::SIGNAL_INDEX_G2 as i32) {
            let string_position = transmit_time_adj.MilliSeconds % 2000;
            
            if string_position < 300 {
                self.is_in_time_marker = true;
                if string_position == 0 {
                    //GNavBit::get_time_marker(&mut self.time_marker_bits);
                }
                let time_marker_bit_index = string_position / 10;
                data_bit = if time_marker_bit_index < 30 { if self.time_marker_bits[time_marker_bit_index as usize] != 0 { -1 } else { 1 } } else { 1 };
                *data_signal = ComplexNumber { real: data_bit as f64, imag: 0.0 };
                *pilot_signal = ComplexNumber::new();
                return true;
            } else {
                self.is_in_time_marker = false;
                let milliseconds = transmit_time_adj.MilliSeconds - 300;
                frame_number = transmit_time_adj.MilliSeconds / self.attribute.frame_length;
                bit_number = ((string_position - 300) * 100 / 1700) as usize;
                if bit_number >= 100 { bit_number = 99; }
                bit_pos = ((string_position - 300) % 17) / self.attribute.code_length;
            }
        } else {
            let galileo_e1_signal = if self.sat_system == GnssSystem::GalileoSystem && self.sat_signal == crate::SIGNAL_INDEX_E1 as i32 { 1 } else { 0 };
            let milliseconds = transmit_time_adj.MilliSeconds + (galileo_e1_signal * 1000);
            frame_number = milliseconds / self.attribute.frame_length;
            let milliseconds_in_frame = milliseconds % self.attribute.frame_length;
            bit_number = (milliseconds_in_frame / bit_length) as usize;
            let milliseconds_in_bit = milliseconds_in_frame % bit_length;
            bit_pos = milliseconds_in_bit / self.attribute.code_length;
        }

        if frame_number != self.current_frame {
            let max_svid = match self.sat_system {
                GnssSystem::BdsSystem => 63,
                GnssSystem::GalileoSystem => 36,
                GnssSystem::GlonassSystem => 24,
                _ => 32,
            };
            if let Some(nav_data) = &mut self.nav_data {
                if self.svid > 0 && self.svid <= max_svid {
                    let param = if (self.sat_system == GnssSystem::GpsSystem && self.sat_signal == crate::SIGNAL_INDEX_L5 as i32) || 
                                     (self.sat_system == GnssSystem::GalileoSystem && self.sat_signal == crate::SIGNAL_INDEX_E1 as i32) { 1 } else { 0 };
                    let mut data_bits_i32 = [0; 4096];
                    nav_data.get_frame_data(transmit_time, self.svid, param, &mut data_bits_i32);
                    for i in 0..4096 {
                        self.data_bits[i] = data_bits_i32[i] as u8;
                    }
                }
            }
            self.current_frame = frame_number;
        }

        if self.sat_system == GnssSystem::GlonassSystem && (self.sat_signal == crate::SIGNAL_INDEX_G1 as i32 || self.sat_signal == crate::SIGNAL_INDEX_G2 as i32) && !self.is_in_time_marker {
            data_bit = if self.data_bits[bit_number] != 0 { -1 } else { 1 };
        } else if !self.is_in_time_marker {
            data_bit = (if self.data_bits[bit_number] != 0 { -1 } else { 1 }) * (if (self.attribute.nh_code & (1 << bit_pos)) != 0 { -1 } else { 1 });
        } else {
            data_bit = 1; // Should be handled by the time marker logic above
        }

        if let Some((secondary_code, _)) = get_pilot_bits(self.sat_system, self.sat_signal as usize, self.svid) {
            let secondary_position = (transmit_time.MilliSeconds / self.attribute.code_length) % secondary_code.len() as i32;
            pilot_bit = if (secondary_code[(secondary_position / 32) as usize] & (1 << (secondary_position & 0x1f))) != 0 { -1 } else { 1 };
        }

        // generate DataSignal and PilotSignal
        match self.sat_system {
            GnssSystem::GpsSystem => match self.sat_signal {
                L1CA => {
                    *data_signal = ComplexNumber { real: data_bit as f64, imag: 0.0 };
                    *pilot_signal = ComplexNumber::new();
                },
                L1C => {
                    *data_signal = ComplexNumber { real: data_bit as f64 * AMPLITUDE_1_4, imag: 0.0 };
                    *pilot_signal = ComplexNumber { real: pilot_bit as f64 * AMPLITUDE_3_4, imag: 0.0 };
                },
                L2C => {
                    *data_signal = ComplexNumber { real: data_bit as f64, imag: 0.0 };
                    *pilot_signal = ComplexNumber { real: 1.0, imag: 0.0 };
                },
                L5 => {
                    *data_signal = ComplexNumber { real: data_bit as f64 * AMPLITUDE_1_2, imag: 0.0 };
                    *pilot_signal = ComplexNumber { real: 0.0, imag: pilot_bit as f64 * AMPLITUDE_1_2 };
                },
                L2P => {
                    *data_signal = ComplexNumber { real: data_bit as f64, imag: 0.0 };
                    *pilot_signal = ComplexNumber::new();
                },
                _ => {},
            },
            GnssSystem::BdsSystem => match self.sat_signal {
                B1C => {
                    *data_signal = ComplexNumber { real: 0.0, imag: -data_bit as f64 * AMPLITUDE_1_4 };
                    *pilot_signal = ComplexNumber { real: pilot_bit as f64 * AMPLITUDE_29_44, imag: 0.0 };
                },
                B1I | B2I | B3I => {
                    *data_signal = ComplexNumber { real: data_bit as f64, imag: 0.0 };
                    *pilot_signal = ComplexNumber::new();
                },
                B2A => {
                    *data_signal = ComplexNumber { real: 0.0, imag: -data_bit as f64 * AMPLITUDE_1_2 };
                    *pilot_signal = ComplexNumber { real: pilot_bit as f64 * AMPLITUDE_1_2, imag: 0.0 };
                },
                B2B => {
                    *data_signal = ComplexNumber { real: 0.0, imag: -data_bit as f64 * AMPLITUDE_1_2 };
                    *pilot_signal = ComplexNumber::new();
                },
                _ => {},
            },
            GnssSystem::GalileoSystem => match self.sat_signal {
                E1 => {
                    *data_signal = ComplexNumber { real: -data_bit as f64 * AMPLITUDE_1_2, imag: 0.0 };
                    *pilot_signal = ComplexNumber { real: pilot_bit as f64 * AMPLITUDE_1_2, imag: 0.0 };
                },
                E5A | E5B => {
                    *data_signal = ComplexNumber { real: 0.0, imag: -data_bit as f64 * AMPLITUDE_1_2 };
                    *pilot_signal = ComplexNumber { real: pilot_bit as f64 * AMPLITUDE_1_2, imag: 0.0 };
                },
                E6 => {
                    *data_signal = ComplexNumber::new();
                    *pilot_signal = ComplexNumber { real: pilot_bit as f64 * AMPLITUDE_1_2, imag: 0.0 };
                },
                _ => {},
            },
            GnssSystem::GlonassSystem => match self.sat_signal {
                G1 | G2 => {
                    if !self.is_in_time_marker {
                        *data_signal = ComplexNumber { real: data_bit as f64, imag: 0.0 };
                        *pilot_signal = ComplexNumber::new();
                    }
                },
                G3 => {
                    *data_signal = ComplexNumber { real: data_bit as f64 * AMPLITUDE_1_2, imag: 0.0 };
                    *pilot_signal = ComplexNumber { real: 0.0, imag: pilot_bit as f64 * AMPLITUDE_1_2 };
                },
                _ => {},
            },
            _ => {},
        }

        true
    }
}











