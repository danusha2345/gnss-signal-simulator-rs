//----------------------------------------------------------------------
// ifdatagen.rs:
//   Main executable file for IF signal generation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use std::fs::File;
use std::io::{Write, BufWriter};
use std::time::{Instant, Duration};
use std::env;
use crate::types::{NavDataType, GnssTime, UtcTime, LlaPosition, LocalSpeed, KinematicInfo, GpsEphemeris, GlonassEphemeris, IonoParam, UtcParam, GpsAlmanac, GlonassAlmanac};
use crate::complex_number::ComplexNumber;
use crate::constants::*;
use crate::{JsonStream, JsonObjectParser as JsonObject};
use crate::{utc_to_gps_time, utc_to_glonass_time_corrected, utc_to_bds_time, lla_to_ecef, speed_local_to_ecef, gps_time_to_utc};
use crate::{LNavBit, L5CNavBit, GNavBit};
use crate::powercontrol::CPowerControl;
use crate::trajectory::CTrajectory;
use crate::satellite_param::SatelliteParam;

// Placeholder types (to be defined later)
pub struct NavData;
pub struct OutputParam {
    pub gps_mask_out: u32,
    pub bds_mask_out: u64,
    pub galileo_mask_out: u64,
    pub glonass_mask_out: u32,
    pub elevation_mask: f64,
}

impl Default for OutputParam {
    fn default() -> Self {
        OutputParam {
            gps_mask_out: 0,
            bds_mask_out: 0,
            galileo_mask_out: 0,
            glonass_mask_out: 0,
            elevation_mask: 0.0,
        }
    }
}

pub struct DelayConfig;

#[derive(Debug, Clone, Copy)]
pub enum OutputFormat {
    IQ8,
    IQ4,
}

pub struct SignalPower {
    pub system: i32,
    pub svid: i32,
    pub time: i32,
    pub cn0: f64,
}

// Placeholder trait for navigation bit generation
pub trait NavBitTrait {
    fn get_frame_data(&self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32;
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris);
    fn set_almanac(&mut self, alm: &[GpsAlmanac]);
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>);
    fn get_type(&self) -> NavDataType;
    fn clone_box(&self) -> Box<dyn NavBitTrait>;
}

// Placeholder navigation bit types
#[derive(Clone)]
pub struct CNavBit;
impl CNavBit { pub fn new() -> Self { CNavBit } }
impl NavBitTrait for CNavBit {
    fn get_frame_data(&self, _start_time: GnssTime, _svid: i32, _param: i32, _nav_bits: &mut [i32]) -> i32 { 0 }
    fn set_ephemeris(&mut self, _svid: i32, _eph: &GpsEphemeris) {}
    fn set_almanac(&mut self, _alm: &[GpsAlmanac]) {}
    fn set_iono_utc(&mut self, _iono_param: Option<&IonoParam>, _utc_param: Option<&UtcParam>) {}
    fn get_type(&self) -> NavDataType { NavDataType::CNav }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

#[derive(Clone)]
pub struct CNav2Bit;
impl CNav2Bit { pub fn new() -> Self { CNav2Bit } }
impl NavBitTrait for CNav2Bit {
    fn get_frame_data(&self, _start_time: GnssTime, _svid: i32, _param: i32, _nav_bits: &mut [i32]) -> i32 { 0 }
    fn set_ephemeris(&mut self, _svid: i32, _eph: &GpsEphemeris) {}
    fn set_almanac(&mut self, _alm: &[GpsAlmanac]) {}
    fn set_iono_utc(&mut self, _iono_param: Option<&IonoParam>, _utc_param: Option<&UtcParam>) {}
    fn get_type(&self) -> NavDataType { NavDataType::CNav2 }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

use crate::sat_if_signal::SatIfSignal;

#[derive(Clone)]
pub struct D1D2NavBit;
impl D1D2NavBit { pub fn new() -> Self { D1D2NavBit } }
impl NavBitTrait for D1D2NavBit {
    fn get_frame_data(&self, _start_time: GnssTime, _svid: i32, _param: i32, _nav_bits: &mut [i32]) -> i32 { 0 }
    fn set_ephemeris(&mut self, _svid: i32, _eph: &GpsEphemeris) {}
    fn set_almanac(&mut self, _alm: &[GpsAlmanac]) {}
    fn set_iono_utc(&mut self, _iono_param: Option<&IonoParam>, _utc_param: Option<&UtcParam>) {}
    fn get_type(&self) -> NavDataType { NavDataType::D1D2Nav }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

#[derive(Clone)]
pub struct INavBit;
impl INavBit { pub fn new() -> Self { INavBit } }
impl NavBitTrait for INavBit {
    fn get_frame_data(&self, _start_time: GnssTime, _svid: i32, _param: i32, _nav_bits: &mut [i32]) -> i32 { 0 }
    fn set_ephemeris(&mut self, _svid: i32, _eph: &GpsEphemeris) {}
    fn set_almanac(&mut self, _alm: &[GpsAlmanac]) {}
    fn set_iono_utc(&mut self, _iono_param: Option<&IonoParam>, _utc_param: Option<&UtcParam>) {}
    fn get_type(&self) -> NavDataType { NavDataType::INav }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

#[derive(Clone)]
pub struct FNavBit;
impl FNavBit { pub fn new() -> Self { FNavBit } }
impl NavBitTrait for FNavBit {
    fn get_frame_data(&self, _start_time: GnssTime, _svid: i32, _param: i32, _nav_bits: &mut [i32]) -> i32 { 0 }
    fn set_ephemeris(&mut self, _svid: i32, _eph: &GpsEphemeris) {}
    fn set_almanac(&mut self, _alm: &[GpsAlmanac]) {}
    fn set_iono_utc(&mut self, _iono_param: Option<&IonoParam>, _utc_param: Option<&UtcParam>) {}
    fn get_type(&self) -> NavDataType { NavDataType::FNav }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

#[derive(Clone)]
pub struct BCNav1Bit;
impl BCNav1Bit { pub fn new() -> Self { BCNav1Bit } }
impl NavBitTrait for BCNav1Bit {
    fn get_frame_data(&self, _start_time: GnssTime, _svid: i32, _param: i32, _nav_bits: &mut [i32]) -> i32 { 0 }
    fn set_ephemeris(&mut self, _svid: i32, _eph: &GpsEphemeris) {}
    fn set_almanac(&mut self, _alm: &[GpsAlmanac]) {}
    fn set_iono_utc(&mut self, _iono_param: Option<&IonoParam>, _utc_param: Option<&UtcParam>) {}
    fn get_type(&self) -> NavDataType { NavDataType::BCNav1 }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

#[derive(Clone)]
pub struct BCNav2Bit;
impl BCNav2Bit { pub fn new() -> Self { BCNav2Bit } }
impl NavBitTrait for BCNav2Bit {
    fn get_frame_data(&self, _start_time: GnssTime, _svid: i32, _param: i32, _nav_bits: &mut [i32]) -> i32 { 0 }
    fn set_ephemeris(&mut self, _svid: i32, _eph: &GpsEphemeris) {}
    fn set_almanac(&mut self, _alm: &[GpsAlmanac]) {}
    fn set_iono_utc(&mut self, _iono_param: Option<&IonoParam>, _utc_param: Option<&UtcParam>) {}
    fn get_type(&self) -> NavDataType { NavDataType::BCNav2 }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

#[derive(Clone)]
pub struct BCNav3Bit;
impl BCNav3Bit { pub fn new() -> Self { BCNav3Bit } }
impl NavBitTrait for BCNav3Bit {
    fn get_frame_data(&self, _start_time: GnssTime, _svid: i32, _param: i32, _nav_bits: &mut [i32]) -> i32 { 0 }
    fn set_ephemeris(&mut self, _svid: i32, _eph: &GpsEphemeris) {}
    fn set_almanac(&mut self, _alm: &[GpsAlmanac]) {}
    fn set_iono_utc(&mut self, _iono_param: Option<&IonoParam>, _utc_param: Option<&UtcParam>) {}
    fn get_type(&self) -> NavDataType { NavDataType::BCNav3 }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

// Implement NavBitTrait for existing types
impl NavBitTrait for LNavBit {
    fn get_frame_data(&self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 {
        let mut nav_bits_300 = [0i32; 300];
        let result = self.get_frame_data(start_time, svid, param, &mut nav_bits_300);
        nav_bits[..300].copy_from_slice(&nav_bits_300);
        result
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { self.set_ephemeris(svid, eph); }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { self.set_almanac(alm); }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) { self.set_iono_utc(iono_param.unwrap(), utc_param.unwrap()); }
    fn get_type(&self) -> NavDataType { NavDataType::LNav }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

impl NavBitTrait for L5CNavBit {
    fn get_frame_data(&self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 {
        let mut nav_bits_600 = [0i32; 600];
        let result = self.get_frame_data(start_time, svid, param, &mut nav_bits_600);
        nav_bits[..600].copy_from_slice(&nav_bits_600);
        result
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { self.set_ephemeris(svid, eph); }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { self.set_almanac(alm); }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) { self.set_iono_utc(iono_param.unwrap(), utc_param.unwrap()); }
    fn get_type(&self) -> NavDataType { NavDataType::L5CNav }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}

impl NavBitTrait for GNavBit {
    fn get_frame_data(&self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 {
        let mut nav_bits_100 = [0i32; 100];
        let result = self.GetFrameData(start_time, svid, param, &mut nav_bits_100);
        nav_bits[..100].copy_from_slice(&nav_bits_100);
        result
    }
    fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) { self.SetEphemeris(svid, eph); }
    fn set_almanac(&mut self, alm: &[GpsAlmanac]) { self.SetAlmanac(alm); }
    fn set_iono_utc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) { self.SetIonoUtc(iono_param, utc_param); }
    fn get_type(&self) -> NavDataType { NavDataType::GNav }
    fn clone_box(&self) -> Box<dyn NavBitTrait> { Box::new(self.clone()) }
}
use crate::fastmath::FastMath;

// Constants for quantization and time conversion
const QUANT_SCALE_IQ4: f64 = 3.0;
const QUANT_SCALE_IQ8: f64 = 25.0;
const WEEK_MS: i32 = 604800000;

const TOTAL_GPS_SAT: usize = 32;
const TOTAL_BDS_SAT: usize = 63;
const TOTAL_GAL_SAT: usize = 36;
const TOTAL_GLO_SAT: usize = 24;
const TOTAL_SAT_CHANNEL: usize = 128;

#[derive(Debug, Clone, Copy)]
pub enum DataBitType {
    DataBitLNav = 0,    // for GPS
    DataBitCNav = 1,
    DataBitCNav2 = 2,
    DataBitL5CNav = 3,
    DataBitGNav = 4,    // for GLONASS
    DataBitGNav2 = 5,
    DataBitD1D2 = 6,    // for BDS
    DataBitBCNav1 = 7,
    DataBitBCNav2 = 8,
    DataBitBCNav3 = 9,
    DataBitINav = 10,   // for Galileo
    DataBitFNav = 11,
    DataBitECNav = 12,
    DataBitSbas = 13,   // for SBAS
}

pub struct IFDataGen {
    pub trajectory: CTrajectory,
    pub power_control: CPowerControl,
    pub nav_data: NavData,
    pub output_param: OutputParam,
    pub cur_time: GnssTime,
    
    // Satellite ephemeris arrays
    pub gps_eph: [Option<GpsEphemeris>; TOTAL_GPS_SAT],
    pub gps_eph_visible: [Option<GpsEphemeris>; TOTAL_GPS_SAT],
    pub bds_eph: [Option<GpsEphemeris>; TOTAL_BDS_SAT],
    pub bds_eph_visible: [Option<GpsEphemeris>; TOTAL_BDS_SAT],
    pub gal_eph: [Option<GpsEphemeris>; TOTAL_GAL_SAT],
    pub gal_eph_visible: [Option<GpsEphemeris>; TOTAL_GAL_SAT],
    pub glo_eph: [Option<GlonassEphemeris>; TOTAL_GLO_SAT],
    pub glo_eph_visible: [Option<GlonassEphemeris>; TOTAL_GLO_SAT],
    
    // Satellite parameter arrays
    pub gps_sat_param: [SatelliteParam; TOTAL_GPS_SAT],
    pub bds_sat_param: [SatelliteParam; TOTAL_BDS_SAT],
    pub gal_sat_param: [SatelliteParam; TOTAL_GAL_SAT],
    pub glo_sat_param: [SatelliteParam; TOTAL_GLO_SAT],
    
    // Satellite numbers
    pub gps_sat_number: usize,
    pub bds_sat_number: usize,
    pub gal_sat_number: usize,
    pub glo_sat_number: usize,
}

// Signal center frequencies in Hz
const SIGNAL_CENTER_FREQ: [[f64; 8]; 4] = [
    [FREQ_GPS_L1, FREQ_GPS_L1, FREQ_GPS_L2, FREQ_GPS_L2, FREQ_GPS_L5, 0.0, 0.0, 0.0],
    [FREQ_BDS_B1C, FREQ_BDS_B1I, FREQ_BDS_B2I, FREQ_BDS_B3I, FREQ_BDS_B2A, FREQ_BDS_B2B, FREQ_BDS_B2AB, 0.0],
    [FREQ_GAL_E1, FREQ_GAL_E5A, FREQ_GAL_E5B, FREQ_GAL_E5, FREQ_GAL_E6, 0.0, 0.0, 0.0],
    [FREQ_GLO_G1, FREQ_GLO_G2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
];

const SIGNAL_NAME: [[&str; 8]; 4] = [
    ["L1CA", "L1C", "L2C", "L2P", "L5", "", "", ""],
    ["B1C", "B1I", "B2I", "B3I", "B2a", "B2b", "B2ab", ""],
    ["E1", "E5a", "E5b", "E5", "E6", "", "", ""],
    ["G1", "G2", "", "", "", "", "", ""],
];

impl IFDataGen {
    pub fn new() -> Self {
        IFDataGen {
            trajectory: CTrajectory::new(),
            power_control: CPowerControl::new(),
            nav_data: NavData,
            output_param: OutputParam::default(),
            cur_time: GnssTime::default(),
            
            gps_eph: [None; TOTAL_GPS_SAT],
            gps_eph_visible: [None; TOTAL_GPS_SAT],
            bds_eph: [None; TOTAL_BDS_SAT],
            bds_eph_visible: [None; TOTAL_BDS_SAT],
            gal_eph: [None; TOTAL_GAL_SAT],
            gal_eph_visible: [None; TOTAL_GAL_SAT],
            glo_eph: [None; TOTAL_GLO_SAT],
            glo_eph_visible: [None; TOTAL_GLO_SAT],
            
            gps_sat_param: [SatelliteParam::default(); TOTAL_GPS_SAT],
            bds_sat_param: [SatelliteParam::default(); TOTAL_BDS_SAT],
            gal_sat_param: [SatelliteParam::default(); TOTAL_GAL_SAT],
            glo_sat_param: [SatelliteParam::default(); TOTAL_GLO_SAT],
            
            gps_sat_number: 0,
            bds_sat_number: 0,
            gal_sat_number: 0,
            glo_sat_number: 0,
        }
    }

    pub fn main(&mut self, args: Vec<String>) -> Result<(), Box<dyn std::error::Error>> {
        println!("\n================================================================================");
        println!("                          IF SIGNAL GENERATION ");
        println!("================================================================================");

        // Read JSON file and assign parameters
        let json_file = if args.len() > 1 {
            if args[1] == "--help" {
                println!("Usage: {} [optional JSON file path]", args[0]);
                return Ok(());
            }
            &args[1]
        } else {
            "IfGenTest.json" // Default JSON file
        };

        // Read the JSON file
        println!("[INFO]\tLoading JSON file: {}", json_file);
        let mut json_stream = JsonStream::new();  // Создаем экземпляр
        let result = json_stream.read_file(json_file);  // Вызываем метод экземпляра

        if result == 0 {
            println!("[INFO]\tJSON file read successfully: {}", json_file);
            // Используем json_stream.root_object или другие поля
        } else {
            eprintln!("[ERROR]\tUnable to read JSON file: {}", json_file);
            return Err("Failed to read JSON file".into());
        }

        let object = json_stream.get_root_object();
        let mut utc_time = UtcTime::new();
        let mut start_pos = LlaPosition::new();
        let mut start_vel = LocalSpeed::new();

        self.assign_parameters(&object, &mut utc_time, &mut start_pos, &mut start_vel)?;

        // Initialize variables
        self.trajectory.reset_trajectory_time();
        self.cur_time = utc_to_gps_time(utc_time);
        let glonass_time = utc_to_glonass_time_corrected(utc_time);
        let bds_time = utc_to_bds_time(utc_time);
        let mut cur_pos = lla_to_ecef(start_pos);
        speed_local_to_ecef(start_pos, start_vel, &mut cur_pos);

        println!("[INFO]\tOpening output file: {}", self.output_param.filename);
        let mut if_file = match File::create(&self.output_param.filename) {
            Ok(file) => {
                println!("[INFO]\tOutput file opened successfully.");
                BufWriter::new(file)
            },
            Err(_) => {
                println!("[ERROR]\tFailed to open output file: {}", self.output_param.filename);
                return Err("Failed to open output file".into());
            }
        };

        // Initialize satellite CN0 values
        for i in 0..TOTAL_GPS_SAT {
            self.gps_sat_param[i].cn0 = (self.power_control.init_cn0 * 100.0 + 0.5) as i32;
        }
        for i in 0..TOTAL_BDS_SAT {
            self.bds_sat_param[i].cn0 = (self.power_control.init_cn0 * 100.0 + 0.5) as i32;
        }
        for i in 0..TOTAL_GAL_SAT {
            self.gal_sat_param[i].cn0 = (self.power_control.init_cn0 * 100.0 + 0.5) as i32;
        }
        for i in 0..TOTAL_GLO_SAT {
            self.glo_sat_param[i].cn0 = (self.power_control.init_cn0 * 100.0 + 0.5) as i32;
        }

        // Create navigation bit instances
        let mut nav_bit_array = self.create_nav_bit_instances();

        self.setup_frequency_filtering();
        self.setup_navigation_data(&mut nav_bit_array, utc_time, glonass_time, bds_time)?;
        self.calculate_visible_satellites(cur_pos, glonass_time)?;

        let mut sat_if_signals = self.create_satellite_signals(&nav_bit_array)?;
        
        self.generate_if_signal(&mut if_file, &mut sat_if_signals, cur_pos)?;

        println!("[INFO]\tIF Signal generation completed!");
        Ok(())
    } 
   fn create_nav_bit_instances(&self) -> Vec<Option<Box<dyn NavBitTrait>>> {
        let mut nav_bit_array: Vec<Option<Box<dyn NavBitTrait>>> = Vec::with_capacity(14);
        
        for i in 0..14 {
            let nav_bit: Option<Box<dyn NavBitTrait>> = match i {
                0 => Some(Box::new(LNavBit::new())),      // DataBitLNav
                1 => Some(Box::new(CNavBit::new())),      // DataBitCNav
                2 => Some(Box::new(CNav2Bit::new())),     // DataBitCNav2
                3 => Some(Box::new(L5CNavBit::new())),    // DataBitL5CNav
                4 => Some(Box::new(GNavBit::new())),      // DataBitGNav
                5 => None,                                // DataBitGNav2
                6 => Some(Box::new(D1D2NavBit::new())),   // DataBitD1D2
                7 => Some(Box::new(BCNav1Bit::new())),    // DataBitBCNav1
                8 => Some(Box::new(BCNav2Bit::new())),    // DataBitBCNav2
                9 => Some(Box::new(BCNav3Bit::new())),    // DataBitBCNav3
                10 => Some(Box::new(INavBit::new())),     // DataBitINav
                11 => Some(Box::new(FNavBit::new())),     // DataBitFNav
                12 => None,                               // DataBitECNav
                13 => None,                               // DataBitSbas
                _ => None,
            };
            nav_bit_array.push(nav_bit);
        }
        
        nav_bit_array
    }

    fn setup_frequency_filtering(&mut self) {
        // Determine whether signal within IF band (expanded bandwidth for multi-system support)
        let bandwidth_expansion_factor = 1.0; // Use normal bandwidth
        let freq_low = ((self.output_param.center_freq - self.output_param.sample_freq * bandwidth_expansion_factor / 2.0) * 1000.0) as i32;
        let freq_high = ((self.output_param.center_freq + self.output_param.sample_freq * bandwidth_expansion_factor / 2.0) * 1000.0) as i32;

        // GPS frequency filtering
        if self.output_param.freq_select[GnssSystem::GpsSystem as usize] != 0 {
            if (self.output_param.freq_select[GnssSystem::GpsSystem as usize] & (1 << SIGNAL_INDEX_L1CA)) != 0 
                && (FREQ_GPS_L1 < freq_low || FREQ_GPS_L1 > freq_high) {
                self.output_param.freq_select[GnssSystem::GpsSystem as usize] &= !(1 << SIGNAL_INDEX_L1CA);
            }
            if (self.output_param.freq_select[GnssSystem::GpsSystem as usize] & (1 << SIGNAL_INDEX_L1C)) != 0 
                && (FREQ_GPS_L1 < freq_low || FREQ_GPS_L1 > freq_high) {
                self.output_param.freq_select[GnssSystem::GpsSystem as usize] &= !(1 << SIGNAL_INDEX_L1C);
            }
            if (self.output_param.freq_select[GnssSystem::GpsSystem as usize] & (1 << SIGNAL_INDEX_L2C)) != 0 
                && (FREQ_GPS_L2 < freq_low || FREQ_GPS_L2 > freq_high) {
                self.output_param.freq_select[GnssSystem::GpsSystem as usize] &= !(1 << SIGNAL_INDEX_L2C);
            }
            if (self.output_param.freq_select[GnssSystem::GpsSystem as usize] & (1 << SIGNAL_INDEX_L2P)) != 0 
                && (FREQ_GPS_L2 < freq_low || FREQ_GPS_L2 > freq_high) {
                self.output_param.freq_select[GnssSystem::GpsSystem as usize] &= !(1 << SIGNAL_INDEX_L2P);
            }
            if (self.output_param.freq_select[GnssSystem::GpsSystem as usize] & (1 << SIGNAL_INDEX_L5)) != 0 
                && (FREQ_GPS_L5 < freq_low || FREQ_GPS_L5 > freq_high) {
                self.output_param.freq_select[GnssSystem::GpsSystem as usize] &= !(1 << SIGNAL_INDEX_L5);
            }
        }

        // Similar filtering for BDS, Galileo, and GLONASS...
        // (Implementation continues with similar pattern for other systems)
    }

    fn setup_navigation_data(&mut self, nav_bit_array: &mut Vec<Option<Box<dyn NavBitTrait>>>, 
                           utc_time: UtcTime, glonass_time: GlonassTime, bds_time: GnssTime) -> Result<(), Box<dyn std::error::Error>> {
        
        // Set Ionosphere and UTC parameters for different navigation data bits
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitLNav as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_gps_iono(), self.nav_data.get_gps_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_gps_iono(), self.nav_data.get_gps_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav2 as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_gps_iono(), self.nav_data.get_gps_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitD1D2 as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_bds_iono(), self.nav_data.get_bds_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav1 as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_bds_iono(), self.nav_data.get_bds_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitINav as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_galileo_iono(), self.nav_data.get_galileo_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitFNav as usize] {
            nav_bit.set_iono_utc(self.nav_data.get_galileo_iono(), self.nav_data.get_galileo_utc_param());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitGNav as usize] {
            nav_bit.set_iono_utc(None, self.nav_data.get_gps_utc_param()); // GLONASS uses GPS UTC parameters
        }

        // Find ephemeris matching current time and fill in data to generate bit stream
        for i in 1..=TOTAL_GPS_SAT {
            self.gps_eph[i-1] = self.nav_data.find_ephemeris(GnssSystem::GpsSystem, self.cur_time, i as i32, 0, 0);
            
            // For L1CA/L1C/L2C/L5, all can use the same ephemeris data
            if let Some(ref eph) = self.gps_eph[i-1] {
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitLNav as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav2 as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitL5CNav as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
            }
        }

        for i in 1..=TOTAL_BDS_SAT {
            self.bds_eph[i-1] = self.nav_data.find_ephemeris(GnssSystem::BdsSystem, bds_time, i as i32, 0, 0);
            
            if let Some(ref eph) = self.bds_eph[i-1] {
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitD1D2 as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav1 as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav2 as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav3 as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
            }
        }

        for i in 1..=TOTAL_GAL_SAT {
            self.gal_eph[i-1] = self.nav_data.find_ephemeris(GnssSystem::GalileoSystem, self.cur_time, i as i32, 0, 0);
            
            if let Some(ref eph) = self.gal_eph[i-1] {
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitINav as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
                if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitFNav as usize] {
                    nav_bit.set_ephemeris(i as i32, eph);
                }
            }
        }

        let mut glo_eph_count = 0;
        for i in 1..=TOTAL_GLO_SAT {
            self.glo_eph[i-1] = self.nav_data.find_glo_ephemeris(glonass_time, i as i32);
            
            if let Some(ref eph) = self.glo_eph[i-1] {
                if eph.flag {
                    if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitGNav as usize] {
                        // Convert GLONASS ephemeris to GPS format for compatibility
                        let gps_eph = self.convert_glonass_to_gps_ephemeris(eph);
                        nav_bit.set_ephemeris(i as i32, &gps_eph);
                    }
                    glo_eph_count += 1;
                }
            }
        }
        println!("[INFO] GLONASS ephemeris loaded: {} satellites", glo_eph_count);

        // Complete almanac data
        self.nav_data.complete_almanac(GnssSystem::GpsSystem, utc_time);
        self.nav_data.complete_almanac(GnssSystem::BdsSystem, utc_time);
        self.nav_data.complete_almanac(GnssSystem::GalileoSystem, utc_time);
        self.nav_data.complete_almanac(GnssSystem::GlonassSystem, utc_time);

        // Set almanac data for navigation bits
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitLNav as usize] {
            nav_bit.set_almanac(self.nav_data.get_gps_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav as usize] {
            nav_bit.set_almanac(self.nav_data.get_gps_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitCNav2 as usize] {
            nav_bit.set_almanac(self.nav_data.get_gps_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitL5CNav as usize] {
            nav_bit.set_almanac(self.nav_data.get_gps_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitD1D2 as usize] {
            nav_bit.set_almanac(self.nav_data.get_bds_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav1 as usize] {
            nav_bit.set_almanac(self.nav_data.get_bds_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav2 as usize] {
            nav_bit.set_almanac(self.nav_data.get_bds_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitBCNav3 as usize] {
            nav_bit.set_almanac(self.nav_data.get_bds_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitINav as usize] {
            nav_bit.set_almanac(self.nav_data.get_galileo_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitFNav as usize] {
            nav_bit.set_almanac(self.nav_data.get_galileo_almanac());
        }
        if let Some(ref mut nav_bit) = nav_bit_array[DataBitType::DataBitGNav as usize] {
            // Convert GLONASS almanac to GPS format for compatibility
            let gps_almanac = self.convert_glonass_to_gps_almanac(self.nav_data.get_glonass_almanac());
            nav_bit.set_almanac(&gps_almanac);
        }

        Ok(())
    }    fn 
calculate_visible_satellites(&mut self, cur_pos: KinematicInfo, glonass_time: GlonassTime) -> Result<(), Box<dyn std::error::Error>> {
        // Calculate visible satellites at start time
        self.gps_sat_number = if self.output_param.freq_select[GnssSystem::GpsSystem as usize] != 0 {
            self.get_visible_satellite(cur_pos, self.cur_time, GnssSystem::GpsSystem, &self.gps_eph, &mut self.gps_eph_visible)
        } else {
            0
        };

        self.bds_sat_number = if self.output_param.freq_select[GnssSystem::BdsSystem as usize] != 0 {
            self.get_visible_satellite(cur_pos, self.cur_time, GnssSystem::BdsSystem, &self.bds_eph, &mut self.bds_eph_visible)
        } else {
            0
        };

        self.gal_sat_number = if self.output_param.freq_select[GnssSystem::GalileoSystem as usize] != 0 {
            self.get_visible_satellite(cur_pos, self.cur_time, GnssSystem::GalileoSystem, &self.gal_eph, &mut self.gal_eph_visible)
        } else {
            0
        };

        self.glo_sat_number = if self.output_param.freq_select[GnssSystem::GlonassSystem as usize] != 0 {
            self.get_glonass_visible_satellite(cur_pos, glonass_time, &self.glo_eph, &mut self.glo_eph_visible)
        } else {
            0
        };

        // Check which satellites are visible and have ephemeris
        if self.glo_sat_number > 0 {
            print!("[GLONASS] Visible satellites: ");
            for j in 0..self.glo_sat_number {
                if let Some(ref eph) = self.glo_eph_visible[j] {
                    print!("SV{:02}(k={:+2}) ", eph.n, eph.freq);
                }
            }
            println!();
        }

        let list_count = self.power_control.get_power_control_list(0);
        let power_list = self.power_control.get_power_list();
        self.update_sat_param_list(self.cur_time, cur_pos, list_count, &power_list, self.nav_data.get_gps_iono());

        Ok(())
    }

    fn create_satellite_signals(&mut self, nav_bit_array: &Vec<Option<Box<dyn NavBitTrait>>>) -> Result<Vec<Option<Box<SatIfSignal>>>, Box<dyn std::error::Error>> {
        let mut sat_if_signals: Vec<Option<Box<SatIfSignal>>> = vec![None; TOTAL_SAT_CHANNEL];
        let mut total_channel_number = 0;

        #[cfg(feature = "openmp")]
        println!("[INFO]\tOpenMP configured for PARALLEL execution");
        #[cfg(not(feature = "openmp"))]
        println!("[INFO]\tOpenMP not available - using sequential processing");

        println!("[INFO]\tGenerating IF data with following satellite signals:\n");

        // Enhanced signal display with cleaner formatting
        println!("[INFO]\tEnabled Signals:");

        // Display enabled signals for each system
        self.display_enabled_signals();

        // Count total signals per system
        let (gps_signal_count, bds_signal_count, gal_signal_count, glo_signal_count) = self.count_signals_per_system();

        self.display_signals_summary_table(gps_signal_count, bds_signal_count, gal_signal_count, glo_signal_count);

        // Create satellite signal instances for each system
        total_channel_number = self.create_gps_signals(&mut sat_if_signals, total_channel_number, nav_bit_array)?;
        total_channel_number = self.create_bds_signals(&mut sat_if_signals, total_channel_number, nav_bit_array)?;
        total_channel_number = self.create_galileo_signals(&mut sat_if_signals, total_channel_number, nav_bit_array)?;
        total_channel_number = self.create_glonass_signals(&mut sat_if_signals, total_channel_number, nav_bit_array)?;

        Ok(sat_if_signals)
    }

    fn generate_if_signal(&mut self, if_file: &mut BufWriter<File>, sat_if_signals: &mut Vec<Option<Box<SatIfSignal>>>, mut cur_pos: KinematicInfo) -> Result<(), Box<dyn std::error::Error>> {
        let mut noise_array = vec![ComplexNumber::new(0.0, 0.0); self.output_param.sample_freq as usize];
        let mut quant_array = vec![0u8; (self.output_param.sample_freq * 2) as usize];

        // Calculate total data size and setup progress tracking
        let total_duration_ms = (self.trajectory.get_time_length() * 1000.0) as i32;
        let bytes_per_ms = self.output_param.sample_freq as f64 * if self.output_param.format == OutputFormat::OutputFormatIQ4 { 1.0 } else { 2.0 };
        let total_mb = (total_duration_ms as f64 * bytes_per_ms) / (1024.0 * 1024.0);
        let mut length = 0;
        let mut total_clipped_samples = 0i64;
        let mut total_samples = 0i64;
        let mut agc_gain = 1.0; // Automatic gain control

        println!("[INFO]\tStarting signal generation loop...");
        println!("[INFO]\tSignal Duration: {:.2} s", total_duration_ms as f64 / 1000.0);
        println!("[INFO]\tSignal Size: {:.2} MB", total_mb);
        println!("[INFO]\tSignal Data format: {}", if self.output_param.format == OutputFormat::OutputFormatIQ4 { "IQ4" } else { "IQ8" });
        println!("[INFO]\tSignal Center freq: {:.4} MHz", self.output_param.center_freq / 1000.0);
        println!("[INFO]\tSignal Sample rate: {:.2} MHz\n", self.output_param.sample_freq / 1000.0);

        let start_time = Instant::now();

        while !self.step_to_next_ms(&mut cur_pos)? {
            // Generate white noise using fast batch generation
            FastMath::generate_noise_block(&mut noise_array, 1.0);

            // Safe optimized parallelization
            for i in 0..sat_if_signals.len() {
                if let Some(ref mut signal) = sat_if_signals[i] {
                    signal.get_if_sample(self.cur_time);
                }
            }

            // Safe optimized accumulation with AGC
            for j in 0..self.output_param.sample_freq as usize {
                let mut sum = noise_array[j];
                for ch in 0..sat_if_signals.len() {
                    if let Some(ref signal) = sat_if_signals[ch] {
                        if let Some(ref sample_array) = signal.get_sample_array() {
                            if j < sample_array.len() {
                                sum = sum + sample_array[j];
                            }
                        }
                    }
                }
                // Apply AGC
                sum.real *= agc_gain;
                sum.imag *= agc_gain;
                noise_array[j] = sum;
            }

            let mut clipped_in_block = 0;
            if self.output_param.format == OutputFormat::OutputFormatIQ4 {
                Self::quant_samples_iq4(&noise_array, &mut quant_array[..self.output_param.sample_freq as usize], &mut clipped_in_block);
                if_file.write_all(&quant_array[..self.output_param.sample_freq as usize])?;
            } else {
                Self::quant_samples_iq8(&noise_array, &mut quant_array, &mut clipped_in_block);
                if_file.write_all(&quant_array)?;
            }

            // Update statistics
            total_clipped_samples += clipped_in_block as i64;
            total_samples += (self.output_param.sample_freq * 2) as i64; // I and Q components

            // Adaptive AGC adjustment every 100 ms
            if (length % 100) == 0 && length > 0 {
                let clipping_rate = total_clipped_samples as f64 / total_samples as f64;
                if clipping_rate > 0.01 {
                    // If more than 1% clipping
                    agc_gain *= 0.95; // Reduce gain by 5%
                    println!("[WARNING]\tAGC: Clipping {:.2}%, reducing gain to {:.3}", clipping_rate * 100.0, agc_gain);
                    total_clipped_samples = 0; // reset statistics
                    total_samples = 0;
                } else if clipping_rate < 0.001 && agc_gain < 1.0 {
                    // If less than 0.1% clipping
                    agc_gain *= 1.02; // Increase gain by 2%
                    if agc_gain > 1.0 {
                        agc_gain = 1.0;
                    }
                    println!("[WARNING]\tAGC: Clipping {:.2}%, increasing gain to {:.3}", clipping_rate * 100.0, agc_gain);
                    total_clipped_samples = 0; // reset statistics
                    total_samples = 0;
                }
            }

            length += 1;
            if (length % 10) == 0 {
                self.display_progress(length, total_duration_ms, total_mb, bytes_per_ms, start_time);
            }
        }

        self.display_final_progress(total_duration_ms, total_mb);
        self.display_completion_stats(total_samples, total_clipped_samples, agc_gain, start_time, total_mb);

        Ok(())
    }

    // Helper methods
    fn step_to_next_ms(&mut self, cur_pos: &mut KinematicInfo) -> Result<bool, Box<dyn std::error::Error>> {
        if !self.trajectory.get_next_pos_vel_ecef(0.001, cur_pos) {
            return Ok(true); // End of trajectory
        }

        let list_count = self.power_control.get_power_control_list(1);
        let power_list = self.power_control.get_power_list();
        
        self.cur_time.MilliSeconds += 1;
        if self.cur_time.MilliSeconds >= WEEK_MS {
            self.cur_time.Week += 1;
            self.cur_time.MilliSeconds -= WEEK_MS;
        }

        // Recalculate visible satellites at minute boundary for long simulations
        if (self.cur_time.MilliSeconds % 60000) == 0 {
            let utc_time = gps_time_to_utc(self.cur_time);
            let glonass_time = utc_to_glonass_time_corrected(utc_time);
            
            self.gps_sat_number = if self.output_param.freq_select[GnssSystem::GpsSystem as usize] != 0 {
                self.get_visible_satellite(*cur_pos, self.cur_time, GnssSystem::GpsSystem, &self.gps_eph, &mut self.gps_eph_visible)
            } else {
                0
            };

            self.bds_sat_number = if self.output_param.freq_select[GnssSystem::BdsSystem as usize] != 0 {
                self.get_visible_satellite(*cur_pos, self.cur_time, GnssSystem::BdsSystem, &self.bds_eph, &mut self.bds_eph_visible)
            } else {
                0
            };

            self.gal_sat_number = if self.output_param.freq_select[GnssSystem::GalileoSystem as usize] != 0 {
                self.get_visible_satellite(*cur_pos, self.cur_time, GnssSystem::GalileoSystem, &self.gal_eph, &mut self.gal_eph_visible)
            } else {
                0
            };

            self.glo_sat_number = if self.output_param.freq_select[GnssSystem::GlonassSystem as usize] != 0 {
                self.get_glonass_visible_satellite(*cur_pos, glonass_time, &self.glo_eph, &mut self.glo_eph_visible)
            } else {
                0
            };
        }

        self.update_sat_param_list(self.cur_time, *cur_pos, list_count, &power_list, self.nav_data.get_gps_iono());
        Ok(false)
    }

    fn get_nav_data(&self, sat_system: GnssSystem, sat_signal_index: i32, nav_bit_array: &Vec<Option<Box<dyn NavBitTrait>>>) -> Option<&Box<dyn NavBitTrait>> {
        match sat_system {
            GnssSystem::GpsSystem => {
                match sat_signal_index {
                    SIGNAL_INDEX_L1CA => nav_bit_array[DataBitType::DataBitLNav as usize].as_ref(),
                    SIGNAL_INDEX_L1C => nav_bit_array[DataBitType::DataBitCNav2 as usize].as_ref(),
                    SIGNAL_INDEX_L2C => nav_bit_array[DataBitType::DataBitCNav as usize].as_ref(),
                    SIGNAL_INDEX_L2P => nav_bit_array[DataBitType::DataBitLNav as usize].as_ref(),
                    SIGNAL_INDEX_L5 => nav_bit_array[DataBitType::DataBitL5CNav as usize].as_ref(),
                    _ => nav_bit_array[DataBitType::DataBitLNav as usize].as_ref(),
                }
            },
            GnssSystem::BdsSystem => {
                match sat_signal_index {
                    SIGNAL_INDEX_B1C => nav_bit_array[DataBitType::DataBitBCNav1 as usize].as_ref(),
                    SIGNAL_INDEX_B1I => nav_bit_array[DataBitType::DataBitD1D2 as usize].as_ref(),
                    SIGNAL_INDEX_B2I => nav_bit_array[DataBitType::DataBitD1D2 as usize].as_ref(),
                    SIGNAL_INDEX_B3I => nav_bit_array[DataBitType::DataBitD1D2 as usize].as_ref(),
                    SIGNAL_INDEX_B2A => nav_bit_array[DataBitType::DataBitBCNav2 as usize].as_ref(),
                    SIGNAL_INDEX_B2B => nav_bit_array[DataBitType::DataBitBCNav3 as usize].as_ref(),
                    _ => nav_bit_array[DataBitType::DataBitD1D2 as usize].as_ref(),
                }
            },
            GnssSystem::GalileoSystem => {
                match sat_signal_index {
                    SIGNAL_INDEX_E1 => nav_bit_array[DataBitType::DataBitINav as usize].as_ref(),
                    SIGNAL_INDEX_E5A => nav_bit_array[DataBitType::DataBitFNav as usize].as_ref(),
                    SIGNAL_INDEX_E5B => nav_bit_array[DataBitType::DataBitINav as usize].as_ref(),
                    SIGNAL_INDEX_E5 => nav_bit_array[DataBitType::DataBitFNav as usize].as_ref(),
                    SIGNAL_INDEX_E6 => nav_bit_array[DataBitType::DataBitINav as usize].as_ref(), // E6 uses I/NAV for now
                    _ => nav_bit_array[DataBitType::DataBitINav as usize].as_ref(),
                }
            },
            GnssSystem::GlonassSystem => {
                match sat_signal_index {
                    SIGNAL_INDEX_G1 => nav_bit_array[DataBitType::DataBitGNav as usize].as_ref(),
                    SIGNAL_INDEX_G2 => nav_bit_array[DataBitType::DataBitGNav as usize].as_ref(),
                    _ => nav_bit_array[DataBitType::DataBitGNav as usize].as_ref(),
                }
            },
            _ => nav_bit_array[DataBitType::DataBitLNav as usize].as_ref(),
        }
    }

    fn quant_samples_iq4(samples: &[ComplexNumber], quant_samples: &mut [u8], clipped_count: &mut i32) {
        *clipped_count = 0;

        for (i, sample) in samples.iter().enumerate() {
            if i >= quant_samples.len() {
                break;
            }

            let value = sample.real.abs();
            let mut quant_value = (value * QUANT_SCALE_IQ4) as u8;
            if quant_value > 7 {
                quant_value = 7;
                *clipped_count += 1;
            }
            quant_value += if sample.real >= 0.0 { 0 } else { 1 << 3 }; // add sign bit as MSB
            let mut quant_sample = quant_value << 4;

            let value = sample.imag.abs();
            let mut quant_value = (value * QUANT_SCALE_IQ4) as u8;
            if quant_value > 7 {
                quant_value = 7;
                *clipped_count += 1;
            }
            quant_value += if sample.imag >= 0.0 { 0 } else { 1 << 3 }; // add sign bit as MSB
            quant_sample |= quant_value;
            quant_samples[i] = quant_sample;
        }
    }

    fn quant_samples_iq8(samples: &[ComplexNumber], quant_samples: &mut [u8], clipped_count: &mut i32) {
        *clipped_count = 0;

        for (i, sample) in samples.iter().enumerate() {
            if i * 2 + 1 >= quant_samples.len() {
                break;
            }

            let val_real = sample.real * QUANT_SCALE_IQ8;
            let quant_value = if val_real > 127.0 {
                *clipped_count += 1;
                127
            } else if val_real < -128.0 {
                *clipped_count += 1;
                -128
            } else {
                val_real as i32
            };
            quant_samples[i * 2] = (quant_value & 0xff) as u8;

            let val_imag = sample.imag * QUANT_SCALE_IQ8;
            let quant_value = if val_imag > 127.0 {
                *clipped_count += 1;
                127
            } else if val_imag < -128.0 {
                *clipped_count += 1;
                -128
            } else {
                val_imag as i32
            };
            quant_samples[i * 2 + 1] = (quant_value & 0xff) as u8;
        }
    }

    // Display and utility methods
    fn display_enabled_signals(&self) {
        // GPS signals
        if self.output_param.freq_select[GnssSystem::GpsSystem as usize] != 0 {
            print!("\tGPS : [ ");
            if (self.output_param.freq_select[GnssSystem::GpsSystem as usize] & (1 << SIGNAL_INDEX_L1CA)) != 0 { print!("L1CA "); }
            if (self.output_param.freq_select[GnssSystem::GpsSystem as usize] & (1 << SIGNAL_INDEX_L1C)) != 0 { print!("L1C "); }
            if (self.output_param.freq_select[GnssSystem::GpsSystem as usize] & (1 << SIGNAL_INDEX_L2C)) != 0 { print!("L2C "); }
            if (self.output_param.freq_select[GnssSystem::GpsSystem as usize] & (1 << SIGNAL_INDEX_L5)) != 0 { print!("L5 "); }
            println!("]");
        }

        // BDS signals
        if self.output_param.freq_select[GnssSystem::BdsSystem as usize] != 0 {
            print!("\tBDS : [ ");
            if (self.output_param.freq_select[GnssSystem::BdsSystem as usize] & (1 << SIGNAL_INDEX_B1C)) != 0 { print!("B1C "); }
            if (self.output_param.freq_select[GnssSystem::BdsSystem as usize] & (1 << SIGNAL_INDEX_B1I)) != 0 { print!("B1I "); }
            if (self.output_param.freq_select[GnssSystem::BdsSystem as usize] & (1 << SIGNAL_INDEX_B2I)) != 0 { print!("B2I "); }
            if (self.output_param.freq_select[GnssSystem::BdsSystem as usize] & (1 << SIGNAL_INDEX_B2A)) != 0 { print!("B2a "); }
            if (self.output_param.freq_select[GnssSystem::BdsSystem as usize] & (1 << SIGNAL_INDEX_B2B)) != 0 { print!("B2b "); }
            if (self.output_param.freq_select[GnssSystem::BdsSystem as usize] & (1 << SIGNAL_INDEX_B3I)) != 0 { print!("B3I "); }
            println!("]");
        }

        // Galileo signals
        if self.output_param.freq_select[GnssSystem::GalileoSystem as usize] != 0 {
            print!("\tGAL : [ ");
            if (self.output_param.freq_select[GnssSystem::GalileoSystem as usize] & (1 << SIGNAL_INDEX_E1)) != 0 { print!("E1 "); }
            if (self.output_param.freq_select[GnssSystem::GalileoSystem as usize] & (1 << SIGNAL_INDEX_E5A)) != 0 { print!("E5a "); }
            if (self.output_param.freq_select[GnssSystem::GalileoSystem as usize] & (1 << SIGNAL_INDEX_E5B)) != 0 { print!("E5b "); }
            if (self.output_param.freq_select[GnssSystem::GalileoSystem as usize] & (1 << SIGNAL_INDEX_E6)) != 0 { print!("E6 "); }
            println!("]");
        }

        // GLONASS signals
        if self.output_param.freq_select[GnssSystem::GlonassSystem as usize] != 0 {
            print!("\tGLO : [ ");
            if (self.output_param.freq_select[GnssSystem::GlonassSystem as usize] & (1 << SIGNAL_INDEX_G1)) != 0 { print!("G1 "); }
            if (self.output_param.freq_select[GnssSystem::GlonassSystem as usize] & (1 << SIGNAL_INDEX_G2)) != 0 { print!("G2 "); }
            println!("]");
        }
        println!();
    }

    fn count_signals_per_system(&self) -> (usize, usize, usize, usize) {
        let mut gps_signal_count = 0;
        let mut bds_signal_count = 0;
        let mut gal_signal_count = 0;
        let mut glo_signal_count = 0;

        for signal_index in SIGNAL_INDEX_L1CA..=SIGNAL_INDEX_L5 {
            if (self.output_param.freq_select[GnssSystem::GpsSystem as usize] & (1 << signal_index)) != 0 {
                gps_signal_count += 1;
            }
        }

        for signal_index in SIGNAL_INDEX_B1C..=SIGNAL_INDEX_B2B {
            if (self.output_param.freq_select[GnssSystem::BdsSystem as usize] & (1 << signal_index)) != 0 {
                bds_signal_count += 1;
            }
        }

        for signal_index in SIGNAL_INDEX_E1..=SIGNAL_INDEX_E6 {
            if (self.output_param.freq_select[GnssSystem::GalileoSystem as usize] & (1 << signal_index)) != 0 {
                gal_signal_count += 1;
            }
        }

        for signal_index in SIGNAL_INDEX_G1..=SIGNAL_INDEX_G2 {
            if (self.output_param.freq_select[GnssSystem::GlonassSystem as usize] & (1 << signal_index)) != 0 {
                glo_signal_count += 1;
            }
        }

        (gps_signal_count, bds_signal_count, gal_signal_count, glo_signal_count)
    }

    fn display_signals_summary_table(&self, gps_signal_count: usize, bds_signal_count: usize, gal_signal_count: usize, glo_signal_count: usize) {
        println!("Signals Summary Table:");
        println!("+---------------+-------------+--------------+------------------------------+");
        println!("| Constellation | Visible SVs | Signals / SV | Total Signals / Visible SVs |");
        println!("+---------------+-------------+--------------+------------------------------+");
        println!("| GPS           | {:11} | {:12} | {:28} |", self.gps_sat_number, gps_signal_count, self.gps_sat_number * gps_signal_count);
        println!("| BeiDou        | {:11} | {:12} | {:28} |", self.bds_sat_number, bds_signal_count, self.bds_sat_number * bds_signal_count);
        println!("| Galileo       | {:11} | {:12} | {:28} |", self.gal_sat_number, gal_signal_count, self.gal_sat_number * gal_signal_count);
        println!("| GLONASS       | {:11} | {:12} | {:28} |", self.glo_sat_number, glo_signal_count, self.glo_sat_number * glo_signal_count);
        println!("+---------------+-------------+--------------+------------------------------+");

        let total_visible_svs = self.gps_sat_number + self.bds_sat_number + self.gal_sat_number + self.glo_sat_number;
        let total_channels = self.gps_sat_number * gps_signal_count + self.bds_sat_number * bds_signal_count + 
                           self.gal_sat_number * gal_signal_count + self.glo_sat_number * glo_signal_count;
        println!("Total Visible SVs = {}, Total channels = {}\n", total_visible_svs, total_channels);
    }

    fn display_progress(&self, length: i32, total_duration_ms: i32, total_mb: f64, bytes_per_ms: f64, start_time: Instant) {
        let elapsed = start_time.elapsed().as_millis() as u64;
        let percentage = length as f64 / total_duration_ms as f64 * 100.0;
        let current_mb = (length as f64 * bytes_per_ms) / (1024.0 * 1024.0);
        let mb_per_sec = if elapsed > 0 { (current_mb * 1000.0) / elapsed as f64 } else { 0.0 };

        // Calculate estimated time remaining
        let eta_ms = if percentage > 0.0 && elapsed > 0 {
            ((elapsed as f64 * (100.0 - percentage)) / percentage) as u64
        } else {
            0
        };

        // Progress bar with percentage in center
        let bar_width = 50;
        let progress = (percentage * bar_width as f64 / 100.0) as usize;
        let progress_str = format!("{:.1}%", percentage);
        let progress_str_len = progress_str.len();
        let center_pos = (bar_width - progress_str_len) / 2;

        print!("\r[");
        for k in 0..bar_width {
            if k >= center_pos && k < center_pos + progress_str_len {
                print!("{}", progress_str.chars().nth(k - center_pos).unwrap_or(' '));
            } else if k < progress {
                print!("=");
            } else if k == progress && percentage < 100.0 {
                print!(">");
            } else if percentage >= 100.0 && k < bar_width {
                print!("=");
            } else {
                print!(" ");
            }
        }

        // Format ETA
        let eta_str = if eta_ms > 0 {
            let eta_seconds = (eta_ms / 1000) as i32;
            let eta_minutes = eta_seconds / 60;
            let eta_seconds = eta_seconds % 60;
            if eta_minutes > 0 {
                format!("ETA: {}m{:02}s   ", eta_minutes, eta_seconds)
            } else {
                format!("ETA: {:02}s   ", eta_seconds)
            }
        } else {
            "ETA: --   ".to_string()
        };

        print!("] {}/{} ms | {:.2}/{:.2} MB | {:.2} MB/s | {}",
               length, total_duration_ms, current_mb, total_mb, mb_per_sec, eta_str);
        std::io::stdout().flush().unwrap();
    }

    fn display_final_progress(&self, total_duration_ms: i32, total_mb: f64) {
        print!("\r[");
        for k in 0..50 {
            if k >= 22 && k < 28 {
                print!("{}", "100.0%".chars().nth(k - 22).unwrap_or(' '));
            } else {
                print!("=");
            }
        }
        println!("] {}/{} ms | {:.2}/{:.2} MB | COMPLETED: --",
                total_duration_ms, total_duration_ms, total_mb, total_mb);
    }

    fn display_completion_stats(&self, total_samples: i64, total_clipped_samples: i64, agc_gain: f64, start_time: Instant, final_mb: f64) {
        let duration = start_time.elapsed();
        let avg_mb_per_sec = if duration.as_millis() > 0 { 
            (final_mb * 1000.0) / duration.as_millis() as f64 
        } else { 
            0.0 
        };

        println!("\n[INFO]\tIF Signal generation completed!");
        println!("------------------------------------------------------------------");
        println!("[INFO]\tTotal samples: {}", total_samples);
        println!("[INFO]\tClipped samples: {} ({:.4}%)", total_clipped_samples, total_clipped_samples as f64 / total_samples as f64 * 100.0);
        println!("[INFO]\tFinal AGC gain: {:.3}", agc_gain);
        if total_clipped_samples as f64 / total_samples as f64 > 0.05 {
            println!("[WARNING]\tHigh clipping rate! Consider reducing initPower in JSON config.");
        }
        println!("[INFO]\tTotal time taken: {:.2} s", duration.as_secs_f64());
        println!("[INFO]\tData generated: {:.2} MB", final_mb);
        println!("[INFO]\tAverage rate: {:.2} MB/s", avg_mb_per_sec);
        println!("------------------------------------------------------------------");
    }

    // Placeholder methods that need to be implemented based on the actual types and interfaces
    fn assign_parameters(&mut self, _object: &JsonObject, _utc_time: &mut UtcTime, _start_pos: &mut LlaPosition, _start_vel: &mut LocalSpeed) -> Result<(), Box<dyn std::error::Error>> {
        // Implementation would parse JSON and assign parameters
        Ok(())
    }

    fn get_visible_satellite(&self, _cur_pos: KinematicInfo, _cur_time: GnssTime, _system: GnssSystem, _eph: &[Option<GpsEphemeris>], _eph_visible: &mut [Option<GpsEphemeris>]) -> usize {
        // Implementation would calculate visible satellites
        0
    }

    fn get_glonass_visible_satellite(&self, _cur_pos: KinematicInfo, _glonass_time: GlonassTime, _eph: &[Option<GlonassEphemeris>], _eph_visible: &mut [Option<GlonassEphemeris>]) -> usize {
        // Implementation would calculate visible GLONASS satellites
        0
    }

    fn update_sat_param_list(&mut self, _cur_time: GnssTime, _cur_pos: KinematicInfo, _list_count: usize, _power_list: &[SignalPower], _iono_param: &IonoParam) {
        // Implementation would update satellite parameters
    }

    fn convert_glonass_to_gps_ephemeris(&self, _glo_eph: &GlonassEphemeris) -> GpsEphemeris {
        // Implementation would convert GLONASS ephemeris to GPS format
        GpsEphemeris::new()
    }

    fn convert_glonass_to_gps_almanac(&self, _glo_alm: &[GlonassAlmanac]) -> Vec<GpsAlmanac> {
        // Implementation would convert GLONASS almanac to GPS format
        Vec::new()
    }

    fn create_gps_signals(&self, sat_if_signals: &mut Vec<Option<Box<SatIfSignal>>>, mut total_channel_number: usize, nav_bit_array: &Vec<Option<Box<dyn NavBitTrait>>>) -> Result<usize, Box<dyn std::error::Error>> {
        for i in 0..self.gps_sat_number {
            if let Some(eph) = &self.gps_eph_visible[i] {
                for signal_index in SIGNAL_INDEX_L1CA..=SIGNAL_INDEX_L5 {
                    if (self.output_param.freq_select[GnssSystem::GpsSystem as usize] & (1 << signal_index)) != 0 {
                        let center_freq = SIGNAL_CENTER_FREQ[GnssSystem::GpsSystem as usize][signal_index];
                        let if_freq = (center_freq - self.output_param.center_freq) as i32;
                        let mut new_signal = SatIfSignal::new(self.output_param.sample_freq as i32, if_freq, GnssSystem::GpsSystem, signal_index as i32, eph.svid as u8);
                        
                        let nav_data = self.get_nav_data(GnssSystem::GpsSystem, signal_index as i32, nav_bit_array);
                        // Cloning the boxed trait object
                        let nav_data_clone = nav_data.map(|nav| nav.clone_box());

                        new_signal.init_state(self.cur_time, &self.gps_sat_param[eph.svid as usize - 1], nav_data_clone);
                        sat_if_signals[total_channel_number] = Some(Box::new(new_signal));
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    fn create_bds_signals(&self, sat_if_signals: &mut Vec<Option<Box<SatIfSignal>>>, mut total_channel_number: usize, nav_bit_array: &Vec<Option<Box<dyn NavBitTrait>>>) -> Result<usize, Box<dyn std::error::Error>> {
        for i in 0..self.bds_sat_number {
            if let Some(eph) = &self.bds_eph_visible[i] {
                for signal_index in SIGNAL_INDEX_B1C..=SIGNAL_INDEX_B2AB {
                    if (self.output_param.freq_select[GnssSystem::BdsSystem as usize] & (1 << signal_index)) != 0 {
                        let center_freq = SIGNAL_CENTER_FREQ[GnssSystem::BdsSystem as usize][signal_index];
                        let if_freq = (center_freq - self.output_param.center_freq) as i32;
                        let mut new_signal = SatIfSignal::new(self.output_param.sample_freq as i32, if_freq, GnssSystem::BdsSystem, signal_index as i32, eph.svid as u8);
                        
                        let nav_data = self.get_nav_data(GnssSystem::BdsSystem, signal_index as i32, nav_bit_array);
                        let nav_data_clone = nav_data.map(|nav| nav.clone_box());

                        new_signal.init_state(self.cur_time, &self.bds_sat_param[eph.svid as usize - 1], nav_data_clone);
                        sat_if_signals[total_channel_number] = Some(Box::new(new_signal));
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    fn create_galileo_signals(&self, sat_if_signals: &mut Vec<Option<Box<SatIfSignal>>>, mut total_channel_number: usize, nav_bit_array: &Vec<Option<Box<dyn NavBitTrait>>>) -> Result<usize, Box<dyn std::error::Error>> {
        for i in 0..self.gal_sat_number {
            if let Some(eph) = &self.gal_eph_visible[i] {
                for signal_index in SIGNAL_INDEX_E1..=SIGNAL_INDEX_E6 {
                    if (self.output_param.freq_select[GnssSystem::GalileoSystem as usize] & (1 << signal_index)) != 0 {
                        let center_freq = SIGNAL_CENTER_FREQ[GnssSystem::GalileoSystem as usize][signal_index];
                        let if_freq = (center_freq - self.output_param.center_freq) as i32;
                        let mut new_signal = SatIfSignal::new(self.output_param.sample_freq as i32, if_freq, GnssSystem::GalileoSystem, signal_index as i32, eph.svid as u8);
                        
                        let nav_data = self.get_nav_data(GnssSystem::GalileoSystem, signal_index as i32, nav_bit_array);
                        let nav_data_clone = nav_data.map(|nav| nav.clone_box());

                        new_signal.init_state(self.cur_time, &self.gal_sat_param[eph.svid as usize - 1], nav_data_clone);
                        sat_if_signals[total_channel_number] = Some(Box::new(new_signal));
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }

    fn create_glonass_signals(&self, sat_if_signals: &mut Vec<Option<Box<SatIfSignal>>>, mut total_channel_number: usize, nav_bit_array: &Vec<Option<Box<dyn NavBitTrait>>>) -> Result<usize, Box<dyn std::error::Error>> {
        for i in 0..self.glo_sat_number {
            if let Some(eph) = &self.glo_eph_visible[i] {
                for signal_index in SIGNAL_INDEX_G1..=SIGNAL_INDEX_G2 {
                    if (self.output_param.freq_select[GnssSystem::GlonassSystem as usize] & (1 << signal_index)) != 0 {
                        let center_freq = SIGNAL_CENTER_FREQ[GnssSystem::GlonassSystem as usize][signal_index] + eph.freq as f64 * 562500.0;
                        let if_freq = (center_freq - self.output_param.center_freq) as i32;
                        let mut new_signal = SatIfSignal::new(self.output_param.sample_freq as i32, if_freq, GnssSystem::GlonassSystem, signal_index as i32, eph.n as u8);
                        
                        let nav_data = self.get_nav_data(GnssSystem::GlonassSystem, signal_index as i32, nav_bit_array);
                        let nav_data_clone = nav_data.map(|nav| nav.clone_box());

                        new_signal.init_state(self.cur_time, &self.glo_sat_param[eph.n as usize - 1], nav_data_clone);
                        sat_if_signals[total_channel_number] = Some(Box::new(new_signal));
                        total_channel_number += 1;
                    }
                }
            }
        }
        Ok(total_channel_number)
    }
}

impl Default for IFDataGen {
    fn default() -> Self {
        Self::new()
    }
}

// Main function for the executable
pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let mut if_data_gen = IFDataGen::new();
    if_data_gen.main(args)
}