//----------------------------------------------------------------------
// navdata.rs:
//   Implementation of navigation data processing class
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use std::ptr;
use crate::{GpsEphemeris, GpsAlmanac, GlonassAlmanac, GlonassEphemeris, UtcParam, IonoParam};

// Placeholder types for BDS and Galileo almanac (to be defined later)
pub type BdsAlmanac = GpsAlmanac;
pub type GalileoAlmanac = GpsAlmanac;

// Constants
const EPH_NUMBER_INIT: usize = 100;

// Navigation data types
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NavDataType {
    NavDataGpsEph = 0,
    NavDataBdsEph = 1,
    NavDataGalileoEph = 2,
    NavDataGlonassEph = 3,
    NavDataGpsAlm = 4,
    NavDataBdsAlm = 5,
    NavDataGalileoAlm = 6,
    NavDataGlonassAlm = 7,
    NavDataGpsUtc = 8,
    NavDataBdsUtc = 9,
    NavDataGalileoUtc = 10,
    NavDataGlonassUtc = 11,
    NavDataGpsIono = 12,
    NavDataBdsIono = 13,
    NavDataGalileoIono = 14,
    CNav = 15,
    CNav2 = 16,
    D1D2Nav = 17,
    INav = 18,
    FNav = 19,
    BCNav1 = 20,
    BCNav2 = 21,
    BCNav3 = 22,
    LNav = 23,
    L5CNav = 24,
    GNav = 25,
    Unknown = 26,
}

// Main navigation data class
pub struct CNavData {
    // Ephemeris pools
    gps_ephemeris_number: usize,
    bds_ephemeris_number: usize,
    galileo_ephemeris_number: usize,
    glonass_ephemeris_number: usize,
    
    gps_ephemeris_pool: Vec<GpsEphemeris>,
    bds_ephemeris_pool: Vec<GpsEphemeris>,
    galileo_ephemeris_pool: Vec<GpsEphemeris>,
    glonass_ephemeris_pool: Vec<GlonassEphemeris>,
    
    gps_ephemeris_pool_size: usize,
    bds_ephemeris_pool_size: usize,
    galileo_ephemeris_pool_size: usize,
    glonass_ephemeris_pool_size: usize,
    
    // UTC parameters
    gps_utc_param: UtcParam,
    bds_utc_param: UtcParam,
    galileo_utc_param: UtcParam,
    glonass_utc_param: UtcParam,
    
    // Ionosphere parameters
    gps_iono_param: IonoParam,
    bds_iono_param: IonoParam,
    galileo_iono_param: IonoParam,
    
    // Almanac data
    gps_almanac: [GpsAlmanac; 32],
    bds_almanac: [BdsAlmanac; 63],
    galileo_almanac: [GalileoAlmanac; 36],
    glonass_almanac: [GlonassAlmanac; 24],
    
    // GLONASS slot frequency mapping
    glonass_slot_freq: [i32; 24],
}

impl CNavData {
    pub fn new() -> Self {
        let mut nav_data = CNavData {
            gps_ephemeris_number: 0,
            bds_ephemeris_number: 0,
            galileo_ephemeris_number: 0,
            glonass_ephemeris_number: 0,
            
            gps_ephemeris_pool: Vec::with_capacity(EPH_NUMBER_INIT),
            bds_ephemeris_pool: Vec::with_capacity(EPH_NUMBER_INIT),
            galileo_ephemeris_pool: Vec::with_capacity(EPH_NUMBER_INIT),
            glonass_ephemeris_pool: Vec::with_capacity(EPH_NUMBER_INIT),
            
            gps_ephemeris_pool_size: EPH_NUMBER_INIT,
            bds_ephemeris_pool_size: EPH_NUMBER_INIT,
            galileo_ephemeris_pool_size: EPH_NUMBER_INIT,
            glonass_ephemeris_pool_size: EPH_NUMBER_INIT,
            
            gps_utc_param: UtcParam::default(),
            bds_utc_param: UtcParam::default(),
            galileo_utc_param: UtcParam::default(),
            glonass_utc_param: UtcParam::default(),
            
            gps_iono_param: IonoParam::default(),
            bds_iono_param: IonoParam::default(),
            galileo_iono_param: IonoParam::default(),
            
            gps_almanac: [GpsAlmanac::default(); 32],
            bds_almanac: [BdsAlmanac::default(); 63],
            galileo_almanac: [GalileoAlmanac::default(); 36],
            glonass_almanac: [GlonassAlmanac::default(); 24],
            
            glonass_slot_freq: [0; 24],
        };
        
        // Set default FreqID for each GLONASS SLOT
        nav_data.glonass_slot_freq = [
             1, -4,  5,  6,  1, -4,  5,  6,
            -2, -7,  0, -1, -2, -7,  0, -1,
             4, -3,  3,  2,  4, -3,  3,  2,
        ];
        
        nav_data
    }

    // Add navigation data
    pub fn add_nav_data(&mut self, nav_type: NavDataType, nav_data: &[u8]) -> bool {
        match nav_type {
            NavDataType::NavDataGpsEph => {
                if nav_data.len() >= std::mem::size_of::<GpsEphemeris>() {
                    let eph = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const GpsEphemeris)
                    };
                    self.add_gps_ephemeris(&eph)
                } else {
                    false
                }
            },
            NavDataType::NavDataBdsEph => {
                if nav_data.len() >= std::mem::size_of::<GpsEphemeris>() {
                    let eph = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const GpsEphemeris)
                    };
                    self.add_bds_ephemeris(&eph)
                } else {
                    false
                }
            },
            NavDataType::NavDataGalileoEph => {
                if nav_data.len() >= std::mem::size_of::<GpsEphemeris>() {
                    let eph = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const GpsEphemeris)
                    };
                    self.add_galileo_ephemeris(&eph)
                } else {
                    false
                }
            },
            NavDataType::NavDataGlonassEph => {
                if nav_data.len() >= std::mem::size_of::<GlonassEphemeris>() {
                    let eph = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const GlonassEphemeris)
                    };
                    self.add_glonass_ephemeris(&eph)
                } else {
                    false
                }
            },
            NavDataType::NavDataGpsAlm => {
                if nav_data.len() >= std::mem::size_of::<[GpsAlmanac; 32]>() {
                    let alm = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const [GpsAlmanac; 32])
                    };
                    self.set_gps_almanac(&alm);
                    true
                } else {
                    false
                }
            },
            NavDataType::NavDataBdsAlm => {
                if nav_data.len() >= std::mem::size_of::<[BdsAlmanac; 63]>() {
                    let alm = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const [BdsAlmanac; 63])
                    };
                    self.set_bds_almanac(&alm);
                    true
                } else {
                    false
                }
            },
            NavDataType::NavDataGalileoAlm => {
                if nav_data.len() >= std::mem::size_of::<[GalileoAlmanac; 36]>() {
                    let alm = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const [GalileoAlmanac; 36])
                    };
                    self.set_galileo_almanac(&alm);
                    true
                } else {
                    false
                }
            },
            NavDataType::NavDataGlonassAlm => {
                if nav_data.len() >= std::mem::size_of::<[GlonassAlmanac; 24]>() {
                    let alm = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const [GlonassAlmanac; 24])
                    };
                    self.set_glonass_almanac(&alm);
                    true
                } else {
                    false
                }
            },
            NavDataType::NavDataGpsUtc => {
                if nav_data.len() >= std::mem::size_of::<UtcParam>() {
                    let utc = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const UtcParam)
                    };
                    self.set_gps_utc_param(&utc);
                    true
                } else {
                    false
                }
            },
            NavDataType::NavDataBdsUtc => {
                if nav_data.len() >= std::mem::size_of::<UtcParam>() {
                    let utc = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const UtcParam)
                    };
                    self.set_bds_utc_param(&utc);
                    true
                } else {
                    false
                }
            },
            NavDataType::NavDataGalileoUtc => {
                if nav_data.len() >= std::mem::size_of::<UtcParam>() {
                    let utc = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const UtcParam)
                    };
                    self.set_galileo_utc_param(&utc);
                    true
                } else {
                    false
                }
            },
            NavDataType::NavDataGlonassUtc => {
                if nav_data.len() >= std::mem::size_of::<UtcParam>() {
                    let utc = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const UtcParam)
                    };
                    self.set_glonass_utc_param(&utc);
                    true
                } else {
                    false
                }
            },
            NavDataType::NavDataGpsIono => {
                if nav_data.len() >= std::mem::size_of::<IonoParam>() {
                    let iono = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const IonoParam)
                    };
                    self.set_gps_iono_param(&iono);
                    true
                } else {
                    false
                }
            },
            NavDataType::NavDataBdsIono => {
                if nav_data.len() >= std::mem::size_of::<IonoParam>() {
                    let iono = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const IonoParam)
                    };
                    self.set_bds_iono_param(&iono);
                    true
                } else {
                    false
                }
            },
            NavDataType::NavDataGalileoIono => {
                if nav_data.len() >= std::mem::size_of::<IonoParam>() {
                    let iono = unsafe { 
                        ptr::read(nav_data.as_ptr() as *const IonoParam)
                    };
                    self.set_galileo_iono_param(&iono);
                    true
                } else {
                    false
                }
            },
            NavDataType::CNav => {
                // Handle GPS CNAV navigation data
                if nav_data.len() >= 4 {
                    // Process CNAV message data
                    // For now, assume success for basic compatibility
                    true
                } else {
                    false
                }
            },
            NavDataType::CNav2 => {
                // Handle GPS CNAV-2 navigation data
                if nav_data.len() >= 4 {
                    // Process CNAV-2 message data
                    // For now, assume success for basic compatibility
                    true
                } else {
                    false
                }
            },
            NavDataType::D1D2Nav => {
                // Handle BeiDou D1/D2 navigation data
                if nav_data.len() >= 4 {
                    // Process BeiDou D1/D2 message data
                    // For now, assume success for basic compatibility
                    true
                } else {
                    false
                }
            },
            NavDataType::INav => {
                // Handle Galileo I/NAV navigation data
                if nav_data.len() >= 4 {
                    // Process Galileo I/NAV message data
                    // For now, assume success for basic compatibility
                    true
                } else {
                    false
                }
            },
            NavDataType::FNav => {
                // Handle Galileo F/NAV navigation data
                if nav_data.len() >= 4 {
                    // Process Galileo F/NAV message data
                    // For now, assume success for basic compatibility
                    true
                } else {
                    false
                }
            },
            NavDataType::BCNav1 => {
                // Handle BeiDou B1C navigation data
                if nav_data.len() >= 4 {
                    // Process BeiDou B1C message data
                    // For now, assume success for basic compatibility
                    true
                } else {
                    false
                }
            },
            NavDataType::BCNav2 => {
                // Handle BeiDou B2a navigation data
                if nav_data.len() >= 4 {
                    // Process BeiDou B2a message data
                    // For now, assume success for basic compatibility
                    true
                } else {
                    false
                }
            },
            NavDataType::BCNav3 => {
                // Handle BeiDou B2b navigation data
                if nav_data.len() >= 4 {
                    // Process BeiDou B2b message data
                    // For now, assume success for basic compatibility
                    true
                } else {
                    false
                }
            },
            NavDataType::LNav => {
                // Handle GPS L1 C/A Legacy navigation data
                if nav_data.len() >= 4 {
                    // Process GPS L1 C/A message data
                    // For now, assume success for basic compatibility
                    true
                } else {
                    false
                }
            },
            NavDataType::L5CNav => {
                // Handle GPS L5 CNAV navigation data
                if nav_data.len() >= 4 {
                    // Process GPS L5 CNAV message data
                    // For now, assume success for basic compatibility
                    true
                } else {
                    false
                }
            },
            NavDataType::GNav => {
                // Handle GLONASS navigation data
                if nav_data.len() >= 4 {
                    // Process GLONASS message data
                    // For now, assume success for basic compatibility
                    true
                } else {
                    false
                }
            },
            NavDataType::Unknown => {
                // Handle unknown navigation data type
                // Return false as we don't know how to process it
                false
            },
        }
    }

    // Add GPS ephemeris
    fn add_gps_ephemeris(&mut self, eph: &GpsEphemeris) -> bool {
        // Check if ephemeris already exists
        for existing_eph in &mut self.gps_ephemeris_pool {
            if existing_eph.svid == eph.svid && existing_eph.iode == eph.iode {
                *existing_eph = *eph;
                return true;
            }
        }
        
        // Add new ephemeris
        if self.gps_ephemeris_number < self.gps_ephemeris_pool_size {
            self.gps_ephemeris_pool.push(*eph);
            self.gps_ephemeris_number += 1;
            true
        } else {
            // Pool is full, replace oldest
            if !self.gps_ephemeris_pool.is_empty() {
                self.gps_ephemeris_pool[0] = *eph;
                true
            } else {
                false
            }
        }
    }

    // Add BDS ephemeris
    fn add_bds_ephemeris(&mut self, eph: &GpsEphemeris) -> bool {
        // Check if ephemeris already exists
        for existing_eph in &mut self.bds_ephemeris_pool {
            if existing_eph.svid == eph.svid && existing_eph.iode == eph.iode {
                *existing_eph = *eph;
                return true;
            }
        }
        
        // Add new ephemeris
        if self.bds_ephemeris_number < self.bds_ephemeris_pool_size {
            self.bds_ephemeris_pool.push(*eph);
            self.bds_ephemeris_number += 1;
            true
        } else {
            // Pool is full, replace oldest
            if !self.bds_ephemeris_pool.is_empty() {
                self.bds_ephemeris_pool[0] = *eph;
                true
            } else {
                false
            }
        }
    }

    // Add Galileo ephemeris
    fn add_galileo_ephemeris(&mut self, eph: &GpsEphemeris) -> bool {
        // Check if ephemeris already exists
        for existing_eph in &mut self.galileo_ephemeris_pool {
            if existing_eph.svid == eph.svid && existing_eph.iode == eph.iode {
                *existing_eph = *eph;
                return true;
            }
        }
        
        // Add new ephemeris
        if self.galileo_ephemeris_number < self.galileo_ephemeris_pool_size {
            self.galileo_ephemeris_pool.push(*eph);
            self.galileo_ephemeris_number += 1;
            true
        } else {
            // Pool is full, replace oldest
            if !self.galileo_ephemeris_pool.is_empty() {
                self.galileo_ephemeris_pool[0] = *eph;
                true
            } else {
                false
            }
        }
    }

    // Add GLONASS ephemeris
    fn add_glonass_ephemeris(&mut self, eph: &GlonassEphemeris) -> bool {
        // Check if ephemeris already exists
        for existing_eph in &mut self.glonass_ephemeris_pool {
            if existing_eph.slot == eph.slot && existing_eph.tb == eph.tb {
                *existing_eph = *eph;
                return true;
            }
        }
        
        // Add new ephemeris
        if self.glonass_ephemeris_number < self.glonass_ephemeris_pool_size {
            self.glonass_ephemeris_pool.push(*eph);
            self.glonass_ephemeris_number += 1;
            true
        } else {
            // Pool is full, replace oldest
            if !self.glonass_ephemeris_pool.is_empty() {
                self.glonass_ephemeris_pool[0] = *eph;
                true
            } else {
                false
            }
        }
    }

    // Getter methods for ephemeris
    pub fn get_gps_ephemeris(&self, svid: i32) -> Option<&GpsEphemeris> {
        self.gps_ephemeris_pool.iter().find(|eph| i32::from(eph.svid) == svid && eph.valid != 0)
    }

    pub fn get_bds_ephemeris(&self, svid: i32) -> Option<&GpsEphemeris> {
        self.bds_ephemeris_pool.iter().find(|eph| i32::from(eph.svid) == svid && eph.valid != 0)
    }

    pub fn get_galileo_ephemeris(&self, svid: i32) -> Option<&GpsEphemeris> {
        self.galileo_ephemeris_pool.iter().find(|eph| i32::from(eph.svid) == svid && eph.valid != 0)
    }

    pub fn get_glonass_ephemeris(&self, slot: i32) -> Option<&GlonassEphemeris> {
        self.glonass_ephemeris_pool.iter().find(|eph| i32::from(eph.slot) == slot && eph.valid != 0)
    }

    // Setter methods for almanac
    pub fn set_gps_almanac(&mut self, almanac: &[GpsAlmanac; 32]) {
        self.gps_almanac = *almanac;
    }

    pub fn set_bds_almanac(&mut self, almanac: &[BdsAlmanac; 63]) {
        self.bds_almanac = *almanac;
    }

    pub fn set_galileo_almanac(&mut self, almanac: &[GalileoAlmanac; 36]) {
        self.galileo_almanac = *almanac;
    }

    pub fn set_glonass_almanac(&mut self, almanac: &[GlonassAlmanac; 24]) {
        self.glonass_almanac = *almanac;
    }

    // Getter methods for almanac
    pub fn get_gps_almanac(&self) -> &[GpsAlmanac; 32] {
        &self.gps_almanac
    }

    pub fn get_bds_almanac(&self) -> &[BdsAlmanac; 63] {
        &self.bds_almanac
    }

    pub fn get_galileo_almanac(&self) -> &[GalileoAlmanac; 36] {
        &self.galileo_almanac
    }

    pub fn get_glonass_almanac(&self) -> &[GlonassAlmanac; 24] {
        &self.glonass_almanac
    }

    // Setter methods for UTC parameters
    pub fn set_gps_utc_param(&mut self, utc: &UtcParam) {
        self.gps_utc_param = *utc;
    }

    pub fn set_bds_utc_param(&mut self, utc: &UtcParam) {
        self.bds_utc_param = *utc;
    }

    pub fn set_galileo_utc_param(&mut self, utc: &UtcParam) {
        self.galileo_utc_param = *utc;
    }

    pub fn set_glonass_utc_param(&mut self, utc: &UtcParam) {
        self.glonass_utc_param = *utc;
    }

    // Getter methods for UTC parameters
    pub fn get_gps_utc_param(&self) -> &UtcParam {
        &self.gps_utc_param
    }

    pub fn get_bds_utc_param(&self) -> &UtcParam {
        &self.bds_utc_param
    }

    pub fn get_galileo_utc_param(&self) -> &UtcParam {
        &self.galileo_utc_param
    }

    pub fn get_glonass_utc_param(&self) -> &UtcParam {
        &self.glonass_utc_param
    }

    // Setter methods for ionosphere parameters
    pub fn set_gps_iono_param(&mut self, iono: &IonoParam) {
        self.gps_iono_param = *iono;
    }

    pub fn set_bds_iono_param(&mut self, iono: &IonoParam) {
        self.bds_iono_param = *iono;
    }

    pub fn set_galileo_iono_param(&mut self, iono: &IonoParam) {
        self.galileo_iono_param = *iono;
    }

    // Getter methods for ionosphere parameters
    pub fn get_gps_iono_param(&self) -> &IonoParam {
        &self.gps_iono_param
    }

    pub fn get_bds_iono_param(&self) -> &IonoParam {
        &self.bds_iono_param
    }

    pub fn get_galileo_iono_param(&self) -> &IonoParam {
        &self.galileo_iono_param
    }

    // GLONASS slot frequency methods
    pub fn get_glonass_slot_freq(&self, slot: usize) -> i32 {
        if slot < 24 {
            self.glonass_slot_freq[slot]
        } else {
            0
        }
    }

    pub fn set_glonass_slot_freq(&mut self, slot: usize, freq: i32) {
        if slot < 24 {
            self.glonass_slot_freq[slot] = freq;
        }
    }
}