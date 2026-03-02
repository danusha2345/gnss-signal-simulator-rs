//----------------------------------------------------------------------
// delay_model.rs:
//   Implementation of ionosphere and troposphere delay model
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//----------------------------------------------------------------------

use crate::coordinate::gps_iono_delay;
use crate::types::IonoParam;

pub trait IonoDelay {
    fn get_delay(&self, time: f64, lat: f64, lon: f64, elevation: f64, azimuth: f64) -> f64;
}

pub struct IonoKlobuchar8 {
    pub iono_param: IonoParam,
}

impl IonoKlobuchar8 {
    pub fn new() -> Self {
        IonoKlobuchar8 {
            iono_param: IonoParam::default(),
        }
    }

    pub fn with_param(param: &IonoParam) -> Self {
        IonoKlobuchar8 {
            iono_param: *param,
        }
    }

    pub fn set_iono_param(&mut self, param: &IonoParam) {
        self.iono_param = *param;
    }
}

impl Default for IonoKlobuchar8 {
    fn default() -> Self {
        Self::new()
    }
}

impl IonoDelay for IonoKlobuchar8 {
    fn get_delay(&self, time: f64, lat: f64, lon: f64, elevation: f64, azimuth: f64) -> f64 {
        gps_iono_delay(&self.iono_param, time, lat, lon, elevation, azimuth)
    }
}
