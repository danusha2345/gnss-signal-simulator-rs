//----------------------------------------------------------------------
// trajectory.rs:
//   Definition of trajectory processing class
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::types::*;
use crate::coordinate::*;
use std::f64::consts::PI;

// Error codes
pub const TRAJECTORY_NO_ERR: i32 = 0;
pub const TRAJECTORY_UNKNOWN_TYPE: i32 = 1;
pub const TRAJECTORY_TYPE_MISMATCH: i32 = 2;
pub const TRAJECTORY_INVALID_PARAM: i32 = 3;
pub const TRAJECTORY_ZERO_SPEED: i32 = 4;
pub const TRAJECTORY_ACC_JERK: i32 = 1;
pub const TRAJECTORY_ZERO_ACC: i32 = 2;
pub const TRAJECTORY_ZERO_ACCRATE: i32 = 3;
pub const TRAJECTORY_ZERO_DEGREE: i32 = 4;
pub const TRAJECTORY_NEGATIVE: i32 = 5;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TrajectoryType {
    TrajTypeUnknown = 0,
    TrajTypeConstSpeed,
    TrajTypeConstAcc,
    TrajTypeVerticalAcc,
    TrajTypeJerk,
    TrajTypeHorizontalCircular,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TrajectoryDataType {
    TrajDataTimeSpan = 0,
    TrajDataAcceleration,
    TrajDataSpeed,
    TrajDataAccRate,
    TrajDataAngle,
    TrajDataAngularRate,
    TrajDataRadius,
}

// Base trait for trajectory segments
pub trait TrajectorySegment {
    fn set_segment_param(
        &mut self,
        prev_segment: Option<&dyn TrajectorySegment>,
        data_type1: TrajectoryDataType,
        data1: f64,
        data_type2: TrajectoryDataType,
        data2: f64,
    ) -> i32;
    
    fn get_pos_vel(&self, relative_time: f64) -> KinematicInfo;
    fn get_trajectory_type(&self) -> TrajectoryType;
    fn get_start_pos_vel(&self) -> &KinematicInfo;
    fn get_local_speed(&self) -> &LocalSpeed;
    fn get_convert_matrix(&self) -> &ConvertMatrix;
    fn get_time_span(&self) -> f64;
    fn set_start_pos_vel(&mut self, pos_vel: KinematicInfo);
    fn set_local_speed(&mut self, speed: LocalSpeed);
    fn set_convert_matrix(&mut self, matrix: ConvertMatrix);
    fn set_time_span(&mut self, time_span: f64);
}

// Base structure for all trajectory segments
#[derive(Debug, Clone)]
pub struct TrajectorySegmentBase {
    pub start_pos_vel: KinematicInfo,
    pub local_speed: LocalSpeed,
    pub convert_matrix: ConvertMatrix,
    pub time_span: f64,
}

impl TrajectorySegmentBase {
    pub fn new() -> Self {
        TrajectorySegmentBase {
            start_pos_vel: KinematicInfo::default(),
            local_speed: LocalSpeed::default(),
            convert_matrix: ConvertMatrix::default(),
            time_span: 0.0,
        }
    }

    pub fn get_speed_projection(&self) -> [f64; 3] {
        let speed = (self.start_pos_vel.vx * self.start_pos_vel.vx + 
                    self.start_pos_vel.vy * self.start_pos_vel.vy + 
                    self.start_pos_vel.vz * self.start_pos_vel.vz).sqrt();
        
        let mut pos_vel = self.start_pos_vel;
        
        if speed < 1e-5 {
            // Zero speed, direction derive from course
            let mut local_speed = self.local_speed;
            local_speed.speed = 1.0;
            speed_course_to_enu(&mut local_speed);
            speed_local_to_ecef(&self.convert_matrix, &local_speed, &mut pos_vel);
            let new_speed = (pos_vel.vx * pos_vel.vx + pos_vel.vy * pos_vel.vy + pos_vel.vz * pos_vel.vz).sqrt();
            [pos_vel.vx / new_speed, pos_vel.vy / new_speed, pos_vel.vz / new_speed]
        } else {
            [pos_vel.vx / speed, pos_vel.vy / speed, pos_vel.vz / speed]
        }
    }

    pub fn init_segment(&mut self, prev_segment: Option<&dyn TrajectorySegment>) {
        if let Some(prev) = prev_segment {
            self.start_pos_vel = prev.get_pos_vel(prev.get_time_span());
            self.local_speed = *prev.get_local_speed();
            self.convert_matrix = *prev.get_convert_matrix();
        }
    }
}

// Constant speed trajectory
#[derive(Debug, Clone)]
pub struct TrajectoryConstSpeed {
    pub base: TrajectorySegmentBase,
}

impl TrajectoryConstSpeed {
    pub fn new() -> Self {
        TrajectoryConstSpeed {
            base: TrajectorySegmentBase::new(),
        }
    }
}

impl TrajectorySegment for TrajectoryConstSpeed {
    fn set_segment_param(
        &mut self,
        prev_segment: Option<&dyn TrajectorySegment>,
        data_type1: TrajectoryDataType,
        data1: f64,
        data_type2: TrajectoryDataType,
        data2: f64,
    ) -> i32 {
        self.base.init_segment(prev_segment);
        
        // Handle different parameter combinations
        match (data_type1, data_type2) {
            (TrajectoryDataType::TrajDataTimeSpan, TrajectoryDataType::TrajDataSpeed) |
            (TrajectoryDataType::TrajDataSpeed, TrajectoryDataType::TrajDataTimeSpan) => {
                let (time_span, speed) = if matches!(data_type1, TrajectoryDataType::TrajDataTimeSpan) {
                    (data1, data2)
                } else {
                    (data2, data1)
                };
                
                if time_span <= 0.0 {
                    return TRAJECTORY_INVALID_PARAM;
                }
                if speed < 0.0 {
                    return TRAJECTORY_NEGATIVE;
                }
                
                self.base.time_span = time_span;
                self.base.local_speed.speed = speed;
                TRAJECTORY_NO_ERR
            },
            _ => TRAJECTORY_TYPE_MISMATCH
        }
    }

    fn get_pos_vel(&self, relative_time: f64) -> KinematicInfo {
        let projection = self.base.get_speed_projection();
        let distance = self.base.local_speed.speed * relative_time;
        
        KinematicInfo {
            x: self.base.start_pos_vel.x + projection[0] * distance,
            y: self.base.start_pos_vel.y + projection[1] * distance,
            z: self.base.start_pos_vel.z + projection[2] * distance,
            vx: projection[0] * self.base.local_speed.speed,
            vy: projection[1] * self.base.local_speed.speed,
            vz: projection[2] * self.base.local_speed.speed,
        }
    }

    fn get_trajectory_type(&self) -> TrajectoryType { TrajectoryType::TrajTypeConstSpeed }
    fn get_start_pos_vel(&self) -> &KinematicInfo { &self.base.start_pos_vel }
    fn get_local_speed(&self) -> &LocalSpeed { &self.base.local_speed }
    fn get_convert_matrix(&self) -> &ConvertMatrix { &self.base.convert_matrix }
    fn get_time_span(&self) -> f64 { self.base.time_span }
    fn set_start_pos_vel(&mut self, pos_vel: KinematicInfo) { self.base.start_pos_vel = pos_vel; }
    fn set_local_speed(&mut self, speed: LocalSpeed) { self.base.local_speed = speed; }
    fn set_convert_matrix(&mut self, matrix: ConvertMatrix) { self.base.convert_matrix = matrix; }
    fn set_time_span(&mut self, time_span: f64) { self.base.time_span = time_span; }
}

// Constant acceleration trajectory
#[derive(Debug, Clone)]
pub struct TrajectoryConstAcc {
    pub base: TrajectorySegmentBase,
    pub ax: f64,
    pub ay: f64,
    pub az: f64,
}

impl TrajectoryConstAcc {
    pub fn new() -> Self {
        TrajectoryConstAcc {
            base: TrajectorySegmentBase::new(),
            ax: 0.0,
            ay: 0.0,
            az: 0.0,
        }
    }
}

impl TrajectorySegment for TrajectoryConstAcc {
    fn set_segment_param(
        &mut self,
        prev_segment: Option<&dyn TrajectorySegment>,
        data_type1: TrajectoryDataType,
        data1: f64,
        data_type2: TrajectoryDataType,
        data2: f64,
    ) -> i32 {
        self.base.init_segment(prev_segment);
        
        match (data_type1, data_type2) {
            (TrajectoryDataType::TrajDataTimeSpan, TrajectoryDataType::TrajDataAcceleration) |
            (TrajectoryDataType::TrajDataAcceleration, TrajectoryDataType::TrajDataTimeSpan) => {
                let (time_span, acceleration) = if matches!(data_type1, TrajectoryDataType::TrajDataTimeSpan) {
                    (data1, data2)
                } else {
                    (data2, data1)
                };
                
                if time_span <= 0.0 {
                    return TRAJECTORY_INVALID_PARAM;
                }
                if acceleration.abs() < 1e-5 {
                    return TRAJECTORY_ZERO_ACC;
                }
                
                self.base.time_span = time_span;
                let projection = self.base.get_speed_projection();
                self.ax = projection[0] * acceleration;
                self.ay = projection[1] * acceleration;
                self.az = projection[2] * acceleration;
                TRAJECTORY_NO_ERR
            },
            _ => TRAJECTORY_TYPE_MISMATCH
        }
    }

    fn get_pos_vel(&self, relative_time: f64) -> KinematicInfo {
        let t2 = relative_time * relative_time;
        
        KinematicInfo {
            x: self.base.start_pos_vel.x + self.base.start_pos_vel.vx * relative_time + 0.5 * self.ax * t2,
            y: self.base.start_pos_vel.y + self.base.start_pos_vel.vy * relative_time + 0.5 * self.ay * t2,
            z: self.base.start_pos_vel.z + self.base.start_pos_vel.vz * relative_time + 0.5 * self.az * t2,
            vx: self.base.start_pos_vel.vx + self.ax * relative_time,
            vy: self.base.start_pos_vel.vy + self.ay * relative_time,
            vz: self.base.start_pos_vel.vz + self.az * relative_time,
        }
    }

    fn get_trajectory_type(&self) -> TrajectoryType { TrajectoryType::TrajTypeConstAcc }
    fn get_start_pos_vel(&self) -> &KinematicInfo { &self.base.start_pos_vel }
    fn get_local_speed(&self) -> &LocalSpeed { &self.base.local_speed }
    fn get_convert_matrix(&self) -> &ConvertMatrix { &self.base.convert_matrix }
    fn get_time_span(&self) -> f64 { self.base.time_span }
    fn set_start_pos_vel(&mut self, pos_vel: KinematicInfo) { self.base.start_pos_vel = pos_vel; }
    fn set_local_speed(&mut self, speed: LocalSpeed) { self.base.local_speed = speed; }
    fn set_convert_matrix(&mut self, matrix: ConvertMatrix) { self.base.convert_matrix = matrix; }
    fn set_time_span(&mut self, time_span: f64) { self.base.time_span = time_span; }
}

// Vertical acceleration trajectory
#[derive(Debug, Clone)]
pub struct TrajectoryVerticalAcc {
    pub base: TrajectorySegmentBase,
    pub ax: f64,
    pub ay: f64,
    pub az: f64,
}

impl TrajectoryVerticalAcc {
    pub fn new() -> Self {
        TrajectoryVerticalAcc {
            base: TrajectorySegmentBase::new(),
            ax: 0.0,
            ay: 0.0,
            az: 0.0,
        }
    }
}

impl TrajectorySegment for TrajectoryVerticalAcc {
    fn set_segment_param(
        &mut self,
        prev_segment: Option<&dyn TrajectorySegment>,
        data_type1: TrajectoryDataType,
        data1: f64,
        data_type2: TrajectoryDataType,
        data2: f64,
    ) -> i32 {
        self.base.init_segment(prev_segment);
        
        match (data_type1, data_type2) {
            (TrajectoryDataType::TrajDataTimeSpan, TrajectoryDataType::TrajDataAcceleration) |
            (TrajectoryDataType::TrajDataAcceleration, TrajectoryDataType::TrajDataTimeSpan) => {
                let (time_span, acceleration) = if matches!(data_type1, TrajectoryDataType::TrajDataTimeSpan) {
                    (data1, data2)
                } else {
                    (data2, data1)
                };
                
                if time_span <= 0.0 {
                    return TRAJECTORY_INVALID_PARAM;
                }
                if acceleration.abs() < 1e-5 {
                    return TRAJECTORY_ZERO_ACC;
                }
                
                self.base.time_span = time_span;
                
                // Calculate vertical direction (up vector)
                let lla_pos = ecef_to_lla(&self.base.start_pos_vel);
                let up_vector = calc_up_vector(&lla_pos);
                
                self.ax = up_vector[0] * acceleration;
                self.ay = up_vector[1] * acceleration;
                self.az = up_vector[2] * acceleration;
                TRAJECTORY_NO_ERR
            },
            _ => TRAJECTORY_TYPE_MISMATCH
        }
    }

    fn get_pos_vel(&self, relative_time: f64) -> KinematicInfo {
        let t2 = relative_time * relative_time;
        
        KinematicInfo {
            x: self.base.start_pos_vel.x + self.base.start_pos_vel.vx * relative_time + 0.5 * self.ax * t2,
            y: self.base.start_pos_vel.y + self.base.start_pos_vel.vy * relative_time + 0.5 * self.ay * t2,
            z: self.base.start_pos_vel.z + self.base.start_pos_vel.vz * relative_time + 0.5 * self.az * t2,
            vx: self.base.start_pos_vel.vx + self.ax * relative_time,
            vy: self.base.start_pos_vel.vy + self.ay * relative_time,
            vz: self.base.start_pos_vel.vz + self.az * relative_time,
        }
    }

    fn get_trajectory_type(&self) -> TrajectoryType { TrajectoryType::TrajTypeVerticalAcc }
    fn get_start_pos_vel(&self) -> &KinematicInfo { &self.base.start_pos_vel }
    fn get_local_speed(&self) -> &LocalSpeed { &self.base.local_speed }
    fn get_convert_matrix(&self) -> &ConvertMatrix { &self.base.convert_matrix }
    fn get_time_span(&self) -> f64 { self.base.time_span }
    fn set_start_pos_vel(&mut self, pos_vel: KinematicInfo) { self.base.start_pos_vel = pos_vel; }
    fn set_local_speed(&mut self, speed: LocalSpeed) { self.base.local_speed = speed; }
    fn set_convert_matrix(&mut self, matrix: ConvertMatrix) { self.base.convert_matrix = matrix; }
    fn set_time_span(&mut self, time_span: f64) { self.base.time_span = time_span; }
}
// Jerk trajectory (constant acceleration rate)
#[derive(Debug, Clone)]
pub struct TrajectoryJerk {
    pub base: TrajectorySegmentBase,
    pub acc: KinematicInfo, // initial acceleration in x, y, z and acceleration rate in vx, vy, vz
}

impl TrajectoryJerk {
    pub fn new() -> Self {
        TrajectoryJerk {
            base: TrajectorySegmentBase::new(),
            acc: KinematicInfo::default(),
        }
    }
}

impl TrajectorySegment for TrajectoryJerk {
    fn set_segment_param(
        &mut self,
        prev_segment: Option<&dyn TrajectorySegment>,
        data_type1: TrajectoryDataType,
        data1: f64,
        data_type2: TrajectoryDataType,
        data2: f64,
    ) -> i32 {
        self.base.init_segment(prev_segment);
        
        match (data_type1, data_type2) {
            (TrajectoryDataType::TrajDataTimeSpan, TrajectoryDataType::TrajDataAccRate) |
            (TrajectoryDataType::TrajDataAccRate, TrajectoryDataType::TrajDataTimeSpan) => {
                let (time_span, acc_rate) = if matches!(data_type1, TrajectoryDataType::TrajDataTimeSpan) {
                    (data1, data2)
                } else {
                    (data2, data1)
                };
                
                if time_span <= 0.0 {
                    return TRAJECTORY_INVALID_PARAM;
                }
                if acc_rate.abs() < 1e-5 {
                    return TRAJECTORY_ZERO_ACCRATE;
                }
                
                self.base.time_span = time_span;
                let projection = self.base.get_speed_projection();
                
                // Initial acceleration is zero, acceleration rate is along velocity direction
                self.acc.x = 0.0;
                self.acc.y = 0.0;
                self.acc.z = 0.0;
                self.acc.vx = projection[0] * acc_rate;
                self.acc.vy = projection[1] * acc_rate;
                self.acc.vz = projection[2] * acc_rate;
                TRAJECTORY_NO_ERR
            },
            _ => TRAJECTORY_TYPE_MISMATCH
        }
    }

    fn get_pos_vel(&self, relative_time: f64) -> KinematicInfo {
        let t2 = relative_time * relative_time;
        let t3 = t2 * relative_time;
        
        KinematicInfo {
            x: self.base.start_pos_vel.x + self.base.start_pos_vel.vx * relative_time + 
               0.5 * self.acc.x * t2 + (1.0/6.0) * self.acc.vx * t3,
            y: self.base.start_pos_vel.y + self.base.start_pos_vel.vy * relative_time + 
               0.5 * self.acc.y * t2 + (1.0/6.0) * self.acc.vy * t3,
            z: self.base.start_pos_vel.z + self.base.start_pos_vel.vz * relative_time + 
               0.5 * self.acc.z * t2 + (1.0/6.0) * self.acc.vz * t3,
            vx: self.base.start_pos_vel.vx + self.acc.x * relative_time + 0.5 * self.acc.vx * t2,
            vy: self.base.start_pos_vel.vy + self.acc.y * relative_time + 0.5 * self.acc.vy * t2,
            vz: self.base.start_pos_vel.vz + self.acc.z * relative_time + 0.5 * self.acc.vz * t2,
        }
    }

    fn get_trajectory_type(&self) -> TrajectoryType { TrajectoryType::TrajTypeJerk }
    fn get_start_pos_vel(&self) -> &KinematicInfo { &self.base.start_pos_vel }
    fn get_local_speed(&self) -> &LocalSpeed { &self.base.local_speed }
    fn get_convert_matrix(&self) -> &ConvertMatrix { &self.base.convert_matrix }
    fn get_time_span(&self) -> f64 { self.base.time_span }
    fn set_start_pos_vel(&mut self, pos_vel: KinematicInfo) { self.base.start_pos_vel = pos_vel; }
    fn set_local_speed(&mut self, speed: LocalSpeed) { self.base.local_speed = speed; }
    fn set_convert_matrix(&mut self, matrix: ConvertMatrix) { self.base.convert_matrix = matrix; }
    fn set_time_span(&mut self, time_span: f64) { self.base.time_span = time_span; }
}

// Horizontal circular trajectory
#[derive(Debug, Clone)]
pub struct TrajectoryHorizontalCircular {
    pub base: TrajectorySegmentBase,
    pub angular_rate: f64,
}

impl TrajectoryHorizontalCircular {
    pub fn new() -> Self {
        TrajectoryHorizontalCircular {
            base: TrajectorySegmentBase::new(),
            angular_rate: 0.0,
        }
    }
}

impl TrajectorySegment for TrajectoryHorizontalCircular {
    fn set_segment_param(
        &mut self,
        prev_segment: Option<&dyn TrajectorySegment>,
        data_type1: TrajectoryDataType,
        data1: f64,
        data_type2: TrajectoryDataType,
        data2: f64,
    ) -> i32 {
        self.base.init_segment(prev_segment);
        
        match (data_type1, data_type2) {
            (TrajectoryDataType::TrajDataTimeSpan, TrajectoryDataType::TrajDataAngularRate) |
            (TrajectoryDataType::TrajDataAngularRate, TrajectoryDataType::TrajDataTimeSpan) => {
                let (time_span, angular_rate) = if matches!(data_type1, TrajectoryDataType::TrajDataTimeSpan) {
                    (data1, data2)
                } else {
                    (data2, data1)
                };
                
                if time_span <= 0.0 {
                    return TRAJECTORY_INVALID_PARAM;
                }
                if angular_rate.abs() < 1e-5 {
                    return TRAJECTORY_ZERO_DEGREE;
                }
                
                self.base.time_span = time_span;
                self.angular_rate = angular_rate * PI / 180.0; // Convert to radians
                TRAJECTORY_NO_ERR
            },
            (TrajectoryDataType::TrajDataTimeSpan, TrajectoryDataType::TrajDataRadius) |
            (TrajectoryDataType::TrajDataRadius, TrajectoryDataType::TrajDataTimeSpan) => {
                let (time_span, radius) = if matches!(data_type1, TrajectoryDataType::TrajDataTimeSpan) {
                    (data1, data2)
                } else {
                    (data2, data1)
                };
                
                if time_span <= 0.0 {
                    return TRAJECTORY_INVALID_PARAM;
                }
                if radius <= 0.0 {
                    return TRAJECTORY_NEGATIVE;
                }
                
                let speed = self.base.local_speed.speed;
                if speed < 1e-5 {
                    return TRAJECTORY_ZERO_SPEED;
                }
                
                self.base.time_span = time_span;
                self.angular_rate = speed / radius; // v = ωr, so ω = v/r
                TRAJECTORY_NO_ERR
            },
            _ => TRAJECTORY_TYPE_MISMATCH
        }
    }

    fn get_pos_vel(&self, relative_time: f64) -> KinematicInfo {
        let angle = self.angular_rate * relative_time;
        let cos_angle = angle.cos();
        let sin_angle = angle.sin();
        
        // Get horizontal velocity components (assuming z is up)
        let vh_x = self.base.start_pos_vel.vx;
        let vh_y = self.base.start_pos_vel.vy;
        let vh_z = self.base.start_pos_vel.vz;
        
        // Rotate velocity vector
        let new_vx = vh_x * cos_angle - vh_y * sin_angle;
        let new_vy = vh_x * sin_angle + vh_y * cos_angle;
        let new_vz = vh_z; // Vertical component unchanged
        
        // Calculate position change (integration of rotated velocity)
        let radius = self.base.local_speed.speed / self.angular_rate;
        let dx = radius * sin_angle;
        let dy = radius * (1.0 - cos_angle);
        
        KinematicInfo {
            x: self.base.start_pos_vel.x + dx,
            y: self.base.start_pos_vel.y + dy,
            z: self.base.start_pos_vel.z + new_vz * relative_time,
            vx: new_vx,
            vy: new_vy,
            vz: new_vz,
        }
    }

    fn get_trajectory_type(&self) -> TrajectoryType { TrajectoryType::TrajTypeHorizontalCircular }
    fn get_start_pos_vel(&self) -> &KinematicInfo { &self.base.start_pos_vel }
    fn get_local_speed(&self) -> &LocalSpeed { &self.base.local_speed }
    fn get_convert_matrix(&self) -> &ConvertMatrix { &self.base.convert_matrix }
    fn get_time_span(&self) -> f64 { self.base.time_span }
    fn set_start_pos_vel(&mut self, pos_vel: KinematicInfo) { self.base.start_pos_vel = pos_vel; }
    fn set_local_speed(&mut self, speed: LocalSpeed) { self.base.local_speed = speed; }
    fn set_convert_matrix(&mut self, matrix: ConvertMatrix) { self.base.convert_matrix = matrix; }
    fn set_time_span(&mut self, time_span: f64) { self.base.time_span = time_span; }
}

// Main trajectory class that manages trajectory segments
pub struct CTrajectory {
    init_pos_vel: KinematicInfo,
    init_local_speed: LocalSpeed,
    trajectory_list: Vec<Box<dyn TrajectorySegment>>,
    current_trajectory_index: usize,
    relative_time: f64,
    trajectory_name: String,
}

impl CTrajectory {
    pub fn new() -> Self {
        CTrajectory {
            init_pos_vel: KinematicInfo::default(),
            init_local_speed: LocalSpeed::default(),
            trajectory_list: Vec::new(),
            current_trajectory_index: 0,
            relative_time: 0.0,
            trajectory_name: String::new(),
        }
    }

    pub fn set_init_pos_vel(&mut self, init_pos_vel: KinematicInfo) {
        self.init_pos_vel = init_pos_vel;
    }

    pub fn set_init_pos_vel_lla(&mut self, init_position: LlaPosition, init_velocity: LocalSpeed, is_enu: bool) {
        self.init_pos_vel = lla_to_ecef(&init_position);
        self.init_local_speed = init_velocity;
        
        if !is_enu {
            // Convert course to ENU if needed
            let mut velocity = init_velocity;
            speed_course_to_enu(&mut velocity);
            self.init_local_speed = velocity;
        }
        
        // Convert local velocity to ECEF
        let convert_matrix = calc_conv_matrix_lla(&init_position);
        speed_local_to_ecef(&convert_matrix, &self.init_local_speed, &mut self.init_pos_vel);
    }

    pub fn clear_trajectory_list(&mut self) {
        self.trajectory_list.clear();
        self.current_trajectory_index = 0;
        self.relative_time = 0.0;
    }

    pub fn append_trajectory(
        &mut self,
        traj_type: TrajectoryType,
        data_type1: TrajectoryDataType,
        data1: f64,
        data_type2: TrajectoryDataType,
        data2: f64,
    ) -> i32 {
        let mut segment: Box<dyn TrajectorySegment> = match traj_type {
            TrajectoryType::TrajTypeConstSpeed => Box::new(TrajectoryConstSpeed::new()),
            TrajectoryType::TrajTypeConstAcc => Box::new(TrajectoryConstAcc::new()),
            TrajectoryType::TrajTypeVerticalAcc => Box::new(TrajectoryVerticalAcc::new()),
            TrajectoryType::TrajTypeJerk => Box::new(TrajectoryJerk::new()),
            TrajectoryType::TrajTypeHorizontalCircular => Box::new(TrajectoryHorizontalCircular::new()),
            _ => return TRAJECTORY_UNKNOWN_TYPE,
        };

        // Set initial conditions for first segment
        if self.trajectory_list.is_empty() {
            segment.set_start_pos_vel(self.init_pos_vel);
            segment.set_local_speed(self.init_local_speed);
            let lla_pos = ecef_to_lla(&self.init_pos_vel);
            let convert_matrix = calc_conv_matrix_lla(&lla_pos);
            segment.set_convert_matrix(convert_matrix);
        }

        let prev_segment = self.get_last_segment();
        let result = segment.set_segment_param(prev_segment, data_type1, data1, data_type2, data2);
        
        if result == TRAJECTORY_NO_ERR {
            self.trajectory_list.push(segment);
        }
        
        result
    }

    pub fn reset_trajectory_time(&mut self) {
        self.current_trajectory_index = 0;
        self.relative_time = 0.0;
    }

    pub fn get_next_pos_vel_ecef(&mut self, time_step: f64, _pos_vel: &mut KinematicInfo) -> bool {
        if self.trajectory_list.is_empty() {
            return false;
        }

        self.relative_time += time_step;

        // Find current trajectory segment
        while self.current_trajectory_index < self.trajectory_list.len() {
            let current_segment = &self.trajectory_list[self.current_trajectory_index];
            let segment_time_span = current_segment.get_time_span();
            
            if self.relative_time <= segment_time_span {
                *_pos_vel = current_segment.get_pos_vel(self.relative_time);
                return true;
            }
            
            // Move to next segment
            self.relative_time -= segment_time_span;
            self.current_trajectory_index += 1;
        }

        // Past all segments
        false
    }

    pub fn get_next_pos_vel_lla(&mut self, time_step: f64, position: &mut LlaPosition, velocity: &mut LocalSpeed) -> bool {
        let mut pos_vel = KinematicInfo::default();
        if self.get_next_pos_vel_ecef(time_step, &mut pos_vel) {
            *position = ecef_to_lla(&pos_vel);
            let convert_matrix = calc_conv_matrix_lla(position);
            speed_ecef_to_local(&convert_matrix, &pos_vel, velocity);
            speed_enu_to_course(velocity);
            true
        } else {
            false
        }
    }

    pub fn get_time_length(&self) -> f64 {
        self.trajectory_list.iter().map(|seg| seg.get_time_span()).sum()
    }

    pub fn set_trajectory_name(&mut self, name: &str) {
        self.trajectory_name = name.to_string();
    }

    pub fn get_trajectory_name(&self) -> &str {
        &self.trajectory_name
    }

    fn get_last_segment(&self) -> Option<&dyn TrajectorySegment> {
        self.trajectory_list.last().map(|seg| seg.as_ref())
    }
}

// Helper function to get trajectory type from trait object
pub fn get_trajectory_type(trajectory: &dyn TrajectorySegment) -> TrajectoryType {
    trajectory.get_trajectory_type()
}
