//----------------------------------------------------------------------
// fastmath.rs:
//   Fast math functions for signal generation optimization
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::ComplexNumber;
use std::sync::Once;
use rand::Rng;

pub struct FastMath;

impl FastMath {
    // Lookup table size - must be power of 2 for fast modulo
    const TRIG_LUT_SIZE: usize = 65536;
    const TRIG_LUT_SCALE: f64 = Self::TRIG_LUT_SIZE as f64 / (2.0 * std::f64::consts::PI);
}

// Static lookup tables
static mut SIN_LUT: [f64; 65536] = [0.0; 65536];
static mut COS_LUT: [f64; 65536] = [0.0; 65536];
static INIT: Once = Once::new();

// Static variables for Gaussian noise generation
static mut HAS_SPARE: bool = false;
static mut SPARE: f64 = 0.0;

impl FastMath {
    // Initialize lookup tables
    fn initialize_lut() {
        INIT.call_once(|| {
            unsafe {
                for i in 0..65536 {
                    let angle = (2.0 * std::f64::consts::PI * i as f64) / 65536_f64;
                    SIN_LUT[i] = angle.sin();
                    COS_LUT[i] = angle.cos();
                }
            }
        });
    }

    // Fast sine using lookup table
    pub fn fast_sin(angle: f64) -> f64 {
        Self::initialize_lut();
        
        // Normalize angle to [0, 2*PI)
        let mut angle = angle % (2.0 * std::f64::consts::PI);
        if angle < 0.0 {
            angle += 2.0 * std::f64::consts::PI;
        }
        
        // Convert to table index
        let index = (angle * (65536.0 / (2.0 * std::f64::consts::PI))) as usize & (65536 - 1);
        unsafe { SIN_LUT[index] }
    }

    // Fast cosine using lookup table
    pub fn fast_cos(angle: f64) -> f64 {
        Self::initialize_lut();
        
        // Normalize angle to [0, 2*PI)
        let mut angle = angle % (2.0 * std::f64::consts::PI);
        if angle < 0.0 {
            angle += 2.0 * std::f64::consts::PI;
        }
        
        // Convert to table index
        let index = (angle * (65536.0 / (2.0 * std::f64::consts::PI))) as usize & (65536 - 1);
        unsafe { COS_LUT[index] }
    }

    // Fast complex rotation using lookup tables
    pub fn fast_rotate(angle: f64) -> ComplexNumber {
        Self::initialize_lut();
        
        // Normalize angle to [0, 2*PI)
        let mut angle = angle % (2.0 * std::f64::consts::PI);
        if angle < 0.0 {
            angle += 2.0 * std::f64::consts::PI;
        }
        
        // Convert to table index
        let index = (angle * (65536.0 / (2.0 * std::f64::consts::PI))) as usize & (65536 - 1);
        unsafe { ComplexNumber { real: COS_LUT[index], imag: SIN_LUT[index] } }
    }

    // Fast noise generation using Box-Muller with cached values
    pub fn fast_gaussian_noise(sigma: f64) -> ComplexNumber {
        unsafe {
            if HAS_SPARE {
                HAS_SPARE = false;
                return ComplexNumber { real: SPARE * sigma, imag: 0.0 };
            }
        }

        let mut rng = rand::thread_rng();
        let (u1, u2, mag) = loop {
            let u1 = 2.0 * rng.gen::<f64>() - 1.0;
            let u2 = 2.0 * rng.gen::<f64>() - 1.0;
            let mag = u1 * u1 + u2 * u2;
            if mag < 1.0 && mag != 0.0 {
                break (u1, u2, mag);
            }
        };

        let factor = (-2.0 * mag.ln() / mag).sqrt();
        
        unsafe {
            SPARE = u2 * factor;
            HAS_SPARE = true;
        }

        ComplexNumber { real: u1 * factor * sigma, imag: u2 * factor * sigma }
    }

    // Batch noise generation for better cache efficiency
    pub fn generate_noise_block(output: &mut [ComplexNumber], sigma: f64) {
        // OPTIMIZED: Generate noise in batches for better cache locality
        const BATCH_SIZE: usize = 64; // Process in batches for better performance
        
        let mut i = 0;
        while i < output.len() {
            let end = (i + BATCH_SIZE).min(output.len());
            for j in i..end {
                output[j] = Self::fast_gaussian_noise(sigma);
            }
            i = end;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_fast_sin() {
        let angle = PI / 4.0;
        let fast_result = FastMath::fast_sin(angle);
        let std_result = angle.sin();
        assert!((fast_result - std_result).abs() < 1e-4);
    }

    #[test]
    fn test_fast_cos() {
        let angle = PI / 3.0;
        let fast_result = FastMath::fast_cos(angle);
        let std_result = angle.cos();
        assert!((fast_result - std_result).abs() < 1e-4);
    }

    #[test]
    fn test_fast_rotate() {
        let angle = PI / 2.0;
        let result = FastMath::fast_rotate(angle);
        assert!((result.real - 0.0).abs() < 1e-4);
        assert!((result.imag - 1.0).abs() < 1e-4);
    }

    #[test]
    fn test_gaussian_noise() {
        let sigma = 1.0;
        let noise = FastMath::fast_gaussian_noise(sigma);
        // Just check that we get some values (statistical test would be more complex)
        assert!(noise.real.is_finite());
        assert!(noise.imag.is_finite());
    }

    #[test]
    fn test_noise_block() {
        let mut output = vec![ComplexNumber::new(); 100];
        FastMath::generate_noise_block(&mut output, 1.0);
        
        // Check that all values are finite
        for sample in &output {
            assert!(sample.real.is_finite());
            assert!(sample.imag.is_finite());
        }
    }
}