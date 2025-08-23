//----------------------------------------------------------------------
// complex_number.rs:
//   Complex number operator implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ComplexNumber {
    pub real: f64,
    pub imag: f64,
}

impl ComplexNumber {
    /// Create a new complex number with zero values
    pub fn new() -> Self {
        ComplexNumber {
            real: 0.0,
            imag: 0.0,
        }
    }

    /// Create a new complex number with specified real and imaginary parts
    pub fn from_parts(real_part: f64, imag_part: f64) -> Self {
        ComplexNumber {
            real: real_part,
            imag: imag_part,
        }
    }

    /// Calculate the absolute value (magnitude) of the complex number
    pub fn abs(&self) -> f64 {
        (self.real * self.real + self.imag * self.imag).sqrt()
    }

    /// Calculate the complex conjugate
    pub fn conj(&self) -> ComplexNumber {
        ComplexNumber {
            real: self.real,
            imag: -self.imag,
        }
    }
}

// Default implementation
impl Default for ComplexNumber {
    fn default() -> Self {
        Self::new()
    }
}

// Addition operator for ComplexNumber + ComplexNumber
impl Add for ComplexNumber {
    type Output = ComplexNumber;

    fn add(self, other: ComplexNumber) -> ComplexNumber {
        ComplexNumber {
            real: self.real + other.real,
            imag: self.imag + other.imag,
        }
    }
}

// Addition assignment operator for ComplexNumber += ComplexNumber
impl AddAssign for ComplexNumber {
    fn add_assign(&mut self, other: ComplexNumber) {
        self.real += other.real;
        self.imag += other.imag;
    }
}

// Subtraction operator for ComplexNumber - ComplexNumber
impl Sub for ComplexNumber {
    type Output = ComplexNumber;

    fn sub(self, other: ComplexNumber) -> ComplexNumber {
        ComplexNumber {
            real: self.real - other.real,
            imag: self.imag - other.imag,
        }
    }
}

// Subtraction assignment operator for ComplexNumber -= ComplexNumber
impl SubAssign for ComplexNumber {
    fn sub_assign(&mut self, other: ComplexNumber) {
        self.real -= other.real;
        self.imag -= other.imag;
    }
}

// Multiplication operator for ComplexNumber * ComplexNumber
impl Mul for ComplexNumber {
    type Output = ComplexNumber;

    fn mul(self, other: ComplexNumber) -> ComplexNumber {
        ComplexNumber {
            real: self.real * other.real - self.imag * other.imag,
            imag: self.real * other.imag + self.imag * other.real,
        }
    }
}

// Multiplication assignment operator for ComplexNumber *= ComplexNumber
impl MulAssign for ComplexNumber {
    fn mul_assign(&mut self, other: ComplexNumber) {
        let temp_real = self.real;
        let temp_imag = self.imag;
        
        self.real = temp_real * other.real - temp_imag * other.imag;
        self.imag = temp_real * other.imag + temp_imag * other.real;
    }
}

// Multiplication operator for ComplexNumber * f64
impl Mul<f64> for ComplexNumber {
    type Output = ComplexNumber;

    fn mul(self, scalar: f64) -> ComplexNumber {
        ComplexNumber {
            real: self.real * scalar,
            imag: self.imag * scalar,
        }
    }
}

// Multiplication assignment operator for ComplexNumber *= f64
impl MulAssign<f64> for ComplexNumber {
    fn mul_assign(&mut self, scalar: f64) {
        self.real *= scalar;
        self.imag *= scalar;
    }
}

// Multiplication operator for f64 * ComplexNumber
impl Mul<ComplexNumber> for f64 {
    type Output = ComplexNumber;

    fn mul(self, complex: ComplexNumber) -> ComplexNumber {
        ComplexNumber {
            real: self * complex.real,
            imag: self * complex.imag,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let c = ComplexNumber::new();
        assert_eq!(c.real, 0.0);
        assert_eq!(c.imag, 0.0);
    }

    #[test]
    fn test_from_parts() {
        let c = ComplexNumber::from_parts(3.0, 4.0);
        assert_eq!(c.real, 3.0);
        assert_eq!(c.imag, 4.0);
    }

    #[test]
    fn test_abs() {
        let c = ComplexNumber::from_parts(3.0, 4.0);
        assert_eq!(c.abs(), 5.0);
    }

    #[test]
    fn test_conj() {
        let c = ComplexNumber::from_parts(3.0, 4.0);
        let conj = c.conj();
        assert_eq!(conj.real, 3.0);
        assert_eq!(conj.imag, -4.0);
    }

    #[test]
    fn test_addition() {
        let c1 = ComplexNumber::from_parts(1.0, 2.0);
        let c2 = ComplexNumber::from_parts(3.0, 4.0);
        let result = c1 + c2;
        assert_eq!(result.real, 4.0);
        assert_eq!(result.imag, 6.0);
    }

    #[test]
    fn test_addition_assign() {
        let mut c1 = ComplexNumber::from_parts(1.0, 2.0);
        let c2 = ComplexNumber::from_parts(3.0, 4.0);
        c1 += c2;
        assert_eq!(c1.real, 4.0);
        assert_eq!(c1.imag, 6.0);
    }

    #[test]
    fn test_subtraction() {
        let c1 = ComplexNumber::from_parts(5.0, 7.0);
        let c2 = ComplexNumber::from_parts(2.0, 3.0);
        let result = c1 - c2;
        assert_eq!(result.real, 3.0);
        assert_eq!(result.imag, 4.0);
    }

    #[test]
    fn test_subtraction_assign() {
        let mut c1 = ComplexNumber::from_parts(5.0, 7.0);
        let c2 = ComplexNumber::from_parts(2.0, 3.0);
        c1 -= c2;
        assert_eq!(c1.real, 3.0);
        assert_eq!(c1.imag, 4.0);
    }

    #[test]
    fn test_multiplication() {
        let c1 = ComplexNumber::from_parts(1.0, 2.0);
        let c2 = ComplexNumber::from_parts(3.0, 4.0);
        let result = c1 * c2;
        // (1 + 2i) * (3 + 4i) = 3 + 4i + 6i + 8i^2 = 3 + 10i - 8 = -5 + 10i
        assert_eq!(result.real, -5.0);
        assert_eq!(result.imag, 10.0);
    }

    #[test]
    fn test_multiplication_assign() {
        let mut c1 = ComplexNumber::from_parts(1.0, 2.0);
        let c2 = ComplexNumber::from_parts(3.0, 4.0);
        c1 *= c2;
        assert_eq!(c1.real, -5.0);
        assert_eq!(c1.imag, 10.0);
    }

    #[test]
    fn test_scalar_multiplication() {
        let c = ComplexNumber::from_parts(2.0, 3.0);
        let result = c * 2.0;
        assert_eq!(result.real, 4.0);
        assert_eq!(result.imag, 6.0);

        let result2 = 2.0 * c;
        assert_eq!(result2.real, 4.0);
        assert_eq!(result2.imag, 6.0);
    }

    #[test]
    fn test_scalar_multiplication_assign() {
        let mut c = ComplexNumber::from_parts(2.0, 3.0);
        c *= 2.0;
        assert_eq!(c.real, 4.0);
        assert_eq!(c.imag, 6.0);
    }
}