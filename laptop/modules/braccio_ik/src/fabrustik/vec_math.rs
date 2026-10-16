//! # Module to provide fixed-point vector arithmetic
//! Author: Neo Vorsatz
//! Last updated: 13 October

// DEPENDENCIES ================================

// External
use fixed; // fixed-point arithmetic
use fixed::traits::ToFixed; // ToFixed trait
// use fixed_sqrt::FixedSqrt; // square root trait
use simple_vectors::Vector; // vector arithmetic
use std::ops::{Add, Sub, Mul, Div, Neg}; // traits for the + and - operators
use cordic; // trigonometric functions

// TYPES ================================

/// Chosen fractional data type
pub type Scalar = fixed::FixedI64<fixed::types::extra::U16>;

/// Arbitrary 3D vector
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Vec3 {
    inner: Vector<Scalar, 3>,
}

// CONSTANTS ================================

// Conversion ratios between radians and degrees
const DEG_TO_RAD: Scalar = Scalar::lit("0.01745329252"); // pi/180
const RAD_TO_DEG: Scalar = Scalar::lit("57.29577951"); // 180/pi
pub const ZERO: Scalar = Scalar::ZERO; // zero

// CONSTRUCTOR IMPLEMENTATION ================================

impl Vec3 {
    pub fn from_coord(coords: [impl ToFixed+Copy; 3]) -> Self {
        Self {
            inner: Vector::new([
                Scalar::from_num(coords[0]),
                Scalar::from_num(coords[1]),
                Scalar::from_num(coords[2]),
            ])
        }
    }
}

// API FUNCTIONS ================================

impl Add for Vec3 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            inner: self.inner + other.inner,
        }
    }
}

impl Sub for Vec3 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            inner: self.inner - other.inner,
        }
    }
}

impl Mul<Scalar> for Vec3 {
    type Output = Vec3;

    fn mul(self, rhs: Scalar) -> Self {
        Self {
            inner: self.inner * rhs,
        }
    }
}

impl Div<Scalar> for Vec3 {
    type Output = Vec3;

    fn div(self, rhs: Scalar) -> Self {
        if rhs == ZERO {
            panic!("Cannot divide Vec3 by zero");
        }
        Self {
            inner: self.inner / rhs,
        }
    }
}

impl Neg for Vec3 {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            inner: -self.inner,
        }
    }
}

impl Vec3 {
    /// Returns an array of the elements
    fn to_array(self) -> [Scalar; 3] {
        self.inner.into()
    }

    /// Dot Product: Returns a scalar value representing the alignment of two vectors
    pub fn dot(self, other: Self) -> Scalar {
        // Extract components
        let [x1, y1, z1] = self.to_array();
        let [x2, y2, z2] = other.to_array();
        
        // Perform dot product
        (x1*x2) + (y1*y2) + (z1*z2)
    }

    /// Cross Product: Returns a new vector perpendicular to both vectors
    pub fn cross(self, other: Self) -> Self {
        // Extract components
        let [x1, y1, z1] = self.to_array();
        let [x2, y2, z2] = other.to_array();

        // Perform cross product
        Self::from_coord([
            (y1 * z2) - (z1 * y2),
            (z1 * x2) - (x1 * z2),
            (x1 * y2) - (y1 * x2),
        ])
    }

    /// Calculates the sum of squared components of the vector
    pub fn sum_sqr(self) -> Scalar {
        self.dot(self)
    }

    /// Calculates the magnitude (length) of the vector
    pub fn mag(self) -> Scalar {
        let sum_of_squares = self.sum_sqr();
        sum_of_squares.sqrt()
    }
}

/// Returns the sign of a scalar, defaulting to 1 if the scalar is 0
pub fn sgn(value: Scalar) -> Scalar {
    // Get sign
    let mut sign: Scalar = value.signum();

    // Check for zero
    if sign == ZERO {
        sign = Scalar::from_num(1); // change to 1
    }

    // Return
    sign
}

/// Returns the cosine of an angle in degrees
pub fn cos(angle: Scalar) -> Scalar {
    // Perform cosine
    cordic::cos(angle*DEG_TO_RAD)
}

/// Returns the sine of an angle in degrees
pub fn sin(angle: Scalar) -> Scalar{
    // Perform sine
    cordic::sin(angle*DEG_TO_RAD)
}

/// Returns the arccosine (in degrees) of a value
pub fn acos(value: Scalar) -> Scalar {
    // Set boundaries
    let max: Scalar = Scalar::from_num(1);
    let min: Scalar = Scalar::from_num(-1);

    // Clamp the input
    let clamped_value: Scalar = if value > max {
        max
    } else if value < min {
        min
    } else {
        value
    };

    // Perform arccosine
    cordic::acos(clamped_value)*RAD_TO_DEG
}

// HELPER FUNCTIONS ================================

// TESTS ================================

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq; // testing for approximate equality

    const TOL_PROP: f32 = 1e-3; // tolerance for proportions
    const TOL_DEG: f32 = 1e-0; // tolerance for degrees

    #[test]
    fn test_dot() {
        let a = Vec3::from_coord([-1, 1, 3]);
        let b = Vec3::from_coord([-3, 2, 0]);
        assert_eq!(a.dot(b).to_num::<f32>(), 5.0f32);
    }

    #[test]
    fn test_cross() {
        let a = Vec3::from_coord([-1, 1, 3]);
        let b = Vec3::from_coord([-3, 2, 0]);
        let expected: Vec3 = Vec3::from_coord([-6, -9, 1]);
        assert_eq!(a.cross(b), expected);
    }

    #[test]
    fn test_sum_sqr() {
        let v = Vec3::from_coord([-1, 1, 3]);
        assert_eq!(v.sum_sqr().to_num::<f32>(), 11.0f32);
    }

    #[test]
    fn test_mag() {
        let v = Vec3::from_coord([0, 3, 4]);
        assert_eq!(v.mag().to_num::<f32>(), 5.0f32);
    }

    #[test]
    fn test_sgn() {
        // sgn(2) = 1
        assert_approx_eq!(sgn(Scalar::from_num(2)).to_num::<f32>(), 1.0f32, TOL_PROP);
        // sgn(-5) = -1
        assert_approx_eq!(sgn(Scalar::from_num(-5)).to_num::<f32>(), -1.0f32, TOL_PROP);
        // sgn(0) = 1
        assert_approx_eq!(sgn(Scalar::from_num(0)).to_num::<f32>(), 1.0f32, TOL_PROP);
    }

    #[test]
    fn test_cos() {
        // cos(0) = 1
        assert_approx_eq!(cos(Scalar::from_num(0)).to_num::<f32>(), 1.0f32, TOL_PROP);
        // cos(90) = 0
        assert_approx_eq!(cos(Scalar::from_num(90)).to_num::<f32>(), 0.0f32, TOL_PROP);
        // cos(180) = -1
        assert_approx_eq!(cos(Scalar::from_num(180)).to_num::<f32>(), -1.0f32, TOL_PROP);
        // cos(-60) = 0.5
        assert_approx_eq!(cos(Scalar::from_num(-60)).to_num::<f32>(), 0.5f32, TOL_PROP);
    }

    #[test]
    fn test_sin() {
        // sin(0) = 0
        assert_approx_eq!(sin(Scalar::from_num(0)).to_num::<f32>(), 0.0f32, TOL_PROP);
        // sin(90) = 1
        assert_approx_eq!(sin(Scalar::from_num(90)).to_num::<f32>(), 1.0f32, TOL_PROP);
        // sin(180) = 0
        assert_approx_eq!(sin(Scalar::from_num(180)).to_num::<f32>(), 0.0f32, TOL_PROP);
        // sin(-30) = -0.5
        assert_approx_eq!(sin(Scalar::from_num(-30)).to_num::<f32>(), -0.5f32, TOL_PROP);
    }

    #[test]
    fn test_acos() {
        // acos(1) = 0
        assert_approx_eq!(acos(Scalar::from_num(1)).to_num::<f32>(), 0.0f32, TOL_DEG);
        // acos(0) = 90
        assert_approx_eq!(acos(Scalar::from_num(0)).to_num::<f32>(), 90.0f32, TOL_DEG);
        // acos(-1) = 180
        assert_approx_eq!(acos(Scalar::from_num(-1)).to_num::<f32>(), 180.0f32, TOL_DEG);
        // acos(0.5) = 60
        assert_approx_eq!(acos(Scalar::from_num(0.5)).to_num::<f32>(), 60.0f32, TOL_DEG);
    }
}
