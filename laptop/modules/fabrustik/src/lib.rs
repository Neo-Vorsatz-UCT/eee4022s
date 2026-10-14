#![doc = include_str!("../README.md")]
//! 
//! Author: Neo Vorsatz
//! Last updated: 13 October

// DEPENDENCIES ================================

// External
use pyo3::prelude::*;

// Internal
mod fabrustik;
use fabrustik::*; // inverse kinematics solver

// PYTHON MODULE ================================

/// The Python module to solve the Braccio's inverse kinematics
#[pymodule]
mod braccio_ik {
    use pyo3::prelude::*;

    /// Formats the sum of two numbers as string.
    #[pyfunction]
    fn sum_as_string(a: usize, b: usize) -> PyResult<String> {
        Ok((a + b).to_string())
    }

    
}

// TYPES ================================

/// Wrapper for the `SerialRobot` struct
#[pyclass]
pub struct PySerialRobot {
    pub inner: SerialRobot,
}

// API FUNCTIONS ================================

// Initialize braccio
// Set target
// Get actuations


/// Initializes a serial robot struct to represent the Braccio
#[pyfunction]
fn init_braccio() -> PySerialRobot {
    let mut braccio: SerialRobot = SerialRobot::new(5);

    // The base
    braccio.set_base(
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0]
    );

    // Base joint
    braccio.conf_joint(
        JointType::Pivot,
        1.0, // TODO: get actual link lengths, for all joints
        true,
        -90.0,
        90.0
    );
    braccio.next_joint();

    // Arm joint 1
    braccio.conf_joint(
        JointType::Hinge,
        1.0,
        true,
        -90.0, // TODO: constraints a little tighter
        90.0
    );
    braccio.next_joint();

    // Arm joint 2
    braccio.conf_joint(
        JointType::Hinge,
        1.0,
        true,
        -90.0,
        90.0
    );
    braccio.next_joint();

    // Arm joint 3
    braccio.conf_joint(
        JointType::Hinge,
        1.0,
        true,
        -90.0, // TODO: constraints a little tighter
        90.0
    );
    braccio.next_joint();

    // Wrist
    braccio.conf_joint(
        JointType::Pivot,
        1.0,
        true,
        -90.0, // TODO: constraints a little tighter
        90.0
    );
    braccio.next_joint(); // not really necessary

    // Return the struct
    PySerialRobot {
        inner: braccio
    }
}

// Sets the target for the inverse kinematics
// #[pyfunction]
// fn set_target(wrapper: &mut PySerialRobot) {
//     let braccio: &mut SerialRobot = &mut wrapper.inner;

// }