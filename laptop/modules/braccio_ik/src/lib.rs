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
fn braccio_ik(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Add API functions
    m.add_function(wrap_pyfunction!(init_ik, m)?)?;
    m.add_function(wrap_pyfunction!(set_target, m)?)?;
    m.add_function(wrap_pyfunction!(get_actuations, m)?)?;
    m.add_function(wrap_pyfunction!(get_num_iterations, m)?)?;
    m.add_function(wrap_pyfunction!(get_error, m)?)?;

    // Return
    Ok(())
}

// TYPES ================================

/// Wrapper for the `SerialRobot` struct
#[pyclass]
pub struct PySerialRobot {
    pub inner: SerialRobot,
}

// API FUNCTIONS ================================

/// Initializes and returns a serial robot struct to represent the Braccio
#[pyfunction]
fn init_ik() -> PySerialRobot {
    let mut braccio: SerialRobot = SerialRobot::new(5);

    // The base
    braccio.set_base(
        [0.0, 0.0, 3.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0]
    );

    // Base joint
    braccio.conf_joint(
        JointType::Pivot,
        68.0,
        true,
        0.0, // REJECT DEPTH (-90.0)
        0.0 // REJECT DEPTH (90.0)
    );
    braccio.next_joint();

    // Arm joint 1
    braccio.conf_joint(
        JointType::Hinge,
        123.0,
        true,
        -75.0,
        75.0
    );
    braccio.next_joint();

    // Arm joint 2
    braccio.conf_joint(
        JointType::Hinge,
        123.0,
        true,
        -90.0,
        90.0
    );
    braccio.next_joint();

    // Arm joint 3
    braccio.conf_joint(
        JointType::Hinge,
        62.0,
        true,
        -90.0,
        90.0
    );
    braccio.next_joint();

    // Wrist
    braccio.conf_joint(
        JointType::Pivot,
        125.0,
        true,
        -90.0,
        90.0
    );
    braccio.next_joint(); // not really necessary

    // Return the struct
    PySerialRobot {
        inner: braccio
    }
}

/// Sets the desired target
/// # Arguments
/// 
/// * `position` - Position of the target
/// * `direction` - Direction of the target
/// * `rotation` - Rotation (auxiliary axis) of the target
#[pyfunction]
fn set_target(wrapper: &mut PySerialRobot, position:[f32; 3], direction:[f32; 3], rotation:[f32; 3]) -> PyResult<bool> {
    Ok(wrapper.inner.set_target(position, direction, rotation))
}

/// Returns the actuation values needed to reach the target previously set
/// 
/// # Arguments
/// 
/// * `error` - Acceptable squared error
/// * `max_iterations` - Maximum number of iterations to run FABRIK
#[pyfunction]
fn get_actuations(wrapper: &mut PySerialRobot, error: f32, max_iterations: usize) -> PyResult<Vec<f32>> {
    // Get actuation values in terms of Fabrustik
    let mut actuations: Box<[f32]> = wrapper.inner.solve(error, max_iterations);

    // Map actuations to Braccio servo motor values
    actuations[0] = 90.0 - actuations[0]; // base [90 -> -90] => [0 -> 180]
    actuations[1] = 90.0 - actuations[1]; // joint1 [90 -> -90] => [0 -> 180]
    actuations[2] = 90.0 - actuations[2]; // joint2 [90 -> -90] => [0 -> 180]
    actuations[3] = 90.0 + actuations[3]; // joint3 [-90 -> 90] => [0 -> 180]
    actuations[4] = 90.0 - actuations[4]; // wrist [90 -> -90] => [0 -> 180]

    // Return values
    Ok(actuations.into_vec())
}

/// Returns the number of iterations used in the previous `get_actuations` call
#[pyfunction]
fn get_num_iterations(wrapper: &mut PySerialRobot) -> PyResult<usize> {
    Ok(wrapper.inner.get_num_iterations())
}

/// Returns the current squared error
#[pyfunction]
fn get_error(wrapper: &mut PySerialRobot) -> PyResult<f32> {
    Ok(wrapper.inner.get_error().to_num())
}
