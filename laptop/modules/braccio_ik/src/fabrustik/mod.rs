//! # Module to solve inverse kinematics using amodified FABRIK algorithm
//! Author: Neo Vorsatz
//! Last updated: 14 October

// DEPENDENCIES ================================

// Internal
mod vec_math;
use vec_math::{Scalar, Vec3, ZERO}; // vector data structures
use vec_math::{sgn, cos, sin, acos}; // trigonometric functions

// TYPES ================================

/// The different types of joints
#[derive(Debug, Clone)]
pub enum JointType {
    Immovable, // used for the world frame and target
    Hinge, // rotates about an axis perpendicular to the link
    Pivot, // rotates about the axis of the link
    Prismatic, // translates along the axis of the link
}

/// Single joint of a robotic manipulator
#[derive(Debug, Clone)]
struct Joint {
    // Kinematics
    pub joint_type: JointType, // describes the way the joint moves
    pub link_len: Scalar, // distance from this joint to the next joint

    // Position and orientation
    pub position: Vec3, // absolute position
    pub axis_link: Vec3, // z-axis, normalised
    pub axis_aux: Vec3, // x-axis, normalised

    // Movement
    pub value: Scalar, // the value of the joint's movement, such as the hinge angle
    pub limited: bool, // whether the joint has a limited range of motion or not
    pub min: Scalar, // minimum angle of rotation or length of extension
    pub max: Scalar, // maximum angle of rotation or length of extension
}

/// A serial robotic manipulator
#[derive(Debug, Clone)]
pub struct SerialRobot {
    edit_index: usize, // index of joint being edited
    joints: Box<[Joint]>, // joints of the robot
    prev_iterations: usize, // number of iterations of previous solution
}

// CONSTRUCTORS ================================

impl Joint {
    /// Creates a new instance.
    /// This instance is immovable, positioned at the origin, and doesn't displace subsequent joints
    pub fn new() -> Self {
        Self {
            // Kinematics
            joint_type: JointType::Immovable,
            link_len: Scalar::from_num(0),

            // Position and orientation
            position: Vec3::from_coord([0, 0, 0]),
            axis_link: Vec3::from_coord([0, 0, 1]),
            axis_aux: Vec3::from_coord([1, 0, 0]),

            // Movement
            value: Scalar::from_num(0),
            limited: true,
            min: Scalar::from_num(0),
            max: Scalar::from_num(0),
        }
    }
}

impl SerialRobot {
    /// Creates a new instance
    /// 
    /// # Arguments
    /// 
    /// * `num_joints` - Number of joints in the robot
    pub fn new(num_joints: usize) -> Self {
        // Generate joints
        let mut joints: Vec<Joint> = Vec::with_capacity(num_joints+3);
        for _ in 0..(num_joints+3) {
            joints.push(Joint::new());
        }

        // Configure the end-effector
        joints[num_joints+1].joint_type = JointType::Prismatic;
        joints[num_joints+1].link_len = ZERO;
        joints[num_joints+1].limited = true;
        joints[num_joints+1].min = ZERO;
        joints[num_joints+1].max = ZERO;

        // Return instance
        Self {
            edit_index: 0,
            joints: joints.into_boxed_slice(),
            prev_iterations: 0,
        }
    }
}

// API FUNCTIONS ================================

impl SerialRobot {
    /// Sets the edit-index to a particular joint
    /// 
    /// # Arguments
    /// 
    /// * `index` - Index of the joint to select
    pub fn select_joint(&mut self, index: usize) {
        // Validate the index
        if index >= self.joints.len()-3 {
            panic!("Joint index out of bounds");
        }

        // Set the index
        self.edit_index = index;
    }

    /// Moves the edit-index to the next joint
    pub fn next_joint(&mut self) {
        self.edit_index = (self.edit_index + 1) % (self.joints.len() - 3);
    }

    /// Sets the joint's configuration
    /// 
    /// # Arguments
    /// 
    /// * `joint_type` - Type of joint
    /// * `link_len` - Distance from this link to the next, or to the end-effector
    /// * `limited` - Whether this joint has a limited range of motion or not
    /// * `min` - Minimum value of the range of motion (recommended to be 0.0 if not limited)
    /// * `max` - Maximum value of the range of motion (recommended to be 0.0 if not limited)
    pub fn conf_joint(&mut self, joint_type: JointType, link_len: f32, limited: bool, min: f32, max: f32) {
        // Get mutable reference to the joint
        let joint = &mut self.joints[self.edit_index+1];

        // Configure the joint
        joint.joint_type = joint_type;
        joint.link_len = Scalar::from_num(link_len);
        joint.limited = limited;
        joint.min = Scalar::from_num(min);
        joint.max = Scalar::from_num(max);
    }

    /// Sets the base's position and orientation,
    /// which forces the position of the first joint
    /// 
    /// # Arguments
    /// 
    /// * `position` - Position of the base
    /// * `direction` - Direction of the base
    /// * `rotation` - Rotation (auxilliary axis) of the base
    pub fn set_base(&mut self, position: [f32; 3], direction: [f32; 3], rotation: [f32; 3]) {
        // Get mutable reference to the base joint
        let base = &mut self.joints[0]; // first element of joints

        // Set position and orientation
        base.set_pos(position, direction, rotation);
    }

    /// Sets the desired target
    /// 
    /// # Arguments
    /// 
    /// * `position` - Position of the target
    /// * `direction` - Direction of the target
    /// * `rotation` - Rotation (auxilliary axis) of the target
    pub fn set_target(&mut self, position: [f32; 3], direction: [f32; 3], rotation: [f32; 3]) {
        // Get mutable reference to the target joint
        let target = &mut self.joints[self.joints.len()-1]; // last element of joints

        // Set position and orientation
        target.set_pos(position, direction, rotation);
    }

    /// Returns the squared error of the end effector compared to the target
    pub fn get_error(&mut self) -> Scalar {
        // Initialize
        let n: usize = self.joints.len(); // number of joints
        let mut diff: Vec3; // vector describing the difference between target and end-effector
        let mut error: Scalar = ZERO; // squared error

        // Add position error
        diff = self.joints[n-1].position - self.joints[n-2].position; // difference between target and end-effector position
        error += diff.sum_sqr();

        // Add direction error
        diff = self.joints[n-1].axis_link - self.joints[n-2].axis_link; // difference between target and end-effector direction
        error += diff.sum_sqr();

        // Add rotation error
        diff = self.joints[n-1].axis_aux - self.joints[n-2].axis_aux; // difference between target and end-effector rotation
        error += diff.sum_sqr();

        // Return
        error
    }

    /// Returns the actuation values needed to reach the target previously set
    /// 
    /// # Arguments
    /// 
    /// * `error` - Acceptable squared error
    /// * `max_iterations` - Maximum number of iterations to run FABRIK
    pub fn solve(&mut self, error: f32, max_iterations: usize) -> Box<[f32]> {
        // Get the number of joints
        let n: usize = self.joints.len();

        // Calculate initial error
        let mut err: Scalar = self.get_error();
        
        // Loop until error reduced or iterations maxed
        let mut i = 0;
        while i < max_iterations && err > error {
            // Forward reaching
            for j in (1..(n-1)).rev() {
                match self.joints[j+1].joint_type {
                    JointType::Immovable => self.forward_immovable(j),
                    JointType::Hinge => self.forward_hinge(j),
                    JointType::Pivot => self.forward_pivot(j),
                    JointType::Prismatic => self.forward_prismatic(j),
                    _ => panic!("Unknown joint type"),
                }
            }

            // Backward reaching
            for j in 1..(n-1) {
                match self.joints[j-1].joint_type {
                    JointType::Immovable => self.backward_immovable(j),
                    JointType::Hinge => self.backward_hinge(j),
                    JointType::Pivot => self.backward_pivot(j),
                    JointType::Prismatic => self.backward_prismatic(j),
                    _ => panic!("Unknown joint type"),
                }
            }

            // End iteration
            err = self.get_error();
            i += 1;
        }

        // Record number of iterations
        self.prev_iterations = i;

        // Return actuation values
        let mut actuations: Box<[f32]> = vec![0.0; n-3].into_boxed_slice();
        for j in 1..(n-2) {
            actuations[j-1] = self.joints[j].value.to_num();
        }
        actuations // return
    }

    /// Returns the number of iterations from the previous call to `solve()`
    pub fn get_num_iterations(&mut self) -> usize {
        self.prev_iterations
    }
}

// HELPER FUNCTIONS ================================

impl Joint {
    /// Sets the position and orientation of a joint,
    /// assuming untrusted arguments
    /// 
    /// # Arguments
    /// 
    /// * `position` - Position of the joint
    /// * `direction` - Direction of the joint
    /// * `rotation` - Rotation (auxilliary axis) of the joint
    pub fn set_pos(&mut self, position: [f32; 3], direction: [f32; 3], rotation: [f32; 3]) {
        // Set position
        self.position = Vec3::from_coord(position);

        // Set direction
        let mut dir: Vec3 = Vec3::from_coord(direction);
        let mag_dir: Scalar = dir.mag();
        if mag_dir == ZERO {
            // panic!("Direction must be non-zero vector");
            dir = Vec3::from_coord([0, 0, 1]); // choose default direction
            self.axis_link = dir;
        } else {
            dir = dir / mag_dir; // normalize
            self.axis_link = dir;
        }

        // Set rotation
        let mut rot: Vec3 = Vec3::from_coord(rotation);
        let mut mag_rot: Scalar = rot.mag();
        if mag_rot == ZERO {
            // panic!("Rotation must be non-zero vector");
            rot = Vec3::from_coord([1, 0, 0]); // choose default rotation
        } else {
            rot  = rot / mag_rot; // normalize, in case rot is nearly zero
        }
        let mut y: Vec3 = dir.cross(rot); // calculate y-axis
        let mag_y: Scalar = y.mag();
        if mag_y == ZERO { // if y-axis is zero, then rotation is parallel to direction
            y = if dir.dot(Vec3::from_coord([0, 1,0 ])).abs() < Scalar::from_num(0.9) {
                // if `dir` isn't parallel to the y-axis, then we can use the y-axis
                Vec3::from_coord([0, 1, 0])
            } else {
                Vec3::from_coord([1, 0, 0]) // use the x-axis
            }
        }
        rot = y.cross(dir); // calculate new rotation vector that's guarenteed to be perpendicular
        mag_rot = rot.mag();
        self.axis_aux = rot / mag_rot;
    }
}

impl SerialRobot {
    /// Moves a joint to a position and orientation,
    /// under the immovable-constraints of another joint,
    /// during the forward-reaching phase of FABRIK
    /// 
    /// # Arguments
    /// 
    /// * `j` - Index of the Joint being moved
    fn forward_immovable(&mut self, j: usize) {
        self.forward_prismatic(j);
    }

    /// Moves a joint to a position and orientation,
    /// under the immovable-constraints of another joint,
    /// during the forward-reaching phase of FABRIK
    /// 
    /// # Arguments
    /// 
    /// * `j` - Index of the Joint being moved
    fn backward_immovable(&mut self, j: usize) {
        self.backward_prismatic(j);
    }

    /// Moves a joint to a position and orientation,
    /// under the hinge-constraints of another joint,
    /// during the forward-reaching phase of FABRIK
    /// 
    /// # Arguments
    /// 
    /// * `j` - Index of the Joint being moved
    fn forward_hinge(&mut self, j: usize) {
    
        if let Ok([joint, hinge]) = self.joints.get_disjoint_mut([j, j+1]) { // get individual mutable references
            // Project distance between joints onto the plane perpendicular to `hinge.axis_aux``
            let d: Vec3 = hinge.position - joint.position; // distance between joints
            let l: Vec3 = d - hinge.axis_aux * d.dot(hinge.axis_aux); // projected d onto perpendicular plane

            // Update joint orientation
            let mag_l: Scalar = l.mag();
            if mag_l == ZERO {
                joint.axis_link = hinge.axis_link;
            } else {
                joint.axis_link = l / mag_l;
            }
            joint.axis_aux = hinge.axis_aux;

            // Check joint limits
            let sign: Scalar = sgn( joint.axis_link.cross(hinge.axis_link).dot(hinge.axis_aux) );
            hinge.value = acos(joint.axis_link.dot(hinge.axis_link)) * sign;
            if hinge.limited {

                // Check upper bound
                if hinge.value > hinge.max {
                    hinge.value = hinge.max; // apply bound
                    let y_axis: Vec3 = hinge.axis_link.cross(hinge.axis_aux); // y = z * x
                    joint.axis_link = hinge.axis_link * cos(hinge.value) // contribution of z-axis
                        + y_axis * sin(hinge.value); // contribution of y-axis
                
                // Check lower bound
                } else if hinge.value < hinge.min {
                    hinge.value = hinge.min; // apply bound
                    let y_axis: Vec3 = hinge.axis_link.cross(hinge.axis_aux); // y = z * x
                    joint.axis_link = hinge.axis_link * cos(hinge.value) // contribution of z-axis
                        + y_axis * sin(hinge.value); // contribution of y-axis
                }
            }

            // Update joint position
            joint.position = hinge.position - joint.axis_link * joint.link_len;
        }
    }

    /// Moves a joint to a position and orientation,
    /// under the hinge-constraints of another joint,
    /// during the backward-reaching phase of FABRIK
    /// 
    /// # Arguments
    /// 
    /// * `j` - Index of the Joint being moved
    fn backward_hinge(&mut self, j: usize) {
    
        if let Ok([joint, hinge, prior]) = self.joints.get_disjoint_mut([j, j-1, j-2]) { // get individual mutable references
            // Project distance between joints onto the plane perpendicular to `hinge.axis_aux``
            let d: Vec3 =  joint.position - hinge.position; // distance between joints
            let l: Vec3 = d - hinge.axis_aux * d.dot(hinge.axis_aux); // projected d onto perpendicular plane

            // Update joint orientation
            let mag_l: Scalar = l.mag();
            if mag_l == 0 {
                hinge.axis_link = prior.axis_link;
            } else {
                hinge.axis_link = l / mag_l;
            }
            joint.axis_aux = hinge.axis_aux;

            // Check joint limits
            let sign: Scalar = sgn( prior.axis_link.cross(hinge.axis_link).dot(hinge.axis_aux) );
            hinge.value = acos(prior.axis_link.dot(hinge.axis_link)) * sign;
            if hinge.limited {

                // Check upper bound
                if hinge.value > hinge.max {
                    hinge.value = hinge.max; // apply bound
                    let y_axis: Vec3 = prior.axis_link.cross(prior.axis_aux); // y = z * x
                    hinge.axis_link = prior.axis_link * cos(hinge.value) // contribution of z-axis
                        - y_axis * sin(hinge.value); // contribution of y-axis
                
                // Check lower bound
                } else if hinge.value < hinge.min {
                    hinge.value = hinge.min; // apply bound
                    let y_axis: Vec3 = prior.axis_link.cross(prior.axis_aux); // y = z * x
                    hinge.axis_link = prior.axis_link * cos(hinge.value) // contribution of z-axis
                        - y_axis * sin(hinge.value); // contribution of y-axis
                }
            }

            // Update joint position
            joint.position = hinge.position + hinge.axis_link * hinge.link_len;
        }
    }

    /// Moves this joint to a position and orientation,
    /// under the pivot-constraints of another joint
    /// during the forward-reaching phase of FABRIK
    /// 
    /// # Arguments
    /// 
    /// * `j` - Index of the Joint being moved
    fn forward_pivot(&mut self, j: usize) {
        
        if let Ok([joint, pivot, next]) = self.joints.get_disjoint_mut([j, j+1, j-1]) { // get individual mutable references
            // Update joint position
            joint.axis_link = pivot.axis_link;
            joint.position = pivot.position - joint.axis_link*joint.link_len;

            // Determine auxiliary axis direction,
            // assuming the next joint's aux axis is constrained similarly to a hinge
            let d: Vec3 = joint.position - next.position;
            let mut a: Vec3 = joint.axis_link.cross(d); // temporary variable for aux axis
            let mag_a: Scalar = a.mag();
            if mag_a == ZERO {
                a = pivot.axis_aux;
            } else {
                a = a / mag_a;
            }

            // Correct the direction of the auxilliary axis
            joint.axis_aux = a * sgn( joint.axis_aux.dot(a) );

            // Check joint limits
            let sign: Scalar = sgn( joint.axis_aux.cross(pivot.axis_aux).dot(joint.axis_link) );
            pivot.value = acos(joint.axis_aux.dot(pivot.axis_aux)) * sign;
            if pivot.limited {

                // Check upper bound
                if pivot.value > pivot.max {
                    pivot.value = pivot.max; // apply bound
                    let y_axis: Vec3 = pivot.axis_link.cross(pivot.axis_aux); // y = z * x
                    joint.axis_aux = pivot.axis_aux * cos(pivot.value) // contribution of z-axis
                        - y_axis * sin(pivot.value); // contribution of y-axis
                
                // Check lower bound
                } else if pivot.value < pivot.min {
                    pivot.value = pivot.min; // apply bound
                    let y_axis: Vec3 = pivot.axis_link.cross(pivot.axis_aux); // y = z * x
                    joint.axis_aux = pivot.axis_aux * cos(pivot.value) // contribution of z-axis
                        - y_axis * sin(pivot.value); // contribution of y-axis
                }
            }
        }
    }

    /// Moves this joint to a position and orientation,
    /// under the pivot-constraints of another joint
    /// during the backward-reaching phase of FABRIK
    /// 
    /// # Arguments
    /// 
    /// * `j` - Index of the Joint being moved
    fn backward_pivot(&mut self, j: usize) {
        
        if let Ok([joint, pivot, next]) = self.joints.get_disjoint_mut([j, j-1, j+1]) { // get individual mutable references
            // Update joint position
            joint.axis_link = pivot.axis_link;
            joint.position = pivot.position + joint.axis_link*pivot.link_len;

            // Determine auxiliary axis direction,
            // using the projected aux axis of the target, otherwise assuming the next joint's aux axis is constrained similarly to a hinge
            let mut a: Vec3;
            match next.joint_type {
                JointType::Immovable => a = next.axis_aux - joint.axis_link * next.axis_aux.dot(joint.axis_link),
                _ => {
                    let d: Vec3 = next.position - joint.position;
                    a = pivot.axis_link.cross(d); // temporary variable for aux axis
                },
            }
            let mag_a: Scalar = a.mag();
            if mag_a == ZERO {
                a = pivot.axis_aux;
            } else {
                a = a / mag_a;
            }

            // Correct the direction of the auxilliary axis
            joint.axis_aux = a * sgn( joint.axis_aux.dot(a) );

            // Check joint limits
            let sign: Scalar = sgn( pivot.axis_aux.cross(joint.axis_aux).dot(pivot.axis_link) );
            pivot.value = acos(pivot.axis_aux.dot(joint.axis_aux)) * sign;
            if pivot.limited {

                // Check upper bound
                if pivot.value > pivot.max {
                    pivot.value = pivot.max; // apply bound
                    let y_axis: Vec3 = pivot.axis_link.cross(pivot.axis_aux); // y = z * x
                    joint.axis_aux = pivot.axis_aux * cos(pivot.value) // contribution of x-axis
                        + y_axis * sin(pivot.value); // contribution of y-axis
                
                // Check lower bound
                } else if pivot.value < pivot.min {
                    pivot.value = pivot.min; // apply bound
                    let y_axis: Vec3 = pivot.axis_link.cross(pivot.axis_aux); // y = z * x
                    joint.axis_aux = pivot.axis_aux * cos(pivot.value) // contribution of x-axis
                        + y_axis * sin(pivot.value); // contribution of y-axis
                }
            }
        }
    }

    /// Moves a joint to a position and orientation,
    /// under the prismatic-constraints of another joint,
    /// during the forward-reaching phase of FABRIK
    /// 
    /// # Arguments
    /// 
    /// * `j` - Index of the Joint being moved
    fn forward_prismatic(&mut self, j: usize) {
    
        if let Ok([joint, prismatic]) = self.joints.get_disjoint_mut([j, j+1]) { // get individual mutable references
            // Update joint orientation
            joint.axis_link = prismatic.axis_link;
            joint.axis_aux = prismatic.axis_aux;

            // Calculate joint length
            let d: Vec3 = prismatic.position - joint.position;
            prismatic.value = d.dot(joint.axis_link) - prismatic.link_len;

            // Check joint limits
            if prismatic.limited {

                // Check upper bound
                if prismatic.value > prismatic.max {
                    prismatic.value = prismatic.max;

                // Check upper bound
                } else if prismatic.value < prismatic.min {
                    prismatic.value = prismatic.min;
                }
            }

            // Update joint position
            joint.position = prismatic.position - joint.axis_link * (prismatic.value + joint.link_len);
        }
    }

    /// Moves a joint to a position and orientation,
    /// under the prismatic-constraints of another joint,
    /// during the backward-reaching phase of FABRIK
    /// 
    /// # Arguments
    /// 
    /// * `j` - Index of the Joint being moved
    fn backward_prismatic(&mut self, j: usize) {
    
        if let Ok([joint, prismatic]) = self.joints.get_disjoint_mut([j, j-1]) { // get individual mutable references
            // Update joint orientation
            joint.axis_link = prismatic.axis_link;
            joint.axis_aux = prismatic.axis_aux;

            // Calculate joint length
            let d: Vec3 = joint.position - prismatic.position;
            prismatic.value = d.dot(joint.axis_link) - prismatic.link_len;

            // Check joint limits
            if prismatic.limited {

                // Check upper bound
                if prismatic.value > prismatic.max {
                    prismatic.value = prismatic.max;

                // Check upper bound
                } else if prismatic.value < prismatic.min {
                    prismatic.value = prismatic.min;
                }
            }

            // Update joint position
            joint.position = prismatic.position + joint.axis_link * (prismatic.value + prismatic.link_len);
        }
    }
}

// TESTS ================================

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq; // testing for approximate equality

    const TARGET_ERR: f32 = 1e-0; // acceptable squared error from the target
    const MAX_ITER: usize = 100; // maximum number of iterations
    const TOL_ACT: f32 = 1e-2; // tolerance for actuations

    // HINGE TESTS ================================

    #[test]
    fn test_hinge_reachable() {
        // Design scenario
        let mut robot: SerialRobot = SerialRobot::new(1);
        robot.set_base( // set base
            [1.0, 2.0, 3.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0]
        );
        robot.conf_joint( // configure joint
            JointType::Hinge,
            5.0,
            false,
            0.0,
            0.0
        );
        robot.set_target( // set target
            [1.0, 2.0+5.0, 3.0],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0]
        );

        // Attempt to solve
        let result: Box<[f32]> = robot.solve(TARGET_ERR, MAX_ITER);
        assert_approx_eq!(result[0], -90.0f32, TOL_ACT);
    }

    #[test]
    fn test_hinge_unreachable() {
        // Design scenario
        let mut robot: SerialRobot = SerialRobot::new(1);
        robot.set_base( // set base
            [1.0, 2.0, 3.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0]
        );
        robot.conf_joint( // configure joint
            JointType::Hinge,
            4.0,
            false,
            0.0,
            0.0
        );
        robot.set_target( // set target
            [1.0+2.0, 2.0-2.8284, 3.0+2.8284],
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0]
        );

        // Attempt to solve
        let result: Box<[f32]> = robot.solve(TARGET_ERR, MAX_ITER);
        assert_approx_eq!(result[0], 45.0f32, TOL_ACT);
    }

    #[test]
    fn test_hinge_constrained() {
        // Design scenario
        let mut robot: SerialRobot = SerialRobot::new(1);
        robot.set_base( // set base
            [1.0, 2.0, 3.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0]
        );
        robot.conf_joint( // configure joint
            JointType::Hinge,
            5.0,
            true,
            -30.0,
            45.0
        );
        robot.set_target( // set target
            [1.0, 2.0+5.0, 3.0],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0]
        );

        // Attempt to solve
        let result: Box<[f32]> = robot.solve(TARGET_ERR, MAX_ITER);
        assert_approx_eq!(result[0], -30.0f32, TOL_ACT);
    }

    // PIVOT TESTS ================================

    #[test]
    fn test_pivot_reachable() {
        // Design scenario
        let mut robot: SerialRobot = SerialRobot::new(1);
        robot.set_base( // set base
            [-2.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0]
        );
        robot.conf_joint( // configure joint
            JointType::Pivot,
            5.0,
            false,
            0.0,
            0.0
        );
        robot.set_target( // set target
            [-2.0, 0.0, 1.0+5.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0]
        );

        // Attempt to solve
        let result: Box<[f32]> = robot.solve(TARGET_ERR, MAX_ITER);
        assert_approx_eq!(result[0], 90.0f32, TOL_ACT);
    }

    #[test]
    fn test_pivot_unreachable() {
        // Design scenario
        let mut robot: SerialRobot = SerialRobot::new(1);
        robot.set_base( // set base
            [-2.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0]
        );
        robot.conf_joint( // configure joint
            JointType::Pivot,
            7.0,
            false,
            0.0,
            0.0
        );
        robot.set_target( // set target
            [-2.0+1.0, 0.0, 1.0+3.0],
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0]
        );

        // Attempt to solve
        let result: Box<[f32]> = robot.solve(TARGET_ERR, MAX_ITER);
        assert_approx_eq!(result[0], -90.0f32, TOL_ACT);
    }

    #[test]
    fn test_pivot_constrained() {
        // Design scenario
        let mut robot: SerialRobot = SerialRobot::new(1);
        robot.set_base( // set base
            [-2.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0]
        );
        robot.conf_joint( // configure joint
            JointType::Pivot,
            5.0,
            true,
            -45.0,
            60.0
        );
        robot.set_target( // set target
            [-2.0, 0.0, 1.0+5.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0]
        );

        // Attempt to solve
        let result: Box<[f32]> = robot.solve(TARGET_ERR, MAX_ITER);
        assert_approx_eq!(result[0], 60.0f32, TOL_ACT);
    }

    // PRISMATIC TESTS ================================

    #[test]
    fn test_prismatic_reachable() {
        // Design scenario
        let mut robot: SerialRobot = SerialRobot::new(1);
        robot.set_base( // set base
            [-1.0, 2.0, -2.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0]
        );
        robot.conf_joint( // configure joint
            JointType::Prismatic,
            1.0,
            false,
            0.0,
            0.0
        );
        robot.set_target( // set target
            [-1.0, 2.0, -2.0+7.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0]
        );

        // Attempt to solve
        let result: Box<[f32]> = robot.solve(TARGET_ERR, MAX_ITER);
        assert_approx_eq!(result[0], 6.0f32, TOL_ACT);
    }

    #[test]
    fn test_prismatic_unreachable() {
        // Design scenario
        let mut robot: SerialRobot = SerialRobot::new(1);
        robot.set_base( // set base
            [-1.0, 2.0, -2.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0]
        );
        robot.conf_joint( // configure joint
            JointType::Prismatic,
            2.0,
            false,
            0.0,
            0.0
        );
        robot.set_target( // set target
            [-1.0-2.0, 2.0, -2.0+3.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        );

        // Attempt to solve
        let result: Box<[f32]> = robot.solve(TARGET_ERR, MAX_ITER);
        assert_approx_eq!(result[0], 1.0f32, TOL_ACT);
    }

    #[test]
    fn test_prismatic_constrained() {
        // Design scenario
        let mut robot: SerialRobot = SerialRobot::new(1);
        robot.set_base( // set base
            [-1.0, 2.0, -2.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0]
        );
        robot.conf_joint( // configure joint
            JointType::Prismatic,
            1.0,
            true,
            -1.0,
            4.5
        );
        robot.set_target( // set target
            [-1.0, 2.0, -2.0+7.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0]
        );

        // Attempt to solve
        let result: Box<[f32]> = robot.solve(TARGET_ERR, MAX_ITER);
        assert_approx_eq!(result[0], 4.5f32, TOL_ACT);
    }
}
