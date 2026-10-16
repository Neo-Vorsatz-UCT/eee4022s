"""
    Program to test the inverse kinematics module
    Author: Neo Vorsatz
    Last updated: 16 October 2026
"""

# IMPORTS ================================

# External
from math import sqrt

# Internal
import braccio_ik as ik

# TYPES ================================

_Target = tuple[list[list[float]],float]|tuple[None,None]

# CONSTANTS ================================

# Low pass filter
_WINDOW_SIZE = 100

# Inverse kinematics solver
_TARGET_ERR = 1 # target acceptable error
_MAX_ITER = 100 # maximum number of iterations

# Physical constraints
_BASE_RADIUS = 100 # distance to stay away form the origin

# Vector components
_X = 0
_Y = 1
_Z = 2

# VARIABLES ================================

_braccio = None # serial robot strucut for `braccio_ik`
_valid_setpoint = None # last valid setpoints
_past_targets = [] # list of past targets

# HELPER FUNCTIONS ================================

def _clamp_position(pos:list[float]) -> list[float]:
    """
    Clamps a position into something more reasonable, such as by:
    - preventing negative y (backwards workspace)
    - preventing negative z (below workspace)
    - preventing coordinates directly on the base (physically unreachable)

    Args:
        pos ([list[float]): The position to clamp
    Returns:
        list[float]: The clamped position
    """
    # Copy
    pos = pos.copy()

    # Prevent negative y
    if pos[_Y] < 0.0:
        pos[_Y] = 0.0

    # Prevent negative z
    if pos[_Z] < 0.0:
        pos[_Z] = 0.0;

    # Prevent coordinates directly on the base
    r = sqrt(pos[_X]**2 + pos[_Y]**2 + pos[_Z]**2) # radius from the origin (centre of the base)
    if r < _BASE_RADIUS: # if the distance is too small
        scale = _BASE_RADIUS / r
        for i in range(3):
            pos[i] *= scale

    # Return
    return pos

def _avg_targets(targets:list[_Target]) -> _Target:
    """
    Averages a list of targets

    Args:
        targets (list[_Target]): The list of targets to average
    Returns:
        _Target: The averaged target
    """
    # Create empty target
    avg_ik_target = [[0,0,0], [0,0,0], [0,0,0]]
    avg_openness = 0

    # Sum all targets
    for target in targets:
        for i in range(3): # for each vector
            for j in range(3): # for each component
                avg_ik_target[i][j] += target[0][i][j]
        avg_openness += target[1]

    # Divide
    for i in range(3): # for each vector
        for j in range(3): # for each component
            avg_ik_target[i][j] /= len(targets)
    avg_openness /= len(targets)

    # Concatenate and return
    return (avg_ik_target, avg_openness)

def _low_pass(target:_Target) -> _Target:
    """
    Performs a low pass filter on the target, assuming the target is valid

    Args:
        target (_Target): The next target
    Returns:
        _Target: The low pass filtered target
    """
    # Global variables
    global _past_targets

    # Append and truncate the buffer
    _past_targets.append(target)
    if len(_past_targets) > _WINDOW_SIZE:
        _past_targets.pop(0)

    # Return average
    return _avg_targets(_past_targets)

# API FUNCTIONS ================================

def init_ik():
    """
    Initializes the inverse kinematics model
    """
    # Global variables
    global _braccio

    # Initialize IK module
    _braccio = ik.init_ik()

def get_setpoints(target:_Target) -> list[float] | None:
    """
    Checks the validity of a target, and converts a target into servo motor setpoints

    Args:
        target (_Target): The target position, direction, rotation, and openness
    Returns:
        list[float] | None: The setpoints, or None if no valid setpoint exists yet
    """
    # Global variables
    global _braccio, _valid_setpoint

    # Check validity
    ik_target, openness = target # the inverse kinematics doesn't consider openness
    if ik_target is None:
        return _valid_setpoint

    # Clean up new target
    ik_target[0] = _clamp_position(ik_target[0]) # clamp the position

    # Perform low pass filter
    ik_target, openness = _low_pass((ik_target, openness))

    # Perform inverse kinematics
    ik.set_target(_braccio, *ik_target)
    setpoints = ik.get_actuations(_braccio, _TARGET_ERR, _MAX_ITER) + [openness]

    # Return
    return setpoints

# MAIN ================================

if __name__ == "__main__":
    # Imports
    import network as net
    import time # timing

    # Desired target
    target_pos = [0, 300, 200]
    target_dir = [0, 1, 0]
    target_rot = [1, 0, 0]

    # Initialize IK
    init_ik()

    # Networking
    net.init_net()
    try:
        while True:
            # Try to connect
            while not net.connect_net():
                print("Retrying to connect in 1 second")
                time.sleep(1.0)

            # Continuously send setpoint
            while True:
                try:
                    setpoints = get_setpoints( ([target_pos, target_dir, target_rot], 180.0) )
                    net.send_setpoints(setpoints)
                    time.sleep(0.01)
                except OSError as e:
                    print(f"Failed to send packet due to error: {e}\nTrying to connect in 1 second")
                    time.sleep(1.0)
                    break

    # Handle exit when using Ctrl+C in the terminal
    except KeyboardInterrupt:
        pass
    finally:
        net.close_net()
