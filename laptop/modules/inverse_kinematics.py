"""
    Module to handle the inverse kinematics - wrapping the `braccio_ik` library
    Author: Neo Vorsatz
    Last updated: 22 October 2026
"""

# IMPORTS ================================

# External
import time

# Internal
import braccio_ik as ik
import modules.data_collection as dc

# TYPES ================================

_Target = tuple[list[list[float]],float]|tuple[None,None]

# CONSTANTS ================================

# Benchmarking
_BENCHMARK = True # make this `False` to omit code that is for benchmarking
_HD_IK_SOLVER = "ik solver (ms)"
_HD_ITERATIONS = "iterations (#)"
_HD_SQR_ERR = "sqr err (mm^2)"

# Inverse kinematics solver
_TARGET_ERR = 1 # target acceptable error
_MAX_ITER = 10000 # maximum number of iterations

# VARIABLES ================================

_braccio = None # serial robot strucut for `braccio_ik`
_valid_setpoints = None # last valid setpoints

# HELPER FUNCTIONS ================================


# API FUNCTIONS ================================

def init_ik():
    """
    Initializes the inverse kinematics model
    """
    # Global variables
    global _braccio

    # Initialize IK module
    _braccio = ik.init_ik()

    # Benchmarking
    if _BENCHMARK:
        dc.add_heading(_HD_IK_SOLVER)
        dc.add_heading(_HD_ITERATIONS)
        dc.add_heading(_HD_SQR_ERR)

def get_setpoints(target:_Target) -> list[float] | None:
    """
    Checks the validity of a target, and converts a target into servo motor setpoints

    Args:
        target (_Target): The target position, direction, rotation, and openness
    Returns:
        list[float] | None: The setpoints, or None if no valid setpoint exists yet
    """
    # Global variables
    global _braccio, _valid_setpoints

    # Check validity
    ik_target, openness = target # the inverse kinematics doesn't consider openness
    if ik_target is None:
        return _valid_setpoints

    # Benchmarking
    if _BENCHMARK:
        bench_start = time.perf_counter()

    # Perform inverse kinematics
    print("End-effector:", round(ik_target[0][0]), round(ik_target[0][1]), round(ik_target[0][2])) # FEEDBACK
    print("Direction:", round(ik_target[1][0],2), round(ik_target[1][1],2), round(ik_target[1][2],2)) # FEEDBACK
    print("Rotation:", round(ik_target[2][0],2), round(ik_target[2][1],2), round(ik_target[2][2],2)) # FEEDBACK
    success = ik.set_target(_braccio, *ik_target)
    if success: # if the target was set successfully
        _valid_setpoints = ik.get_actuations(_braccio, _TARGET_ERR, _MAX_ITER) + [openness]
    print("Setpoint:", round(_valid_setpoints[4])) # FEEDBACK

    # Benchmarking
    if _BENCHMARK:
        bench_delta = (time.perf_counter()-bench_start)*1000
        dc.record(bench_delta, _HD_IK_SOLVER)
        if success:
            dc.record(ik.get_num_iterations(_braccio), _HD_ITERATIONS)
        else:
            dc.record(0, _HD_ITERATIONS) # no iterations were run
        dc.record(ik.get_error(_braccio), _HD_SQR_ERR)

    # Return
    return _valid_setpoints

# MAIN ================================

if __name__ == "__main__":
    # Imports
    import network as net
    from math import floor

    # Desired target
    targets = [
        [ [0,500,100], [0,5,0], [1,0,0] ],
        [ [0,-50,500], [0,0,5], [1,0,0] ],
        # [ [0,100,400], [0,1,4], [1,0,0] ],
        # [ [0,200,300], [0,2,3], [1,0,0] ],
        # [ [0,300,200], [0,3,0], [1,0,0] ],
        # [ [0,200,100], [0,2,-1], [1,0,0] ],
    ]

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

            # Initial time
            t0 = time.time()

            # Continuously send setpoint
            while True:
                try:
                    # Determine target
                    dt = time.time() - t0
                    index = floor(dt / 5)
                    if index >= len(targets):
                        index = len(targets) - 1
                    print("Move:", index)
                    target = targets[index]

                    # Send setpoints
                    setpoints = get_setpoints( (target, 180.0) )
                    print("Setpoints:", setpoints)
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
