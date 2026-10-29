"""
    Program to handle the operator-side of the Braccio teleoperation
    Author: Neo Vorsatz
    Last updated: 27 October 2026
"""

# IMPORTS ================================

# External
import time # delays
import sys # system exit

# Internal
import modules.computer_vision as cv
import modules.network as net
import modules.inverse_kinematics as ik
import modules.data_collection as dc

# CONSTANTS ================================

# Benchmarking
_BENCHMARK = True # make this `False` to omit code that is for benchmarking

# HELPER FUNCTIONS ================================

def _init():
    """
    Initializes the computer vision, networking and inverse kinematics
    """
    # Computer vision
    cv.init_cv()
    try:
        cv.calibrate_cv()
    except (ValueError, KeyboardInterrupt) as e:
        cv.cleanup_cv()
        print(e)
        sys.exit() # exit with error

    # Network
    net.init_net()

    # Inverse kinematics
    ik.init_ik()

def _main_loop() -> OSError:
    """
    Connects the network, and continuously gets and sends setpoints.
    If a connection breaks, the function ends and returns the exception

    Returns:
        OSError: The exception associated with the failure of the network
    Raises:
        KeyboardInterrupt: When the computer vision pipeline ends
    """
    # Try to connect
    while not net.connect_net():
        print("Retrying to connect in 1 second")
        time.sleep(1.0)

    # Continuously get and send setpoints
    while True:
        try:
            # Update computer vision
            if not cv.update_cv() or cv.get_key_press() == ord("q"):
                raise KeyboardInterrupt # end the program
            target = cv.get_target()

            # Perform cleaning and inverse kinematics
            setpoints = ik.get_setpoints(target)
            
            # Send setpoints
            if setpoints is not None:
                net.send_setpoints(setpoints)
            print("=====") # FEEDBACK

            # Benchmarking
            if _BENCHMARK:
                dc.flush() # write all of the data that has been pending

            # Add delay for network traffic to keep up
            # time.sleep(0.01)

        # Failed to send packet
        except OSError as e:
            return e

# MAIN ================================

def main():
    # Initialization
    _init()

    # Main loop
    try:
        while True:
            e = _main_loop()
            print(f"Failed to send packet due to error: {e}\nTrying to connect in 1 second")
            time.sleep(1.0)

    # Handle exit when Ctrl+C is used in the terminal or when `_main_loop()` raises this exception
    except KeyboardInterrupt:
        pass
    finally:
        net.close_net()
        cv.cleanup_cv()
        dc.finish()

if __name__ == "__main__":
    main()
