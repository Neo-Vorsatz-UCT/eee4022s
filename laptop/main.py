"""
    Program to handle the operator-side of the Braccio teleoperation
    Author: Neo Vorsatz
    Last updated: 8 October 2026
"""

# IMPORTS ================================

# External

# Internal
import modules.computer_vision as cv

# TYPES ================================


# CONSTANTS ================================


# HELPER FUNCTIONS ================================

def _callibrate():
    pass

# MAIN ================================

def main():
    # Initialization
    cv.init_cv()
    pose_handler = cv.PoseHandler()

    while cv.update_cv():
        pose_handler.set_pose(cv.get_landmarks())

if __name__ == "__main__":
    main()
