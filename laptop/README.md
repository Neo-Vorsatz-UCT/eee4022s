# Overview
This is the software for the operator-side of the teleoperation system. It includes the computer vision subsystem, inverse kinematics subsystem, and part of the networking subsystem.

Please ignore the Makefile; it's more just notes to myself than functional.

# Computer Vision
The computer vision uses Google's [MediaPipe *Hand Landmark Detection*](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker) to the operator's hand pose and map that to a target for the Braccio's end-effector. At the time of developing this project, MediaPipe is only supported for up to Python 3.12. Because of this, the virtual environment was made for Python 3.12.

# Inverse Kinematics
The inverse kinematics was implemented in Rust, using a variation of FABRIK described by [Moreno *et al.*](https://doi.org/10.1109/ICEV56253.2022.9959663). Maturin was used to convert the Rust library into a Python module.

# Networking
Communication between the operator-side and ESP32 happens over the DTLS1.2 protocol. At the time of developing this project, mbedTLS 4.x only supported up to DTLS1.2. For authentication and encryption, pre-shared keys were used (visible in this repository).

# Installation & Dependencies
## Virtual Environment
I highly recommend setting up a virtual environment. I used Micromamba, with which creating the virtual environment can be done using
```bash
micromamba env create -f env/environment.yml
```
This will create a virtual environment called `eee4022s`, and automatically install the Python dependencies from `requirements.txt`.

If you create the virtual environment some other way, and have pip installed in that environment, then you can install the Python dependencies by activating your virtual environment and using
```bash
pip install -r env/requirements.txt
```

## MediaPipe Hand Landmark Detection
To download the MediaPipe *Hand Landmark Detection* model, run
```bash
wget -P ./mediapipe_models https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
```

## Compile Inverse Kinematics
To compile the inverse kinematics module, and install it into your virtual environment, activate your virtual environment and run
```bash
maturin develop --release
```

If you're using Micromamba and don't want to activate your virtual environment, you can run
```bash
micromamba run -n eee4022s maturin develop --release
```
(assuming your environment's name is `eee4022s`)

# License
This directory and all its parts, where developed by me (not imported from a third party), are released under the same license as the entire project.
