# Overview
This is the GitHub repository of my work for the EEE4022S course, which forms part of the University of Cape Town's ***BSc(Eng) in Mechatronics*** degree.

The [final report](<report/EEE4022S 2026 FINAL REPORT VRSNEO001 VORSATZ N - RAMOTSOELA.pdf>) can be found in PDF form, along with source LaTeX files.

## Abstract
Teleoperation systems are used to allow humans to control robotic systems over large distances. An ideal user experience involves non-intrusive sensing and intuitive spatial mapping. Few implementations of teleoperation discuss the effects of applying cybersecurity principles to the resource constrained hardware inherent to some robotic systems. This report discusses the implementation of an intuitive and non-invasive teleoperation system, using computer vision, while keeping network security and constraints in consideration. The system uses a pre-trained computer vision pipeline, along with an inverse kinematics algorithm to determine the actuation values for a Tinkerkit Braccio. Packets sent from the operator-side to the Braccio-side are managed by the DTLS protocol, using pre-shared keys. Due to monocular depth ambiguity, the motion of the Braccio was constrained to 2 dimensions. Through experiments, it was found that the smoothing algorithm used to reduce aggressive movements contributed most (88.3%) to the system’s total latency. The latency due to encryption and decryption were otherwise insignificant, and this is more apparent over slower networks, such as due to large geographical separation.

## Directories
```
┣━ arduino/ # the code for the Arduino
┣━ data/ # data collected during tests
┃   ┗━ EEE4022S_TDR-04_Demo.mp4 # demo video of the project
┣━ esp32/ # the code for the ESP32
┣━ laptop/ # the code for the operator-side
┗━ report/ # the LaTeX source files for the report
    ┗━ EEE4022S 2026 FINAL REPORT VRSNEO001 VORSATZ N - RAMOTSOELA.pdf # final report
```

# Demo video
<video src="data/EEE4022S_TDR-04_Demo.mp4" controls width="100%"></video>
[(link to video file)](data/EEE4022S_TDR-04_Demo.mp4)

# License
This project is released on the GNU Lesser General Public License v2.1.

# Installation & Dependencies
It's recommended to copy this entire repository into somewhere you keep your installed libraries, because some of the modules depend on other modules within this repository.
```bash
git clone https://github.com/Neo-Vorsatz/Neo-Libraries.git
```
