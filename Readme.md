#Overview
Yet Another Pilot (aka YAP) is a autopilot system for multicopters and any remote controlled system.

The modular design makes it easy to port between different microprocessors, sensors, even to linux based boards.

most features on common commercial quads are supported, such as position holding, return to launch, controlling via cellphones.

and some "not so common" features are also included, such as OpticalFlow, sonar, one key acrobatic.

Currently most YAP systems runs on STM32F4xx. Due to computation power issue, minimum MCU recomendation is CortexM3 @ 72Mhz, with EKF disabled.


#System Stucture and Work Flow
TODO
##
## Bootloader

#Coding Guidelines
use C+class, minimun template and basic operator override is acceptable.

platform specified assembly acceleration is allowed, but a C/C++ fallback is required.

use tabs, not spaces.

use stack and global/member variables. heap allocation is forbidden.

non-blocking functions in time-critical thread only. put unpredictable works to "AsyncWorker".

#Coordinate Systems
see [CoordinateSystems.md](CoordinateSystems.md)

#TODOs
Precision IMU calibration.
Better state estimation, such as UKF, more detailed dynamic physical models.
System Identification and Advanced Controlling Algorithm.
Software Defined Radio baseband.
Battery Management System.
FOC ESCs.
Gimbals.
Image Stabilizer, or "E-Gimbal".
Redundant System.