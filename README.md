# Autonomous-Waypoint-EV

This repository contains the source code and documentation for my undergraduate final project: a low-level control system for an autonomous electric vehicle. This project was developed to enable precise waypoint-based navigation and was recognized as **one of the first autonomous electric vehicle builds in Indonesia.**

The core research focused on the low-level controller's ability to execute trajectory tracking commands from a high-level planner.

**📄[Read the IEEE Conference Paper](https://doi.org/10.1109/ICVEE59738.2023.10348231)**

![Autonomous Car Body](https://github.com/DewaPr/Autonomous-Waypoint-EV/blob/main/images/Official%20testing%20with%20the%20directorate%20general%20of%20universities%2C%20Prof.%20Dr.%20Ir.%20Nizam%20M.Sc.%20Ph.D.jpg)

## Test Videos
* **[Obstacle Avoidance Outside Car POV](https://drive.google.com/file/d/1uJbg33MiOxGYNZFdr3-Tg0fdbmy_W6BV/view?usp=sharing)**
* **[Obstacle Avoidance Steering POV](https://drive.google.com/file/d/1a7Viich5UEiji-1FOAW4bwDuT_RbOzFk/view?usp=sharing)**
* **[Path4 Outside Car POV](https://drive.google.com/file/d/1nScOytIyXUA_hNI2v9JQ6n9wpI861W92/view?usp=sharing)**
* **[Path4 Steering POV](https://drive.google.com/file/d/15KvCHcvea8IMGCnsBSTB9wiy_GfPaA42/view?usp=sharing)**
* **[Path5 Outside Car POV](https://drive.google.com/file/d/1kHZRzk53AjELgQN9o9s0ZDN-GLw2w79a/view?usp=sharing)**
* **[Path5 Sterring POV](https://drive.google.com/file/d/1KwWKs7TE-Yf4vmhg593MW81YMnaruxMK/view?usp=sharing)**
* **[Path10 Outside Car POV](https://drive.google.com/file/d/13tGAtkVPWCAK_A_Q4I5gw0dKqqMXEaAY/view?usp=sharing)**
* **[Path10 Steering POV](https://drive.google.com/file/d/1_yt8tsWci4t8FFlRQIWo6Av3mCr4Lhnl/view?usp=sharing)**

## Key Achievements
* **First Mover:** Developed one of the pioneering autonomous electric vehicle platforms in Indonesia.
* **IEEE Publication:** Research findings and control methodologies were published in an IEEE conference proceeding.
* **Precision Control:** Implemented a robust low-level control loop for steering and velocity tracking.

## System Architecture

The system is divided into two main layers: High-Level Planning and Low-Level Control.

### 1. High-Level Planner (Navigation)
* **Function:** Generates the global path and calculates the desired steering angle and velocity based on GPS waypoints.
* **Algorithm:** Pure Pursuit Waypoint for path tracking.
* **Sensors:**
    * **GPS/GNSS:** For global positioning.
    * **IMU:** For heading and orientation estimation.
    * **Stereo Camera:** For obstacle detection.

### 2. Low-Level Controller (The Core Project)
This repository focuses on this layer, which translates high-level commands into physical actuator signals.
* **Microcontroller:** STM32F411CCU6
* **Actuators:**
    * **Steering:** Power Sterring controlled via PID.
    * **Throttle/Brake:** DAC output to the EV motor controller.
* **Control Loop:** A PID-based control loop running ensures the steering angle matches the target setpoint from the planner.

## IEEE Abstract
*"The Design and Implementation of a Waypoint-Based Autonomous Control System on a Low-Level Controller for Self-Driving Vehicles"*

> Autonomous land transportation vehicles and Electric Vehicles (EV) have become the main choice of the majority of society due to their environmentally friendly nature. Furthermore, the acceleration of autonomous vehicle research can also reduce traffic accidents caused by human error, which have been significantly high in the last 10 years. However, the cost required to obtain assistance from autonomous systems still tends to be expensive. This paper presents the development of a device utilizing an Advanced RISC Machine (ARM) Cortex-an Advanced RISC Machine (ARM) Cortex-M4 STM32F411 microcontroller system with 32-bit architecture. This system offers optimal performance at a cost-effective price point. The proposed autonomous system is designed based on the waypoint method as a reference to the vehicle’s position. The Global Navigation Satellite System (GNSS) accessed through the U-Blox NEO-M8P module provides position data, whereas the HMC5883L magnetometer provides orientation data. The position data is further processed by utilizing a Kalman filter to enhance accuracy and reduce computational burden. The error detection angle read by the E-Compass is only 0.31% with the ground speed testing in comparison with true speed from car odometry with an average error of 1%, thus supporting good data accuracy, achieved with a track error of only 0.06 meters in real-scale environmental testing. This demonstrates the successful implementation of autonomous control algorithms on the low-level controller.

## Repository Structure
* `/firmware`: C/C++ source code for the Low-Level Controller (STM32CubeIde).
* `/hardware`: Wiring diagrams and H-Bridge driver schematics for the steering motor.
* `/images`: The documentation image of testing and actual hardware.
* `/datasheets`: The component datasheets we use.

## Demonstration
more image and testing on my LinkedIn: [dewapramudya](https://linkedin.com/in/dewapramudya)

---
*Developed by Dewa Pramydta at Electronic Engineering Polytechnic Institute of Surabaya (EEPIS).*
