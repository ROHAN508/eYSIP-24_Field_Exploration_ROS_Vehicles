# Advancing Field Exploration Using ROS-powered Robotic Vehicles - ARMS Lab IIT Bombay , e-Yantra IITB

## Table of Contents
1. [Introduction](#introduction)
2. [Project Goal](#project-goal)
3. [Tech Stack](#tech-stack)
4. [Setup](#setup)
    - [Prerequisites](#prerequisites)
5. [Hardware Development](#hardware-development)
6. [ROS2-Teleoperation](#ros2-teleoperation)
7. [Localization and Odometry](#localization-and-odometry)
8. [Motion Planning](#motion-planning)
9. [SLAM](#slam)

## Introduction
Field exploration robots have gained significant attention in recent years due to their potential applications in various domains such as agriculture, environmental monitoring, and autonomous driving research. These robots are designed to operate autonomously in diverse environments, collecting data and performing tasks that would otherwise be labor-intensive or hazardous for humans. This project focuses on developing a ROS-powered robotic vehicle designed for field exploration. The vehicle is capable of traversing varied ground surfaces autonomously and can be used for tasks such as autonomous field mapping, agricultural plant monitoring, and testing autonomous driving algorithms.

## Project Goal
The primary goal of this project is to develop a ROS2-enabled four-wheel-drive vehicle with Ackerman steering capable of waypoint navigation and obstacle avoidance. This vehicle will be equipped with various sensors and an onboard computer to perform Simultaneous Localization and Mapping (SLAM). The project encompasses hardware development, sensor integration, firmware and algorithm development, and extensive testing and validation to ensure the vehicle’s performance and reliability in real-world scenarios.

## Tech Stack
![ROS Logo](https://upload.wikimedia.org/wikipedia/commons/thumb/b/bb/Ros_logo.svg/85px-Ros_logo.svg.png) 
<img src="https://github.com/user-attachments/assets/221016e3-f4f1-4bc7-b64c-08f458c31085" alt="Jazzy Logo" width="80" height="80"/>
![Arduino Logo](https://upload.wikimedia.org/wikipedia/commons/thumb/8/87/Arduino_Logo.svg/85px-Arduino_Logo.svg.png)  ![Raspberry Pi Logo](https://upload.wikimedia.org/wikipedia/en/thumb/c/cb/Raspberry_Pi_Logo.svg/50px-Raspberry_Pi_Logo.svg.png)
<img src="https://github.com/user-attachments/assets/d05d5f7d-a439-47d1-9e0e-8f6272b9ea4c" alt="Nav2 Logo" width="80" height="80"/>






- **[ROS2 jazzy](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html):** Middleware for robot software development.
- **[Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/):** Onboard computer for processing and communication.
- **[Esp32](https://www.espressif.com/en/products/socs/esp32#:~:text=ESP32%20is%20highly%2Dintegrated%20with,Hybrid%20Wi%2DFi%20%26%20Bluetooth%20Chip):** Microcontroller for low-level motor and servo control interfaced using [ArduinoIDE]( https://www.arduino.cc/en/software)
- **[Nav2](https://docs.nav2.org/getting_started/index.html):** Navigation stack for ROS 2.
- **[RViz](https://wiki.ros.org/rviz):** Visualization tool for ROS.

## Setup

### Prerequisites
- **Operating System:** Ubuntu 20.04 or later
- **ROS 2 Distribution:** Foxy Fitzroy or later
- **Hardware:** Raspberry Pi 4 or later, ESP32, BMX-160 IMU, LIDAR
- **Software:** ROS 2, Arduino IDE, Python 3.x

1. Install ROS 2 on your Ubuntu system by following the [official ROS 2 installation guide](https://docs.ros.org/en/foxy/Installation.html).
2. Set up your Raspberry Pi and ensure it is connected to your network.
3. Install necessary libraries and dependencies on your Raspberry Pi.
4. Flash the ESP32 with the custom firmware using Arduino IDE.
5. Connect and configure all hardware components as per the provided schematic.

## Hardware Development
### Actuation and Steering System
- The motor actuation and steering system features five-pin servo control using PID for precision and DC motor control for throttle.
- A custom PCB board was developed to ensure optimum power supply to the ESP32, motor drivers, and Raspberry Pi.
- The mechanical design involved creating laser-cut CAD models using acrylic for a two-layered system, with mounts for all hardware components including a LIDAR.
- Additionally, the BMX-160 IMU was interfaced and calibrated to publish accurate orientation and acceleration data on a ROS-2 network.

**Figure 1:** Control System Design

### Key Challenges
- **Low-level Control:** The primary challenge we faced in low-level control was interfacing a custom servo with five pins for which no pin diagrams or references were available online. Achieving accurate control at this level required extensive experimentation and fine-tuning.
- **Localization and Odometry:** For localization and odometry, we relied solely on an IMU to estimate the robot’s pose. The IMU’s orientation measurements exhibited significant drift over time. Additionally, the accelerometer’s bias was continuously affected by the robot’s suspension system.
- **Motion Planning:** In motion planning, traditional control algorithms used for differential or omnidirectional drives were ineffective. This required the development of an approach tailored to the dynamics of an Ackermann steering system.
- **Environment Mapping:** For environment mapping, we had to adapt mapping techniques to work with Ackermann kinematics. This included ensuring accurate data collection from LIDAR and integrating these readings into a coherent map.

**Figure 2:** Vicon Odometry vs IMU Odometry

## ROS2-Teleoperation
### Methodologies
- Achieved complete low-level control of the car through keyboard commands using ROS2 for communication over a shared network.
- Implement teleoperation nodes in ROS 2 to send commands to the robot.

**Implementation Steps:**
1. Install necessary ROS 2 packages for teleoperation.
2. Write a ROS 2 node to capture keyboard inputs and publish velocity commands.
3. Configure the robot to subscribe to these velocity commands and actuate the motors accordingly.
4. Test the teleoperation setup to ensure reliable and responsive control.

## Localization and Odometry
### Methodologies
- **Modified Kalman Filter:** Fused car velocity data (calculated from Vicon) with IMU accelerometer readings to get a more accurate state estimate (position and velocity) by reducing drift.

## Motion Planning
### Methodologies
- **Optimized Pure Pursuit Algorithm:** Implemented an optimized version of this path-tracking algorithm to ensure appropriate steering angles using the lookahead distance concept.

## SLAM
### Methodologies
- **SLAM:** Mapped the environment using LiDAR scan data and fused it with odometry data. This, along with the Adaptive Monte Carlo Localization (AMCL) algorithm, provided an accurate pose (position and orientation) estimate of the car within the map.

**Figure 3:** Prototype Vehicle
