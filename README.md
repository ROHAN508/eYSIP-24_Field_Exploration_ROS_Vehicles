# Advancing Field Exploration Using ROS-powered Robotic Vehicles 
## ARMS Lab, Systems and Control engineering department IIT Bombay & e-Yantra

<img src="https://github.com/user-attachments/assets/1f34676f-148d-4278-898e-5757917b775c" alt="arms lab Logo" width="150" height="80"/>
<img src="https://github.com/user-attachments/assets/728cd6f2-2ba5-4a79-9b8e-a3cf8086c92e" alt="eyantra Logo" width="80" height="80"/>


![GitHub repo size](https://img.shields.io/github/repo-size/TahsinOP/eYSIP-24_Field_Exploration_ROS_Vehicles)
![GitHub contributors](https://img.shields.io/github/contributors/TahsinOP/eYSIP-24_Field_Exploration_ROS_Vehicles)
![GitHub last commit](https://img.shields.io/github/last-commit/TahsinOP/eYSIP-24_Field_Exploration_ROS_Vehicles)
![GitHub issues](https://img.shields.io/github/issues/TahsinOP/eYSIP-24_Field_Exploration_ROS_Vehicles)
![GitHub pull requests](https://img.shields.io/github/issues-pr/TahsinOP/eYSIP-24_Field_Exploration_ROS_Vehicles)
![GitHub](https://img.shields.io/github/license/TahsinOP/eYSIP-24_Field_Exploration_ROS_Vehicles)

## Table of Contents
1. [Introduction](#introduction)
2. [Project Goal](#project-goal)
3. [Tech Stack](#tech-stack)

4. [Setup](#setup)
    - [Prerequisites](#prerequisites)
    - [Remote Desktop Setup](#remote-desktop-setup)
    - [Interfacing Hardware ](#interfacing-hardware)
         - [Raspberry Pi 5](#raspberry-pi-5)
         - [BMX 160 IMU ](#bmx-160-imu)
         - [Ydlidar Tmini-Pro](#ydlidar-tmini-pro)
5. [Key Challenges](#key-challenges)
6. [Hardware Development](#hardware-development)
7. [Control System Design](#control-system-design)
8. [ROS2-Teleoperation](#ros2-teleoperation)
9. [Localization and Odometry](#localization-and-odometry)
10. [Motion Planning](#motion-planning)
11. [SLAM](#slam)

## Introduction
Field exploration robots have gained significant attention in recent years due to their potential applications in various domains such as agriculture, environmental monitoring, and autonomous driving research. These robots are designed to operate autonomously in diverse environments, collecting data and performing tasks that would otherwise be labor-intensive or hazardous for humans. This project focuses on developing a ``ROS-powered robotic`` vehicle designed for ``field exploration``. The vehicle is capable of traversing varied ground surfaces autonomously and can be used for tasks such as autonomous field mapping, agricultural plant monitoring, and testing autonomous driving algorithms.

## Project Goal
The primary goal of this project is to develop a ROS2-enabled four-wheel-drive vehicle with ``Ackermann steering`` capable of ``waypoint navigation`` and ``obstacle avoidance``. This vehicle will have various sensors and an onboard computer to perform`` Simultaneous Localization and Mapping (SLAM``). The project encompasses hardware development, sensor integration, firmware and algorithm development, and extensive testing and validation to ensure the vehicle’s performance and reliability in real-world scenarios.

## Tech Stack
![ROS Logo](https://upload.wikimedia.org/wikipedia/commons/thumb/b/bb/Ros_logo.svg/85px-Ros_logo.svg.png) 
<img src="https://github.com/user-attachments/assets/221016e3-f4f1-4bc7-b64c-08f458c31085" alt="Jazzy Logo" width="80" height="80"/>
![Arduino Logo](https://upload.wikimedia.org/wikipedia/commons/thumb/8/87/Arduino_Logo.svg/85px-Arduino_Logo.svg.png)![Raspberry Pi Logo](https://upload.wikimedia.org/wikipedia/en/thumb/c/cb/Raspberry_Pi_Logo.svg/50px-Raspberry_Pi_Logo.svg.png)
<img src="https://github.com/user-attachments/assets/d05d5f7d-a439-47d1-9e0e-8f6272b9ea4c" alt="Nav2 Logo" width="80" height="80"/>


- **[ROS2 jazzy](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html):** Middleware for robot software development.
- **[Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/):** Onboard computer for processing and communication.
- **[Esp32](https://www.espressif.com/en/products/socs/esp32#:~:text=ESP32%20is%20highly%2Dintegrated%20with,Hybrid%20Wi%2DFi%20%26%20Bluetooth%20Chip):** Microcontroller for low-level motor and servo control interfaced using [ArduinoIDE]( https://www.arduino.cc/en/software)
- **[Nav2](https://docs.nav2.org/getting_started/index.html):** Navigation stack for ROS 2.
- **[RViz2](https://wiki.ros.org/rviz):** Visualization tool for ROS 2.

## Setup

## Prerequisites

- **Operating System:** ``Ubuntu 24 LTS`` noble or later
- **ROS 2 Distribution:** ``Jazzy Jelisco`` 
- **Hardware:** ``Raspberry Pi5`` ,`` BMX 160 IMU`` , ``Ydlidar Tmini-Pro`` , ``Esp32`` , ``7V Lipo Battery`` 
- **Software:**  ``Arduino IDE``, ``Python 3.12``

## Remote Desktop Setup 
1. **Install ROS2 Jazzy:**

   After installing run this command once 
   ```bash 
   source /opt/ros/humble/setup.bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc

   ```
2. **Create a workspace and clone the repository**

   ```bash
   mkdir -p ~/ros2_ws/src      # Create a workspace , builld and source it 
   cd ~/ros2_ws
   colcon build --symlink install
   source install/local_setup.bash

   cd /pi_ws/src      # To clone the repo inside src folder 
   git clone https://github.com/TahsinOP/eYSIP-24_Field_Exploration_ROS_Vehicles.git

   ```
   To avoid sourcing everytime you open a new terminal instance run this once ,
   ```bash
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc      # source bashrc as we have made changes
   ```

## Interfacing Hardware 

### Raspberry Pi 5 
- **Flash SD Card**
1. **Download Ubuntu 24:** 
   Download the ``Ubuntu 24`` image for Raspberry Pi from the official Ubuntu website.
2. **Flash the SD Card:** 
   Use tools like ``Balena Etcher``/``Rpi Imager`` to flash the Ubuntu image to a ``32GB`` or ``64GB`` SD card.

- **Install ROS 2 Jazzy**
1. **Follow the ROS 2 Installation Guide:** 
   Official ROS 2 Jazzy installation instructions can be found [here](https://docs.ros.org/en/ros2/Installation.html).

- **Workspace Setup**
1. **Create ROS 2 Workspace:**
   ```bash
   mkdir -p ~/pi_ws/src
   cd ~/pi_ws
   colcon build --symlink install
   source install/local_setup.bash
   
   echo "source ~/pi_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc      # source bashrc as we have made changes
   ```
   
2. **Clone the repository**
   ```bash
   cd /pi_ws/src
   git clone https://github.com/TahsinOP/eYSIP-24_Field_Exploration_ROS_Vehicles.git
   ```
- **USB serial Permissions** 
**Add ``dialout``to ``groups``**

  ```bash
   sudo usermod -aG dialout $USER
   ```

### BMX 160 IMU
#### Dependencies
1. **Install smbus :**
   ```bash
   sudo apt update
   sudo apt install python3-smbus
   ```
#### IMU node
2. **Launch IMU node:**
   ```bash
   ros2 launch imu_pkg imu_rviz_robot.launch.py
   ```
#### Logging Data    
3. **Check IMU data:**
   ```bash
   ros2 topic echo /imu_raw
   ```
### Ydlidar Tmini-Pro
### Connect and Launch the Lidar driver 
1. **Ensure the YDLidar is connected to a USB port and powered on**

2. **Launch the YDLidar node with the following command:**
   ```bash
   ros2 launch ydlidar_ros2_driver ydlidar_launch.py
   ```
3. **Visualize Lidar scan data:**
   ```bash
   ros2 launch rcar_viz display.launch.py
   ````
   This will launch ``Rviz2`` with a robot model and ``fixed frame`` as ``"odom"``

4. **Optional: Run IMU Pose Estimation:**
   To see the Odom data, you can run the following node (optional):
   ```bash
   ros2 run imu_pose_estimation estimator
    ```
    Alternatively, set the fixed frame to ``"base_link"`` in RViz to see laser data on the ``/scan``topic.

   
          
   



## Key Challenges
- **Low-level Control:** The primary challenge we faced in low-level control was interfacing a custom servo with five pins for which no pin diagrams or references were available online. Achieving accurate control at this level required extensive experimentation and fine-tuning.
- **Localization and Odometry:** We relied solely on an IMU to estimate the robot’s pose for localization and odometry. The IMU’s orientation measurements exhibited significant drift over time. Additionally, the robot’s suspension system continuously affected the accelerometer's bias.
- **Motion Planning:** In motion planning, traditional control algorithms used for differential or omnidirectional drives were ineffective. This required the development of an approach tailored to the dynamics of an Ackermann steering system.
- **Environment Mapping:** For environment mapping, we had to adapt mapping techniques to work with Ackermann kinematics. This included ensuring accurate data collection from LIDAR and integrating these readings into a coherent map.
  **Figure 2:** Vicon Odometry vs IMU Odometry

## Hardware Development

- The motor actuation and steering system features a ``five-pin servo`` control using ``PID`` for precision and DC motor control for throttle.
- A custom ``PCB`` board was developed to ensure optimum power supply to the ``ESP32``, ``motor drivers``, and ``aspberry Pi``.
- The mechanical design involved creating laser-cut CAD models using acrylic for a ``two-layered system`` with mounts for all hardware components including a ``LIDAR``.
- ``BMX-160`` IMU was interfaced and calibrated to publish accurate orientation and acceleration data on a ROS-2 network.
- ``Ydlidar Tmini-Pro`` interfaced with ``Raspberry Pi5`` on ``ROS2 Jazzy`` to publish laser data on ``/scan`` topic 

![car](https://github.com/user-attachments/assets/293aae25-fba5-43d2-993a-406b976a6d7d)

  **Figure 1:** Prototype Vehicle

## Control System Design 


<img src="https://github.com/user-attachments/assets/86c5c324-89aa-4556-b372-714f7f6e5913" alt="System Design" width="1000" height="800"/>



Figure 2: Control System Design 

### 1. Low-Level Control

- **ESP32**:
  - **Role**: Acts as the intermediary between the motor drivers and the Raspberry Pi 5.
  - **Communication**: Receives ``angle`` and ``PWM``values via ``UART Serial`` from the Raspberry Pi 5.
  - **Outputs**: Sends control signals to the motor drivers.

- **Motor Drivers**:
  - **Role**: Control the steering and throttle motors based on signals received from the ESP32.

- **Servo Motor (Steering)**:
  - **Role**: Adjusts the ``steering angle`` of the vehicle.
  - **Control**: Receives signals from the motor driver.

- **DC Motor (Throttle)**:
  - **Role**: Controls the ``throttle`` of the vehicle.
  - **Control**: Receives signals from the motor driver.

### 2. Onboard Raspberry Pi 5

- **Operating System**: Ubuntu 24LTS
- **Optimized Pure Pursuit Algorithm**:
  - **Role**: Calculates the desired ``angle`` for steering based on the current ``position`` and the ``goal``.
  - **Output**: Angle and PWM values sent to the ESP32 for motor control.

- **BMX160 IMU**:
  - **Role**: Provides IMU data for odometry.
  - **Connection**: Connected via ``GPIO`` pins on Rasbperry Pi 5.
  - **Data Published**: `/imu_data` topic.

- **YDLidar Tmini Pro**:
  - **Role**: Provides ``LiDAR`` data for SLAM.
  - **Connection**: Connected via ``USB``.
  - **Data Published**: `/scan` topic.

- **Odometry (Modified Kalman Filter)**:
  - **Role**: Processes ``IMU`` and other ``Vicon`` car velocity (corresponding to PWM) data to provide an accurate estimate of the vehicle’s ``position`` and ``velocity``.
  - **Output**: Publishes odometry data to the `/odom` topic.

### 3. Remote PC

- **Operating System**: ROS2 Jazzy
- **SLAM (Simultaneous Localization and Mapping)**:
  - **Components**: ``EKF (Extended Kalman Filter)`` and ``AMCL (Adaptive Monte Carlo Localization)``.
  - **Role**: Utilizes LiDAR and odometry data to build and update a ``map`` of the environment.
  - **Inputs**: Receives `/scan` and `/odom` topics.

- **Navigation Stack (Nav2)**:
  - **Role**: Handles ``path planning``and ``navigation``.
  - **Inputs**: Receives odometry data and processed map information.
  - **Outputs**: Publishes path data to the `/plan` topic.

- **Goal Node**:
  - **Role**: Defines the navigation ``goals` for the vehicle.
  - **Output**: Publishes ``waypoints`` to the `/goal_topic`.

### Summary

- The **Low-Level Control** segment, managed by the ESP32, translates commands from the Raspberry Pi into physical movements of the vehicle.
- The **Onboard Raspberry Pi 5** is the central processing unit for sensor data collection and preliminary processing (IMU, LiDAR), and it runs the Pure Pursuit algorithm for immediate steering adjustments.
- The **Remote PC** is responsible for advanced processing tasks, including SLAM for mapping and localization, and the Nav2 stack for path planning and navigation, ensuring the vehicle reaches its defined goals efficiently.




## ROS2-Teleoperation
- Achieved complete low-level control of the car through keyboard commands using ROS2 for communication over a shared network.
- Implement teleoperation nodes in ROS 2 to send commands to the robot.

**Implementation Steps:**
1. Install necessary ROS 2 packages for teleoperation.
2. Write a ROS 2 node to capture keyboard inputs and publish velocity commands.
3. Configure the robot to subscribe to these velocity commands and actuate the motors accordingly.
4. Test the teleoperation setup to ensure reliable and responsive control.

## Localization and Odometry
- **Modified Kalman Filter:** Fused car velocity data (calculated from Vicon) with IMU accelerometer readings to get a more accurate state estimate (position and velocity) by reducing drift.

## Motion Planning
- **Optimized Pure Pursuit Algorithm:** Implemented an optimized version of this path-tracking algorithm to ensure appropriate steering angles using the lookahead distance concept.

## SLAM
- **SLAM:** Mapped the environment using LiDAR scan data and fused it with odometry data. This, along with the Adaptive Monte Carlo Localization (AMCL) algorithm, provided an accurate pose (position and orientation) estimate of the car within the map.

**Figure 3:** Prototype Vehicle
