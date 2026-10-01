<div align="center">

# Southeast University
## Department of Computer Science and Engineering
### Program: B.Sc. in CSE

**Course Code:** CSE382.8
**Course Title:** Introduction to Embedded Systems Lab
**Section:** 65

### Submitted to
**Jayed Bin Azad**
Lecturer, Southeast University

### Submitted by
**Group G**

| Name | Student ID |
| :--- | :--- |
| Abdullah Mohammad Arman | 2023200000750 |
| Md. Shanto Khan | 2023200000342 |
| Fahim-Al-Sharear | 2023200000354 |
| Md. Asib Khan | 2023200000364 |

**Contact Email:** 2023200000364@seu.edu.bd
**Date of Submission:** 06.09.2026

</div>

---

<div align="center">

## Autonomous Goal-Oriented & Pathfinding Robotic Rover
### with Bluetooth Navigation, Dead Reckoning, and Ultrasonic Radar Sweeping

</div>

### 1. Project Description
This project develops a fully autonomous smart obstacle-avoiding and pathfinding robotic rover powered by an **ESP-32U DevKit V1 microcontroller**. Unlike simple reactive robots, this rover navigates from a starting point (Point A) to a user-defined target destination (Point B) in unknown, dynamic indoor environments. It achieves this by continuously calculating its heading and distance to the goal, scanning its surroundings, identifying obstacles, and computing the most optimal, collision-free trajectory in real time.

The system utilizes the **ESP32's built-in Bluetooth** capabilities to interface with a custom web-based Remote Control application. To maintain precise straight-line movement, it employs a custom PID (Proportional-Integral-Derivative) controller acting on dual LM393 Wheel Encoders. For dynamic obstacle avoidance, it uses an HC-SR04 Ultrasonic Distance Sensor mounted on a 180-degree SG90 Micro Servo Motor. Under autonomous mode, the rover drives straight forward. When an obstacle is detected within a danger threshold (e.g., 25 cm):

*   **Immediate Brake:** The rover stops instantly to avoid physical collision.
*   **Radar Sweeping:** The servo pans the ultrasonic sensor 60° to the right and left to measure clearance distances.
*   **Smart Bypass:** The ESP32 evaluates the clearances, executes a differential turn toward the most open path, and drives forward to bypass the obstacle.
*   **Path Recalculation:** Once the obstacle is cleared, the rover updates its coordinates using the wheel encoders, recalculates the vector to Point B, and adjusts its heading.
*   **Dead-End & Trap Recovery:** If both left and right directions are obstructed (<20 cm), the rover sounds a buzzer warning, reverses, and pivots to escape the dead end.

An on-board 16×2 I2C LCD screen displays real-time navigation telemetry (current coordinates, distance to goal, and obstacle status). This robotic platform models industrial Automated Guided Vehicles (AGVs), autonomous delivery drones, and search-and-rescue reconnaissance systems.

### 2. Project Functionalities

| # | Functionality | Description |
| :--- | :--- | :--- |
| 1 | Wireless Web Controller | Provides real-time telemetry and full manual/autonomous control from a smartphone via Bluetooth Web API. |
| 2 | Autonomous Avoidance | Drives continuously while proactively scanning for and bypassing obstacles using dual DC motors. |
| 3 | Odometry & Dead Reckoning | Tracks the rover's exact (X,Y) position relative to its starting point using optical wheel encoders counting motor revolutions. |
| 4 | Closed-Loop PID Synchronization | Monitors the rover's wheel speeds and actively fights curvature to ensure perfect straight-line navigation. |
| 5 | Real-Time Distance Ranging | Continuously computes accurate distance to front obstacles using high-frequency ultrasonic ping timing. |
| 6 | Servo Radar Spatial Scanning | When blocked, rotates the sensor horizontally (left/right) to scan environmental clearance angles. |
| 7 | Intelligent Path Decision Logic | Automatically halts, evaluates clearances, bypasses obstacles, and resumes forward progression. |
| 8 | Live Telemetry LCD Display | Displays real-time position, distance to goal, and active maneuvers on a 16×2 I2C LCD module. |

### 3. Components / Hardware

| # | Component | Qty | Purpose |
| :--- | :--- | :--- | :--- |
| 1 | ESP-32U Development Board | 1 | Main controller with external U.FL antenna processing logic and Bluetooth Web comms. |
| 2 | L298N Dual H-Bridge Motor Driver | 1 | Drives high-current DC geared motors with bidirectional PWM speed regulation. |
| 3 | TT Dual-Axis DC Geared Motors + Wheels | 2 | Provides reliable differential drive traction and steering torque. |
| 4 | High-Torque Battery Shield | 1 | Supplies stable high-current 5V/7.4V power to eliminate microcontroller brownouts. |
| 5 | LM393 Speed Sensors (Encoders) | 2 | Optical sensors reading slotted motor discs to track distance traveled (Odometry). |
| 6 | HC-SR04 Ultrasonic Sensor | 1 | Emits 40kHz ultrasound bursts to measure obstacle distances from 2cm to 400cm. |
| 7 | SG90 Micro Servo Motor (180°) | 1 | Pans ultrasonic sensor to scan left and right clearance angles. |
| 8 | 16×2 I2C LCD Display Module | 1 | Provides on-chassis diagnostic display via 2-wire I2C communication. |
| 9 | 2WD Smart Robotic Chassis Kit | 1 | Acrylic frame, front caster wheel, motor mounts, and hardware fasteners. |
| 10 | Active Buzzer (5V) | 1 | Acoustic annunciator for reverse maneuvers, obstacle alerts, and startup chimes. |
| 11 | 18650 Li-ion Batteries (3.7V × 2) | 1 set | High-drain rechargeable power pack supplying motors and ESP32. |
| 12 | Mini Breadboard & Jumper Wires | ~40 | Facilitates circuit assembly, distribution rails, and interconnections. |
| 13 | SPST Power Switch | 1 | Master DC power ON/OFF switch for safe hardware operation. |
| 14 | 1kΩ & 2kΩ Resistors | 1 set | Voltage divider for HC-SR04 Echo pin (5V → 3.3V safe for ESP32). |


### 4. Expected Output

When the robotic rover is deployed and powered on:

1.  **Startup Initialization:** The LCD initializes with a greeting ("ROVER ACTIVE"), and the ESP32 begins broadcasting a Bluetooth Low Energy (BLE) server.
2.  **Awaiting Command:** The rover waits for a connection from the Web Controller. The user selects Manual or Auto mode.
3.  **Autonomous Navigation:** In AUTO mode, the dual encoders and PID controller forcefully synchronize the wheel speeds, driving the rover in a mathematically straight line.
4.  **Live Telemetry:** The rover constantly beams real-time data back to the app, displaying live wheel speeds (cm/s), front obstacle distance, and operating state.
5.  **Obstacle Detection & Bypass:** If an obstacle breaches the 25 cm threshold, the rover halts. The SG90 servo swings the HC-SR04 to check left and right clearances.
6.  **Path Recalculation:** The rover executes a pivot turn toward the clearer path, drives to bypass the object, and then restores its previous navigation vector.
7.  **Continuous Operation:** The rover will continue navigating and avoiding obstacles autonomously indefinitely until the user manually sends a STOP command via the Web Controller, safely parking it.

