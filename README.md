# LEGO-EV3-Self-Driving-Car-Perfect-Mission-with-Pybricks-OpenMV

[![Pybricks](https://img.shields.io/badge/Powered%20by-Pybricks-blue.svg)](https://pybricks.com/)
[![Language](https://img.shields.io/badge/Language-MicroPython-yellow.svg)](https://micropython.org/)

This project is an advanced autonomous driving robot built with LEGO Mindstorms EV3 and the Pybricks framework. It goes beyond simple line following to systematically execute complex, sequential missions using a State Machine design. A key feature is its integration with an external OpenMV camera to process visual information, enabling it to handle sophisticated driving scenarios with precision.

## 🎥 Demonstration Video

See the robot in action! Click the image below to watch the full mission on YouTube.

[![EV3 Autonomous Robot Video](https://img.youtube.com/vi/F8zVAMhfwYk/0.jpg)](https://www.youtube.com/watch?v=F8zVAMhfwYk)



## 🎯 Project Goals and Features

*   **Complex Mission Execution:** Implements scenario-based autonomous driving, performing a sequence of distinct actions like stopping, reversing, accelerating, decelerating, and waiting for signals.
*   **State-Based Design:** Utilizes a State Machine centered around the RobotState class. This ensures robust and extensible code by isolating tasks and preventing convoluted logic.
*   **External Hardware Integration:** Communicates with an OpenMV camera in real-time via UART to recognize external environmental cues (like traffic lights) that are beyond the capabilities of standard EV3 sensors.
*   **Custom P-Control Logic:** Implements a **unique Proportional-control algorithm that compares the average values of left and right sensor groups**, enabling stable and responsive line tracing without relying on a central sensor.
*   **Time-Based Sequential Actions:** Precisely executes multi-stage, time-based missions (e.g., boost → normal speed → final boost) in a non-blocking manner using the StopWatch utility.

---

## 📜 Mission Scenario: Detailed Breakdown

The robot starts at the starting line and performs the following missions sequentially as it passes five red lines.

1.  **Start & Basic Driving**
    *   Pressing the Center Button initiates line tracing at the DRIVE_SPEED_LINETRACE speed.

2.  **School Zone (On Yellow Line Detection)**
    *   Upon first detecting a yellow line, the robot stops immediately.
    *   It flashes the EV3 brick's LEDs as a warning signal.
    *   **Phase 1:** Proceeds through the school zone at normal speed for 7 seconds.
    *   **Phase 2:** Accelerates to maximum speed (DRIVE_SPEED_MAX_LINETRACE) for the next 3 seconds to quickly exit the zone.

3.  **2nd Red Line: Boost Section**
    *   When the 2nd red line is detected, the robot boosts to maximum speed for SECOND_RED_BOOST_DURATION_MS (2.6 seconds).
    *   It then returns to normal speed.

4.  **3rd Red Line: Traffic Light Wait**
    *   The robot stops immediately upon detecting the 3rd red line.
    *   It reverses REVERSE_DISTANCE_CM (10 cm) to position itself accurately at the stop line.
    *   It enters the TRAFFIC_LIGHT_WAITING state and waits for a signal from the OpenMV camera.
    *   Once the OpenMV camera sends the string "two" (signifying a green light) via UART, the robot resumes line tracing.

5.  **4th Red Line: Complex Maneuver**
    *   Detecting the 4th red line triggers a predefined 3-stage sequence.
    *   **Stage 1 (Initial Boost):** Accelerates to maximum speed for FOURTH_RED_INITIAL_BOOST_MS (1.5 seconds).
    *   **Stage 2 (Cruise):** Drives at normal speed for POST_BOOST_WAIT_DURATION_MS (12.5 seconds).
    *   **Stage 3 (Final Boost):** Executes a final burst at maximum speed for FINAL_BOOST_DURATION_MS (1 second).

6.  **5th Red Line: Mission Complete**
    *   Upon detecting the 5th red line (the finish line), the robot stops all motors.
    *   It displays a Mission Complete! message and plays a victory tune to conclude the run.

---

## 🛠️ Hardware and Software

### Hardware Configuration

| Port            | Device                                  | Role and Description                                           |
|-----------------|-----------------------------------------|----------------------------------------------------------------|
| **Motor Port A**  | EV3 Large Motor                         | **Drive Motor:** Powers the robot's forward and reverse movement. |
| **Motor Port D**  | EV3 Large/Medium Motor                  | **Steering Motor:** Controls the angle of the front wheels for turning. |
| **Sensor Port 1** | Mindsensors LSA (Line Sensor Array) | **Line Sensor:** Precisely measures the line's position with 8 IR sensors (I2C Address: 0x14). |
| **Sensor Port 3** | OpenMV Cam (via UART)                   | **External Camera:** Recognizes traffic lights and sends "one", "two", or "three" via UART (Baud: 115200). |
| **Sensor Port 4** | EV3 Color Sensor                        | **Color Sensor:** Detects the red and yellow lines on the track. |

### Software and Installation Guide

1.  **Install Pybricks Firmware:**
    *   Flash your EV3 brick with the latest firmware from the [official Pybricks website](https://pybricks.com/install/).

2.  **Install Required Library:**
    *   This project requires the mindsensorsPYB library for the LSA sensor.
    *   Connect to your EV3 via SSH or use the terminal in a Pybricks-compatible IDE (like VS Code) and run:
    
bash
    micropython -m pip install mindsensorsPYB


3.  **Upload Project Code:**
    *   Download the main.py file from this repository to your EV3.
    *   **OpenMV Setup:** The OpenMV camera must be programmed separately with a MicroPython script that recognizes traffic light signals (e.g., numbers 1, 2, 3) and sends the corresponding strings ("one", "two", "three") over UART.

---

## 🔧 Technical Deep Dive: The Custom P-Control Logic

The line tracing in this robot does not use a typical single-sensor method or a standard center-weighted PID controller. Instead, it implements a **custom Proportional (P) control algorithm that leverages the outer sensor groups of the LSA (Line Sensor Array)**. This design achieves both high stability and excellent responsiveness.

Here is a step-by-step breakdown of how the algorithm works:

### 1. Sensor Grouping and Ignoring the Center
The 8 sensors of the LSA [0, 1, 2, 3, 4, 5, 6, 7] are divided into three groups:
*   **Left Group:** [0, 1, 2]
*   **Right Group:** [5, 6, 7]
*   **Center Group:** [3, 4] (These are **intentionally ignored** in the calculation).

**Design Rationale:** By ignoring the center sensors, the algorithm encourages the robot to **straddle the line between the two outer sensor groups** rather than trying to stay perfectly centered. This approach is less susceptible to minor line imperfections or noise and provides a more robust and intuitive control mechanism based on balancing the two groups.

### 2. Calculating Group 'Darkness' Averages
The algorithm averages the 'darkness' level for each group. The lsa.ReadRaw_Calibrated() function returns a normalized value from 0 (brightest) to 100 (darkest).
*   left_dark = (hap[0] + hap[1] + hap[2]) / 3
*   right_dark = (hap[5] + hap[6] + hap[7]) / 3

left_dark represents how much the left group is on the black line, and right_dark represents the same for the right group.

### 3. Calculating the Error
The error is defined as the difference between the two group averages.
error = left_dark - right_dark

*   If **error > 0**: The left group is darker than the right. This means the robot is **drifting to the right**, and the line is on its left. -> It needs to **steer left**.
*   If **error < 0**: The right group is darker than the left. This means the robot is **drifting to the left**, and the line is on its right. -> It needs to **steer right**.
*   If **error ≈ 0**: Both groups have a similar darkness level. The robot is **perfectly balanced** on the line.

### 4. Dynamic Kp and Calculating the Correction
The final steering adjustment is calculated by multiplying the error by a proportional gain (Kp).
correction = current_kp * error

Crucially, the Kp value is changed dynamically based on the robot's speed to maximize stability.
*   **Normal Speed:** current_kp = STEERING_KP_LINETRACE (e.g., -7.0)
*   **High-Speed Mode:** current_kp = STEERING_KP_LINETRACE * HIGH_SPEED_KP_FACTOR (e.g., -7.0 * 0.1 = -0.7)

**Design Rationale:** At high speeds, a sensitive steering response can cause the robot to oscillate violently (overshooting). By intentionally reducing the Kp value, the steering becomes less aggressive, enabling a smoother and more stable ride even at maximum velocity.

### 5. Clamping the Steering Angle
The calculated correction value is capped to a maximum and minimum to prevent it from exceeding physical limits.
correction = max(-90, min(90, correction))

This prevents sending impossible commands to the motor and reduces the risk of tipping over on sharp turns by limiting extreme steering angles.

### Summary: Control Logic Flowchart

[Collect 8 Sensor Values] -> [Split into Left/Right Groups] -> [Calculate Group Averages] -> [Calculate Error] -> [Check Current Speed] -> [Determine Dynamic Kp] -> [Calculate Correction] -> [Clamp Steering Angle] -> [Execute Steering Motor]


This custom P-control method is a core technology of the project, delivering fast and reliable line tracing performance without the need for a complex PID tuning process.

---

## ⚙️ Configuration and Tuning Guide

The robot's performance can vary depending on environmental factors like track conditions and ambient lighting. You can optimize the robot by modifying the constants defined in the **configuration section** at the top of the code.

| Constant Name                     | Description                                                              | Tuning Advice                                                                                                                                      |
|-----------------------------------|--------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| DRIVE_SPEED_LINETRACE           | The robot's normal driving speed.                                        | If set too high, cornering may become unstable. Adjust between 200-400.                                                                             |
| DRIVE_SPEED_MAX_LINETRACE       | The maximum speed during boost sections.                                 | Set this within the motor's capabilities. Too high a value increases the risk of derailing.                                                        |
| STEERING_KP_LINETRACE           | The sensitivity of the line follower (the Kp gain).                      | **(Most Important)** This is a negative value. A larger absolute value means a faster response. Too high, and it will zigzag; too low, and it will fail to turn. Fine-tune between -5.0 and -10.0. |
| HIGH_SPEED_KP_FACTOR            | The factor to reduce Kp by during high-speed driving.                    | Set between 0.1 and 0.3 to ensure stability at high speeds.                                                                                        |
| *_BOOST_DURATION_MS             | The duration of time-based actions (boosts, waits, etc.) in milliseconds.| Adjust these values to meet specific mission requirements.                                                                                         |
| RED_*_MIN/MAX, YELLOW_*_MIN/MAX | The RGB thresholds for the Color Sensor to detect colors.                | Highly sensitive to ambient lighting. If color detection fails, these values **must** be recalibrated by observing real-time RGB values in Pybricks. |

## 📄 License

This project is distributed under the MIT License. You are free to use, modify, and distribute it.
