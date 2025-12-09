# Autonomous Agricultural UAV: Variable Mass PID Control System

![Status](https://img.shields.io/badge/Status-Completed-success)
![Platform](https://img.shields.io/badge/Platform-STM32%20%2F%20Pixhawk-blue)
![Simulation](https://img.shields.io/badge/Simulation-MATLAB%20%2F%20Simulink-orange)

## 🚁 Project Overview
This project addresses a critical aerodynamic challenge in heavy-lift agricultural drones: **Variable Mass Dynamics**.

Standard PID controllers assume a constant mass. However, an agricultural drone spraying pesticide loses up to **50% of its total mass** (from 22kg to 12kg) during a single mission. This rapid change in inertia causes standard controllers to overshoot, leading to oscillation or instability.

**The Solution:**
I designed a **"Digital Twin"** simulation and an embedded control architecture that dynamically adjusts the control law based on real-time mass estimation, ensuring stable flight throughout the spraying lifecycle.

---

## 🏗️ System Architecture

The system bridges high-level mission planning with low-level actuator control.

### Hardware-Software Interface
![System Architecture](./block_diagram.png)
*(Note: Ensure your block diagram image is named 'block_diagram.png')*

* **Flight Controller:** Cube Orange (Pixhawk 2.1) running ArduCopter firmware.
* **Sensors:** IMU (Accel/Gyro), Barometer, GPS Module.
* **Actuators:** T-Motor U8 Series (135 KV) + Sprayer Pump Mechanism.
* **Communication:**
    * **I2C:** Sensor data acquisition.
    * **PWM:** ESC Motor control (80A High-Voltage).
    * **UART:** Telemetry and Optical Flow integration.

---

## 🧮 Control Logic & Mathematics

### 1. The Variable Mass Problem
The drone's mass changes continuously as a function of the flow rate ($q$) and time ($t$):

$$m(t) = m_{initial} - \int_{0}^{t} q(\tau) d\tau$$

* **Initial Mass:** 22 kg
* **Flow Rate:** 0.05 kg/s

### 2. The Adaptive Control Law
To compensate for the changing $m(t)$, the PID loop includes a mass-dependent term to adjust the throttle output ($U_1$).

**Standard PID Equation:**
$$U_1(t) = K_p \cdot e(t) + K_i \int e(\tau)d\tau + K_d \frac{de(t)}{dt}$$

**Tuned Gains:**
* $K_p = 25$ (Proportional)
* $K_i = 10$ (Integral - Critical for mass compensation)
* $K_d = 45$ (Derivative)

---

## 📊 Simulation Results (MATLAB/Simulink)

We validated the control law using a high-fidelity Simulink model. The results demonstrate the controller's robustness against dynamic physical changes.

### 1. The Challenge: Mass Depletion
![Mass Depletion Graph](./mass_depletion.png)
> **Physics:** The graph above shows the drone's total mass decreasing linearly from **22kg to 12kg** over a 200-second mission duration due to pesticide spraying.

### 2. The Solution: Altitude Stability
![Altitude Response Graph](./altitude_response.png)
> **Control Response:** Despite the 45% loss in mass (shown above), the PID loop successfully maintains the target altitude of **-30m**.
> * **Settling Time:** ~5.0 seconds
> * **Overshoot:** 0% (Stable Hover)
> * **Steady State Error:** 0.00 m

---

## 💻 Tech Stack
* **Embedded:** C/C++, ArduCopter Firmware
* **Simulation:** MATLAB R2024a, Aerospace Blockset, Simulink
* **Hardware:** Octocopter Frame (1400mm), 22,000 mAh 6S Li-Po Battery

---

## 👤 Author
**Pravar Dev**
* **Focus:** Embedded Systems Architecture & Flight Dynamics
* [LinkedIn Profile](https://www.linkedin.com/in/pravar-dev-354bb82a7/)
