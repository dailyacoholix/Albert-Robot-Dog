# 🐾 Albert: The Modular Robot Dog Framework

**Albert** is a 3D-printed, ESP32-powered quadruped robot dog designed for smooth, organic movement and modular extensibility. This repository contains the core firmware built to handle complex gait cycles, inverse kinematics-style positioning, and remote control via Bluetooth.



## 🚀 Key Features

* **Sinusoidal Gait Engine**: Uses mathematical sine waves to produce fluid, lifelike walking patterns rather than jerky linear movements.
* **Synchronized Interpolation**: A blocking move system that ensures all legs reach their target position simultaneously, regardless of the distance traveled.
* **Dual-Control Interface**: Supports commands via Serial (USB) and Bluetooth (ESP32).
* **Manual Override**: A custom parser allows for real-time manual servo positioning (e.g., `0,325-15,400`).
* **Pre-programmed Poses**: Built-in sequences for "Gallop," "Handshake," "Sit," and "Top" poses.

## 🛠️ Hardware Requirements

* **Microcontroller**: ESP32 (recommended) or Arduino-compatible board.
* **Servo Driver**: Adafruit PCA9685 16-Channel 12-bit PWM Driver.
* **Servos**: 8x for legs (2 degrees of freedom per leg) + 1x for the neck.
* **Power**: 5V High-current source (A hacked 5V Power Bank is a great budget solution).



## 💻 Software Architecture

The code is organized into several distinct layers:

1.  **Configuration**: Define servo limits (`SERVOMIN`/`SERVOMAX`) and center points.
2.  **Interpolation Engine**: `moveUntilReachedAll()` handles smooth transitions between static poses.
3.  **Gait Engine**: `runWalkSequence()` uses a frequency/amplitude model to drive the legs in a continuous loop.
4.  **Command Parser**: Interprets string commands into specific robot modes or manual servo movements.



## 🕹️ Control Commands

Commands can be sent via **Serial Monitor** (115200 baud) or **Bluetooth Serial**:

| Command | Action |
| :--- | :--- |
| `WALK` | Walk forward in a continuous loop |
| `LEFT` / `RIGHT` | Dynamic turning while moving |
| `LS` / `RS` | Spin on the spot (Left/Right) |
| `STOP` | Immediate stop and hold current position |
| `UP` / `DOWN` | Adjust stand height |
| `GALLOP` | A high-energy forward lunging sequence |
| `HAND` | Perform a handshake gesture |

**Manual Mode Example:** `0,400-1,200-15,350` (Moves Servo 0 to 400, Servo 1 to 200, and Neck to 350 simultaneously).

## 📈 Gait Logic (The Math)

Albert's walk is powered by a sinusoidal oscillator:

$$pulse = Center + Offset + (Amplitude \cdot \cos(t + Phase))$$



By shifting the `Phase` for each leg, we create the classic "trot" gait. The lift servos (odd indices) use a rectified cosine function to ensure the feet only move upward during the swing phase.

## 📥 Installation

1.  Clone this repository.
2.  Install the **Adafruit PWM Servo Driver Library** via the Arduino Library Manager.
3.  Ensure the **ESP32 Board Manager** is installed in your IDE.
4.  Upload the code and open the **Serial Plotter** to see the gait waves in real-time.

---

### 🤝 Contributing
Albert is open-source! Feel free to fork the repo, add new gait patterns, or improve the LLM-generated logic. Created by **Thinking Things**.
