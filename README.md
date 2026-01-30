# VEX Robotics: Team Error Error 🤖

This repository contains the software stack for **VEX Robotics Team Error Error**, developed for the *Spin Up* competition season. The project features a custom control architecture implemented in Python, focusing on autonomous precision, real-time physics simulations, and advanced state estimation.

## 🏗 System Architecture

The codebase is organized around a central `Robot` class that manages state estimation, subsystem control, and autonomous execution. It uses an asynchronous update loop to handle sensor polling and control calculations.

### 1. State Estimation & Sensor Fusion
The robot maintains a high-fidelity estimate of its field position by fusing data from multiple sensors.
*   **Coordinate System:** Real-time $(x, y, \theta)$ tracking using GPS for absolute references and wheel encoders for high-frequency relative displacement.
*   **Sensor Fusion Logic:** `Main/src/main.py` ([lines 1238-1327](Main/src/main.py#L1238)).
*   **Vector Math:** Custom `Vector` class and `rotate_vector_2d` utilities for coordinate frame transformations ([lines 292-329](Main/src/main.py#L292)).

### 2. Ballistics & Flywheel Control (Aimbot)
Automated scoring is handled via a kinematics engine that models the field in code to predict projectile paths.
*   **Field Modeling:** The robot uses a mathematical model of the game field to calculate the exact angle and velocity required to hit the target from any position. See `Main/src/main.py` ([lines 436-608](Main/src/main.py#L436)).
*   **Computer Vision Aimbot:** In addition to geometric modeling, the robot employs a Vision Sensor-based "aimbot" to track goal signatures and dynamically adjust heading. (`Main/src/main.py` [lines 2451-2495](Main/src/main.py#L2451)).
*   **Recovery PID:** A specialized PID loop designed for high-inertia flywheels. It features a "recovery mode" that ignores derivative noise during the 0.5s window following a disc launch to minimize RPM drop ([lines 1445-1502](Main/src/main.py#L1445)).

### 3. Automated PID Tuning via Gradient Descent
To optimize the flywheel PID constants ($k_P$, $k_I$, $k_D$), I developed a **Gradient Descent** algorithm.
*   **Iterative Optimization:** Instead of manual tuning, the algorithm iterates over flywheel performance data to identify the optimal constants that minimize RPM error and recovery time.
*   **Implementation:** The optimization logic is housed in `Main/gradient_descent.py`. This script implements a custom gradient calculation and parameter update loop to automate the tuning process.

### 4. Command-Based Autonomous Engine
Autonomous routines are defined as a sequence of state-target dictionaries, allowing for readable and modular scripting of complex behaviors.
*   **Execution Engine:** The `run_autonomous` function processes step-wise instructions with support for timeouts, tolerances, and asynchronous function callbacks ([lines 2088-2182](Main/src/main.py#L2088)).
*   **Example Routine:** `match_auto_two_squares` demonstrates a full competition path involving odometry-based movement and subsystem coordination ([lines 2616-2715](Main/src/main.py#L2616)).

### 5. Custom UI Framework (V5 Brain)
To facilitate on-field debugging and configuration, this project includes a lightweight UI framework built on the V5 touchscreen.
*   **UI Elements:** Implementation of `Button`, `Switch`, and `Text` classes with callback support ([lines 611-740](Main/src/main.py#L611)).
*   **Telemetry Dashboard:** A multi-page GUI providing real-time motor temperatures, battery voltage, and odometry data ([lines 3418-3510](Main/src/main.py#L3418)).

## 📊 Data & Analysis

The repository includes scripts used for offline performance tuning and scouting.
*   **Performance Tuning:** `Main/dataAnalyzing.py` processes telemetry logs using Pandas to identify drivetrain slip and flywheel consistency.
*   **Regional Scouting:** `Main/3dgraphs.py` visualizes regional competition data to track scoring trends and team progression.

## 🛠 Tech Stack
*   **Platform:** VEX V5 System
*   **Language:** Python 3.6 (VEX Python Runtime)
*   **Libraries:** Math, Time, VEX (Robot API), Pandas/Matplotlib (Offline Analysis)
*   **Control Theory:** PID Control, Gradient Descent Optimization, Alpha-Beta Filters, Kinematics

## 📁 Project Structure
*   `Main/src/main.py`: Primary robot control logic and UI implementation.
*   `Main/gradient_descent.py`: Autonomous PID tuning script.
*   `Main/kalmanFilter.py`: Prototype code for noise-resistant sensor fusion.
*   `Main/gps_math.py`: Pure-math utilities for field geometry.

---
*Developed by Team Error Error*
