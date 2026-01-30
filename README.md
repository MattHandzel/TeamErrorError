# VEX Robotics: Team Error Error 🤖

This repository contains the software stack for **VEX Robotics Team Error Error**, developed for the *Spin Up* competition season. The project features a custom control architecture implemented in Python, focusing on autonomous precision, real-time physics simulations, and advanced state estimation.

## 🏗 System Architecture

The codebase is organized around a central `Robot` class that manages state estimation, subsystem control, and autonomous execution. It uses an asynchronous update loop to handle sensor polling and control calculations.

### 1. State Estimation & Sensor Fusion
The robot maintains a high-fidelity estimate of its field position by fusing data from multiple sensors.
*   **Coordinate System:** Real-time $(x, y, \theta)$ tracking using GPS for absolute references and wheel encoders for high-frequency relative displacement.
*   **Sensor Fusion Logic:** [`Main/src/main.py` (lines 1238-1327)](Main/src/main.py#L1238)
*   **Vector Math:** Custom `Vector` class and `rotate_vector_2d` utilities for coordinate frame transformations in [`Main/src/main.py` (lines 292-329)](Main/src/main.py#L292).

### 2. Ballistics & Flywheel Control (Aimbot)
Automated scoring is handled via a kinematics engine that models the field in code to predict projectile paths.
*   **Field Modeling:** The robot uses a mathematical model of the game field to calculate the exact angle and velocity required to hit the target from any position. See [`Main/src/main.py` (lines 436-608)](Main/src/main.py#L436).
*   **Computer Vision Aimbot:** In addition to geometric modeling, the robot employs a Vision Sensor-based "aimbot" to track goal signatures and dynamically adjust heading. [`Main/src/main.py` (lines 2451-2495)](Main/src/main.py#L2451)
*   **Recovery PID:** A specialized PID loop designed for high-inertia flywheels. It features a "recovery mode" that ignores derivative noise during the 0.5s window following a disc launch to minimize RPM drop. [`Main/src/main.py` (lines 1445-1502)](Main/src/main.py#L1445)

### 3. Trajectory Design & Pathing Library
I developed a custom toolchain for designing and executing autonomous paths.
*   **Visual Path Designer:** A Pygame-based desktop application [`Main/drawtrajectories.py`](Main/drawtrajectories.py) allows for drawing robot trajectories over a digital field map and exporting them as coordinate data.
*   **Autonomous Recording & Replay:** The system supports a "Recording Mode" where driver movements are sampled and stored as a series of state-targets, allowing complex manual maneuvers to be replayed as autonomous routines. [`Main/src/main.py` (lines 2188-2202)](Main/src/main.py#L2188)
*   **Intersection Geometry:** Uses line-segment intersection algorithms to determine when the robot has passed specific checkpoints. [`Main/src/main.py` (lines 1362-1372)](Main/src/main.py#L1362)

### 4. PID Optimization via Custom Gradient Descent
To identify the optimal flywheel PID constants ($k_P$, $k_I$, $k_D$), I developed a **Gradient Descent** algorithm from scratch.
*   **Methodology:** I implemented the optimization algorithm to process flywheel performance data and minimize RPM error. This allowed for a more mathematical approach to tuning than traditional trial-and-error.
*   **Implementation:** [`Main/gradient_descent.py`](Main/gradient_descent.py) contains the from-scratch implementation of the Gradient Descent algorithm, including custom vector math and gradient calculation logic.
*   **Production Constants:** The resulting optimized constants are implemented in the flywheel control loop in [`Main/src/main.py` (lines 1460-1462)](Main/src/main.py#L1460).

### 5. Physical Modeling & Simulation
Before testing on hardware, I built mathematical models to predict robot performance limits.
*   **Drivetrain Modeling:** [`Main/robot_speed_calculator.py`](Main/robot_speed_calculator.py) models motor torque curves, robot weight, and coefficients of friction to calculate theoretical maximum velocities and acceleration profiles.

### 6. Custom UI Framework (V5 Brain)
To facilitate on-field debugging and configuration, this project includes a lightweight UI framework built on the V5 touchscreen.
*   **UI Elements:** Implementation of `Button`, `Switch`, and `Text` classes with callback support in [`Main/src/main.py` (lines 611-740)](Main/src/main.py#L611).
*   **Telemetry Dashboard:** A multi-page GUI providing real-time motor temperatures, battery voltage, and odometry data. [`Main/src/main.py` (lines 3418-3510)](Main/src/main.py#L3418)

## 📊 Data & Analysis

The repository includes scripts used for offline performance tuning and scouting.
*   **Performance Tuning:** [`Main/dataAnalyzing.py`](Main/dataAnalyzing.py) processes telemetry logs using Pandas to identify drivetrain slip and flywheel consistency.
*   **Regional Scouting:** [`Main/3dgraphs.py`](Main/3dgraphs.py) visualizes regional competition data to track scoring trends and team progression.

## 🛠 Tech Stack
*   **Platform:** VEX V5 System
*   **Language:** Python 3.6 (VEX Python Runtime), Python 3.10+ (Desktop Tools)
*   **Libraries:** Pygame (Path Design), Pandas/Matplotlib (Offline Analysis), NumPy
*   **Control Theory:** PID Control, Gradient Descent Optimization, Alpha-Beta Filters, Kinematics

## 📁 Project Structure
*   [`Main/src/main.py`](Main/src/main.py): Primary robot control logic and UI implementation.
*   [`Main/drawtrajectories.py`](Main/drawtrajectories.py): Pygame pathing tool.
*   [`Main/gradient_descent.py`](Main/gradient_descent.py): From-scratch implementation of Gradient Descent used for PID tuning.
*   [`Main/robot_speed_calculator.py`](Main/robot_speed_calculator.py): Drivetrain physics simulator.
*   [`Main/kalmanFilter.py`](Main/kalmanFilter.py): Prototype code for noise-resistant sensor fusion.

---
*Developed by Team Error Error*
