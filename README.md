# EKFusion2D — Extended Kalman‑Filter Sensor Fusion for 2D Localization

![EKFusion2D Demo Plot](docs/ekf_demo_plot.png)
_A sample visualization of ground truth, noisy sensor data, and the EKF's estimated path with covariance ellipses._

## Table of Contents
* [Introduction](#introduction)
* [About the Project](#about-the-project)
    * [The Challenge](#the-challenge)
    * [Our Solution: The Extended Kalman Filter](#our-solution-the-extended-kalman-filter)
    * [Key Features](#key-features)
* [Mathematical Formulation](#mathematical-formulation)
    * [Motion Model (Prediction)](#motion-model-prediction)
    * [Measurement Models (Update)](#measurement-models-update)
* [Getting Started](#getting-started)
    * [Prerequisites](#prerequisites)
    * [Building the C++ Project](#building-the-c-project)
    * [Running the Simulation & EKF](#running-the-simulation--ekf)
    * [Generating Plots](#generating-plots)
* [Project Structure](#project-structure)
* [Results & Analysis](#results--analysis)
* [Future Work](#future-work)
* [Contributing](#contributing)
* [License](#license)
* [Contact](#contact)

---

## Introduction

**EKFusion2D** is a robust C++ implementation of an **Extended Kalman Filter (EKF)** for 2D vehicle localization. This project demonstrates how to effectively fuse diverse, noisy sensor data—specifically simulated **wheel odometry** (linear and angular velocities) and **GPS/IMU-style** (position and orientation) measurements—to achieve a highly accurate and reliable estimate of a vehicle's true state (x, y, and heading angle $\theta$) over time.

This repository serves as a practical, hands-on demonstration of fundamental concepts in robotics state estimation and sensor fusion, crucial for autonomous systems operating in uncertain environments.

## About the Project

### The Challenge

Autonomous robots and vehicles must have precise knowledge of their position and orientation to navigate safely and effectively. However, real-world sensors are inherently imperfect. Wheel odometry accumulates drift, while GPS signals can be intermittent, inaccurate in certain areas, or suffer from multipath effects. Relying on any single sensor often leads to unreliable state estimates. The core problem is to intelligently combine these disparate, noisy data streams to derive a more robust, precise, and continuous understanding of the system's true state.

### Our Solution: The Extended Kalman Filter

**EKFusion2D** addresses this challenge by employing the **Extended Kalman Filter**, a powerful recursive algorithm widely adopted in robotics, aerospace, and control systems. The EKF operates in two primary phases:

1.  **Prediction Phase:** Based on a non-linear motion model and the previous state estimate, the filter predicts the vehicle's next state and updates its associated uncertainty (covariance).
2.  **Update Phase:** When new sensor measurements become available, the filter corrects its prediction by optimally blending the measurement with the predicted state. This process reduces uncertainty and refines the state estimate, leveraging the measurement and motion model Jacobians to linearize the non-linear dynamics around the current estimate.

### Key Features

* **Custom C++ EKF Implementation:** A from-scratch implementation of the Extended Kalman Filter in C++, leveraging the high-performance **Eigen** library for efficient matrix operations.
* **Realistic 2D Simulation Environment:** Simulates ground truth vehicle motion using a constant velocity and constant angular velocity non-linear motion model.
* **Diverse Noisy Sensor Data Generation:**
    * **Wheel Odometry:** Simulates noisy linear and angular velocity readings with Gaussian noise.
    * **GPS/IMU:** Simulates noisy 2D position (x, y) and orientation ($\theta$) readings with Gaussian noise.
* **Robust Sensor Fusion:** Demonstrates the fusion of two distinct sensor types with different measurement models within a single EKF framework.
* **Comprehensive Data Output:** C++ EKF outputs ground truth, raw sensor data, and estimated states with full covariance matrices to a CSV file for post-processing.
* **Professional Data Visualization (Python/Matplotlib):**
    * Visualization of the **Ground Truth Path**, **Noisy Sensor Readings**, and the **Filtered EKF Estimate Path**.
    * Crucially, dynamic plotting of **Covariance Ellipses** at key points along the path, providing a clear visual representation of the filter's evolving uncertainty in the state estimate.
    * Analysis of filter consistency through plots of **Innovation (Residual)** and its **Normalized Squared Value ($\chi^2$ statistic)**, which helps validate the filter's noise assumptions.

## Mathematical Formulation

This section outlines the core equations used in the EKF for this project.

### Motion Model (Prediction)

The vehicle's state is defined as $X = [x, y, \theta]^T$, representing its 2D position and orientation.
The non-linear motion model used is:
$$
\begin{aligned}
x_{k+1} &= x_k + \frac{v}{\omega} (\sin(\theta_k + \omega \Delta t) - \sin(\theta_k)) \\
y_{k+1} &= y_k + \frac{v}{\omega} (-\cos(\theta_k + \omega \Delta t) + \cos(\theta_k)) \\
\theta_{k+1} &= \theta_k + \omega \Delta t
\end{aligned}
$$
where $v$ is the linear velocity and $\omega$ is the angular velocity. If $\omega \approx 0$, a linear approximation is used to avoid division by zero.

The **state transition Jacobian (Ft)**, $\mathbf{F}_t = \frac{\partial f(X_k, U_k)}{\partial X_k}$, is derived from this model to linearize it for the EKF prediction step.

### Measurement Models (Update)

1.  **Wheel Odometry (Velocity/Angular Velocity):**
    The measurement $Z_{odom} = [v_{odom}, \omega_{odom}]^T$.
    The measurement function $h_{odom}(X_k)$ is simply the velocity components extracted from the motion model.
    The **measurement Jacobian (Ht)**, $\mathbf{H}_{t,odom} = \frac{\partial h_{odom}(X_k)}{\partial X_k}$, is derived for this measurement.

2.  **GPS/IMU (Position/Orientation):**
    The measurement $Z_{gps} = [x_{gps}, y_{gps}, \theta_{gps}]^T$.
    The measurement function $h_{gps}(X_k)$ is simply $X_k$ itself.
    The **measurement Jacobian (Ht)**, $\mathbf{H}_{t,gps} = \frac{\partial h_{gps}(X_k)}{\partial X_k}$, for this simple case is an identity matrix.

Further details on the specific Jacobian derivations are available in the source code comments and potentially a dedicated `docs/` folder (highly recommended for a real project!).

## Getting Started

Follow these instructions to set up, build, and run the EKFusion2D project.

### Prerequisites

* **C++ Compiler:** C++11 or newer (e.g., g++ for Linux/macOS, MSVC for Windows).
* **CMake:** Version 3.10 or higher.
* **Eigen Library:** Will be managed by CMake or can be manually installed.
* **Python 3.x:** For plotting.
* **Python Libraries:** `matplotlib`, `numpy`, `pandas`. Install with pip:
    ```bash
    pip install matplotlib numpy pandas
    ```

### Building the C++ Project

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/0xphen/EKFusion2D.git](https://github.com/0xphen/EKFusion2D.git)
    cd EKFusion2D
    ```
2.  **Create a build directory and run CMake:**
    ```bash
    mkdir build
    cd build
    cmake ..
    ```
3.  **Build the project:**
    ```bash
    make # On Linux/macOS
    # Or for Visual Studio (Windows): open the generated .sln file in VS and build
    ```

### Running the Simulation & EKF

After building, an executable named `ekfusion2d_sim` (or similar, depending on your CMakeLists.txt) will be in your `build` directory.

```bash
./ekfusion2d_sim