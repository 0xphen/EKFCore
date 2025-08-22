# EKFCore — A Templated Extended Kalman Filter Library

![EKFCore Demo Plot](docs/ekf_demo_plot.png)
_A sample visualization of ground truth, noisy sensor data, and the EKF's estimated path with covariance ellipses._

---

## Table of Contents
* [Introduction](#introduction)
* [About the Project](#about-the-project)
    * [The Challenge](#the-challenge)
    * [Our Solution: The Extended Kalman Filter](#our-solution-the-extended-kalman-filter)
    * [Key Features](#key-features)
* [Mathematical Formulation](#mathematical-formulation)
* [Getting Started](#getting-started)
    * [Prerequisites](#prerequisites)
    * [Building the C++ Project](#building-the-c-project)
    * [Running the Simulation](#running-the-simulation)
* [Project Structure](#project-structure)
* [Results & Analysis](#results--analysis)
* [Future Work](#future-work)
* [Contributing](#contributing)
* [License](#license)
* [Contact](#contact)

---

## Introduction

**EKFCore** is a robust C++ implementation of a templated **Extended Kalman Filter (EKF)** library for state estimation. This project provides a generic, reusable framework for fusing noisy sensor data to achieve a highly accurate and reliable estimate of a system's true state. The templated design allows the same core filter logic to be applied to any state size and number of measurements.

This repository serves as a practical, hands-on demonstration of fundamental concepts in robotics state estimation and sensor fusion, crucial for autonomous systems operating in uncertain environments.

---

## About the Project

### The Challenge

Autonomous systems must have precise knowledge of their state to navigate safely and effectively. However, real-world sensors are inherently imperfect. The core problem is to intelligently combine disparate, noisy data streams to derive a more robust, precise, and continuous understanding of the system's true state.

### Our Solution: The Extended Kalman Filter

**EKFCore** addresses this challenge by providing a clean implementation of the **Extended Kalman Filter**, a powerful recursive algorithm widely adopted in robotics, aerospace, and control systems. The EKF operates in a predict-update cycle, leveraging the measurement and motion model Jacobians to linearize the non-linear dynamics around the current estimate.

### Key Features

* **Templated C++ EKF Implementation:** A from-scratch implementation of the EKF in C++, leveraging the high-performance **Eigen** library for efficient matrix operations. The templated design ensures **compile-time dimensional safety** and reusability across various state and measurement sizes.
* **Realistic Simulation Environment:** The simulator's ground truth path can be configured to include realistic process noise, modeling real-world disturbances.
* **Correlated Noise Generation:** A dedicated function `generateCorrelatedNoise` is included, which uses the **Cholesky decomposition** algorithm to produce statistically accurate noise vectors from user-defined covariance matrices.
* **Robust Sensor Fusion:** The framework is designed to handle the fusion of distinct sensor types with different measurement models.
* **Comprehensive Data Output:** The C++ EKF outputs ground truth, raw sensor data, and estimated states with full covariance matrices to a CSV file for post-processing.
* **Data Visualization (Python/Matplotlib):** Visualization of the **Ground Truth Path**, **Noisy Sensor Readings**, and the **Filtered EKF Estimate Path**, including **Covariance Ellipses** to represent filter uncertainty.

---

## Mathematical Formulation

This section outlines the core equations and a 2D vehicle example to illustrate the general, templated EKF framework.

### Motion Model (Prediction)

For a 2D vehicle, the state is defined as $X = [x, y, \theta]^T$. The non-linear motion model and its corresponding **state transition Jacobian ($F_t$)** are derived from this model to linearize it for the EKF prediction step.

### Measurement Models (Update)

The framework supports multiple measurement models. For a 2D vehicle, examples include:
* **GPS/IMU:** The measurement $Z_{gps} = [x_{gps}, y_{gps}, \theta_{gps}]^T$. The measurement function $h_{gps}(X_k)$ is simply $X_k$ itself.
* **Wheel Odometry:** The measurement $Z_{odom} = [v_{odom}, \omega_{odom}]^T$. The measurement function $h_{odom}(X_k)$ is the velocity components extracted from the motion model.

---

## Getting Started

Follow these instructions to set up, build, and run the EKFCore project.

### Prerequisites

* **C++ Compiler:** C++17 or newer (e.g., g++ for Linux/macOS, MSVC for Windows).
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
    git clone [https://github.com/0xphen/EKFCore.git](https://github.com/0xphen/EKFCore.git)
    cd EKFCore
    ```
2.  **Create a build directory and run CMake:**
    ```bash
    mkdir build
    cd build
    cmake ..
    ```
3.  **Build the project:**
    ```bash
    make
    ```

### Running the Simulation

After building, an executable named `ekfcore_sim` will be in your `build` directory.

```bash
./ekfcore_sim
