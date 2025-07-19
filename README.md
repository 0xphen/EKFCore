# EKFusion2D

![EKF Demo Plot](docs/ekf_demo_plot.png) ## Intro

**EKFusion2D** is a C++ project demonstrating a custom-built **Extended Kalman Filter (EKF)** for 2D vehicle localization. It shows how we can take noisy sensor inputs—like wheel odometry and GPS/IMU data—and fuse them together to get a much more accurate and robust estimate of a vehicle's true position and orientation. Think of it as making sense out of chaos, giving robots and autonomous systems a clear picture of where they are.

## About the Project

### The Problem

Robots and autonomous vehicles need to know exactly where they are. But real-world sensors are messy: wheel odometry drifts, and GPS can be spotty. Relying on just one sensor often isn't good enough. How do you combine these imperfect signals to get a reliable state estimate?

### Our Solution: EKFusion2D

This project tackles that with an **Extended Kalman Filter**. The EKF is a powerful algorithm widely used in robotics. 

1.  **Simulated Reality:** We start by simulating a 2D vehicle's true path and then generate realistic, noisy sensor data for both **wheel odometry** (velocity and angular velocity) and **GPS/IMU-style** (position and orientation) readings.
2.  **C++ EKF Engine:** The core EKF logic is implemented in C++, leveraging the **Eigen** library for efficient matrix math. It predicts the vehicle's next state based on a non-linear motion model and then corrects that prediction with incoming sensor measurements, all while managing uncertainty.
3.  **Visual Proof:** We use Python and Matplotlib to visualize everything: the actual "ground truth" path, the noisy sensor readings, and most importantly, the EKF's estimated path. We also plot **covariance ellipses** to visually represent the filter's uncertainty, showing how confidence in the estimate changes over time.
