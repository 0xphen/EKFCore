# TODO - Project Enhancement Plan

This document outlines key features to implement to make the EKFCore project more robust.

---

## High-Impact Features

- [ ]  **Implement an Outlier Rejection Mechanism**
    - [ ]  Calculate the Mahalanobis distance for each new measurement's innovation.
    - [ ]  Compare the Mahalanobis distance to a threshold based on a chi-squared ($\chi^2$) distribution.
    - [ ]  If the measurement is an outlier, discard it and skip the update step.

- [ ]  **Demonstrate Multi-Sensor Fusion**
    - [ ]  Create a single, generic `update` method that can handle different measurement types.
    - [ ]  Inside the `update` method, apply the correct measurement model ($H$) and noise covariance ($R$) based on the incoming sensor data.
    - [ ]  Verify that the filter can seamlessly switch between fusing different sensor data streams.

---

## Validation and Analysis

- [ ]  **Add Consistency Checks ($\chi^2$ Tests)**
    - [ ]  Calculate the Normalized Innovation Squared (NIS) for each measurement.
    - [ ]  Plot the NIS values over time and compare them to the theoretical $\chi^2$ distribution.
    - [ ]  Use these plots to identify if the filter's covariance matrices ($Q$ and $R$) are well-tuned.

- [ ]  **Plot Covariance Bounds**
    - [ ]  Generate plots that show the filter's estimated path with $\pm 1\sigma$ (standard deviation) bounds.
    - [ ]  Overlay these bounds with the ground truth path to visually confirm that the filter is accurately capturing its own uncertainty.

---

## Future Work & Additional Ideas

- [ ]  **Implement a UKF (Unscented Kalman Filter)** for comparison with the EKF.
- [ ]  **Add Dynamic Covariance Tuning** to allow `Q` and `R` to change based on context (e.g., higher noise in a GPS when in a tunnel).
- [ ]  **Integrate with a real-world dataset** or a real robotics framework like ROS 2.
- [ ]  **Implement a Full SLAM Solution** by extending the state to include landmarks.