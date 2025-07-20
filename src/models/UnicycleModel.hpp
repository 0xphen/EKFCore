#pragma once

#include <Eigen/Dense>

#include "IVehicleModel.hpp"

/**
 * @brief Implements the kinematic unicycle model for 2D vehicle motion.
 *
 * This concrete model uses linear velocity (v) and angular velocity (omega) as its control inputs,
 * suitable for differential drive robots or simplified car-like kinematics.
 * It provides the non-linear motion function and its Jacobian for EKF prediction.
 */
class UnicycleModel : public IVehicleModel {
public:
    UnicycleModel();

    /**
     * @brief Returns the dimension of the state vector.
     * @return Always 3 for the unicycle model (x, y, theta).
     * @copydoc IVehicleModel::getStateDim
     */
    int getStateDim() const override;

    /**
     * @brief Returns the dimension of the control input vector.
     * @return Always 2 for the unicycle model (linear_velocity, angular_velocity).
     * @copydoc IVehicleModel::getInputDimension
     */
    int getInputDimension() const override;

    /**
     * @brief Predicts the next state using the unicycle motion equations.
     * Expects control_input to be a 2D vector [linear_velocity, angular_velocity].
     * @copydoc IVehicleModel::getNextState
     */
    Eigen::Vector3d getNextState(const Eigen::Vector3d &current_state,
                                 const Eigen::VectorXd &control_input,
                                 double dt) const override;

    /**
     * @brief Computes the state transition Jacobian (Ft) for the unicycle model.
     * Expects control_input to be a 2D vector [linear_velocity, angular_velocity].
     * @copydoc IVehicleModel::computeFt
     */
    Eigen::Matrix3d computeFt(const Eigen::Vector3d &current_state,
                              const Eigen::VectorXd &control_input,
                              double dt) const override;
};