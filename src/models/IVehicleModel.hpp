#include <Eigen/Dense>

#include "../types.hpp"

namespace models {
/**
 * @brief Abstract base class defining the interface for a vehicle motion model.
 *
 * This class serves as a contract for all concrete vehicle models (e.g.,
 * UnicycleModel, KinematicBicycleModel). It declares the essential
 * functionalities that any motion model must provide for use in an EKF.
 */
class IVehicleModel {
public:
  virtual ~IVehicleModel() = default;

  /**
   * @brief Returns the dimension of the state vector this model operates on.
   * @return The integer dimension of the state vector (e.g., 3 for [x, y,
   * theta]).
   */
  virtual int getStateDim() const = 0;

  /**
   * @brief Returns the dimension of the control input vector this model
   * expects.
   * @return The integer dimension of the control input vector (e.g., 2 for [v,
   * omega]).
   */
  virtual int getInputDimension() const = 0;

  /**
   * @brief Predicts the next state of the vehicle based on the current state
   * and control input. This implements the non-linear motion function f(X_k,
   * U_k, dt).
   * @param current_state The vehicle's current state vector [x, y, theta].
   * @param control_input The control input vector [v, omega].
   * @param dt The time step for the prediction.
   * @return The predicted next state vector [x_new, y_new, theta_new].
   */
  virtual StateVector getNextState(const StateVector &state,
                                   const ControlInput &control_input,
                                   double dt) const = 0;

  // /**
  //  * @brief Computes the state transition Jacobian (Ft) for the motion model.
  //  * This linearizes the non-linear motion function around the current state
  //  and
  //  * control input.
  //  * @param current_state The vehicle's current state vector [x, y, theta].
  //  * @param control_input The control input vector [v, omega].
  //  * @param dt The time step, often required for Jacobian computation.
  //  * @return The 3x3 state transition Jacobian matrix (Ft).
  //  */
  // virtual Eigen::Matrix3d computeFt(const StateVector &current_state,
  //                                   const Eigen::VectorXd &control_input,
  //                                   double dt) const = 0;
};
} // namespace models