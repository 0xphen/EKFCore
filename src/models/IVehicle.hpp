#include <Eigen/Dense>

namespace models {
/**
 * @brief Abstract base class for vehicle motion models used in an EKF.
 *
 * This class defines the interface for concrete vehicle models (e.g.,
 * UnicycleModel) by specifying the essential functionalities required
 * for EKF state prediction and Jacobian computation.
 */
template <int StateSize, int ControlSize> class IVehicle {
public:
  using StateVector = Eigen::Matrix<double, StateSize, 1>;
  using ControlVector = Eigen::Matrix<double, ControlSize, 1>;
  using StateMatrix = Eigen::Matrix<double, StateSize, StateSize>;

  virtual ~IVehicle() = default;

  /**
   * @brief Predicts the next state of the vehicle using a non-linear motion
   * function.
   *
   * This method computes f(X_k, U_k, dt) and returns the predicted state.
   *
   * @param current_state The vehicle's current state vector.
   * @param control_input The control input vector for the motion.
   * @param dt The time step for the prediction.
   * @return The predicted next state vector.
   */
  virtual StateVector getNextState(const StateVector &current_state,
                                   const ControlVector &control_input,
                                   double dt) const = 0;

  /**
   * @brief Computes the state transition Jacobian (Ft) for the motion model.
   *
   * This matrix linearizes the non-linear motion function around the current
   * state and control input, which is essential for the EKF's covariance
   * prediction step.
   *
   * @param current_state The vehicle's current state vector.
   * @param control_input The control input vector for the motion.
   * @param dt The time step, required for Jacobian computation.
   * @return The state transition Jacobian (Ft) matrix.
   */
  virtual StateMatrix computeFt(const StateVector &current_state,
                                const ControlVector &control_input,
                                double dt) const = 0;
};
} // namespace models