#include "ekf/ExtendedKalmanFilter.hpp"
#include "common/EkfTraits.hpp"

namespace ekf {
template <int StateSize, int ControlSize>
common::EkfPrediction<StateSize> Ekf<StateSize, ControlSize>::predict(
    const StateVector &X, const ControlVector &U, const StateMatrix &P,
    const ControlInputMatrix &B, const StateMatrix &Ft, const StateMatrix &Q) {
  StateVector x_predict = Ft * X + B * U;
  StateMatrix p_predict = Ft * P * Ft.transpose() + Q;

  return common::EkfPrediction<StateSize>{x_predict, p_predict};
}

template <int StateSize, int ControlSize, int MeasurementSize>
common::EkfCorrection<StateSize>
update(const typename Ekf<StateSize, ControlSize>::StateVector &x_predict,
       const typename Ekf<StateSize, ControlSize>::StateMatrix &P,
       const typename common::SensorTypes<
           MeasurementSize, StateSize>::MeasurementVector &measurement,
       const typename common::SensorTypes<MeasurementSize,
                                          StateSize>::CovarianceMatrix &R,
       const typename common::SensorTypes<MeasurementSize,
                                          StateSize>::MeasurementMatrix &H) {
  Eigen::Matrix<double, MeasurementSize, 1> innovation =
      measurement - H * x_predict;

  Eigen::Matrix<double, MeasurementSize, MeasurementSize> S =
      H * P * H.transpose() + R;

  Eigen::Matrix<double, StateSize, MeasurementSize> kalman_gain =
      P * H.transpose() * S.inverse();

  Eigen::Matrix<double, StateSize, 1> x_final =
      x_predict + kalman_gain * innovation;

  Eigen::Matrix<double, StateSize, StateSize> I =
      Eigen::Matrix<double, StateSize, StateSize>::Identity();

  Eigen::Matrix<double, StateSize, StateSize> P_final =
      (I - kalman_gain * H) * P;

  return common::EkfCorrection<StateSize>{x_final, P_final};
}
} // namespace ekf