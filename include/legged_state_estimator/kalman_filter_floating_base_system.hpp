#ifndef LEGGED_STATE_ESTIMATOR_KALMAN_FILTER_FLOATING_BASE_SYSTEM_HPP_
#define LEGGED_STATE_ESTIMATOR_KALMAN_FILTER_FLOATING_BASE_SYSTEM_HPP_

#include "Eigen/Core"

#include "legged_state_estimator/macros.hpp"


namespace legged_state_estimator {

///
/// second-order system of the floating base (single rigid-body)
/// x: (pos, lin_vel) (R^6)
/// w: noise on lin_vel (R^3)
/// u: linear acceleration (R^3)
/// y: (height, lin_vel) (from leg) (R^3)
///
/// second-order system of the floating base (single rigid-body)
template <typename Scalar>
class KalmanFilterFloatingBaseSystem {
public:
  static constexpr int dimx = 6;
  static constexpr int dimu = 3;
  static constexpr int dimy = 4;
  static constexpr int dimw = 3;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using StateVector   = Eigen::Matrix<Scalar, dimx, 1>;
  using ControlVector = Eigen::Matrix<Scalar, dimu, 1>;
  using OutputVector  = Eigen::Matrix<Scalar, dimy, 1>;
  using StateMatrix            = Eigen::Matrix<Scalar, dimx, dimx>;
  using ControlInputMatrix     = Eigen::Matrix<Scalar, dimx, dimu>;
  using StateNoiseMatrix       = Eigen::Matrix<Scalar, dimx, dimw>;
  using OutputMatrix           = Eigen::Matrix<Scalar, dimy, dimx>;
  using StateNoiseCovarianceMatrix = Eigen::Matrix<Scalar, dimw, dimw>;
  using StateCovarianceMatrix      = Eigen::Matrix<Scalar, dimx, dimx>;
  using OutputCovarianceMatrix     = Eigen::Matrix<Scalar, dimy, dimy>;
  using KalmanGainMatrix           = Eigen::Matrix<Scalar, dimx, dimy>;

  KalmanFilterFloatingBaseSystem(const Scalar dt=0.)
    : dt(dt),
      f(StateVector::Zero()), 
      h(OutputVector::Zero()),
      h_tmp(OutputVector::Zero()),
      A(StateMatrix::Zero()),
      B(ControlInputMatrix::Zero()),
      G(StateNoiseMatrix::Zero()),
      C(OutputMatrix::Zero()),
      Q(StateNoiseCovarianceMatrix::Zero()),
      GQGt(StateCovarianceMatrix::Zero()),
      R(OutputCovarianceMatrix::Zero()) {
  }

  ~KalmanFilterFloatingBaseSystem() {}

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(KalmanFilterFloatingBaseSystem)
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(KalmanFilterFloatingBaseSystem)
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(KalmanFilterFloatingBaseSystem)
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(KalmanFilterFloatingBaseSystem)

  void init() {
    A << 1.0, 0.0, 0.0,  dt, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,  dt, 0,0,
         0.0, 0.0, 1.0, 0.0, 0.0,  dt,
         0.0, 0.0, 0.0, 1.0, 0.0, 0,0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0,0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1,0;
    B << 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0,
         0.0, 0.0, 0.0,
          dt, 0.0, 0.0,
         0.0,  dt, 0.0,
         0.0, 0.0,  dt;
    G << 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0,
         0.0, 0.0, 0.0,
         1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
    C << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0,0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0,0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    setGQGt();
  }

  void setGQGt() {
    GQGt = G * Q * G.transpose();
  }

  ///
  /// second-order system of the floating base (single rigid-body)
  /// x: (pos, lin_vel) (R^6)
  /// w: noise on lin_vel (R^3)
  /// u: linear acceleration (R^3) from imu
  /// y: (height, lin_vel) (R^6) from leg
  ///
  void eval_f(const StateVector& x, const ControlVector& u) {
    f.noalias() = A * x + B * u;
  }

  void linearize_f(const StateVector& x, const ControlVector& u) {
    // do nothing
  }

  void eval_h(const StateVector& x) {
    h.noalias() = C * x;
  }

  void linearize_h(const StateVector& x) {
    // do nothing
  }

  Scalar dt;
  StateVector f;
  OutputVector h, h_tmp;
  StateMatrix A;
  ControlInputMatrix B;
  StateNoiseMatrix G;
  OutputMatrix C;
  StateNoiseCovarianceMatrix Q;
  StateCovarianceMatrix GQGt;
  OutputCovarianceMatrix R;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_KALMAN_FILTER_FLOATING_BASE_SYSTEM_HPP_ 