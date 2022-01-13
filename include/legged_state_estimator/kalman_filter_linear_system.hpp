#ifndef LEGGED_STATE_ESTIMATOR_KALMAN_FILTER_LINEAR_SYSTEM_HPP_
#define LEGGED_STATE_ESTIMATOR_KALMAN_FILTER_LINEAR_SYSTEM_HPP_ 

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {

template <typename Scalar, int _dimx, int _dimu, int _dimy, int _dimw>
class KalmanFilterLinearSystem {
public:
  static constexpr int dimx = _dimx;
  static constexpr int dimu = _dimu;
  static constexpr int dimy = _dimy;
  static constexpr int dimw = _dimw;
  using StateVector   = Vector<Scalar, dimx>;
  using ControlVector = Vector<Scalar, dimu>;
  using OutputVector  = Vector<Scalar, dimy>;
  using StateMatrix            = Matrix<Scalar, dimx, dimx>;
  using ControlInputMatrix     = Matrix<Scalar, dimx, dimu>;
  using StateNoiseMatrix       = Matrix<Scalar, dimx, dimw>;
  using OutputMatrix           = Matrix<Scalar, dimy, dimx>;
  using StateNoiseCovarianceMatrix = Matrix<Scalar, dimw, dimw>;
  using StateCovarianceMatrix      = Matrix<Scalar, dimx, dimx>;
  using OutputCovarianceMatrix     = Matrix<Scalar, dimy, dimy>;
  using KalmanGainMatrix           = Matrix<Scalar, dimx, dimy>;

  KalmanFilterLinearSystem()
    : f(StateVector::Zero()), 
      h(OutputVector::Zero()),
      A(StateMatrix::Zero()),
      B(ControlInputMatrix::Zero()),
      G(StateNoiseMatrix::Zero()),
      C(OutputMatrix::Zero()),
      Q(StateNoiseCovarianceMatrix::Zero()),
      GQGt(StateCovarianceMatrix::Zero()),
      R(OutputCovarianceMatrix::Zero()) {
  }

  ~KalmanFilterLinearSystem() {}

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(KalmanFilterLinearSystem);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(KalmanFilterLinearSystem);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(KalmanFilterLinearSystem);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(KalmanFilterLinearSystem);

  void init() {
    // do nothing
  }

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

  StateVector f;
  OutputVector h;
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

#endif // LEGGED_STATE_ESTIMATOR_KALMAN_FILTER_LINEAR_SYSTEM_HPP_ 