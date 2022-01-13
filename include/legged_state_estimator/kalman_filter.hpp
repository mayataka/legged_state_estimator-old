#ifndef LEGGED_STATE_ESTIMATOR_KALMAN_FILTER_HPP_
#define LEGGED_STATE_ESTIMATOR_KALMAN_FILTER_HPP_

#include "Eigen/LU"

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {

template <typename System, typename Scalar>
class KalmanFilter {
public:
  static constexpr int dimx = System::dimx;
  static constexpr int dimu = System::dimu;
  static constexpr int dimy = System::dimy;
  static constexpr int dimw = System::dimw;
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

  KalmanFilter(const System& _system)
    : system(_system),
      x_hat_(StateVector::Zero()), 
      x_pred_(StateVector::Zero()),
      e_(OutputVector::Zero()),
      P_hat_(StateCovarianceMatrix::Zero()), 
      P_pred_(StateCovarianceMatrix::Zero()), 
      Mxx_tmp_(StateCovarianceMatrix::Zero()),
      S_(OutputCovarianceMatrix::Zero()), 
      S_inv_(OutputCovarianceMatrix::Zero()), 
      Myy_tmp_(OutputCovarianceMatrix::Zero()),
      K_(KalmanGainMatrix::Zero()), 
      Mxy_tmp_(KalmanGainMatrix::Zero()),
      cholesky_() {
  }

  KalmanFilter()
    : system(),
      x_hat_(StateVector::Zero()), 
      x_pred_(StateVector::Zero()),
      e_(OutputVector::Zero()),
      P_hat_(StateCovarianceMatrix::Zero()), 
      P_pred_(StateCovarianceMatrix::Zero()), 
      Mxx_tmp_(StateCovarianceMatrix::Zero()),
      S_(OutputCovarianceMatrix::Zero()), 
      S_inv_(OutputCovarianceMatrix::Zero()), 
      Myy_tmp_(OutputCovarianceMatrix::Zero()),
      K_(KalmanGainMatrix::Zero()), 
      Mxy_tmp_(KalmanGainMatrix::Zero()),
      cholesky_() {
  }

  ~KalmanFilter() {}

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(KalmanFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(KalmanFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(KalmanFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(KalmanFilter);

  void init(const StateVector& x, const StateCovarianceMatrix& P) {
    x_hat_ = x;
    P_hat_ = P;
    system.init();
  }

  void predictionStep(const ControlVector& u=ControlVector::Zero()) {
    system.eval_f(x_hat_, u);
    system.linearize_f(x_hat_, u);
    x_pred_ = system.f;
    P_pred_.noalias() = system.A * P_hat_;
    Mxx_tmp_.noalias() = P_pred_ * system.A.transpose();
    P_pred_.noalias() = 0.5 * (Mxx_tmp_ + Mxx_tmp_.transpose()) + system.GQGt;
  }

  void filteringStep(const OutputVector& y) {
    system.eval_h(x_hat_);
    system.linearize_h(x_hat_);
    e_ = y - system.h;
    Mxy_tmp_.noalias() = P_pred_ * system.C.transpose();
    Myy_tmp_.noalias() = system.C * Mxy_tmp_;
    S_.noalias() = 0.5 * (Myy_tmp_ + Myy_tmp_.transpose()) + system.R;
    if constexpr (dimw <= 3) {
      S_inv_.noalias() = S_.inverse();
    }
    else {
      cholesky_.compute(S_);
      S_inv_.noalias() = cholesky_.solve(OutputCovarianceMatrix::Identity());
    }
    K_.noalias() = Mxy_tmp_ * S_inv_;
    x_hat_.noalias() = x_pred_ + K_ * e_;
    Mxx_tmp_.noalias() = StateCovarianceMatrix::Identity() - K_ * system.C;
    P_hat_.noalias() = Mxx_tmp_ * P_pred_;
    Mxx_tmp_.noalias() = 0.5 * (P_hat_ + P_hat_.transpose()); 
    P_hat_ = Mxx_tmp_;
  }

  const StateVector& x_hat() const {
    return x_hat_;
  }

  const StateCovarianceMatrix& P_hat() const {
    return P_hat_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  System system;
  // state equation:     x^+ = system.f(x, u, w)
  // its linearization:  x^+ = system.A * x + system.B * u + system.G * w
  // output equation:    y = system.h(x, v)
  // its linearization:  y = system.C * x + v
  // noise covariance:   w : system.Q, v : system.R

private:
  // KF datas
  StateVector x_hat_, x_pred_;
  OutputVector e_;
  StateCovarianceMatrix P_hat_, P_pred_, Mxx_tmp_;
  OutputCovarianceMatrix S_, S_inv_, Myy_tmp_;
  KalmanGainMatrix K_, Mxy_tmp_;
  // Eigen::LLT<OutputCovarianceMatrix> cholesky_;
  Eigen::LDLT<OutputCovarianceMatrix> cholesky_;

};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_KALMAN_FILTER_HPP_ 