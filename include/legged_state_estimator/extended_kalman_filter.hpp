#ifndef LEGGED_STATE_ESTIMATOR_EXTENDED_KALMAN_FILTER_HPP_
#define LEGGED_STATE_ESTIMATOR_EXTENDED_KALMAN_FILTER_HPP_

#include "Eigen/LU"

#include "legged_state_estimator/macros.hpp"
#include "legged_state_estimator/types.hpp"


namespace legged_state_estimator {

template <typename Scalar, int dimx, int dimdx, int dimy>
class ExtendedKalmanFilter {
public:
  template <int dim>
  using Vector = types::Vector<Scalar, dim>;
  template <int rows, int cols>
  using Matrix = types::Matrix<Scalar, rows, cols>;

  ExtendedKalmanFilter()
    : x_pred(Vector<dimx>::Zero()), 
      x_hat(Vector<dimx>::Zero()), 
      dx_hat(Vector<dimdx>::Zero()), 
      y_pred(Vector<dimy>::Zero()),
      y(Vector<dimy>::Zero()),
      A(Matrix<dimdx, dimdx>::Zero()), 
      Q(Matrix<dimdx, dimdx>::Zero()), 
      C(Matrix<dimy, dimdx>::Zero()), 
      R(Matrix<dimy, dimy>::Zero()), 
      P_hat_(Matrix<dimdx, dimdx>::Zero()), 
      P_pred_(Matrix<dimdx, dimdx>::Zero()), 
      Mxx_tmp_(Matrix<dimdx, dimdx>::Zero()),
      S_(Matrix<dimy, dimy>::Zero()), 
      S_inv_(Matrix<dimy, dimy>::Zero()), 
      Myy_tmp_(Matrix<dimy, dimy>::Zero()),
      K_(Matrix<dimdx, dimy>::Zero()), 
      Mxy_tmp_(Matrix<dimdx, dimy>::Zero()),
      cholesky_() {
  }

  ~ExtendedKalmanFilter() {}

  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(ExtendedKalmanFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(ExtendedKalmanFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(ExtendedKalmanFilter);
  LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(ExtendedKalmanFilter);

  void predictionStep() {
    P_pred_.noalias()  = A * P_hat_;
    Mxx_tmp_.noalias() = P_pred_ * A.transpose();
    P_pred_.noalias()  = 0.5 * (Mxx_tmp_ + Mxx_tmp_.transpose()) + Q;
  }

  void filteringStep() {
    Mxy_tmp_.noalias() = P_pred_ * C.transpose();
    Myy_tmp_.noalias() = C * Mxy_tmp_;
    S_.noalias() = 0.5 * (Myy_tmp_ + Myy_tmp_.transpose()) + R;
    if constexpr (dimy <= 3) {
      S_inv_.noalias() = S_.inverse();
    }
    else {
      cholesky_.compute(S_);
      S_inv_.noalias() = cholesky_.solve(Matrix<dimy, dimy>::Identity());
    }
    K_.noalias() = Mxy_tmp_ * S_inv_;
    dx_hat.noalias()   = K_ * (y - y_pred);
    Mxx_tmp_.noalias() = Matrix<dimdx, dimdx>::Identity() - K_ * C;
    P_hat_.noalias()   = Mxx_tmp_ * P_pred_;
    Mxx_tmp_.noalias() = 0.5 * (P_hat_ + P_hat_.transpose()); 
    P_hat_ = Mxx_tmp_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Plant model
  // - state equation:     x^+ = plant.f(x, u, w)
  // - its linearization:  x^+ = plant.A * x + system.G * w
  // - output equation:    y = plant.h(x, v)
  // - its linearization:  y = plant.C * x + v
  // - noise covariance:   w : plant.Q, v : plant.R
  Vector<dimx> x_pred, x_hat;
  Vector<dimx> dx_hat;
  Vector<dimy> y_pred, y;
  Matrix<dimdx, dimdx> A, Q;
  Matrix<dimy, dimdx> C;
  Matrix<dimy, dimy> R;

private:
  // Internal EKF datas
  Matrix<dimdx, dimdx> P_hat_, P_pred_, Mxx_tmp_;
  Matrix<dimy, dimy> S_, S_inv_, Myy_tmp_;
  Matrix<dimdx, dimy> K_, Mxy_tmp_;
  Eigen::LDLT<Matrix<dimy, dimy>> cholesky_;
  // Eigen::LLT<Matrix<dimy, dimy>> cholesky_;

};

} // namespace legged_state_estimator

#endif // LEGGED_STATE_ESTIMATOR_EXTENDED_KALMAN_FILTER_HPP_ 