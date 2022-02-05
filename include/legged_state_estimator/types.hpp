#ifndef LEGGED_STATE_ESTIMATOR_TYPES_HPP_
#define LEGGED_STATE_ESTIMATOR_TYPES_HPP_

#include "Eigen/Core"
#include "Eigen/Geometry"


namespace legged_state_estimator {
namespace types {

template <typename Scalar, int cols, int rows>
using Matrix = Eigen::Matrix<Scalar, cols, rows>;

template <typename Scalar>
using Matrix6 = Matrix<Scalar, 6, 6>;

template <typename Scalar>
using Matrix4 = Matrix<Scalar, 4, 4>;

template <typename Scalar>
using Matrix3 = Matrix<Scalar, 3, 3>;

template <typename Scalar, int dim>
using Vector = Matrix<Scalar, dim, 1>;

template <typename Scalar>
using Vector19 = Vector<Scalar, 19>;

template <typename Scalar>
using Vector18 = Vector<Scalar, 18>;

template <typename Scalar>
using Vector12 = Vector<Scalar, 12>;

template <typename Scalar>
using Vector7 = Vector<Scalar, 7>;

template <typename Scalar>
using Vector6 = Vector<Scalar, 6>;

template <typename Scalar>
using Vector4 = Vector<Scalar, 4>;

template <typename Scalar>
using Vector3 = Vector<Scalar, 3>;

template <typename Scalar>
using Quaternion = Eigen::Quaternion<Scalar>;

} // namespace types
} // namespace legged_state_estimator


#endif // LEGGED_STATE_ESTIMATOR_TYPES_HPP_