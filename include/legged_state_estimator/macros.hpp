#ifndef LEGGED_STATE_ESTIMATOR_MACROS_HPP_
#define LEGGED_STATE_ESTIMATOR_MACROS_HPP_


#define LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_CONSTRUCTOR(ClassName) ClassName(const ClassName&) = default;
#define LEGGED_STATE_ESTIMATOR_USE_DEFAULT_COPY_ASSIGN_OPERATOR(ClassName) ClassName& operator=(const ClassName&) = default;
#define LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_CONSTRUCTOR(ClassName) ClassName(ClassName&&) noexcept = default;
#define LEGGED_STATE_ESTIMATOR_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(ClassName) ClassName& operator=(ClassName&&) noexcept = default;


#endif // LEGGED_STATE_ESTIMATOR_MACROS_HPP_