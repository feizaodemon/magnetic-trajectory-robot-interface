#include "colmag_ros/fr3_cartesian_teleop_core.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace colmag_ros
{
namespace
{

using Matrix3 = std::array<double, 9>;
using Matrix4 = std::array<double, 16>;
using Jacobian = std::array<double, 42>;
using Vector6 = std::array<double, 6>;
using Matrix6 = std::array<double, 36>;

bool finite(double value)
{
  return std::isfinite(value);
}

template <std::size_t N>
bool finiteArray(const std::array<double, N>& values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
    return finite(value);
  });
}

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

template <std::size_t N>
double norm(const std::array<double, N>& values)
{
  double squared = 0.0;
  for (double value : values)
  {
    squared += value * value;
  }
  return std::sqrt(squared);
}

template <std::size_t N>
std::array<double, N> limitedNorm(const std::array<double, N>& values,
                                  double limit)
{
  const double magnitude = norm(values);
  if (magnitude <= limit || magnitude <= 1e-12)
  {
    return values;
  }
  std::array<double, N> result = values;
  for (double& value : result)
  {
    value *= limit / magnitude;
  }
  return result;
}

Matrix4 identity4()
{
  Matrix4 result{{0.0}};
  for (std::size_t index = 0; index < 4; ++index)
  {
    result[index * 4 + index] = 1.0;
  }
  return result;
}

Matrix4 multiply4(const Matrix4& lhs, const Matrix4& rhs)
{
  Matrix4 result{{0.0}};
  for (std::size_t row = 0; row < 4; ++row)
  {
    for (std::size_t column = 0; column < 4; ++column)
    {
      for (std::size_t inner = 0; inner < 4; ++inner)
      {
        result[row * 4 + column] +=
            lhs[row * 4 + inner] * rhs[inner * 4 + column];
      }
    }
  }
  return result;
}

Matrix4 modifiedDh(double a, double d, double alpha, double theta)
{
  const double cosine_theta = std::cos(theta);
  const double sine_theta = std::sin(theta);
  const double cosine_alpha = std::cos(alpha);
  const double sine_alpha = std::sin(alpha);
  return Matrix4{{
      cosine_theta, -sine_theta, 0.0, a,
      sine_theta * cosine_alpha, cosine_theta * cosine_alpha,
      -sine_alpha, -d * sine_alpha,
      sine_theta * sine_alpha, cosine_theta * sine_alpha,
      cosine_alpha, d * cosine_alpha,
      0.0, 0.0, 0.0, 1.0}};
}

Matrix4 forwardKinematics(const Fr3JointVector& joints)
{
  constexpr double kPi = 3.14159265358979323846;
  const std::array<double, 8> a{{0.0, 0.0, 0.0, 0.0825,
                                 -0.0825, 0.0, 0.088, 0.0}};
  const std::array<double, 8> d{{0.333, 0.0, 0.316, 0.0,
                                 0.384, 0.0, 0.0, 0.107}};
  const std::array<double, 8> alpha{{0.0, -kPi / 2.0, kPi / 2.0,
                                     kPi / 2.0, -kPi / 2.0, kPi / 2.0,
                                     kPi / 2.0, 0.0}};
  Matrix4 transform = identity4();
  for (std::size_t index = 0; index < 8; ++index)
  {
    const double theta = index < 7 ? joints[index] : 0.0;
    transform = multiply4(transform, modifiedDh(a[index], d[index], alpha[index], theta));
  }
  return transform;
}

CartesianVector translation(const Matrix4& transform)
{
  return CartesianVector{{transform[3], transform[7], transform[11]}};
}

Matrix3 rotation(const Matrix4& transform)
{
  return Matrix3{{transform[0], transform[1], transform[2],
                  transform[4], transform[5], transform[6],
                  transform[8], transform[9], transform[10]}};
}

CartesianVector cross(const CartesianVector& lhs, const CartesianVector& rhs)
{
  return CartesianVector{{lhs[1] * rhs[2] - lhs[2] * rhs[1],
                          lhs[2] * rhs[0] - lhs[0] * rhs[2],
                          lhs[0] * rhs[1] - lhs[1] * rhs[0]}};
}

CartesianVector column(const Matrix3& matrix, std::size_t index)
{
  return CartesianVector{{matrix[index], matrix[3 + index], matrix[6 + index]}};
}

CartesianVector orientationError(const Matrix3& current, const Matrix3& desired)
{
  CartesianVector result{{0.0, 0.0, 0.0}};
  for (std::size_t index = 0; index < 3; ++index)
  {
    const CartesianVector term = cross(column(current, index), column(desired, index));
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      result[axis] += 0.5 * term[axis];
    }
  }
  return result;
}

Jacobian numericalJacobian(const Fr3JointVector& joints, double epsilon)
{
  const Matrix4 base = forwardKinematics(joints);
  const CartesianVector base_position = translation(base);
  const Matrix3 base_rotation = rotation(base);
  Jacobian jacobian{{0.0}};
  for (std::size_t joint = 0; joint < 7; ++joint)
  {
    Fr3JointVector perturbed = joints;
    perturbed[joint] += epsilon;
    const Matrix4 sample = forwardKinematics(perturbed);
    const CartesianVector sample_position = translation(sample);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      jacobian[axis * 7 + joint] =
          (sample_position[axis] - base_position[axis]) / epsilon;
    }

    const Matrix3 sample_rotation = rotation(sample);
    Matrix3 delta_rotation{{0.0}};
    for (std::size_t row = 0; row < 3; ++row)
    {
      for (std::size_t column_index = 0; column_index < 3; ++column_index)
      {
        for (std::size_t inner = 0; inner < 3; ++inner)
        {
          delta_rotation[row * 3 + column_index] +=
              (sample_rotation[row * 3 + inner] - base_rotation[row * 3 + inner]) *
              base_rotation[column_index * 3 + inner];
        }
      }
    }
    jacobian[3 * 7 + joint] = delta_rotation[7] / epsilon;
    jacobian[4 * 7 + joint] = delta_rotation[2] / epsilon;
    jacobian[5 * 7 + joint] = delta_rotation[3] / epsilon;
  }
  return jacobian;
}

template <std::size_t N>
bool solveLinearSystem(std::array<double, N * N> matrix,
                       std::array<double, N> rhs,
                       std::array<double, N>* solution)
{
  for (std::size_t pivot = 0; pivot < N; ++pivot)
  {
    std::size_t best = pivot;
    for (std::size_t row = pivot + 1; row < N; ++row)
    {
      if (std::fabs(matrix[row * N + pivot]) >
          std::fabs(matrix[best * N + pivot]))
      {
        best = row;
      }
    }
    if (std::fabs(matrix[best * N + pivot]) < 1e-12)
    {
      return false;
    }
    if (best != pivot)
    {
      for (std::size_t column_index = pivot; column_index < N; ++column_index)
      {
        std::swap(matrix[pivot * N + column_index],
                  matrix[best * N + column_index]);
      }
      std::swap(rhs[pivot], rhs[best]);
    }

    const double scale = matrix[pivot * N + pivot];
    for (std::size_t column_index = pivot; column_index < N; ++column_index)
    {
      matrix[pivot * N + column_index] /= scale;
    }
    rhs[pivot] /= scale;
    for (std::size_t row = 0; row < N; ++row)
    {
      if (row == pivot)
      {
        continue;
      }
      const double factor = matrix[row * N + pivot];
      for (std::size_t column_index = pivot; column_index < N; ++column_index)
      {
        matrix[row * N + column_index] -=
            factor * matrix[pivot * N + column_index];
      }
      rhs[row] -= factor * rhs[pivot];
    }
  }
  *solution = rhs;
  return finiteArray(*solution);
}

bool solveIk(const CartesianVector& target_position,
             const Matrix3& target_orientation,
             const Fr3JointVector& seed,
             const Fr3CartesianTeleopConfig& config,
             Fr3JointVector* solution,
             double* position_residual,
             double* orientation_residual)
{
  Fr3JointVector joints = seed;
  for (int iteration = 0; iteration < config.ik_max_iterations; ++iteration)
  {
    const Matrix4 current_transform = forwardKinematics(joints);
    const CartesianVector current_position = translation(current_transform);
    const Matrix3 current_orientation = rotation(current_transform);
    CartesianVector position_error{{
        target_position[0] - current_position[0],
        target_position[1] - current_position[1],
        target_position[2] - current_position[2]}};
    CartesianVector rotation_error = orientationError(current_orientation, target_orientation);
    if (norm(position_error) < 1e-4 && norm(rotation_error) < 1e-3)
    {
      break;
    }
    position_error = limitedNorm(position_error, config.ik_position_step_limit_m);
    rotation_error = limitedNorm(rotation_error, config.ik_orientation_step_limit_rad);
    Vector6 error{{position_error[0], position_error[1], position_error[2],
                   rotation_error[0], rotation_error[1], rotation_error[2]}};
    const Jacobian jacobian = numericalJacobian(joints, config.jacobian_epsilon);
    Matrix6 damped{{0.0}};
    for (std::size_t row = 0; row < 6; ++row)
    {
      for (std::size_t column_index = 0; column_index < 6; ++column_index)
      {
        for (std::size_t joint = 0; joint < 7; ++joint)
        {
          damped[row * 6 + column_index] +=
              jacobian[row * 7 + joint] * jacobian[column_index * 7 + joint];
        }
      }
      damped[row * 6 + row] += config.dls_damping * config.dls_damping;
    }
    Vector6 intermediate{{0.0}};
    if (!solveLinearSystem<6>(damped, error, &intermediate))
    {
      return false;
    }
    for (std::size_t joint = 0; joint < 7; ++joint)
    {
      double delta = 0.0;
      for (std::size_t row = 0; row < 6; ++row)
      {
        delta += jacobian[row * 7 + joint] * intermediate[row];
      }
      joints[joint] = clamp(
          joints[joint] + delta,
          config.joint_lower[joint] + config.joint_limit_margin_rad,
          config.joint_upper[joint] - config.joint_limit_margin_rad);
    }
  }

  const Matrix4 final_transform = forwardKinematics(joints);
  const CartesianVector final_position = translation(final_transform);
  const CartesianVector final_orientation_error =
      orientationError(rotation(final_transform), target_orientation);
  CartesianVector final_position_error{{
      target_position[0] - final_position[0],
      target_position[1] - final_position[1],
      target_position[2] - final_position[2]}};
  *position_residual = norm(final_position_error);
  *orientation_residual = norm(final_orientation_error);
  *solution = joints;
  return finiteArray(joints) && finite(*position_residual) &&
         finite(*orientation_residual);
}

double normalizedInput(double value, double lower, double upper)
{
  return 2.0 * clamp((value - lower) / (upper - lower), 0.0, 1.0) - 1.0;
}

double normalizedHeight(double value,
                        double lower,
                        double upper,
                        double gain)
{
  const double linear = clamp((std::fabs(value) - lower) / (upper - lower), 0.0, 1.0);
  if (gain <= 1e-9)
  {
    return linear;
  }
  return -std::expm1(-gain * linear) / -std::expm1(-gain);
}

CartesianVector mapBoardPosition(const MagneticBoardSample& sample,
                                 const Fr3CartesianTeleopConfig& config,
                                 double held_z)
{
  const double input_x = normalizedInput(sample.x, config.input_x_min, config.input_x_max);
  const double input_y = normalizedInput(sample.y, config.input_y_min, config.input_y_max);
  const double robot_x = config.swap_xy ? input_y : input_x;
  const double robot_y = config.swap_xy ? input_x : input_y;
  CartesianVector target{{
      0.5 * (config.workspace_min[0] + config.workspace_max[0]) +
          0.5 * (config.workspace_max[0] - config.workspace_min[0]) *
              config.map_x_sign * robot_x,
      0.5 * (config.workspace_min[1] + config.workspace_max[1]) +
          0.5 * (config.workspace_max[1] - config.workspace_min[1]) *
              config.map_y_sign * robot_y,
      held_z}};
  if (config.map_z)
  {
    const double fraction = normalizedHeight(
        sample.z, config.input_z_min, config.input_z_max, config.height_gain);
    target[2] = config.workspace_min[2] +
                fraction * (config.workspace_max[2] - config.workspace_min[2]);
  }
  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    target[axis] = clamp(target[axis], config.workspace_min[axis],
                         config.workspace_max[axis]);
  }
  return target;
}

struct ShapedCartesianState
{
  CartesianVector position{{0.0, 0.0, 0.0}};
  CartesianVector velocity{{0.0, 0.0, 0.0}};
  CartesianVector acceleration{{0.0, 0.0, 0.0}};
};

ShapedCartesianState sCurveStep(const CartesianVector& position,
                                const CartesianVector& velocity,
                                const CartesianVector& acceleration,
                                const CartesianVector& desired,
                                double dt,
                                const Fr3CartesianTeleopConfig& config)
{
  CartesianVector displacement{{desired[0] - position[0],
                                desired[1] - position[1],
                                desired[2] - position[2]}};
  if (norm(displacement) <= 0.00025 &&
      norm(velocity) <= config.max_linear_acceleration_m_s2 * dt &&
      norm(acceleration) <= config.max_linear_jerk_m_s3 * dt)
  {
    ShapedCartesianState result;
    result.position = desired;
    return result;
  }

  const double distance = norm(displacement);
  double duration = std::max(
      config.s_curve_min_duration_sec,
      std::max(1.875 * distance / config.max_linear_speed_m_s,
               std::max(std::sqrt(5.8 * distance /
                                  config.max_linear_acceleration_m_s2),
                        std::cbrt(60.0 * distance /
                                  config.max_linear_jerk_m_s3))));
  ShapedCartesianState result;
  for (int attempt = 0; attempt < 12; ++attempt)
  {
    const double sample_time = std::min(dt, duration);
    CartesianVector start_jerk{{0.0, 0.0, 0.0}};
    CartesianVector end_jerk{{0.0, 0.0, 0.0}};
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      const double c0 = position[axis];
      const double c1 = velocity[axis];
      const double c2 = 0.5 * acceleration[axis];
      const std::array<double, 9> matrix{{
          std::pow(duration, 3), std::pow(duration, 4), std::pow(duration, 5),
          3.0 * std::pow(duration, 2), 4.0 * std::pow(duration, 3),
          5.0 * std::pow(duration, 4),
          6.0 * duration, 12.0 * std::pow(duration, 2),
          20.0 * std::pow(duration, 3)}};
      const std::array<double, 3> rhs{{
          desired[axis] - (c0 + c1 * duration + c2 * duration * duration),
          -c1 - 2.0 * c2 * duration,
          -2.0 * c2}};
      std::array<double, 3> coefficients{{0.0, 0.0, 0.0}};
      if (!solveLinearSystem<3>(matrix, rhs, &coefficients))
      {
        result.position = position;
        return result;
      }
      const double c3 = coefficients[0];
      const double c4 = coefficients[1];
      const double c5 = coefficients[2];
      result.position[axis] =
          c0 + c1 * sample_time + c2 * sample_time * sample_time +
          c3 * std::pow(sample_time, 3) + c4 * std::pow(sample_time, 4) +
          c5 * std::pow(sample_time, 5);
      result.velocity[axis] =
          c1 + 2.0 * c2 * sample_time + 3.0 * c3 * std::pow(sample_time, 2) +
          4.0 * c4 * std::pow(sample_time, 3) +
          5.0 * c5 * std::pow(sample_time, 4);
      result.acceleration[axis] =
          2.0 * c2 + 6.0 * c3 * sample_time +
          12.0 * c4 * std::pow(sample_time, 2) +
          20.0 * c5 * std::pow(sample_time, 3);
      start_jerk[axis] = 6.0 * c3;
      end_jerk[axis] = 6.0 * c3 + 24.0 * c4 * sample_time +
                       60.0 * c5 * sample_time * sample_time;
    }
    if (norm(result.velocity) <= config.max_linear_speed_m_s + 1e-12 &&
        norm(result.acceleration) <=
            config.max_linear_acceleration_m_s2 + 1e-12 &&
        std::max(norm(start_jerk), norm(end_jerk)) <=
            config.max_linear_jerk_m_s3 + 1e-12)
    {
      return result;
    }
    duration *= 1.35;
  }
  result.velocity = limitedNorm(result.velocity, config.max_linear_speed_m_s);
  result.acceleration = limitedNorm(
      result.acceleration, config.max_linear_acceleration_m_s2);
  return result;
}

Fr3CartesianTeleopStep rejectedStep(const std::string& reason)
{
  Fr3CartesianTeleopStep result;
  result.reason = reason;
  return result;
}

}  // namespace

Fr3CartesianTeleopCore::Fr3CartesianTeleopCore(
    const Fr3CartesianTeleopConfig& config)
  : config_(config)
{
}

bool Fr3CartesianTeleopCore::validateConfiguration(std::string* reason) const
{
  if (!finite(config_.input_x_min) || !finite(config_.input_x_max) ||
      !finite(config_.input_y_min) || !finite(config_.input_y_max) ||
      !finite(config_.input_z_min) || !finite(config_.input_z_max) ||
      config_.input_x_min >= config_.input_x_max ||
      config_.input_y_min >= config_.input_y_max ||
      config_.input_z_min >= config_.input_z_max)
  {
    *reason = "invalid_input_range";
    return false;
  }
  if (!finite(config_.map_x_sign) || !finite(config_.map_y_sign) ||
      std::fabs(config_.map_x_sign) < 1e-12 ||
      std::fabs(config_.map_y_sign) < 1e-12 ||
      !finite(config_.height_gain) || config_.height_gain < 0.0)
  {
    *reason = "invalid_mapping_parameter";
    return false;
  }
  if (!finiteArray(config_.workspace_min) || !finiteArray(config_.workspace_max))
  {
    *reason = "non_finite_workspace";
    return false;
  }
  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    if (config_.workspace_min[axis] >= config_.workspace_max[axis])
    {
      *reason = "invalid_workspace_bounds";
      return false;
    }
  }
  const std::array<double, 13> positive{{
      config_.max_linear_speed_m_s,
      config_.max_linear_acceleration_m_s2,
      config_.max_linear_jerk_m_s3,
      config_.s_curve_min_duration_sec,
      config_.dls_damping,
      config_.jacobian_epsilon,
      config_.ik_position_tolerance_m,
      config_.ik_orientation_tolerance_rad,
      config_.ik_position_step_limit_m,
      config_.ik_orientation_step_limit_rad,
      config_.max_joint_step_rad,
      config_.joint_velocity_filter_alpha,
      config_.joint_limit_margin_rad}};
  if (!finiteArray(positive) ||
      std::any_of(positive.begin(), positive.end(), [](double value) {
        return value <= 0.0;
      }) ||
      !finite(config_.target_smoothing_tau_sec) ||
      config_.target_smoothing_tau_sec < 0.0 ||
      config_.joint_velocity_filter_alpha > 1.0 ||
      config_.ik_max_iterations <= 0)
  {
    *reason = "invalid_motion_or_ik_limit";
    return false;
  }
  if (!finiteArray(config_.joint_lower) || !finiteArray(config_.joint_upper))
  {
    *reason = "non_finite_joint_limit";
    return false;
  }
  for (std::size_t joint = 0; joint < 7; ++joint)
  {
    if (config_.joint_lower[joint] + config_.joint_limit_margin_rad >=
        config_.joint_upper[joint] - config_.joint_limit_margin_rad)
    {
      *reason = "invalid_joint_limit";
      return false;
    }
  }
  reason->clear();
  return true;
}

void Fr3CartesianTeleopCore::reset()
{
  initialized_ = false;
  cartesian_velocity_.fill(0.0);
  cartesian_acceleration_.fill(0.0);
  filtered_joint_velocity_.fill(0.0);
}

Fr3CartesianTeleopStep Fr3CartesianTeleopCore::step(
    const MagneticBoardSample& sample,
    const Fr3JointVector& current_joints,
    double dt_sec)
{
  if (!sample.interaction_engaged)
  {
    reset();
    return rejectedStep("interaction_not_engaged");
  }
  if (!finite(sample.x) || !finite(sample.y) || !finite(sample.z))
  {
    reset();
    return rejectedStep("non_finite_magnetic_input");
  }

  std::string reason;
  if (!prepareStep(current_joints, dt_sec, &reason))
  {
    return rejectedStep(reason);
  }
  return stepPreparedTarget(
      mapBoardPosition(sample, config_, target_position_[2]),
      dt_sec);
}

Fr3CartesianTeleopStep Fr3CartesianTeleopCore::stepNormalizedCartesian(
    const NormalizedCartesianInput& input,
    const Fr3JointVector& current_joints,
    double dt_sec)
{
  if (!input.interaction_engaged)
  {
    reset();
    return rejectedStep("interaction_not_engaged");
  }
  if (!finiteArray(input.direction))
  {
    reset();
    return rejectedStep("non_finite_normalized_input");
  }
  if (std::any_of(input.direction.begin(), input.direction.end(), [](double value) {
        return std::fabs(value) > 1.0 + 1e-12;
      }))
  {
    reset();
    return rejectedStep("normalized_input_out_of_range");
  }

  std::string reason;
  if (!prepareStep(current_joints, dt_sec, &reason))
  {
    return rejectedStep(reason);
  }
  const CartesianVector direction = limitedNorm(input.direction, 1.0);
  CartesianVector destination = filtered_destination_;
  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    destination[axis] = clamp(
        destination[axis] +
            direction[axis] * config_.max_linear_speed_m_s * dt_sec,
        config_.workspace_min[axis], config_.workspace_max[axis]);
  }
  return stepPreparedTarget(destination, dt_sec);
}

bool Fr3CartesianTeleopCore::prepareStep(
    const Fr3JointVector& current_joints,
    double dt_sec,
    std::string* reason)
{
  if (!validateConfiguration(reason))
  {
    reset();
    return false;
  }
  if (!finiteArray(current_joints) || !finite(dt_sec) || dt_sec <= 0.0)
  {
    reset();
    *reason = "invalid_joint_state_or_period";
    return false;
  }
  for (std::size_t joint = 0; joint < 7; ++joint)
  {
    if (current_joints[joint] <
            config_.joint_lower[joint] + config_.joint_limit_margin_rad ||
        current_joints[joint] >
            config_.joint_upper[joint] - config_.joint_limit_margin_rad)
    {
      reset();
      *reason = "current_joint_outside_limits";
      return false;
    }
  }

  if (!initialized_)
  {
    const Matrix4 current_transform = forwardKinematics(current_joints);
    target_position_ = translation(current_transform);
    filtered_destination_ = target_position_;
    target_orientation_ = rotation(current_transform);
    commanded_joints_ = current_joints;
    cartesian_velocity_.fill(0.0);
    cartesian_acceleration_.fill(0.0);
    filtered_joint_velocity_.fill(0.0);
    initialized_ = true;
  }

  reason->clear();
  return true;
}

Fr3CartesianTeleopStep Fr3CartesianTeleopCore::stepPreparedTarget(
    const CartesianVector& workspace_target,
    double dt_sec)
{
  if (!finiteArray(workspace_target))
  {
    reset();
    return rejectedStep("non_finite_workspace_target");
  }
  CartesianVector bounded_target = workspace_target;
  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    bounded_target[axis] = clamp(
        bounded_target[axis], config_.workspace_min[axis],
        config_.workspace_max[axis]);
  }
  const double alpha = config_.target_smoothing_tau_sec > 0.0
                           ? dt_sec / (config_.target_smoothing_tau_sec + dt_sec)
                           : 1.0;
  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    filtered_destination_[axis] +=
        alpha * (bounded_target[axis] - filtered_destination_[axis]);
  }
  const ShapedCartesianState shaped = sCurveStep(
      target_position_, cartesian_velocity_, cartesian_acceleration_,
      filtered_destination_, dt_sec, config_);
  if (!finiteArray(shaped.position) || !finiteArray(shaped.velocity) ||
      !finiteArray(shaped.acceleration))
  {
    reset();
    return rejectedStep("non_finite_cartesian_shape");
  }

  Fr3JointVector ik_solution{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double position_residual = std::numeric_limits<double>::infinity();
  double orientation_residual = std::numeric_limits<double>::infinity();
  if (!solveIk(shaped.position, target_orientation_, commanded_joints_, config_,
               &ik_solution, &position_residual, &orientation_residual))
  {
    return rejectedStep("dls_solve_failed");
  }
  if (position_residual > config_.ik_position_tolerance_m ||
      orientation_residual > config_.ik_orientation_tolerance_rad)
  {
    return rejectedStep("ik_residual_exceeded");
  }

  Fr3JointVector next_joints = commanded_joints_;
  Fr3JointVector next_velocity = filtered_joint_velocity_;
  for (std::size_t joint = 0; joint < 7; ++joint)
  {
    const double delta = clamp(
        ik_solution[joint] - commanded_joints_[joint],
        -config_.max_joint_step_rad,
        config_.max_joint_step_rad);
    next_joints[joint] = commanded_joints_[joint] + delta;
    const double raw_velocity = delta / dt_sec;
    next_velocity[joint] += config_.joint_velocity_filter_alpha *
                            (raw_velocity - next_velocity[joint]);
  }
  if (!finiteArray(next_joints) || !finiteArray(next_velocity))
  {
    return rejectedStep("non_finite_joint_target");
  }

  target_position_ = shaped.position;
  cartesian_velocity_ = shaped.velocity;
  cartesian_acceleration_ = shaped.acceleration;
  commanded_joints_ = next_joints;
  filtered_joint_velocity_ = next_velocity;

  Fr3CartesianTeleopStep result;
  result.accepted = true;
  result.workspace_target = bounded_target;
  result.shaped_target = shaped.position;
  result.joint_positions = next_joints;
  result.joint_velocities = next_velocity;
  result.position_residual_m = position_residual;
  result.orientation_residual_rad = orientation_residual;
  return result;
}

}  // namespace colmag_ros
