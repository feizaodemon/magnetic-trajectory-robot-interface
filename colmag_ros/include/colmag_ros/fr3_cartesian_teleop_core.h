#pragma once

#include <array>
#include <string>

namespace colmag_ros
{

using CartesianVector = std::array<double, 3>;
using Fr3JointVector = std::array<double, 7>;

struct Fr3CartesianTeleopConfig
{
  double input_x_min = -0.05;
  double input_x_max = 0.05;
  double input_y_min = -0.05;
  double input_y_max = 0.05;
  double input_z_min = 0.007;
  double input_z_max = 0.150;
  bool swap_xy = true;
  double map_x_sign = -1.0;
  double map_y_sign = 1.0;
  bool map_z = true;
  double height_gain = 2.0;

  CartesianVector workspace_min{{0.30, -0.30, 0.121}};
  CartesianVector workspace_max{{0.60, 0.30, 0.680}};
  double target_smoothing_tau_sec = 0.08;
  double max_linear_speed_m_s = 0.10;
  double max_linear_acceleration_m_s2 = 0.20;
  double max_linear_jerk_m_s3 = 0.80;
  double s_curve_min_duration_sec = 0.75;

  int ik_max_iterations = 60;
  double dls_damping = 0.03;
  double jacobian_epsilon = 1e-6;
  double ik_position_tolerance_m = 0.005;
  double ik_orientation_tolerance_rad = 0.02;
  double ik_position_step_limit_m = 0.02;
  double ik_orientation_step_limit_rad = 0.05;
  double max_joint_step_rad = 0.05;
  double joint_velocity_filter_alpha = 0.5;
  double joint_limit_margin_rad = 0.02;

  Fr3JointVector joint_lower{{
      -2.7437, -1.7628, -2.8973, -3.0421, -2.8065, 0.5445, -2.8973}};
  Fr3JointVector joint_upper{{
      2.7437, 1.7628, 2.8973, -0.1518, 2.8065, 3.7525, 2.8973}};
};

struct MagneticBoardSample
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  bool interaction_engaged = false;
};

struct NormalizedCartesianInput
{
  CartesianVector direction{{0.0, 0.0, 0.0}};
  bool interaction_engaged = false;
};

struct Fr3CartesianTeleopStep
{
  bool accepted = false;
  std::string reason;
  CartesianVector workspace_target{{0.0, 0.0, 0.0}};
  CartesianVector shaped_target{{0.0, 0.0, 0.0}};
  Fr3JointVector joint_positions{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Fr3JointVector joint_velocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double position_residual_m = 0.0;
  double orientation_residual_rad = 0.0;
};

class Fr3CartesianTeleopCore
{
public:
  explicit Fr3CartesianTeleopCore(const Fr3CartesianTeleopConfig& config);

  bool validateConfiguration(std::string* reason) const;
  void reset();
  Fr3CartesianTeleopStep step(const MagneticBoardSample& sample,
                              const Fr3JointVector& current_joints,
                              double dt_sec);
  Fr3CartesianTeleopStep stepNormalizedCartesian(
      const NormalizedCartesianInput& input,
      const Fr3JointVector& current_joints,
      double dt_sec);

private:
  bool prepareStep(const Fr3JointVector& current_joints,
                   double dt_sec,
                   std::string* reason);
  Fr3CartesianTeleopStep stepPreparedTarget(
      const CartesianVector& workspace_target,
      double dt_sec);

  Fr3CartesianTeleopConfig config_;
  bool initialized_ = false;
  CartesianVector target_position_{{0.0, 0.0, 0.0}};
  CartesianVector filtered_destination_{{0.0, 0.0, 0.0}};
  CartesianVector cartesian_velocity_{{0.0, 0.0, 0.0}};
  CartesianVector cartesian_acceleration_{{0.0, 0.0, 0.0}};
  std::array<double, 9> target_orientation_{{0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0}};
  Fr3JointVector commanded_joints_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Fr3JointVector filtered_joint_velocity_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

}  // namespace colmag_ros
