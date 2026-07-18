#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "colmag_ros/fr3_trajectory_safety.h"

namespace colmag_ros
{

struct JointExcursionTrajectoryRequest
{
  std::vector<std::string> joint_names;
  std::vector<double> current_positions;
  std::string target_joint_name;
  double signed_excursion_rad = 0.0;
  double max_abs_excursion_rad = 0.0;
  double minimum_leg_duration_sec = 0.0;
  double max_velocity_rad_s = 0.0;
  double max_acceleration_rad_s2 = 0.0;
  double sample_period_sec = 0.0;
  JointPositionLimit target_limit;
  double joint_limit_margin_rad = 0.0;
};

struct JointExcursionTrajectory
{
  bool valid = false;
  std::string reason;
  std::size_t target_joint_index = 0;
  double peak_position_rad = 0.0;
  std::vector<JointTrajectorySample> samples;
};

JointExcursionTrajectory generateJointExcursionTrajectory(
    const JointExcursionTrajectoryRequest& request);

}  // namespace colmag_ros
