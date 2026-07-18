#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "colmag_ros/fr3_trajectory_safety.h"

namespace colmag_ros
{

struct JointSpaceHexagonRequest
{
  std::vector<std::string> joint_names;
  std::vector<double> current_positions;
  std::string primary_joint_name;
  std::string secondary_joint_name;
  double primary_offset_rad = 0.0;
  double secondary_offset_rad = 0.0;
  double max_abs_offset_rad = 0.0;
};

struct JointSpaceHexagon
{
  bool valid = false;
  std::string reason;
  std::size_t primary_joint_index = 0;
  std::size_t secondary_joint_index = 0;
  std::vector<std::vector<double>> waypoints;
};

JointSpaceHexagon generateClosedJointSpaceHexagon(
    const JointSpaceHexagonRequest& request);

struct HexagonTrajectoryRequest
{
  JointSpaceHexagonRequest geometry;
  std::vector<JointPositionLimit> joint_limits;
  double joint_limit_margin_rad = 0.0;
  double minimum_edge_duration_sec = 0.0;
  double max_velocity_rad_s = 0.0;
  double max_acceleration_rad_s2 = 0.0;
  double sample_period_sec = 0.0;
};

struct HexagonTrajectory
{
  bool valid = false;
  std::string reason;
  JointSpaceHexagon geometry;
  std::vector<JointTrajectorySample> samples;
};

HexagonTrajectory generateHexagonTrajectory(const HexagonTrajectoryRequest& request);

}  // namespace colmag_ros
