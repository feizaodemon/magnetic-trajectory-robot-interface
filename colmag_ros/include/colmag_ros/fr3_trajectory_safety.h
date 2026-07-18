#pragma once

#include <string>
#include <vector>

namespace colmag_ros
{

struct JointPositionLimit
{
  double lower = 0.0;
  double upper = 0.0;
};

struct JointNudgeTrajectoryRequest
{
  std::vector<std::string> joint_names;
  std::vector<double> current_positions;
  std::string target_joint_name;
  double requested_delta_rad = 0.0;
  double max_abs_delta_rad = 0.0;
  double minimum_duration_sec = 0.0;
  double max_velocity_rad_s = 0.0;
  double max_acceleration_rad_s2 = 0.0;
  double sample_period_sec = 0.0;
  JointPositionLimit target_limit;
  double joint_limit_margin_rad = 0.0;
};

struct JointTrajectorySample
{
  double time_from_start_sec = 0.0;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
};

struct JointNudgeTrajectory
{
  bool valid = false;
  std::string reason;
  std::size_t target_joint_index = 0;
  double target_position_rad = 0.0;
  std::vector<JointTrajectorySample> samples;
};

bool finiteVector(const std::vector<double>& values);

bool validateJointNameContract(const std::vector<std::string>& configured_names,
                               const std::vector<std::string>& controller_names,
                               const std::vector<std::string>& observed_names,
                               std::string* reason);

bool validateTargetWithinJointLimit(double target,
                                    const JointPositionLimit& limit,
                                    double margin,
                                    std::string* reason);

JointNudgeTrajectory generateJointNudgeTrajectory(const JointNudgeTrajectoryRequest& request);

}  // namespace colmag_ros
