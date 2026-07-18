#include <cassert>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "colmag_ros/fr3_trajectory_safety.h"

namespace
{

colmag_ros::JointNudgeTrajectoryRequest request(double delta = 0.03)
{
  colmag_ros::JointNudgeTrajectoryRequest value;
  value.joint_names = {"fr3_joint1", "fr3_joint2", "fr3_joint3"};
  value.current_positions = {0.2, -0.4, 0.6};
  value.target_joint_name = "fr3_joint2";
  value.requested_delta_rad = delta;
  value.max_abs_delta_rad = 0.05;
  value.minimum_duration_sec = 3.0;
  value.max_velocity_rad_s = 0.12;
  value.max_acceleration_rad_s2 = 0.18;
  value.sample_period_sec = 0.10;
  value.target_limit.lower = -1.5;
  value.target_limit.upper = 1.5;
  value.joint_limit_margin_rad = 0.02;
  return value;
}

void expectValidSamples(const colmag_ros::JointNudgeTrajectory& trajectory,
                        const colmag_ros::JointNudgeTrajectoryRequest& input)
{
  assert(trajectory.valid);
  assert(trajectory.samples.size() > 3);
  double previous_time = 0.0;
  for (const colmag_ros::JointTrajectorySample& sample : trajectory.samples)
  {
    assert(sample.time_from_start_sec > previous_time);
    previous_time = sample.time_from_start_sec;
    assert(sample.positions.size() == input.joint_names.size());
    assert(sample.velocities.size() == input.joint_names.size());
    assert(sample.accelerations.size() == input.joint_names.size());
    assert(colmag_ros::finiteVector(sample.positions));
    assert(colmag_ros::finiteVector(sample.velocities));
    assert(colmag_ros::finiteVector(sample.accelerations));
    assert(sample.positions[0] == input.current_positions[0]);
    assert(sample.positions[2] == input.current_positions[2]);
    assert(sample.velocities[0] == 0.0 && sample.velocities[2] == 0.0);
    assert(sample.accelerations[0] == 0.0 && sample.accelerations[2] == 0.0);
  }
  assert(trajectory.samples.front().velocities[trajectory.target_joint_index] == 0.0);
  assert(trajectory.samples.back().velocities[trajectory.target_joint_index] == 0.0);
  assert(trajectory.samples.back().accelerations[trajectory.target_joint_index] == 0.0);
  assert(std::fabs(trajectory.samples.back().positions[trajectory.target_joint_index] -
                   trajectory.target_position_rad) < 1e-12);
}

}  // namespace

int main()
{
  const colmag_ros::JointNudgeTrajectoryRequest normal = request();
  const colmag_ros::JointNudgeTrajectory trajectory =
      colmag_ros::generateJointNudgeTrajectory(normal);
  expectValidSamples(trajectory, normal);
  assert(trajectory.target_joint_index == 1);
  assert(std::fabs(trajectory.target_position_rad - (-0.37)) < 1e-12);

  const colmag_ros::JointNudgeTrajectoryRequest bounded = request(0.08);
  const colmag_ros::JointNudgeTrajectory bounded_trajectory =
      colmag_ros::generateJointNudgeTrajectory(bounded);
  expectValidSamples(bounded_trajectory, bounded);
  assert(std::fabs(bounded_trajectory.target_position_rad - (-0.35)) < 1e-12);
  for (const colmag_ros::JointTrajectorySample& sample : bounded_trajectory.samples)
  {
    assert(sample.positions[1] <= bounded_trajectory.target_position_rad + 1e-12);
  }

  colmag_ros::JointNudgeTrajectoryRequest trapezoid = request(0.05);
  trapezoid.minimum_duration_sec = 0.1;
  trapezoid.max_velocity_rad_s = 0.04;
  trapezoid.max_acceleration_rad_s2 = 0.2;
  expectValidSamples(colmag_ros::generateJointNudgeTrajectory(trapezoid), trapezoid);

  colmag_ros::JointNudgeTrajectoryRequest near_limit = request();
  near_limit.current_positions[1] = 1.47;
  assert(!colmag_ros::generateJointNudgeTrajectory(near_limit).valid);

  colmag_ros::JointNudgeTrajectoryRequest missing_joint = request();
  missing_joint.target_joint_name = "fr3_joint7";
  assert(colmag_ros::generateJointNudgeTrajectory(missing_joint).reason ==
         "target_joint_not_configured");

  colmag_ros::JointNudgeTrajectoryRequest non_finite = request();
  non_finite.current_positions[0] = std::numeric_limits<double>::quiet_NaN();
  assert(colmag_ros::generateJointNudgeTrajectory(non_finite).reason ==
         "non_finite_trajectory_input");

  std::string reason;
  const std::vector<std::string> names = {"fr3_joint1", "fr3_joint2"};
  assert(colmag_ros::validateJointNameContract(names, names, names, &reason));
  assert(!colmag_ros::validateJointNameContract(names, {"panda_joint1", "panda_joint2"},
                                                names, &reason));
  assert(reason == "controller_joint_names_mismatch");
  assert(!colmag_ros::validateJointNameContract(names, names,
                                                {"fr3_joint2", "fr3_joint1"}, &reason));
  assert(reason == "joint_state_names_mismatch");

  return 0;
}
