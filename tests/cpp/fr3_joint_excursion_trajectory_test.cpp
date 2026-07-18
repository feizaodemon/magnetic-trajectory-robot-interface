#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include "colmag_ros/fr3_joint_excursion_trajectory.h"

namespace
{

colmag_ros::JointExcursionTrajectoryRequest request(double excursion)
{
  colmag_ros::JointExcursionTrajectoryRequest value;
  value.joint_names = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
                       "fr3_joint5", "fr3_joint6", "fr3_joint7"};
  value.current_positions = {0.1, -0.2, 0.3, -0.4, 0.5, 0.6, -0.7};
  value.target_joint_name = "fr3_joint1";
  value.signed_excursion_rad = excursion;
  value.max_abs_excursion_rad = 0.12;
  value.minimum_leg_duration_sec = 3.0;
  value.max_velocity_rad_s = 0.12;
  value.max_acceleration_rad_s2 = 0.18;
  value.sample_period_sec = 0.1;
  value.target_limit.lower = -2.0;
  value.target_limit.upper = 2.0;
  value.joint_limit_margin_rad = 0.02;
  return value;
}

}  // namespace

int main()
{
  const colmag_ros::JointExcursionTrajectoryRequest positive = request(0.10);
  const colmag_ros::JointExcursionTrajectory trajectory =
      colmag_ros::generateJointExcursionTrajectory(positive);
  assert(trajectory.valid);
  assert(trajectory.target_joint_index == 0);
  assert(std::fabs(trajectory.peak_position_rad - 0.20) < 1e-12);
  assert(trajectory.samples.front().positions == positive.current_positions);
  assert(trajectory.samples.back().positions == positive.current_positions);
  assert(trajectory.samples.size() > 3);

  bool saw_peak = false;
  double previous_time = 0.0;
  for (const colmag_ros::JointTrajectorySample& sample : trajectory.samples)
  {
    assert(sample.time_from_start_sec > previous_time);
    previous_time = sample.time_from_start_sec;
    assert(sample.positions.size() == positive.current_positions.size());
    assert(sample.velocities.size() == sample.positions.size());
    assert(sample.accelerations.size() == sample.positions.size());
    for (std::size_t joint = 0; joint < sample.positions.size(); ++joint)
    {
      assert(std::isfinite(sample.positions[joint]));
      assert(std::isfinite(sample.velocities[joint]));
      assert(std::isfinite(sample.accelerations[joint]));
      if (joint != trajectory.target_joint_index)
      {
        assert(sample.positions[joint] == positive.current_positions[joint]);
      }
    }
    if (std::fabs(sample.positions[trajectory.target_joint_index] -
                  trajectory.peak_position_rad) < 1e-12)
    {
      saw_peak = true;
      assert(sample.velocities[trajectory.target_joint_index] == 0.0);
    }
  }
  assert(saw_peak);
  assert(trajectory.samples.front().velocities ==
         std::vector<double>(positive.joint_names.size(), 0.0));
  assert(trajectory.samples.back().velocities ==
         std::vector<double>(positive.joint_names.size(), 0.0));

  const colmag_ros::JointExcursionTrajectoryRequest negative = request(-0.10);
  const colmag_ros::JointExcursionTrajectory negative_trajectory =
      colmag_ros::generateJointExcursionTrajectory(negative);
  assert(negative_trajectory.valid);
  assert(std::fabs(negative_trajectory.peak_position_rad) < 1e-12);
  assert(negative_trajectory.samples.back().positions == negative.current_positions);

  colmag_ros::JointExcursionTrajectoryRequest exact_cap = request(0.12);
  assert(colmag_ros::generateJointExcursionTrajectory(exact_cap).valid);

  colmag_ros::JointExcursionTrajectoryRequest over_cap = request(0.120001);
  const colmag_ros::JointExcursionTrajectory rejected_cap =
      colmag_ros::generateJointExcursionTrajectory(over_cap);
  assert(!rejected_cap.valid);
  assert(rejected_cap.reason == "joint_excursion_exceeds_limit");
  assert(rejected_cap.samples.empty());

  for (const double invalid : {
           0.0, std::numeric_limits<double>::quiet_NaN(),
           std::numeric_limits<double>::infinity()})
  {
    colmag_ros::JointExcursionTrajectoryRequest invalid_request = request(invalid);
    assert(colmag_ros::generateJointExcursionTrajectory(invalid_request).reason ==
           "joint_excursion_invalid");
  }

  colmag_ros::JointExcursionTrajectoryRequest outside_limit = request(0.10);
  outside_limit.current_positions[0] = 1.97;
  const colmag_ros::JointExcursionTrajectory rejected_limit =
      colmag_ros::generateJointExcursionTrajectory(outside_limit);
  assert(!rejected_limit.valid);
  assert(rejected_limit.reason == "target_outside_absolute_joint_limits");
  assert(rejected_limit.samples.empty());
  return 0;
}
