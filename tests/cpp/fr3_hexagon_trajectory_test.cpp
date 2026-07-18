#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include "colmag_ros/fr3_hexagon_trajectory.h"
#include "colmag_ros/fr3_hardware_safety.h"

namespace
{

colmag_ros::JointSpaceHexagonRequest geometryRequest()
{
  colmag_ros::JointSpaceHexagonRequest request;
  request.joint_names = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
                         "fr3_joint5", "fr3_joint6", "fr3_joint7"};
  request.current_positions = {0.1, -0.2, 0.3, -0.4, 0.5, 0.6, -0.7};
  request.primary_joint_name = "fr3_joint1";
  request.secondary_joint_name = "fr3_joint6";
  request.primary_offset_rad = 0.08;
  request.secondary_offset_rad = 0.10;
  request.max_abs_offset_rad = 0.12;
  return request;
}

colmag_ros::HexagonTrajectoryRequest trajectoryRequest()
{
  colmag_ros::HexagonTrajectoryRequest request;
  request.geometry = geometryRequest();
  colmag_ros::JointPositionLimit limit;
  limit.lower = -2.0;
  limit.upper = 2.0;
  request.joint_limits.assign(request.geometry.joint_names.size(), limit);
  request.joint_limit_margin_rad = 0.02;
  request.minimum_edge_duration_sec = 1.0;
  request.max_velocity_rad_s = 0.12;
  request.max_acceleration_rad_s2 = 0.18;
  request.sample_period_sec = 0.1;
  return request;
}

}  // namespace

int main()
{
  const colmag_ros::JointSpaceHexagonRequest request = geometryRequest();
  const colmag_ros::JointSpaceHexagon geometry =
      colmag_ros::generateClosedJointSpaceHexagon(request);

  assert(geometry.valid);
  assert(geometry.reason.empty());
  assert(geometry.primary_joint_index == 0);
  assert(geometry.secondary_joint_index == 5);
  assert(geometry.waypoints.size() == 7);  // six vertices plus closed return
  assert(geometry.waypoints.front() == request.current_positions);
  assert(geometry.waypoints.back() == request.current_positions);

  for (std::size_t waypoint = 0; waypoint < geometry.waypoints.size(); ++waypoint)
  {
    assert(geometry.waypoints[waypoint].size() == request.current_positions.size());
    for (std::size_t joint : {1U, 2U, 3U, 4U, 6U})
    {
      assert(geometry.waypoints[waypoint][joint] == request.current_positions[joint]);
    }
    assert(std::fabs(geometry.waypoints[waypoint][0] - request.current_positions[0]) <=
           0.08 + 1e-12);
    assert(std::fabs(geometry.waypoints[waypoint][5] - request.current_positions[5]) <=
           0.10 + 1e-12);
    if (waypoint > 0)
    {
      assert(geometry.waypoints[waypoint] != geometry.waypoints[waypoint - 1]);
    }
  }

  colmag_ros::JointSpaceHexagonRequest shifted = geometryRequest();
  shifted.current_positions = {-0.5, 0.4, -0.3, 0.2, -0.1, -0.8, 0.9};
  const colmag_ros::JointSpaceHexagon shifted_geometry =
      colmag_ros::generateClosedJointSpaceHexagon(shifted);
  assert(shifted_geometry.valid);
  assert(shifted_geometry.waypoints.front() == shifted.current_positions);
  assert(shifted_geometry.waypoints.back() == shifted.current_positions);

  for (const double invalid : {0.0, -0.01,
                               std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity()})
  {
    colmag_ros::JointSpaceHexagonRequest invalid_radius = geometryRequest();
    invalid_radius.primary_offset_rad = invalid;
    assert(colmag_ros::generateClosedJointSpaceHexagon(invalid_radius).reason ==
           "hexagon_offset_invalid");
  }

  colmag_ros::JointSpaceHexagonRequest above_bound = geometryRequest();
  above_bound.secondary_offset_rad = 0.1200001;
  assert(colmag_ros::generateClosedJointSpaceHexagon(above_bound).reason ==
         "hexagon_offset_exceeds_limit");

  colmag_ros::JointSpaceHexagonRequest weakened_bound = geometryRequest();
  weakened_bound.max_abs_offset_rad = 0.120001;
  assert(colmag_ros::generateClosedJointSpaceHexagon(weakened_bound).reason ==
         "hexagon_offset_exceeds_limit");

  colmag_ros::JointSpaceHexagonRequest missing = geometryRequest();
  missing.secondary_joint_name = "fr3_joint8";
  assert(colmag_ros::generateClosedJointSpaceHexagon(missing).reason ==
         "hexagon_joint_missing");

  colmag_ros::JointSpaceHexagonRequest duplicate = geometryRequest();
  duplicate.joint_names[4] = "fr3_joint1";
  assert(colmag_ros::generateClosedJointSpaceHexagon(duplicate).reason ==
         "hexagon_joint_missing");

  const colmag_ros::HexagonTrajectoryRequest trajectory_request = trajectoryRequest();
  const colmag_ros::HexagonTrajectory trajectory =
      colmag_ros::generateHexagonTrajectory(trajectory_request);
  assert(trajectory.valid);
  assert(trajectory.reason.empty());
  assert(trajectory.samples.size() > 7);
  assert(trajectory.samples.front().positions == trajectory_request.geometry.current_positions);
  assert(trajectory.samples.back().positions == trajectory_request.geometry.current_positions);
  assert(trajectory.samples.back().time_from_start_sec >= 9.0);
  assert(trajectory.samples.back().time_from_start_sec < 11.0);

  double previous_time = 0.0;
  std::size_t corners_seen = 0;
  for (const colmag_ros::JointTrajectorySample& sample : trajectory.samples)
  {
    assert(sample.time_from_start_sec > previous_time);
    previous_time = sample.time_from_start_sec;
    assert(sample.positions.size() == trajectory_request.geometry.joint_names.size());
    assert(sample.velocities.size() == sample.positions.size());
    assert(sample.accelerations.size() == sample.positions.size());
    for (std::size_t joint = 0; joint < sample.positions.size(); ++joint)
    {
      assert(std::isfinite(sample.positions[joint]));
      assert(std::isfinite(sample.velocities[joint]));
      assert(std::isfinite(sample.accelerations[joint]));
      assert(std::fabs(sample.velocities[joint]) <=
             trajectory_request.max_velocity_rad_s + 1e-12);
      assert(std::fabs(sample.accelerations[joint]) <=
             trajectory_request.max_acceleration_rad_s2 + 1e-12);
    }
    for (const std::vector<double>& corner : trajectory.geometry.waypoints)
    {
      if (sample.positions == corner)
      {
        for (double velocity : sample.velocities) assert(velocity == 0.0);
        for (double acceleration : sample.accelerations) assert(acceleration == 0.0);
        ++corners_seen;
        break;
      }
    }
  }
  assert(corners_seen == 7);

  colmag_ros::HexagonTrajectoryRequest outside_limit = trajectoryRequest();
  outside_limit.geometry.current_positions[0] = -1.99;
  const colmag_ros::HexagonTrajectory rejected_limit =
      colmag_ros::generateHexagonTrajectory(outside_limit);
  assert(!rejected_limit.valid);
  assert(rejected_limit.reason == "hexagon_waypoint_outside_joint_limit");
  assert(rejected_limit.samples.empty());

  colmag_ros::HexagonTrajectoryRequest just_above_hard_bound = trajectoryRequest();
  just_above_hard_bound.geometry.primary_offset_rad = 0.1200001;
  const colmag_ros::HexagonTrajectory rejected_bound =
      colmag_ros::generateHexagonTrajectory(just_above_hard_bound);
  assert(!rejected_bound.valid);
  assert(rejected_bound.reason == "hexagon_offset_exceeds_limit");
  assert(rejected_bound.samples.empty());

  colmag_ros::OneShotMotionGate one_shot(true, 1);
  std::string quota_reason;
  assert(one_shot.canSend(&quota_reason));
  one_shot.recordSendAttempt();  // one complete multi-point trajectory, not per waypoint
  assert(one_shot.sendAttempts() == 1);
  assert(!one_shot.canSend(&quota_reason));
  assert(quota_reason == "one_shot_motion_limit_reached");

  return 0;
}
