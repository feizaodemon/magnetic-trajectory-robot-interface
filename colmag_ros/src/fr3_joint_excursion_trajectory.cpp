#include "colmag_ros/fr3_joint_excursion_trajectory.h"

#include <algorithm>
#include <cmath>

#include "colmag_ros/fr3_visible_motion_limits.h"

namespace colmag_ros
{
namespace
{

JointExcursionTrajectory invalidTrajectory(const std::string& reason)
{
  JointExcursionTrajectory trajectory;
  trajectory.reason = reason;
  return trajectory;
}

JointNudgeTrajectoryRequest legRequest(
    const JointExcursionTrajectoryRequest& request,
    const std::vector<double>& start,
    double delta)
{
  JointNudgeTrajectoryRequest leg;
  leg.joint_names = request.joint_names;
  leg.current_positions = start;
  leg.target_joint_name = request.target_joint_name;
  leg.requested_delta_rad = delta;
  leg.max_abs_delta_rad = request.max_abs_excursion_rad;
  leg.minimum_duration_sec = request.minimum_leg_duration_sec;
  leg.max_velocity_rad_s = request.max_velocity_rad_s;
  leg.max_acceleration_rad_s2 = request.max_acceleration_rad_s2;
  leg.sample_period_sec = request.sample_period_sec;
  leg.target_limit = request.target_limit;
  leg.joint_limit_margin_rad = request.joint_limit_margin_rad;
  return leg;
}

}  // namespace

JointExcursionTrajectory generateJointExcursionTrajectory(
    const JointExcursionTrajectoryRequest& request)
{
  if (!std::isfinite(request.signed_excursion_rad) ||
      !std::isfinite(request.max_abs_excursion_rad) ||
      request.signed_excursion_rad == 0.0 ||
      request.max_abs_excursion_rad <= 0.0)
  {
    return invalidTrajectory("joint_excursion_invalid");
  }
  if (request.max_abs_excursion_rad > kVisibleMotionHardLimitRad ||
      std::fabs(request.signed_excursion_rad) > request.max_abs_excursion_rad)
  {
    return invalidTrajectory("joint_excursion_exceeds_limit");
  }

  const JointNudgeTrajectory outbound = generateJointNudgeTrajectory(
      legRequest(request, request.current_positions, request.signed_excursion_rad));
  if (!outbound.valid)
  {
    return invalidTrajectory(outbound.reason);
  }
  const std::vector<double>& peak = outbound.samples.back().positions;
  const JointNudgeTrajectory inbound = generateJointNudgeTrajectory(
      legRequest(request, peak, -request.signed_excursion_rad));
  if (!inbound.valid)
  {
    return invalidTrajectory(inbound.reason);
  }

  JointExcursionTrajectory trajectory;
  trajectory.target_joint_index = outbound.target_joint_index;
  trajectory.peak_position_rad = outbound.target_position_rad;
  trajectory.samples = outbound.samples;
  const double time_offset = trajectory.samples.back().time_from_start_sec -
                             inbound.samples.front().time_from_start_sec;
  for (std::size_t index = 1; index < inbound.samples.size(); ++index)
  {
    JointTrajectorySample sample = inbound.samples[index];
    sample.time_from_start_sec += time_offset;
    trajectory.samples.push_back(sample);
  }
  if (trajectory.samples.back().positions != request.current_positions)
  {
    return invalidTrajectory("joint_excursion_invalid");
  }
  trajectory.valid = true;
  return trajectory;
}

}  // namespace colmag_ros
