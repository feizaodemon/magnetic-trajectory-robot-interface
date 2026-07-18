#include "colmag_ros/fr3_hexagon_trajectory.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "colmag_ros/fr3_trajectory_safety.h"
#include "colmag_ros/fr3_visible_motion_limits.h"

namespace colmag_ros
{
namespace
{

JointSpaceHexagon invalidHexagon(const std::string& reason)
{
  JointSpaceHexagon result;
  result.reason = reason;
  return result;
}

HexagonTrajectory invalidTrajectory(const std::string& reason)
{
  HexagonTrajectory result;
  result.reason = reason;
  return result;
}

bool validTrajectoryParameters(const HexagonTrajectoryRequest& request)
{
  return std::isfinite(request.joint_limit_margin_rad) &&
         std::isfinite(request.minimum_edge_duration_sec) &&
         std::isfinite(request.max_velocity_rad_s) &&
         std::isfinite(request.max_acceleration_rad_s2) &&
         std::isfinite(request.sample_period_sec) &&
         request.joint_limit_margin_rad >= 0.0 &&
         request.minimum_edge_duration_sec > 0.0 &&
         request.max_velocity_rad_s > 0.0 &&
         request.max_acceleration_rad_s2 > 0.0 &&
         request.sample_period_sec > 0.0;
}

bool withinAbsoluteLimits(const std::vector<double>& positions,
                          const HexagonTrajectoryRequest& request)
{
  if (positions.size() != request.joint_limits.size())
  {
    return false;
  }
  for (std::size_t index = 0; index < positions.size(); ++index)
  {
    std::string reason;
    if (!validateTargetWithinJointLimit(positions[index], request.joint_limits[index],
                                        request.joint_limit_margin_rad, &reason))
    {
      return false;
    }
  }
  return true;
}

double segmentDuration(const std::vector<double>& start,
                       const std::vector<double>& target,
                       const HexagonTrajectoryRequest& request)
{
  double maximum_delta = 0.0;
  for (std::size_t index = 0; index < start.size(); ++index)
  {
    maximum_delta = std::max(maximum_delta, std::fabs(target[index] - start[index]));
  }
  const double velocity_duration = 1.875 * maximum_delta / request.max_velocity_rad_s;
  const double acceleration_duration =
      std::sqrt((10.0 / std::sqrt(3.0)) * maximum_delta /
                request.max_acceleration_rad_s2);
  return std::max(request.minimum_edge_duration_sec,
                  std::max(velocity_duration, acceleration_duration));
}

JointTrajectorySample quinticSample(const std::vector<double>& start,
                                    const std::vector<double>& target,
                                    double segment_time,
                                    double duration,
                                    double message_time)
{
  const double phase = segment_time / duration;
  const double phase2 = phase * phase;
  const double phase3 = phase2 * phase;
  const double phase4 = phase3 * phase;
  const double phase5 = phase4 * phase;
  const double progress = 10.0 * phase3 - 15.0 * phase4 + 6.0 * phase5;
  const double progress_velocity =
      (30.0 * phase2 - 60.0 * phase3 + 30.0 * phase4) / duration;
  const double progress_acceleration =
      (60.0 * phase - 180.0 * phase2 + 120.0 * phase3) /
      (duration * duration);

  JointTrajectorySample sample;
  sample.time_from_start_sec = message_time;
  sample.positions.resize(start.size());
  sample.velocities.resize(start.size());
  sample.accelerations.resize(start.size());
  for (std::size_t index = 0; index < start.size(); ++index)
  {
    const double delta = target[index] - start[index];
    sample.positions[index] = start[index] + progress * delta;
    sample.velocities[index] = progress_velocity * delta;
    sample.accelerations[index] = progress_acceleration * delta;
  }
  return sample;
}

bool validGeneratedSample(const JointTrajectorySample& sample,
                          const HexagonTrajectoryRequest& request,
                          double previous_time)
{
  const std::size_t joint_count = request.geometry.joint_names.size();
  if (!std::isfinite(sample.time_from_start_sec) ||
      sample.time_from_start_sec <= previous_time ||
      sample.positions.size() != joint_count ||
      sample.velocities.size() != joint_count ||
      sample.accelerations.size() != joint_count ||
      !finiteVector(sample.positions) || !finiteVector(sample.velocities) ||
      !finiteVector(sample.accelerations) ||
      !withinAbsoluteLimits(sample.positions, request))
  {
    return false;
  }
  for (std::size_t index = 0; index < joint_count; ++index)
  {
    if (std::fabs(sample.positions[index] - request.geometry.current_positions[index]) >
            request.geometry.max_abs_offset_rad + 1e-12 ||
        std::fabs(sample.velocities[index]) > request.max_velocity_rad_s + 1e-12 ||
        std::fabs(sample.accelerations[index]) > request.max_acceleration_rad_s2 + 1e-12)
    {
      return false;
    }
  }
  return true;
}

}  // namespace

JointSpaceHexagon generateClosedJointSpaceHexagon(
    const JointSpaceHexagonRequest& request)
{
  if (request.joint_names.empty() ||
      request.joint_names.size() != request.current_positions.size() ||
      !finiteVector(request.current_positions))
  {
    return invalidHexagon("hexagon_trajectory_invalid");
  }
  const std::unordered_set<std::string> unique_names(
      request.joint_names.begin(), request.joint_names.end());
  if (unique_names.size() != request.joint_names.size() ||
      unique_names.count("") != 0U)
  {
    return invalidHexagon("hexagon_joint_missing");
  }
  if (!std::isfinite(request.primary_offset_rad) ||
      !std::isfinite(request.secondary_offset_rad) ||
      !std::isfinite(request.max_abs_offset_rad) ||
      request.primary_offset_rad <= 0.0 || request.secondary_offset_rad <= 0.0 ||
      request.max_abs_offset_rad <= 0.0)
  {
    return invalidHexagon("hexagon_offset_invalid");
  }
  if (request.max_abs_offset_rad > kVisibleMotionHardLimitRad ||
      request.primary_offset_rad > request.max_abs_offset_rad ||
      request.secondary_offset_rad > request.max_abs_offset_rad)
  {
    return invalidHexagon("hexagon_offset_exceeds_limit");
  }

  const std::vector<std::string>::const_iterator primary =
      std::find(request.joint_names.begin(), request.joint_names.end(), request.primary_joint_name);
  const std::vector<std::string>::const_iterator secondary =
      std::find(request.joint_names.begin(), request.joint_names.end(), request.secondary_joint_name);
  if (primary == request.joint_names.end() || secondary == request.joint_names.end() ||
      primary == secondary)
  {
    return invalidHexagon("hexagon_joint_missing");
  }

  JointSpaceHexagon result;
  result.primary_joint_index = static_cast<std::size_t>(primary - request.joint_names.begin());
  result.secondary_joint_index = static_cast<std::size_t>(secondary - request.joint_names.begin());
  const double primary_offsets[] = {
      0.0,
      -0.25 * request.primary_offset_rad,
      -0.75 * request.primary_offset_rad,
      -request.primary_offset_rad,
      -0.75 * request.primary_offset_rad,
      -0.25 * request.primary_offset_rad,
      0.0,
  };
  const double secondary_offsets[] = {
      0.0,
      request.secondary_offset_rad,
      request.secondary_offset_rad,
      0.0,
      -request.secondary_offset_rad,
      -request.secondary_offset_rad,
      0.0,
  };
  for (std::size_t index = 0; index < 7; ++index)
  {
    std::vector<double> waypoint = request.current_positions;
    waypoint[result.primary_joint_index] += primary_offsets[index];
    waypoint[result.secondary_joint_index] += secondary_offsets[index];
    if (!finiteVector(waypoint) ||
        (!result.waypoints.empty() && result.waypoints.back() == waypoint))
    {
      return invalidHexagon("hexagon_trajectory_invalid");
    }
    result.waypoints.push_back(waypoint);
  }
  result.valid = true;
  return result;
}

HexagonTrajectory generateHexagonTrajectory(const HexagonTrajectoryRequest& request)
{
  const JointSpaceHexagon geometry = generateClosedJointSpaceHexagon(request.geometry);
  if (!geometry.valid)
  {
    return invalidTrajectory(geometry.reason);
  }
  if (!validTrajectoryParameters(request) ||
      request.joint_limits.size() != request.geometry.joint_names.size())
  {
    return invalidTrajectory("hexagon_trajectory_invalid");
  }
  for (const std::vector<double>& waypoint : geometry.waypoints)
  {
    if (!withinAbsoluteLimits(waypoint, request))
    {
      return invalidTrajectory("hexagon_waypoint_outside_joint_limit");
    }
  }

  HexagonTrajectory result;
  result.geometry = geometry;
  JointTrajectorySample hold;
  hold.time_from_start_sec = request.sample_period_sec;
  hold.positions = request.geometry.current_positions;
  hold.velocities.assign(request.geometry.joint_names.size(), 0.0);
  hold.accelerations.assign(request.geometry.joint_names.size(), 0.0);
  result.samples.push_back(hold);

  double message_time = hold.time_from_start_sec;
  for (std::size_t edge = 0; edge + 1 < geometry.waypoints.size(); ++edge)
  {
    const std::vector<double>& start = geometry.waypoints[edge];
    const std::vector<double>& target = geometry.waypoints[edge + 1];
    const double duration = segmentDuration(start, target, request);
    const std::size_t sample_count = static_cast<std::size_t>(
        std::ceil(duration / request.sample_period_sec));
    for (std::size_t sample_index = 1; sample_index <= sample_count; ++sample_index)
    {
      const double segment_time = duration * static_cast<double>(sample_index) /
                                  static_cast<double>(sample_count);
      JointTrajectorySample sample = quinticSample(
          start, target, segment_time, duration, message_time + segment_time);
      if (sample_index == sample_count)
      {
        sample.positions = target;
        std::fill(sample.velocities.begin(), sample.velocities.end(), 0.0);
        std::fill(sample.accelerations.begin(), sample.accelerations.end(), 0.0);
      }
      result.samples.push_back(sample);
    }
    message_time += duration;
  }

  double previous_time = 0.0;
  for (const JointTrajectorySample& sample : result.samples)
  {
    if (!validGeneratedSample(sample, request, previous_time))
    {
      return invalidTrajectory("hexagon_trajectory_invalid");
    }
    previous_time = sample.time_from_start_sec;
  }
  if (result.samples.back().positions != request.geometry.current_positions)
  {
    return invalidTrajectory("hexagon_trajectory_invalid");
  }
  result.valid = true;
  return result;
}

}  // namespace colmag_ros
