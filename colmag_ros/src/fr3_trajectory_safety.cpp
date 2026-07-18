#include "colmag_ros/fr3_trajectory_safety.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace colmag_ros
{
namespace
{

JointNudgeTrajectory invalidTrajectory(const std::string& reason)
{
  JointNudgeTrajectory trajectory;
  trajectory.reason = reason;
  return trajectory;
}

double boundedDelta(double requested, double maximum)
{
  return std::max(-maximum, std::min(maximum, requested));
}

struct ProfileTiming
{
  bool valid = false;
  double duration = 0.0;
  double acceleration_time = 0.0;
  double cruise_time = 0.0;
  double acceleration = 0.0;
  double peak_velocity = 0.0;
};

ProfileTiming profileTiming(double distance,
                            double minimum_duration,
                            double max_velocity,
                            double max_acceleration)
{
  ProfileTiming timing;
  const double accel_to_max_velocity = max_velocity / max_acceleration;
  const double accel_distance = max_acceleration * accel_to_max_velocity * accel_to_max_velocity;
  double fastest_duration = 0.0;
  if (distance <= accel_distance)
  {
    fastest_duration = 2.0 * std::sqrt(distance / max_acceleration);
  }
  else
  {
    fastest_duration = 2.0 * accel_to_max_velocity +
                       (distance - accel_distance) / max_velocity;
  }

  timing.duration = std::max(minimum_duration, fastest_duration);
  const double triangular_acceleration = 4.0 * distance / (timing.duration * timing.duration);
  const double triangular_peak_velocity = 2.0 * distance / timing.duration;
  if (triangular_acceleration <= max_acceleration && triangular_peak_velocity <= max_velocity)
  {
    timing.acceleration_time = timing.duration / 2.0;
    timing.acceleration = triangular_acceleration;
    timing.peak_velocity = triangular_peak_velocity;
  }
  else
  {
    const double discriminant = timing.duration * timing.duration -
                                4.0 * distance / max_acceleration;
    if (discriminant < 0.0)
    {
      return timing;
    }
    timing.acceleration_time =
        0.5 * (timing.duration - std::sqrt(std::max(0.0, discriminant)));
    timing.cruise_time = timing.duration - 2.0 * timing.acceleration_time;
    timing.acceleration = max_acceleration;
    timing.peak_velocity = timing.acceleration * timing.acceleration_time;
  }

  timing.valid = timing.duration > 0.0 && timing.acceleration_time > 0.0 &&
                 timing.acceleration > 0.0 && timing.peak_velocity > 0.0 &&
                 timing.peak_velocity <= max_velocity + 1e-12;
  return timing;
}

void profileState(const ProfileTiming& timing,
                  double distance,
                  double time,
                  double* position,
                  double* velocity,
                  double* acceleration)
{
  if (time <= timing.acceleration_time)
  {
    *acceleration = timing.acceleration;
    *velocity = timing.acceleration * time;
    *position = 0.5 * timing.acceleration * time * time;
    return;
  }

  const double deceleration_start = timing.acceleration_time + timing.cruise_time;
  const double acceleration_distance =
      0.5 * timing.acceleration * timing.acceleration_time * timing.acceleration_time;
  if (time < deceleration_start)
  {
    *acceleration = 0.0;
    *velocity = timing.peak_velocity;
    *position = acceleration_distance +
                timing.peak_velocity * (time - timing.acceleration_time);
    return;
  }

  const double time_left = std::max(0.0, timing.duration - time);
  *acceleration = -timing.acceleration;
  *velocity = timing.acceleration * time_left;
  *position = distance - 0.5 * timing.acceleration * time_left * time_left;
}

JointTrajectorySample sampleAt(const JointNudgeTrajectoryRequest& request,
                               std::size_t target_index,
                               double direction,
                               double distance,
                               const ProfileTiming& timing,
                               double profile_time,
                               double message_time)
{
  JointTrajectorySample sample;
  sample.time_from_start_sec = message_time;
  sample.positions = request.current_positions;
  sample.velocities.assign(request.joint_names.size(), 0.0);
  sample.accelerations.assign(request.joint_names.size(), 0.0);

  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  profileState(timing, distance, profile_time,
               &position, &velocity, &acceleration);
  sample.positions[target_index] += direction * position;
  sample.velocities[target_index] = direction * velocity;
  sample.accelerations[target_index] = direction * acceleration;
  return sample;
}

}  // namespace

bool finiteVector(const std::vector<double>& values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

bool validateJointNameContract(const std::vector<std::string>& configured_names,
                               const std::vector<std::string>& controller_names,
                               const std::vector<std::string>& observed_names,
                               std::string* reason)
{
  if (configured_names.empty())
  {
    *reason = "configured_joint_names_empty";
    return false;
  }
  if (controller_names != configured_names)
  {
    *reason = "controller_joint_names_mismatch";
    return false;
  }
  if (observed_names != configured_names)
  {
    *reason = "joint_state_names_mismatch";
    return false;
  }
  reason->clear();
  return true;
}

bool validateTargetWithinJointLimit(double target,
                                    const JointPositionLimit& limit,
                                    double margin,
                                    std::string* reason)
{
  if (!std::isfinite(target) || !std::isfinite(limit.lower) ||
      !std::isfinite(limit.upper) || !std::isfinite(margin))
  {
    *reason = "non_finite_joint_limit_input";
    return false;
  }
  if (margin < 0.0 || limit.lower >= limit.upper ||
      limit.lower + margin >= limit.upper - margin)
  {
    *reason = "invalid_joint_limit_or_margin";
    return false;
  }
  if (target < limit.lower + margin || target > limit.upper - margin)
  {
    *reason = "target_outside_absolute_joint_limits";
    return false;
  }
  reason->clear();
  return true;
}

JointNudgeTrajectory generateJointNudgeTrajectory(const JointNudgeTrajectoryRequest& request)
{
  if (request.joint_names.empty() || request.joint_names.size() != request.current_positions.size())
  {
    return invalidTrajectory("joint_vector_size_mismatch");
  }
  if (!finiteVector(request.current_positions) ||
      !std::isfinite(request.requested_delta_rad) ||
      !std::isfinite(request.max_abs_delta_rad) ||
      !std::isfinite(request.minimum_duration_sec) ||
      !std::isfinite(request.max_velocity_rad_s) ||
      !std::isfinite(request.max_acceleration_rad_s2) ||
      !std::isfinite(request.sample_period_sec))
  {
    return invalidTrajectory("non_finite_trajectory_input");
  }
  if (request.max_abs_delta_rad <= 0.0 || request.minimum_duration_sec <= 0.0 ||
      request.max_velocity_rad_s <= 0.0 || request.max_acceleration_rad_s2 <= 0.0 ||
      request.sample_period_sec <= 0.0)
  {
    return invalidTrajectory("non_positive_trajectory_limit");
  }

  const std::vector<std::string>::const_iterator target =
      std::find(request.joint_names.begin(), request.joint_names.end(), request.target_joint_name);
  if (target == request.joint_names.end())
  {
    return invalidTrajectory("target_joint_not_configured");
  }
  const std::size_t target_index = static_cast<std::size_t>(target - request.joint_names.begin());
  const double delta = boundedDelta(request.requested_delta_rad, request.max_abs_delta_rad);
  if (std::fabs(delta) <= std::numeric_limits<double>::epsilon())
  {
    return invalidTrajectory("zero_joint_delta");
  }
  const double target_position = request.current_positions[target_index] + delta;
  std::string limit_reason;
  if (!validateTargetWithinJointLimit(target_position, request.target_limit,
                                      request.joint_limit_margin_rad, &limit_reason))
  {
    return invalidTrajectory(limit_reason);
  }

  const ProfileTiming timing = profileTiming(std::fabs(delta), request.minimum_duration_sec,
                                             request.max_velocity_rad_s,
                                             request.max_acceleration_rad_s2);
  if (!timing.valid)
  {
    return invalidTrajectory("trajectory_profile_infeasible");
  }

  JointNudgeTrajectory trajectory;
  trajectory.target_joint_index = target_index;
  trajectory.target_position_rad = target_position;
  const double hold_time = request.sample_period_sec;
  JointTrajectorySample hold;
  hold.time_from_start_sec = hold_time;
  hold.positions = request.current_positions;
  hold.velocities.assign(request.joint_names.size(), 0.0);
  hold.accelerations.assign(request.joint_names.size(), 0.0);
  trajectory.samples.push_back(hold);

  for (double profile_time = request.sample_period_sec;
       profile_time < timing.duration;
       profile_time += request.sample_period_sec)
  {
    trajectory.samples.push_back(sampleAt(request, target_index, delta > 0.0 ? 1.0 : -1.0,
                                          std::fabs(delta), timing, profile_time,
                                          hold_time + profile_time));
  }
  JointTrajectorySample final_sample =
      sampleAt(request, target_index, delta > 0.0 ? 1.0 : -1.0,
               std::fabs(delta), timing, timing.duration, hold_time + timing.duration);
  final_sample.positions[target_index] = target_position;
  final_sample.velocities[target_index] = 0.0;
  final_sample.accelerations[target_index] = 0.0;
  trajectory.samples.push_back(final_sample);

  double previous_time = 0.0;
  for (const JointTrajectorySample& sample : trajectory.samples)
  {
    if (!std::isfinite(sample.time_from_start_sec) || sample.time_from_start_sec <= previous_time ||
        !finiteVector(sample.positions) || !finiteVector(sample.velocities) ||
        !finiteVector(sample.accelerations))
    {
      return invalidTrajectory("invalid_generated_trajectory_point");
    }
    previous_time = sample.time_from_start_sec;
  }

  trajectory.valid = true;
  return trajectory;
}

}  // namespace colmag_ros
