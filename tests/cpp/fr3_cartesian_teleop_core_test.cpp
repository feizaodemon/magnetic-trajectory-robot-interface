#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <string>

#include "colmag_ros/fr3_cartesian_teleop_core.h"

namespace
{

colmag_ros::Fr3JointVector home()
{
  return colmag_ros::Fr3JointVector{{
      0.0, -0.785398163, 0.0, -2.35619449,
      0.0, 1.57079632679, 0.785398163397}};
}

colmag_ros::MagneticBoardSample engagedCenter()
{
  colmag_ros::MagneticBoardSample sample;
  sample.x = 0.0;
  sample.y = 0.0;
  sample.z = 0.05;
  sample.interaction_engaged = true;
  return sample;
}

// Drives the normalized input at 30 Hz and tracks the shaped target's
// finite-difference acceleration and jerk.
struct KeyboardRun
{
  KeyboardRun(colmag_ros::Fr3CartesianTeleopCore& core_ref,
              const colmag_ros::Fr3CartesianTeleopConfig& config_ref)
    : core(core_ref), config(config_ref), joints(home())
  {
  }

  void step(const colmag_ros::CartesianVector& direction)
  {
    colmag_ros::NormalizedCartesianInput input;
    input.direction = direction;
    input.interaction_engaged = true;
    const colmag_ros::Fr3CartesianTeleopStep result =
        core.stepNormalizedCartesian(input, joints, dt);
    assert(result.accepted);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      assert(result.shaped_target[axis] >= config.workspace_min[axis] - 1e-12);
      assert(result.shaped_target[axis] <= config.workspace_max[axis] + 1e-12);
    }
    joints = result.joint_positions;
    colmag_ros::CartesianVector velocity{{0.0, 0.0, 0.0}};
    colmag_ros::CartesianVector acceleration{{0.0, 0.0, 0.0}};
    double acceleration_squared = 0.0;
    double jerk_squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      velocity[axis] = (result.shaped_target[axis] - position[axis]) / dt;
      acceleration[axis] = (velocity[axis] - this->velocity[axis]) / dt;
      acceleration_squared += acceleration[axis] * acceleration[axis];
      const double jerk = (acceleration[axis] - this->acceleration[axis]) / dt;
      jerk_squared += jerk * jerk;
    }
    // The first samples difference against the uninitialized pose.
    if (samples >= 2)
    {
      max_acceleration =
          std::max(max_acceleration, std::sqrt(acceleration_squared));
    }
    if (samples >= 3)
    {
      max_jerk = std::max(max_jerk, std::sqrt(jerk_squared));
    }
    position = result.shaped_target;
    this->velocity = velocity;
    this->acceleration = acceleration;
    ++samples;
  }

  void hold(const colmag_ros::CartesianVector& direction, int ticks)
  {
    for (int tick = 0; tick < ticks; ++tick)
    {
      step(direction);
    }
  }

  int release(int max_ticks)
  {
    int ticks = 0;
    while (core.isMoving() && ticks < max_ticks)
    {
      step(colmag_ros::CartesianVector{{0.0, 0.0, 0.0}});
      ++ticks;
    }
    return ticks;
  }

  colmag_ros::Fr3CartesianTeleopCore& core;
  const colmag_ros::Fr3CartesianTeleopConfig& config;
  const double dt = 1.0 / 30.0;
  colmag_ros::Fr3JointVector joints;
  colmag_ros::CartesianVector position{{0.0, 0.0, 0.0}};
  colmag_ros::CartesianVector velocity{{0.0, 0.0, 0.0}};
  colmag_ros::CartesianVector acceleration{{0.0, 0.0, 0.0}};
  double max_acceleration = 0.0;
  double max_jerk = 0.0;
  int samples = 0;
};

}  // namespace

int main()
{
  colmag_ros::Fr3CartesianTeleopConfig config;
  colmag_ros::Fr3CartesianTeleopCore core(config);
  std::string reason;
  assert(core.validateConfiguration(&reason));

  colmag_ros::MagneticBoardSample disengaged = engagedCenter();
  disengaged.interaction_engaged = false;
  assert(!core.step(disengaged, home(), 1.0 / 30.0).accepted);
  assert(core.step(disengaged, home(), 1.0 / 30.0).reason ==
         "interaction_not_engaged");

  colmag_ros::MagneticBoardSample invalid = engagedCenter();
  invalid.x = std::numeric_limits<double>::quiet_NaN();
  assert(core.step(invalid, home(), 1.0 / 30.0).reason ==
         "non_finite_magnetic_input");

  const colmag_ros::Fr3CartesianTeleopStep first =
      core.step(engagedCenter(), home(), 1.0 / 30.0);
  assert(first.accepted);
  for (std::size_t joint = 0; joint < 7; ++joint)
  {
    assert(std::isfinite(first.joint_positions[joint]));
    assert(std::isfinite(first.joint_velocities[joint]));
    assert(std::fabs(first.joint_positions[joint] - home()[joint]) <=
           config.max_joint_step_rad + 1e-12);
  }
  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    assert(first.workspace_target[axis] >= config.workspace_min[axis]);
    assert(first.workspace_target[axis] <= config.workspace_max[axis]);
  }

  core.reset();
  colmag_ros::NormalizedCartesianInput keyboard_input;
  keyboard_input.direction = colmag_ros::CartesianVector{{1.0, 0.0, 0.0}};
  keyboard_input.interaction_engaged = true;
  const colmag_ros::Fr3CartesianTeleopStep keyboard_step =
      core.stepNormalizedCartesian(keyboard_input, home(), 1.0 / 30.0);
  assert(keyboard_step.accepted);
  assert(keyboard_step.workspace_target[0] >= keyboard_step.shaped_target[0]);

  colmag_ros::NormalizedCartesianInput out_of_range = keyboard_input;
  out_of_range.direction[0] = 1.1;
  assert(core.stepNormalizedCartesian(out_of_range, home(), 1.0 / 30.0).reason ==
         "normalized_input_out_of_range");

  // A held key is a velocity command: it cruises at max_linear_speed_m_s,
  // a direction change and a release stay inside the acceleration and jerk
  // limits, and holding toward a bound brakes inside the workspace.
  core.reset();
  KeyboardRun run(core, config);
  run.hold({{1.0, 0.0, 0.0}}, 60);
  const double cruise_start = run.position[0];
  run.hold({{1.0, 0.0, 0.0}}, 15);
  const double held_speed = (run.position[0] - cruise_start) / (15 * run.dt);
  assert(held_speed >= 0.95 * config.max_linear_speed_m_s);
  assert(held_speed <= config.max_linear_speed_m_s + 1e-9);

  const double turn_x = run.position[0];
  run.hold({{0.0, 1.0, 0.0}}, 60);
  assert(run.position[0] - turn_x <= 0.05);

  const int stop_ticks = run.release(90);
  assert(stop_ticks < 60);
  assert(!core.isMoving());
  assert(run.max_acceleration <= config.max_linear_acceleration_m_s2 + 1e-6);
  assert(run.max_jerk <= config.max_linear_jerk_m_s3 + 1e-6);

  // A fresh run: the settle-to-rest snap followed by a restart is a
  // sub-millimetre-per-second step that the finite-difference jerk exaggerates.
  core.reset();
  KeyboardRun climb(core, config);
  climb.hold({{0.0, 0.0, 1.0}}, 300);
  assert(climb.position[2] <= config.workspace_max[2] + 1e-12);
  assert(climb.position[2] >= config.workspace_max[2] - 0.005);
  assert(climb.max_acceleration <= config.max_linear_acceleration_m_s2 + 1e-6);
  assert(climb.max_jerk <= config.max_linear_jerk_m_s3 + 1e-6);

  colmag_ros::Fr3CartesianTeleopConfig invalid_config = config;
  invalid_config.workspace_min[0] = invalid_config.workspace_max[0];
  colmag_ros::Fr3CartesianTeleopCore invalid_core(invalid_config);
  assert(!invalid_core.validateConfiguration(&reason));
  assert(reason == "invalid_workspace_bounds");

  colmag_ros::Fr3CartesianTeleopConfig strict_config = config;
  strict_config.ik_max_iterations = 1;
  strict_config.ik_position_tolerance_m = 1e-12;
  strict_config.ik_orientation_tolerance_rad = 1e-12;
  colmag_ros::Fr3CartesianTeleopCore strict_core(strict_config);
  colmag_ros::MagneticBoardSample edge = engagedCenter();
  edge.x = config.input_x_max;
  edge.y = config.input_y_max;
  edge.z = config.input_z_max;
  const colmag_ros::Fr3CartesianTeleopStep rejected =
      strict_core.step(edge, home(), 1.0 / 30.0);
  assert(!rejected.accepted);
  assert(rejected.reason == "ik_residual_exceeded" ||
         rejected.reason == "dls_solve_failed");

  return 0;
}
