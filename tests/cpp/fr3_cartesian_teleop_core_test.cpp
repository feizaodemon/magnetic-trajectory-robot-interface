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
