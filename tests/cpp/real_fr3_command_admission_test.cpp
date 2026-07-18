#include <cassert>
#include <string>
#include <vector>

#include "colmag_ros/real_fr3_command_admission.h"

namespace
{

using colmag_ros::RealFr3AdmissionDecision;
using colmag_ros::RealFr3AdmissionInput;
using colmag_ros::RealFr3AdmissionPolicy;
using colmag_ros::RealFr3AdmissionReason;
using colmag_ros::evaluateRealFr3CommandAdmission;
using colmag_ros::realFr3AdmissionReasonCode;

RealFr3AdmissionInput command(const std::string& task)
{
  RealFr3AdmissionInput input;
  input.task = task;
  input.confirmed = true;
  input.accepted = true;
  input.gazebo_only = false;
  input.controls_real_robot = false;
  return input;
}

RealFr3AdmissionPolicy policy(const std::vector<std::string>& allowed_tasks)
{
  RealFr3AdmissionPolicy value;
  value.require_confirmed = true;
  value.require_accepted = true;
  value.allowed_tasks = allowed_tasks;
  value.reject_gazebo_only_commands = true;
  value.reject_controls_real_robot_true = true;
  return value;
}

void expectDecision(const RealFr3AdmissionDecision& decision,
                    bool allowed,
                    RealFr3AdmissionReason reason,
                    const std::string& normalized_task)
{
  assert(decision.allowed == allowed);
  assert(decision.reason == reason);
  assert(decision.normalized_task == normalized_task);
}

}  // namespace

int main()
{
  const RealFr3AdmissionPolicy strategy_a =
      policy({"MOVE_LEFT", "HEXAGON_TRAJECTORY", "MOVE_RIGHT", "STOP_OR_CANCEL"});
  const RealFr3AdmissionPolicy strategy_b = policy({"HEXAGON_TRAJECTORY", "STOP_OR_CANCEL"});

  expectDecision(evaluateRealFr3CommandAdmission(command("move_left"), strategy_a),
                 true, RealFr3AdmissionReason::kAllowed, "MOVE_LEFT");
  expectDecision(evaluateRealFr3CommandAdmission(command("joint_nudge_positive"), strategy_a),
                 true, RealFr3AdmissionReason::kAllowed, "MOVE_LEFT");
  expectDecision(evaluateRealFr3CommandAdmission(command("hexagon_trajectory"), strategy_b),
                 true, RealFr3AdmissionReason::kAllowed, "HEXAGON_TRAJECTORY");

  expectDecision(evaluateRealFr3CommandAdmission(command("unknown"), strategy_a),
                 false, RealFr3AdmissionReason::kTaskNotAllowed, "UNKNOWN");
  expectDecision(evaluateRealFr3CommandAdmission(command(""), strategy_a),
                 false, RealFr3AdmissionReason::kTaskNotAllowed, "");
  expectDecision(evaluateRealFr3CommandAdmission(command("move_left"), strategy_b),
                 false, RealFr3AdmissionReason::kTaskNotAllowed, "MOVE_LEFT");

  RealFr3AdmissionInput rejected = command("MOVE_LEFT");
  rejected.confirmed = false;
  rejected.accepted = false;
  rejected.gazebo_only = true;
  rejected.controls_real_robot = true;
  expectDecision(evaluateRealFr3CommandAdmission(rejected, strategy_a),
                 false, RealFr3AdmissionReason::kConfirmedFalse, "MOVE_LEFT");

  rejected.confirmed = true;
  expectDecision(evaluateRealFr3CommandAdmission(rejected, strategy_a),
                 false, RealFr3AdmissionReason::kAcceptedFalse, "MOVE_LEFT");

  rejected.accepted = true;
  expectDecision(evaluateRealFr3CommandAdmission(rejected, strategy_b),
                 false, RealFr3AdmissionReason::kTaskNotAllowed, "MOVE_LEFT");

  expectDecision(evaluateRealFr3CommandAdmission(rejected, strategy_a),
                 false, RealFr3AdmissionReason::kGazeboOnlyCommand, "MOVE_LEFT");

  rejected.gazebo_only = false;
  expectDecision(evaluateRealFr3CommandAdmission(rejected, strategy_a),
                 false, RealFr3AdmissionReason::kControlsRealRobotTrue, "MOVE_LEFT");

  RealFr3AdmissionInput stop = command("stop_or_cancel");
  stop.gazebo_only = true;
  expectDecision(evaluateRealFr3CommandAdmission(stop, strategy_a),
                 true, RealFr3AdmissionReason::kAllowed, "STOP_OR_CANCEL");
  expectDecision(evaluateRealFr3CommandAdmission(stop, strategy_b),
                 true, RealFr3AdmissionReason::kAllowed, "STOP_OR_CANCEL");
  stop.controls_real_robot = true;
  stop.confirmed = false;
  stop.accepted = false;
  expectDecision(evaluateRealFr3CommandAdmission(stop, strategy_a),
                 true, RealFr3AdmissionReason::kAllowed, "STOP_OR_CANCEL");

  RealFr3AdmissionPolicy relaxed = strategy_a;
  relaxed.require_confirmed = false;
  relaxed.require_accepted = false;
  relaxed.reject_gazebo_only_commands = false;
  relaxed.reject_controls_real_robot_true = false;
  expectDecision(evaluateRealFr3CommandAdmission(rejected, relaxed),
                 true, RealFr3AdmissionReason::kAllowed, "MOVE_LEFT");

  assert(std::string(realFr3AdmissionReasonCode(RealFr3AdmissionReason::kConfirmedFalse)) ==
         "confirmed_false");
  assert(std::string(realFr3AdmissionReasonCode(RealFr3AdmissionReason::kAcceptedFalse)) ==
         "accepted_false");
  assert(std::string(realFr3AdmissionReasonCode(RealFr3AdmissionReason::kTaskNotAllowed)) ==
         "task_not_allowed");
  assert(std::string(realFr3AdmissionReasonCode(RealFr3AdmissionReason::kGazeboOnlyCommand)) ==
         "gazebo_only_command_not_valid_for_real_fr3");

  return 0;
}
