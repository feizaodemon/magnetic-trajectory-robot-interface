#pragma once

#include <string>
#include <vector>

namespace colmag_ros
{

struct RealFr3AdmissionInput
{
  std::string task;
  bool confirmed = false;
  bool accepted = false;
  bool gazebo_only = false;
  bool controls_real_robot = false;
};

struct RealFr3AdmissionPolicy
{
  bool require_confirmed = true;
  bool require_accepted = true;
  std::vector<std::string> allowed_tasks;
  bool reject_gazebo_only_commands = true;
  bool reject_controls_real_robot_true = true;
};

enum class RealFr3AdmissionReason
{
  kAllowed,
  kConfirmedFalse,
  kAcceptedFalse,
  kTaskNotAllowed,
  kGazeboOnlyCommand,
  kControlsRealRobotTrue,
};

struct RealFr3AdmissionDecision
{
  bool allowed = false;
  RealFr3AdmissionReason reason = RealFr3AdmissionReason::kTaskNotAllowed;
  std::string normalized_task = "NO_OP";
};

std::string normalizeRealFr3Task(const std::string& task);

const char* realFr3AdmissionReasonCode(RealFr3AdmissionReason reason);

RealFr3AdmissionDecision evaluateRealFr3CommandAdmission(
    const RealFr3AdmissionInput& input,
    const RealFr3AdmissionPolicy& policy);

}  // namespace colmag_ros
