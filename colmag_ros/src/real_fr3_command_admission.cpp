#include "colmag_ros/real_fr3_command_admission.h"

#include <algorithm>
#include <cctype>

namespace colmag_ros
{

namespace
{

RealFr3AdmissionDecision rejected(const std::string& task, RealFr3AdmissionReason reason)
{
  RealFr3AdmissionDecision decision;
  decision.allowed = false;
  decision.reason = reason;
  decision.normalized_task = task;
  return decision;
}

bool containsNormalizedTask(const std::vector<std::string>& tasks, const std::string& task)
{
  return std::find_if(tasks.begin(), tasks.end(), [&task](const std::string& candidate) {
    return normalizeRealFr3Task(candidate) == task;
  }) != tasks.end();
}

}  // namespace

std::string normalizeRealFr3Task(const std::string& task)
{
  std::string normalized = task;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
    return static_cast<char>(std::toupper(ch));
  });
  if (normalized == "JOINT_NUDGE_POSITIVE")
  {
    return "MOVE_LEFT";
  }
  return normalized;
}

const char* realFr3AdmissionReasonCode(RealFr3AdmissionReason reason)
{
  switch (reason)
  {
    case RealFr3AdmissionReason::kAllowed:
      return "";
    case RealFr3AdmissionReason::kConfirmedFalse:
      return "confirmed_false";
    case RealFr3AdmissionReason::kAcceptedFalse:
      return "accepted_false";
    case RealFr3AdmissionReason::kTaskNotAllowed:
      return "task_not_allowed";
    case RealFr3AdmissionReason::kGazeboOnlyCommand:
      return "gazebo_only_command_not_valid_for_real_fr3";
    case RealFr3AdmissionReason::kControlsRealRobotTrue:
      return "controls_real_robot_true";
  }
  return "unknown_admission_reason";
}

RealFr3AdmissionDecision evaluateRealFr3CommandAdmission(
    const RealFr3AdmissionInput& input,
    const RealFr3AdmissionPolicy& policy)
{
  const std::string task = normalizeRealFr3Task(input.task);
  if (!containsNormalizedTask(policy.allowed_tasks, task))
  {
    return rejected(task, RealFr3AdmissionReason::kTaskNotAllowed);
  }
  if (task == "STOP_OR_CANCEL")
  {
    RealFr3AdmissionDecision decision;
    decision.allowed = true;
    decision.reason = RealFr3AdmissionReason::kAllowed;
    decision.normalized_task = task;
    return decision;
  }
  if (policy.require_confirmed && !input.confirmed)
  {
    return rejected(task, RealFr3AdmissionReason::kConfirmedFalse);
  }
  if (policy.require_accepted && !input.accepted)
  {
    return rejected(task, RealFr3AdmissionReason::kAcceptedFalse);
  }
  if (policy.reject_gazebo_only_commands && input.gazebo_only)
  {
    return rejected(task, RealFr3AdmissionReason::kGazeboOnlyCommand);
  }
  if (policy.reject_controls_real_robot_true && input.controls_real_robot)
  {
    return rejected(task, RealFr3AdmissionReason::kControlsRealRobotTrue);
  }

  RealFr3AdmissionDecision decision;
  decision.allowed = true;
  decision.reason = RealFr3AdmissionReason::kAllowed;
  decision.normalized_task = task;
  return decision;
}

}  // namespace colmag_ros
