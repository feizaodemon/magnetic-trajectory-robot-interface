#include "colmag_ros/fr3_hardware_safety.h"

#include <algorithm>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cmath>
#include <limits>

namespace colmag_ros
{
namespace
{

double runtimeDurationUpperBound(const std::string& parameter)
{
  if (parameter == "action_server_wait_sec") return 5.0;
  if (parameter == "execution_timeout_sec") return 30.0;
  if (parameter == "final_state_timeout_sec") return 5.0;
  if (parameter == "trajectory_start_delay_sec") return 2.0;
  if (parameter == "joint_state_max_age_sec") return 2.0;
  if (parameter == "command_max_age_sec") return 10.0;
  if (parameter == "watchdog_period_sec") return 1.0;
  return 0.0;
}

}  // namespace

SteadyDuration steadyDurationFromSeconds(double seconds)
{
  if (!std::isfinite(seconds) || seconds <= 0.0)
  {
    return SteadyDuration::zero();
  }
  return std::chrono::duration_cast<SteadyDuration>(std::chrono::duration<double>(seconds));
}

std::string newAdapterSessionId()
{
  return boost::uuids::to_string(boost::uuids::random_generator()());
}

bool validateRuntimeDuration(const std::string& parameter,
                             double value_sec,
                             std::string* reason)
{
  const double maximum = runtimeDurationUpperBound(parameter);
  if (maximum <= 0.0)
  {
    *reason = "unknown_runtime_duration_parameter:" + parameter;
    return false;
  }
  if (!std::isfinite(value_sec) || value_sec <= 0.0)
  {
    *reason = "invalid_runtime_duration:" + parameter;
    return false;
  }
  if (value_sec > maximum)
  {
    *reason = "runtime_duration_above_maximum:" + parameter;
    return false;
  }
  reason->clear();
  return true;
}

CommandReplayGuard::CommandReplayGuard(double max_age_sec,
                                       double future_tolerance_sec,
                                       std::size_t cache_size,
                                       int supported_schema_version,
                                       const std::string& adapter_session_id)
  : max_age_sec_(max_age_sec),
    future_tolerance_sec_(future_tolerance_sec),
    cache_size_(cache_size),
    supported_schema_version_(supported_schema_version),
    adapter_session_id_(adapter_session_id)
{
}

CommandGuardDecision CommandReplayGuard::evaluate(const CommandMetadata& metadata,
                                                  double now_sec,
                                                  bool hardware_active,
                                                  bool motion_command)
{
  (void)motion_command;
  CommandGuardDecision decision;
  const bool metadata_complete = !metadata.command_id.empty() && metadata.has_issued_at &&
                                 metadata.has_schema_version &&
                                 !metadata.target_adapter_session_id.empty();
  if (!hardware_active && !metadata_complete)
  {
    decision.allowed = true;
    decision.compatibility_warning = true;
    decision.reason = "legacy_command_metadata_missing_dry_run_only";
    return decision;
  }
  if (metadata.command_id.empty())
  {
    decision.reason = "missing_command_id";
    return decision;
  }
  if (recent_id_set_.count(metadata.command_id) != 0)
  {
    decision.reason = "duplicate_command_id";
    return decision;
  }
  if (!metadata.has_issued_at || !std::isfinite(metadata.issued_at_sec) || !std::isfinite(now_sec))
  {
    decision.reason = "missing_or_non_finite_issued_at";
    return decision;
  }
  if (hardware_active && now_sec <= 0.0)
  {
    decision.reason = "invalid_current_time";
    return decision;
  }
  if (!metadata.has_schema_version || metadata.schema_version != supported_schema_version_)
  {
    decision.reason = "unsupported_schema_version";
    return decision;
  }
  if (hardware_active && metadata.target_adapter_session_id != adapter_session_id_)
  {
    decision.reason = "command_adapter_session_mismatch";
    return decision;
  }
  if (hardware_active && have_last_accepted_ros_time_ && now_sec < last_accepted_ros_time_sec_)
  {
    decision.reason = "command_ros_time_moved_backwards";
    return decision;
  }

  const double age_sec = now_sec - metadata.issued_at_sec;
  if (age_sec > max_age_sec_)
  {
    decision.reason = "stale_command";
    return decision;
  }
  if (age_sec < -future_tolerance_sec_)
  {
    decision.reason = "future_command";
    return decision;
  }
  remember(metadata.command_id);
  if (hardware_active)
  {
    have_last_accepted_ros_time_ = true;
    last_accepted_ros_time_sec_ = now_sec;
  }
  decision.allowed = true;
  return decision;
}

void CommandReplayGuard::remember(const std::string& command_id)
{
  if (cache_size_ == 0)
  {
    return;
  }
  recent_ids_.push_back(command_id);
  recent_id_set_.insert(command_id);
  while (recent_ids_.size() > cache_size_)
  {
    recent_id_set_.erase(recent_ids_.front());
    recent_ids_.pop_front();
  }
}

OneShotMotionGate::OneShotMotionGate(bool enabled, std::size_t maximum_motion_goals)
  : enabled_(enabled), maximum_motion_goals_(maximum_motion_goals)
{
}

bool OneShotMotionGate::canSend(std::string* reason) const
{
  if (enabled_ && maximum_motion_goals_ != 0 && send_attempts_ >= maximum_motion_goals_)
  {
    *reason = "one_shot_motion_limit_reached";
    return false;
  }
  reason->clear();
  return true;
}

void OneShotMotionGate::recordSendAttempt()
{
  ++send_attempts_;
}

std::size_t OneShotMotionGate::sendAttempts() const
{
  return send_attempts_;
}

bool RosTimeRollbackGuard::evaluate(double now_sec,
                                    const std::string& rollback_reason,
                                    std::string* reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(now_sec) || now_sec <= 0.0)
  {
    *reason = "invalid_ros_time";
    return false;
  }
  if (have_last_time_ && now_sec < last_time_sec_)
  {
    *reason = rollback_reason;
    return false;
  }
  have_last_time_ = true;
  last_time_sec_ = now_sec;
  reason->clear();
  return true;
}

const char* goalLifecycleStateName(GoalLifecycleState state)
{
  switch (state)
  {
    case GoalLifecycleState::kReady:
      return "READY";
    case GoalLifecycleState::kGoalPrepared:
      return "GOAL_PREPARED";
    case GoalLifecycleState::kGoalSending:
      return "GOAL_SENDING";
    case GoalLifecycleState::kGoalSent:
      return "GOAL_SENT";
    case GoalLifecycleState::kGoalActive:
      return "GOAL_ACTIVE";
    case GoalLifecycleState::kFinalStateVerifying:
      return "FINAL_STATE_VERIFYING";
    case GoalLifecycleState::kGoalRejected:
      return "GOAL_REJECTED";
    case GoalLifecycleState::kGoalAborted:
      return "GOAL_ABORTED";
    case GoalLifecycleState::kGoalPreempted:
      return "GOAL_PREEMPTED";
    case GoalLifecycleState::kGoalTimeout:
      return "GOAL_TIMEOUT";
    case GoalLifecycleState::kFinalStateVerified:
      return "FINAL_STATE_VERIFIED";
    case GoalLifecycleState::kFinalStateVerifyFailed:
      return "FINAL_STATE_VERIFY_FAILED";
    case GoalLifecycleState::kSoftwareCancelRequested:
      return "SOFTWARE_CANCEL_REQUESTED";
    case GoalLifecycleState::kSoftwareCancelComplete:
      return "SOFTWARE_CANCEL_COMPLETE";
    case GoalLifecycleState::kSendFailed:
      return "SEND_FAILED";
    case GoalLifecycleState::kJointStateFailed:
      return "JOINT_STATE_FAILED";
    case GoalLifecycleState::kShutdownCancelled:
      return "SHUTDOWN_CANCELLED";
  }
  return "UNKNOWN";
}

const char* transitionDispositionName(TransitionDisposition disposition)
{
  switch (disposition)
  {
    case TransitionDisposition::kAccepted:
      return "accepted";
    case TransitionDisposition::kDeferred:
      return "deferred";
    case TransitionDisposition::kIgnoredStaleGeneration:
      return "ignored_stale_generation";
    case TransitionDisposition::kIgnoredAlreadyTerminal:
      return "ignored_already_terminal";
    case TransitionDisposition::kInvalidTransition:
      return "invalid_transition";
  }
  return "unknown";
}

LifecycleStatusProjection projectLifecycleStatus(
    const LifecycleSnapshot& lifecycle,
    const std::string& requested_state,
    const std::string& diagnostic_code,
    const std::string& diagnostic_command_id,
    const std::string& fallback_command_id,
    const std::string& fallback_source_id)
{
  LifecycleStatusProjection projection;
  const bool lifecycle_exists = lifecycle.goal_generation != 0;
  projection.state = lifecycle_exists ? goalLifecycleStateName(lifecycle.state) : requested_state;
  projection.command_id = lifecycle_exists ? lifecycle.identity.command_id : fallback_command_id;
  projection.source_id = lifecycle_exists ? lifecycle.identity.source_id : fallback_source_id;
  projection.diagnostic_code = diagnostic_code;
  projection.diagnostic_command_id = diagnostic_command_id;
  projection.lifecycle_state = lifecycle.state;
  projection.goal_generation = lifecycle.goal_generation;
  projection.lifecycle_event_sequence = lifecycle.event_sequence;
  projection.terminal = lifecycle.terminal;
  return projection;
}

LifecycleTransition GoalLifecycle::transitionLocked(TransitionDisposition disposition,
                                                    const std::string& reason,
                                                    bool cancel_owned_goal)
{
  LifecycleTransition transition;
  transition.disposition = disposition;
  transition.goal_generation = generation_;
  transition.event_sequence = event_sequence_;
  transition.state = state_;
  transition.terminal = terminal_;
  transition.cancel_owned_goal = cancel_owned_goal;
  transition.reason = reason;
  return transition;
}

LifecycleTransition GoalLifecycle::acceptedTransitionLocked(const std::string& reason,
                                                            bool cancel_owned_goal)
{
  ++event_sequence_;
  return transitionLocked(TransitionDisposition::kAccepted, reason, cancel_owned_goal);
}

LifecycleTransition GoalLifecycle::terminalTransitionLocked(GoalLifecycleState state,
                                                            const std::string& reason,
                                                            bool cancel_owned_goal)
{
  state_ = state;
  terminal_ = true;
  owned_active_goal_ = false;
  awaiting_final_verification_ = false;
  ++terminal_transition_count_;
  return acceptedTransitionLocked(reason, cancel_owned_goal);
}

LifecycleTransition GoalLifecycle::beginGoal(const GoalIdentity& identity)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (shutting_down_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal, "shutting_down");
  }
  if (generation_ != 0 && !terminal_)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition, "goal_lifecycle_busy");
  }
  if (generation_ != 0 && state_ != GoalLifecycleState::kFinalStateVerified)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal,
                            "goal_lifecycle_requires_restart");
  }
  ++generation_;
  identity_ = identity;
  state_ = GoalLifecycleState::kGoalPrepared;
  terminal_ = false;
  owned_active_goal_ = false;
  awaiting_final_verification_ = false;
  transport_established_ = false;
  cancel_dispatched_ = false;
  done_received_ = false;
  deferred_active_ = false;
  deferred_done_ = false;
  execution_deadline_ = SteadyTimePoint();
  final_state_deadline_ = SteadyTimePoint();
  result_receive_steady_time_ = SteadyTimePoint();
  has_execution_deadline_ = false;
  has_final_state_deadline_ = false;
  result_ros_time_sec_ = 0.0;
  joint_state_sequence_baseline_ = 0;
  final_joint_error_ = std::numeric_limits<double>::quiet_NaN();
  result_error_code_ = 0;
  terminal_transition_count_ = 0;
  final_verification_start_count_ = 0;
  return acceptedTransitionLocked("goal_prepared");
}

LifecycleTransition GoalLifecycle::beginSend(std::uint64_t generation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != generation_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredStaleGeneration);
  }
  if (shutting_down_ || terminal_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal);
  }
  if (state_ != GoalLifecycleState::kGoalPrepared)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition);
  }
  state_ = GoalLifecycleState::kGoalSending;
  return acceptedTransitionLocked("send_attempt_started");
}

std::vector<LifecycleTransition> GoalLifecycle::markSendSucceeded(
    std::uint64_t generation, SteadyTimePoint now, SteadyDuration execution_timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<LifecycleTransition> transitions;
  if (generation != generation_)
  {
    transitions.push_back(transitionLocked(TransitionDisposition::kIgnoredStaleGeneration));
    return transitions;
  }
  if (shutting_down_ || terminal_)
  {
    if (state_ == GoalLifecycleState::kShutdownCancelled &&
        state_ != GoalLifecycleState::kSendFailed && !transport_established_)
    {
      transport_established_ = true;
      const bool cancel = !cancel_dispatched_;
      cancel_dispatched_ = true;
      transitions.push_back(transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal,
                                             "shutdown_after_transport_send", cancel));
    }
    else
    {
      transitions.push_back(transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal));
    }
    return transitions;
  }
  if (state_ != GoalLifecycleState::kGoalSending || execution_timeout <= SteadyDuration::zero())
  {
    transitions.push_back(transitionLocked(TransitionDisposition::kInvalidTransition));
    return transitions;
  }
  state_ = GoalLifecycleState::kGoalSent;
  transport_established_ = true;
  owned_active_goal_ = true;
  execution_deadline_ = now + execution_timeout;
  has_execution_deadline_ = true;
  transitions.push_back(acceptedTransitionLocked("goal_sent"));
  if (deferred_active_)
  {
    deferred_active_ = false;
    transitions.push_back(applyActiveLocked(deferred_active_time_));
  }
  if (deferred_done_)
  {
    deferred_done_ = false;
    transitions.push_back(applyDoneLocked(deferred_result_));
  }
  return transitions;
}

LifecycleTransition GoalLifecycle::markSendFailed(std::uint64_t generation, int error_code)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != generation_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredStaleGeneration);
  }
  if (shutting_down_ || terminal_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal);
  }
  if (state_ != GoalLifecycleState::kGoalSending)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition);
  }
  result_error_code_ = error_code;
  deferred_active_ = false;
  deferred_done_ = false;
  return terminalTransitionLocked(GoalLifecycleState::kSendFailed, "send_failed");
}

LifecycleTransition GoalLifecycle::handleJointStateFailure(std::uint64_t generation,
                                                           const std::string& detail)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != generation_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredStaleGeneration);
  }
  if (generation_ == 0 || !transport_established_)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition,
                            "joint_state_failure_before_motion_ownership");
  }
  if (shutting_down_ || terminal_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal);
  }
  const bool cancel = owned_active_goal_ && !cancel_dispatched_;
  cancel_dispatched_ = cancel_dispatched_ || cancel;
  return terminalTransitionLocked(
      GoalLifecycleState::kJointStateFailed,
      detail.empty() ? "post_send_joint_state_failure" :
                       "post_send_joint_state_failure:" + detail,
      cancel);
}

LifecycleTransition GoalLifecycle::executionTimeoutIfReachedLocked(SteadyTimePoint now)
{
  if (!has_execution_deadline_ || now < execution_deadline_ || done_received_ ||
      state_ == GoalLifecycleState::kFinalStateVerifying || !owned_active_goal_)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition,
                            "execution_deadline_not_reached");
  }
  const bool cancel = transport_established_ && !cancel_dispatched_;
  cancel_dispatched_ = cancel_dispatched_ || cancel;
  return terminalTransitionLocked(GoalLifecycleState::kGoalTimeout,
                                  "execution_timeout", cancel);
}

LifecycleTransition GoalLifecycle::finalTimeoutIfReachedLocked(SteadyTimePoint now)
{
  if (!has_final_state_deadline_ || now < final_state_deadline_ ||
      state_ != GoalLifecycleState::kFinalStateVerifying || !awaiting_final_verification_)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition,
                            "final_state_deadline_not_reached");
  }
  return finalFailureLocked("final_state_timeout");
}

LifecycleTransition GoalLifecycle::applyActiveLocked(SteadyTimePoint now)
{
  if (terminal_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal);
  }
  if (state_ == GoalLifecycleState::kSoftwareCancelRequested ||
      state_ == GoalLifecycleState::kFinalStateVerifying || done_received_)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition);
  }
  if (state_ != GoalLifecycleState::kGoalSent && state_ != GoalLifecycleState::kGoalActive)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition);
  }
  const LifecycleTransition deadline = executionTimeoutIfReachedLocked(now);
  if (deadline.accepted())
  {
    return deadline;
  }
  if (state_ == GoalLifecycleState::kGoalActive)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition, "duplicate_active");
  }
  state_ = GoalLifecycleState::kGoalActive;
  return acceptedTransitionLocked("goal_active");
}

LifecycleTransition GoalLifecycle::handleActive(std::uint64_t generation, SteadyTimePoint now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != generation_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredStaleGeneration);
  }
  if (shutting_down_ || terminal_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal);
  }
  const LifecycleTransition final_deadline = finalTimeoutIfReachedLocked(now);
  if (final_deadline.accepted())
  {
    return final_deadline;
  }
  if (state_ == GoalLifecycleState::kGoalSending)
  {
    deferred_active_ = true;
    deferred_active_time_ = now;
    return transitionLocked(TransitionDisposition::kDeferred, "active_deferred_until_send_returns");
  }
  return applyActiveLocked(now);
}

LifecycleTransition GoalLifecycle::applyDoneLocked(const GoalResultEvent& result)
{
  if (terminal_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal);
  }
  if (done_received_ || state_ == GoalLifecycleState::kFinalStateVerifying)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition, "duplicate_done");
  }
  const LifecycleTransition deadline = executionTimeoutIfReachedLocked(
      result.result_receive_steady_time);
  if (deadline.accepted())
  {
    result_error_code_ = result.result_error_code;
    return deadline;
  }
  if (!transport_established_ ||
      (state_ != GoalLifecycleState::kGoalSent &&
       state_ != GoalLifecycleState::kGoalActive &&
       state_ != GoalLifecycleState::kSoftwareCancelRequested))
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition);
  }
  done_received_ = true;
  owned_active_goal_ = false;
  result_error_code_ = result.result_error_code;
  if (state_ == GoalLifecycleState::kSoftwareCancelRequested)
  {
    return terminalTransitionLocked(GoalLifecycleState::kSoftwareCancelComplete,
                                    "software_cancel_transport_complete");
  }
  if (result.terminal_state == GoalTerminalState::kSucceeded && result.result_error_code == 0)
  {
    if (!std::isfinite(result.result_ros_time_sec) || result.result_ros_time_sec <= 0.0 ||
        result.final_state_timeout <= SteadyDuration::zero())
    {
      return terminalTransitionLocked(GoalLifecycleState::kFinalStateVerifyFailed,
                                      "invalid_result_time_baseline");
    }
    state_ = GoalLifecycleState::kFinalStateVerifying;
    awaiting_final_verification_ = true;
    result_receive_steady_time_ = result.result_receive_steady_time;
    result_ros_time_sec_ = result.result_ros_time_sec;
    joint_state_sequence_baseline_ = result.joint_state_sequence_baseline;
    final_state_deadline_ = result.result_receive_steady_time + result.final_state_timeout;
    has_final_state_deadline_ = true;
    ++final_verification_start_count_;
    return acceptedTransitionLocked("action_succeeded_final_verification_required");
  }
  if (result.terminal_state == GoalTerminalState::kRejected)
  {
    return terminalTransitionLocked(GoalLifecycleState::kGoalRejected, "action_rejected");
  }
  if (result.terminal_state == GoalTerminalState::kPreempted)
  {
    return terminalTransitionLocked(GoalLifecycleState::kGoalPreempted, "action_preempted");
  }
  return terminalTransitionLocked(GoalLifecycleState::kGoalAborted, "action_aborted");
}

LifecycleTransition GoalLifecycle::handleDone(std::uint64_t generation,
                                              const GoalResultEvent& result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != generation_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredStaleGeneration);
  }
  if (shutting_down_ || terminal_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal);
  }
  const LifecycleTransition final_deadline = finalTimeoutIfReachedLocked(
      result.result_receive_steady_time);
  if (final_deadline.accepted())
  {
    return final_deadline;
  }
  if (state_ == GoalLifecycleState::kGoalSending)
  {
    if (deferred_done_)
    {
      return transitionLocked(TransitionDisposition::kInvalidTransition, "duplicate_deferred_done");
    }
    deferred_done_ = true;
    deferred_result_ = result;
    return transitionLocked(TransitionDisposition::kDeferred, "done_deferred_until_send_returns");
  }
  return applyDoneLocked(result);
}

LifecycleTransition GoalLifecycle::requestCancel(std::uint64_t generation, SteadyTimePoint now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != generation_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredStaleGeneration);
  }
  if (shutting_down_ || terminal_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal);
  }
  const LifecycleTransition deadline = executionTimeoutIfReachedLocked(now);
  if (deadline.accepted())
  {
    return deadline;
  }
  const LifecycleTransition final_deadline = finalTimeoutIfReachedLocked(now);
  if (final_deadline.accepted())
  {
    return final_deadline;
  }
  if (!owned_active_goal_ || state_ == GoalLifecycleState::kFinalStateVerifying)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition, "no_owned_active_goal");
  }
  if (state_ == GoalLifecycleState::kSoftwareCancelRequested)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition, "cancel_already_requested");
  }
  state_ = GoalLifecycleState::kSoftwareCancelRequested;
  const bool cancel = transport_established_ && !cancel_dispatched_;
  cancel_dispatched_ = cancel_dispatched_ || cancel;
  return acceptedTransitionLocked("software_cancel_requested", cancel);
}

LifecycleTransition GoalLifecycle::handleTimeout(std::uint64_t generation, SteadyTimePoint now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != generation_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredStaleGeneration);
  }
  if (shutting_down_ || terminal_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal);
  }
  const LifecycleTransition execution_deadline = executionTimeoutIfReachedLocked(now);
  if (execution_deadline.accepted())
  {
    return execution_deadline;
  }
  return finalTimeoutIfReachedLocked(now);
}

LifecycleTransition GoalLifecycle::finalFailureLocked(const std::string& reason)
{
  return terminalTransitionLocked(GoalLifecycleState::kFinalStateVerifyFailed, reason);
}

LifecycleTransition GoalLifecycle::verifyFinalState(std::uint64_t generation,
                                                    const FinalStateSample& sample,
                                                    const FinalStatePolicy& policy)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != generation_ || sample.goal_generation != generation_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredStaleGeneration);
  }
  if (shutting_down_ || terminal_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal);
  }
  if (state_ != GoalLifecycleState::kFinalStateVerifying || !awaiting_final_verification_)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition);
  }
  const LifecycleTransition deadline = finalTimeoutIfReachedLocked(sample.receive_steady_time);
  if (deadline.accepted())
  {
    return deadline;
  }
  if (sample.receive_sequence <= joint_state_sequence_baseline_)
  {
    return transitionLocked(TransitionDisposition::kInvalidTransition,
                            "joint_state_not_received_after_result");
  }
  if (sample.receive_steady_time <= result_receive_steady_time_)
  {
    return finalFailureLocked("joint_state_receive_time_not_after_result");
  }
  if (!std::isfinite(sample.current_ros_time_sec) || sample.current_ros_time_sec <= 0.0 ||
      !std::isfinite(sample.header_stamp_sec) || sample.header_stamp_sec <= 0.0 ||
      !std::isfinite(result_ros_time_sec_) || result_ros_time_sec_ <= 0.0 ||
      !std::isfinite(policy.max_age_sec) || policy.max_age_sec <= 0.0 ||
      !std::isfinite(policy.future_tolerance_sec) || policy.future_tolerance_sec < 0.0 ||
      !std::isfinite(policy.error_tolerance) || policy.error_tolerance < 0.0)
  {
    return finalFailureLocked("invalid_final_time_or_tolerance");
  }
  if (sample.current_ros_time_sec < result_ros_time_sec_)
  {
    return finalFailureLocked("final_state_ros_time_moved_backwards");
  }
  const double age_sec = sample.current_ros_time_sec - sample.header_stamp_sec;
  if (age_sec > policy.max_age_sec)
  {
    return finalFailureLocked("stale_final_joint_state");
  }
  if (age_sec < -policy.future_tolerance_sec)
  {
    return finalFailureLocked("future_final_joint_state");
  }
  if (sample.names != policy.configured_joint_names ||
      sample.positions.size() != sample.names.size())
  {
    return finalFailureLocked("final_joint_state_names_mismatch");
  }
  if (!identity_.final_joint_names.empty() || !identity_.final_joint_positions.empty())
  {
    if (identity_.final_joint_names != policy.configured_joint_names ||
        identity_.final_joint_positions.size() != identity_.final_joint_names.size() ||
        !std::all_of(identity_.final_joint_positions.begin(),
                     identity_.final_joint_positions.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(sample.positions.begin(), sample.positions.end(),
                     [](double value) { return std::isfinite(value); }))
    {
      return finalFailureLocked("final_joint_state_names_mismatch");
    }
    final_joint_error_ = 0.0;
    for (std::size_t index = 0; index < sample.positions.size(); ++index)
    {
      final_joint_error_ = std::max(
          final_joint_error_,
          std::fabs(sample.positions[index] - identity_.final_joint_positions[index]));
    }
    if (final_joint_error_ > policy.error_tolerance)
    {
      return finalFailureLocked("final_joint_error_above_tolerance");
    }
    return terminalTransitionLocked(GoalLifecycleState::kFinalStateVerified,
                                    "final_state_verified");
  }
  const std::size_t target_count = static_cast<std::size_t>(
      std::count(sample.names.begin(), sample.names.end(), identity_.target_joint));
  if (target_count != 1)
  {
    return finalFailureLocked(target_count == 0 ? "final_target_joint_missing" :
                                                  "final_target_joint_duplicate");
  }
  const std::vector<std::string>::const_iterator target =
      std::find(sample.names.begin(), sample.names.end(), identity_.target_joint);
  const std::size_t index = static_cast<std::size_t>(target - sample.names.begin());
  if (index >= sample.positions.size() || !std::isfinite(sample.positions[index]) ||
      !std::isfinite(identity_.target_position))
  {
    return finalFailureLocked("non_finite_final_joint_state");
  }
  final_joint_error_ = std::fabs(sample.positions[index] - identity_.target_position);
  if (final_joint_error_ > policy.error_tolerance)
  {
    return finalFailureLocked("final_joint_error_above_tolerance");
  }
  return terminalTransitionLocked(GoalLifecycleState::kFinalStateVerified,
                                  "final_state_verified");
}

LifecycleTransition GoalLifecycle::handleFinalStateTimeout(std::uint64_t generation,
                                                           SteadyTimePoint now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != generation_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredStaleGeneration);
  }
  if (shutting_down_ || terminal_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal);
  }
  return finalTimeoutIfReachedLocked(now);
}

LifecycleTransition GoalLifecycle::beginShutdown(SteadyTimePoint now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (shutting_down_)
  {
    return transitionLocked(TransitionDisposition::kIgnoredAlreadyTerminal, "shutdown_already_started");
  }
  shutting_down_ = true;
  deferred_active_ = false;
  deferred_done_ = false;
  if (generation_ == 0 || terminal_)
  {
    return acceptedTransitionLocked("shutdown_without_active_goal");
  }
  const LifecycleTransition execution_deadline = executionTimeoutIfReachedLocked(now);
  if (execution_deadline.accepted())
  {
    return execution_deadline;
  }
  const LifecycleTransition final_deadline = finalTimeoutIfReachedLocked(now);
  if (final_deadline.accepted())
  {
    return final_deadline;
  }
  const bool cancel = owned_active_goal_ && transport_established_ && !cancel_dispatched_;
  cancel_dispatched_ = cancel_dispatched_ || cancel;
  return terminalTransitionLocked(GoalLifecycleState::kShutdownCancelled,
                                  "shutdown_cancelled", cancel);
}

LifecycleSnapshot GoalLifecycle::snapshotLocked() const
{
  LifecycleSnapshot value;
  value.state = state_;
  value.goal_generation = generation_;
  value.event_sequence = event_sequence_;
  value.identity = identity_;
  value.terminal = terminal_;
  value.owned_active_goal = owned_active_goal_;
  value.awaiting_final_verification = awaiting_final_verification_;
  value.shutting_down = shutting_down_;
  value.execution_deadline = execution_deadline_;
  value.final_state_deadline = final_state_deadline_;
  value.final_joint_error = final_joint_error_;
  value.result_error_code = result_error_code_;
  value.terminal_transition_count = terminal_transition_count_;
  value.final_verification_start_count = final_verification_start_count_;
  return value;
}

LifecycleSnapshot GoalLifecycle::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshotLocked();
}

}  // namespace colmag_ros
