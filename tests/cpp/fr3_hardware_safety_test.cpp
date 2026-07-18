#include <atomic>
#include <cassert>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "colmag_ros/fr3_hardware_safety.h"

namespace
{

struct FakeTransportClient
{
  FakeTransportClient(std::atomic<int>* cancel_count,
                      std::atomic<int>* destruction_count)
    : cancel_count(cancel_count), destruction_count(destruction_count)
  {
  }

  ~FakeTransportClient()
  {
    if (callback.joinable())
    {
      callback.join();
    }
    ++(*destruction_count);
  }

  void cancelGoal() { ++(*cancel_count); }
  void startCallback(const std::function<void()>& callback_body)
  {
    callback = std::thread(callback_body);
  }

  std::atomic<int>* cancel_count;
  std::atomic<int>* destruction_count;
  std::thread callback;
};

colmag_ros::SteadyTimePoint steady(double seconds)
{
  return colmag_ros::SteadyTimePoint(
      std::chrono::duration_cast<colmag_ros::SteadyDuration>(
          std::chrono::duration<double>(seconds)));
}

colmag_ros::CommandMetadata metadata(const std::string& id,
                                     double issued_at,
                                     const std::string& session = "session-current")
{
  colmag_ros::CommandMetadata value;
  value.command_id = id;
  value.target_adapter_session_id = session;
  value.has_issued_at = true;
  value.issued_at_sec = issued_at;
  value.has_schema_version = true;
  value.schema_version = 1;
  return value;
}

colmag_ros::GoalIdentity identity(const std::string& command_id)
{
  colmag_ros::GoalIdentity value;
  value.command_id = command_id;
  value.source_id = "test-source";
  value.target_joint = "fr3_joint7";
  value.target_position = 0.25;
  return value;
}

colmag_ros::GoalResultEvent result(colmag_ros::GoalTerminalState terminal_state,
                                   int error_code,
                                   std::uint64_t joint_state_sequence_baseline,
                                   double receive_time = 6.0)
{
  colmag_ros::GoalResultEvent value;
  value.terminal_state = terminal_state;
  value.result_error_code = error_code;
  value.result_receive_steady_time = steady(receive_time);
  value.result_ros_time_sec = 10.0;
  value.final_state_timeout = colmag_ros::steadyDurationFromSeconds(1.0);
  value.joint_state_sequence_baseline = joint_state_sequence_baseline;
  return value;
}

colmag_ros::FinalStateSample finalSample(std::uint64_t generation,
                                        std::uint64_t receive_sequence,
                                        double header_stamp = 10.1,
                                        double receive_time = 6.1,
                                        double position = 0.25)
{
  colmag_ros::FinalStateSample value;
  value.goal_generation = generation;
  value.receive_sequence = receive_sequence;
  value.receive_steady_time = steady(receive_time);
  value.header_stamp_sec = header_stamp;
  value.current_ros_time_sec = 10.2;
  value.names = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
                 "fr3_joint5", "fr3_joint6", "fr3_joint7"};
  value.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, position};
  return value;
}

colmag_ros::FinalStatePolicy finalPolicy()
{
  colmag_ros::FinalStatePolicy value;
  value.configured_joint_names = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
                                  "fr3_joint5", "fr3_joint6", "fr3_joint7"};
  value.max_age_sec = 0.5;
  value.future_tolerance_sec = 0.05;
  value.error_tolerance = 0.01;
  return value;
}

std::uint64_t startActiveGoal(colmag_ros::GoalLifecycle* lifecycle,
                              const std::string& command_id = "motion")
{
  colmag_ros::LifecycleTransition prepared = lifecycle->beginGoal(identity(command_id));
  assert(prepared.accepted());
  const std::uint64_t generation = prepared.goal_generation;
  assert(lifecycle->beginSend(generation).accepted());
  const std::vector<colmag_ros::LifecycleTransition> sent =
      lifecycle->markSendSucceeded(generation, steady(5.0),
                                   colmag_ros::steadyDurationFromSeconds(3.0));
  assert(sent.size() == 1);
  assert(sent.front().state == colmag_ros::GoalLifecycleState::kGoalSent);
  assert(lifecycle->handleActive(generation, steady(5.1)).accepted());
  return generation;
}

std::uint64_t startFinalVerification(colmag_ros::GoalLifecycle* lifecycle,
                                     std::uint64_t baseline = 10)
{
  const std::uint64_t generation = startActiveGoal(lifecycle);
  colmag_ros::LifecycleTransition done = lifecycle->handleDone(
      generation, result(colmag_ros::GoalTerminalState::kSucceeded, 0, baseline));
  assert(done.accepted());
  assert(done.state == colmag_ros::GoalLifecycleState::kFinalStateVerifying);
  return generation;
}

}  // namespace

int main()
{
  const std::string process_a_session = colmag_ros::newAdapterSessionId();
  const std::string process_b_session = colmag_ros::newAdapterSessionId();
  assert(!process_a_session.empty());
  assert(!process_b_session.empty());
  assert(process_a_session != process_b_session);

  colmag_ros::CommandReplayGuard guard(2.0, 0.1, 2, 1, "session-current");
  colmag_ros::CommandMetadata empty;
  colmag_ros::CommandGuardDecision dry_run = guard.evaluate(empty, 10.0, false);
  assert(dry_run.allowed && dry_run.compatibility_warning);
  assert(!guard.evaluate(empty, 10.0, true).allowed);

  assert(guard.evaluate(metadata("new", 10.01), 10.02, true).allowed);
  assert(guard.evaluate(metadata("new", 10.02), 10.03, true).reason == "duplicate_command_id");
  assert(guard.evaluate(metadata("rollback", 10.01), 10.01, true).reason ==
         "command_ros_time_moved_backwards");
  assert(guard.evaluate(metadata("stale", 7.0), 10.02, true).reason == "stale_command");
  assert(guard.evaluate(metadata("future", 10.3), 10.02, true).reason == "future_command");
  assert(guard.evaluate(metadata("zero-now", 10.01), 0.0, true).reason ==
         "invalid_current_time");
  assert(guard.evaluate(metadata("old-stop", 9.99), 10.02, true, false).allowed);

  colmag_ros::CommandReplayGuard restarted_guard(2.0, 0.1, 2, 1, process_b_session);
  colmag_ros::OneShotMotionGate replay_one_shot(true, 1);
  assert(restarted_guard.evaluate(metadata("old-session", 10.01, process_a_session),
                                  10.02, true).reason ==
         "command_adapter_session_mismatch");
  assert(replay_one_shot.sendAttempts() == 0);
  assert(restarted_guard.evaluate(metadata("missing-session", 10.01, ""),
                                  10.02, true).reason ==
         "command_adapter_session_mismatch");
  assert(restarted_guard.evaluate(metadata("current-session", 10.01, process_b_session),
                                  10.02, true).allowed);
  colmag_ros::CommandReplayGuard rollback_replay_guard(
      2.0, 0.1, 2, 1, process_b_session);
  assert(rollback_replay_guard.evaluate(
      metadata("old-latched-rollback", 100.0, process_a_session), 99.96, true).reason ==
         "command_adapter_session_mismatch");
  assert(rollback_replay_guard.evaluate(
      metadata("old-stop", 100.0, process_a_session), 99.96, true, false).reason ==
         "command_adapter_session_mismatch");
  colmag_ros::CommandReplayGuard dry_session_guard(2.0, 0.1, 2, 1, process_b_session);
  const colmag_ros::CommandGuardDecision legacy_dry = dry_session_guard.evaluate(
      metadata("legacy-dry", 10.0, ""), 10.01, false);
  assert(legacy_dry.allowed && legacy_dry.compatibility_warning);
  colmag_ros::GoalLifecycle restarted_stop;
  assert(restarted_stop.requestCancel(0, steady(1.0)).disposition ==
         colmag_ros::TransitionDisposition::kInvalidTransition);
  assert(restarted_stop.snapshot().goal_generation == 0);
  colmag_ros::CommandMetadata unsupported = metadata("bad-schema", 10.01);
  unsupported.schema_version = 2;
  assert(guard.evaluate(unsupported, 10.02, true).reason == "unsupported_schema_version");

  colmag_ros::CommandReplayGuard dry_guard(2.0, 0.1, 2, 1, "session-current");
  assert(dry_guard.evaluate(empty, 0.0, false).allowed);
  assert(dry_guard.evaluate(metadata("zero-now", 0.01), 0.0, true).reason ==
         "invalid_current_time");

  colmag_ros::CommandReplayGuard busy_guard(2.0, 0.1, 2, 1, "session-current");
  colmag_ros::GoalLifecycle busy_lifecycle;
  const std::uint64_t busy_generation = startActiveGoal(&busy_lifecycle, "active-command");
  const colmag_ros::CommandMetadata busy_command =
      metadata("busy-command", 10.01, "session-current");
  assert(busy_guard.evaluate(busy_command, 10.02, true).allowed);
  assert(busy_lifecycle.beginGoal(identity("busy-command")).reason == "goal_lifecycle_busy");
  assert(busy_lifecycle.handleDone(
      busy_generation, result(colmag_ros::GoalTerminalState::kSucceeded, 0, 1)).accepted());
  assert(busy_lifecycle.verifyFinalState(
      busy_generation, finalSample(busy_generation, 2), finalPolicy()).state ==
         colmag_ros::GoalLifecycleState::kFinalStateVerified);
  assert(busy_guard.evaluate(busy_command, 10.03, true).reason == "duplicate_command_id");
  assert(busy_lifecycle.snapshot().goal_generation == busy_generation);

  colmag_ros::OneShotMotionGate one_shot(true, 1);
  std::string reason;
  assert(one_shot.canSend(&reason));
  one_shot.recordSendAttempt();
  assert(!one_shot.canSend(&reason));
  assert(reason == "one_shot_motion_limit_reached");
  assert(one_shot.sendAttempts() == 1);

  colmag_ros::OneShotMotionGate bounded(true, 3);
  for (int attempt = 0; attempt < 3; ++attempt)
  {
    assert(bounded.canSend(&reason));
    bounded.recordSendAttempt();
  }
  assert(!bounded.canSend(&reason));
  assert(bounded.sendAttempts() == 3);

  colmag_ros::OneShotMotionGate unlimited(true, 0);
  for (int attempt = 0; attempt < 5; ++attempt)
  {
    assert(unlimited.canSend(&reason));
    unlimited.recordSendAttempt();
  }
  assert(unlimited.canSend(&reason));
  assert(unlimited.sendAttempts() == 5);

  colmag_ros::OneShotMotionGate failed_send_quota(true, 0);
  failed_send_quota.recordSendAttempt();
  assert(failed_send_quota.sendAttempts() == 1);

  colmag_ros::RosTimeRollbackGuard joint_ros_time;
  assert(joint_ros_time.evaluate(10.0, "joint_state_ros_time_moved_backwards", &reason));
  assert(joint_ros_time.evaluate(10.01, "joint_state_ros_time_moved_backwards", &reason));
  assert(!joint_ros_time.evaluate(10.005, "joint_state_ros_time_moved_backwards", &reason));
  assert(reason == "joint_state_ros_time_moved_backwards");

  for (const std::pair<std::string, double>& limit : {
           std::make_pair(std::string("action_server_wait_sec"), 5.0),
           std::make_pair(std::string("execution_timeout_sec"), 30.0),
           std::make_pair(std::string("final_state_timeout_sec"), 5.0),
           std::make_pair(std::string("trajectory_start_delay_sec"), 2.0),
           std::make_pair(std::string("joint_state_max_age_sec"), 2.0),
           std::make_pair(std::string("command_max_age_sec"), 10.0),
           std::make_pair(std::string("watchdog_period_sec"), 1.0)})
  {
    assert(colmag_ros::validateRuntimeDuration(limit.first, limit.second, &reason));
    assert(!colmag_ros::validateRuntimeDuration(limit.first, 0.0, &reason));
    assert(!colmag_ros::validateRuntimeDuration(limit.first, -1.0, &reason));
    assert(!colmag_ros::validateRuntimeDuration(
        limit.first, std::numeric_limits<double>::quiet_NaN(), &reason));
    assert(!colmag_ros::validateRuntimeDuration(
        limit.first, std::numeric_limits<double>::infinity(), &reason));
    assert(!colmag_ros::validateRuntimeDuration(limit.first, limit.second + 0.001, &reason));
    assert(!colmag_ros::validateRuntimeDuration(limit.first, 1.0e100, &reason));
  }
  for (const std::pair<std::string, double>& default_value : {
           std::make_pair(std::string("action_server_wait_sec"), 0.5),
           std::make_pair(std::string("execution_timeout_sec"), 6.0),
           std::make_pair(std::string("final_state_timeout_sec"), 1.0),
           std::make_pair(std::string("trajectory_start_delay_sec"), 0.2),
           std::make_pair(std::string("joint_state_max_age_sec"), 0.5),
           std::make_pair(std::string("command_max_age_sec"), 2.0),
           std::make_pair(std::string("watchdog_period_sec"), 0.05)})
  {
    assert(colmag_ros::validateRuntimeDuration(
        default_value.first, default_value.second, &reason));
  }

  // A callback that arrives before sendGoal() returns is deferred. GOAL_SENT is
  // always the first publishable transport state.
  colmag_ros::GoalLifecycle ordered;
  const std::uint64_t ordered_generation = ordered.beginGoal(identity("ordered")).goal_generation;
  assert(ordered.beginSend(ordered_generation).accepted());
  assert(ordered.handleActive(ordered_generation, steady(4.9)).disposition ==
         colmag_ros::TransitionDisposition::kDeferred);
  assert(ordered.handleDone(ordered_generation,
                            result(colmag_ros::GoalTerminalState::kSucceeded, 0, 4)).disposition ==
         colmag_ros::TransitionDisposition::kDeferred);
  const std::vector<colmag_ros::LifecycleTransition> ordered_transitions =
      ordered.markSendSucceeded(ordered_generation, steady(5.0),
                                colmag_ros::steadyDurationFromSeconds(3.0));
  assert(ordered_transitions.size() == 3);
  assert(ordered_transitions[0].state == colmag_ros::GoalLifecycleState::kGoalSent);
  assert(ordered_transitions[1].state == colmag_ros::GoalLifecycleState::kGoalActive);
  assert(ordered_transitions[2].state == colmag_ros::GoalLifecycleState::kFinalStateVerifying);
  assert(ordered_transitions[0].event_sequence < ordered_transitions[1].event_sequence);
  assert(ordered_transitions[1].event_sequence < ordered_transitions[2].event_sequence);

  // timeout wins exactly once; every late transport event is ignored.
  colmag_ros::GoalLifecycle timeout;
  const std::uint64_t timeout_generation = startActiveGoal(&timeout, "timeout");
  colmag_ros::LifecycleTransition timed_out = timeout.handleTimeout(timeout_generation, steady(8.1));
  assert(timed_out.accepted() && timed_out.cancel_owned_goal);
  assert(timed_out.state == colmag_ros::GoalLifecycleState::kGoalTimeout);
  assert(timed_out.terminal);
  assert(timeout.handleDone(timeout_generation,
                            result(colmag_ros::GoalTerminalState::kSucceeded, 0, 7)).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
  assert(timeout.handleDone(timeout_generation,
                            result(colmag_ros::GoalTerminalState::kPreempted, 0, 7)).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
  assert(timeout.handleDone(timeout_generation,
                            result(colmag_ros::GoalTerminalState::kAborted, -4, 7)).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
  assert(timeout.handleActive(timeout_generation, steady(8.2)).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
  assert(timeout.snapshot().state == colmag_ros::GoalLifecycleState::kGoalTimeout);
  assert(timeout.snapshot().terminal_transition_count == 1);
  assert(timeout.snapshot().final_verification_start_count == 0);

  for (const colmag_ros::GoalTerminalState terminal_state : {
           colmag_ros::GoalTerminalState::kSucceeded,
           colmag_ros::GoalTerminalState::kPreempted,
           colmag_ros::GoalTerminalState::kAborted})
  {
    colmag_ros::GoalLifecycle late_done;
    const std::uint64_t generation = startActiveGoal(&late_done, "late-done");
    const colmag_ros::LifecycleTransition at_deadline = late_done.handleDone(
        generation, result(terminal_state, terminal_state == colmag_ros::GoalTerminalState::kAborted ? -4 : 0,
                           3, 8.0));
    assert(at_deadline.accepted());
    assert(at_deadline.state == colmag_ros::GoalLifecycleState::kGoalTimeout);
    assert(at_deadline.cancel_owned_goal);
    assert(late_done.snapshot().terminal_transition_count == 1);
    assert(late_done.snapshot().final_verification_start_count == 0);
  }

  colmag_ros::GoalLifecycle before_deadline;
  const std::uint64_t before_generation = startActiveGoal(&before_deadline, "before-deadline");
  assert(before_deadline.handleDone(
      before_generation,
      result(colmag_ros::GoalTerminalState::kSucceeded, 0, 3, 7.999)).state ==
         colmag_ros::GoalLifecycleState::kFinalStateVerifying);

  // If success wins before timeout, final verification remains the one outcome.
  colmag_ros::GoalLifecycle success_first;
  const std::uint64_t success_first_generation = startActiveGoal(&success_first, "success-first");
  assert(success_first.handleDone(
      success_first_generation,
      result(colmag_ros::GoalTerminalState::kSucceeded, 0, 3)).accepted());
  assert(success_first.handleTimeout(success_first_generation, steady(6.5)).disposition ==
         colmag_ros::TransitionDisposition::kInvalidTransition);

  for (int iteration = 0; iteration < 100; ++iteration)
  {
    colmag_ros::GoalLifecycle concurrent;
    const std::uint64_t concurrent_generation = startActiveGoal(&concurrent, "concurrent");
    std::atomic<bool> go(false);
    colmag_ros::LifecycleTransition timeout_race;
    colmag_ros::LifecycleTransition done_race;
    std::thread timeout_thread([&]() {
      while (!go.load()) {}
      timeout_race = concurrent.handleTimeout(concurrent_generation, steady(8.1));
    });
    std::thread done_thread([&]() {
      while (!go.load()) {}
      done_race = concurrent.handleDone(
          concurrent_generation,
          result(colmag_ros::GoalTerminalState::kSucceeded, 0, 3));
    });
    go.store(true);
    timeout_thread.join();
    done_thread.join();
    const colmag_ros::LifecycleSnapshot race_snapshot = concurrent.snapshot();
    assert(race_snapshot.state == colmag_ros::GoalLifecycleState::kGoalTimeout ||
           race_snapshot.state == colmag_ros::GoalLifecycleState::kFinalStateVerifyFailed);
    assert(race_snapshot.terminal_transition_count == 1);
    assert(race_snapshot.final_verification_start_count <= 1);
  }

  // The event's receive time, not scheduler callback order, is authoritative.
  colmag_ros::GoalLifecycle delayed_scheduler;
  const std::uint64_t delayed_generation = startActiveGoal(&delayed_scheduler, "delayed-scheduler");
  assert(delayed_scheduler.handleDone(
      delayed_generation,
      result(colmag_ros::GoalTerminalState::kSucceeded, 0, 3, 8.1)).state ==
         colmag_ros::GoalLifecycleState::kGoalTimeout);
  assert(delayed_scheduler.handleTimeout(delayed_generation, steady(8.2)).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);

  // Cancel owns only the current generation. Late active cannot overwrite it,
  // and PREEMPTED produces one software-cancel terminal.
  colmag_ros::GoalLifecycle cancel;
  const std::uint64_t cancel_generation = startActiveGoal(&cancel, "cancel");
  colmag_ros::LifecycleTransition cancel_requested = cancel.requestCancel(cancel_generation, steady(6.0));
  assert(cancel_requested.accepted() && cancel_requested.cancel_owned_goal);
  assert(cancel.handleActive(cancel_generation, steady(6.1)).disposition ==
         colmag_ros::TransitionDisposition::kInvalidTransition);
  colmag_ros::LifecycleTransition cancel_done = cancel.handleDone(
      cancel_generation, result(colmag_ros::GoalTerminalState::kPreempted, 0, 2));
  assert(cancel_done.accepted());
  assert(cancel_done.state == colmag_ros::GoalLifecycleState::kSoftwareCancelComplete);
  assert(cancel_done.terminal);
  assert(cancel.snapshot().terminal_transition_count == 1);

  // STOP while final verification is pending is a no-op and cannot clear the
  // generation, target, deadline, or verification state.
  colmag_ros::GoalLifecycle verifying_stop;
  const std::uint64_t verifying_generation = startFinalVerification(&verifying_stop, 10);
  const colmag_ros::LifecycleSnapshot before_stop = verifying_stop.snapshot();
  const colmag_ros::LifecycleTransition stop = verifying_stop.requestCancel(
      verifying_generation, steady(6.1));
  assert(stop.disposition == colmag_ros::TransitionDisposition::kInvalidTransition);
  const colmag_ros::LifecycleSnapshot after_stop = verifying_stop.snapshot();
  assert(after_stop.goal_generation == verifying_generation);
  assert(after_stop.state == colmag_ros::GoalLifecycleState::kFinalStateVerifying);
  assert(after_stop.final_state_deadline == before_stop.final_state_deadline);

  // Only a fully verified success can start the next generation. Per-goal
  // deadlines and verification state reset while event ordering stays global.
  colmag_ros::GoalLifecycle generations;
  const std::uint64_t generation_one = startActiveGoal(&generations, "generation-one");
  assert(generations.handleDone(
      generation_one, result(colmag_ros::GoalTerminalState::kSucceeded, 0, 1)).accepted());
  const colmag_ros::LifecycleTransition verified_one = generations.verifyFinalState(
      generation_one, finalSample(generation_one, 2), finalPolicy());
  assert(verified_one.state == colmag_ros::GoalLifecycleState::kFinalStateVerified);
  const std::uint64_t first_terminal_event = verified_one.event_sequence;

  const colmag_ros::LifecycleTransition prepared_two =
      generations.beginGoal(identity("generation-two"));
  assert(prepared_two.accepted());
  const std::uint64_t generation_two = prepared_two.goal_generation;
  assert(generation_two > generation_one);
  assert(prepared_two.event_sequence > first_terminal_event);
  const colmag_ros::LifecycleSnapshot reset = generations.snapshot();
  assert(!reset.terminal);
  assert(!reset.owned_active_goal);
  assert(!reset.awaiting_final_verification);
  assert(reset.execution_deadline == colmag_ros::SteadyTimePoint());
  assert(reset.final_state_deadline == colmag_ros::SteadyTimePoint());
  assert(std::isnan(reset.final_joint_error));
  assert(generations.beginSend(generation_two).accepted());
  assert(generations.markSendSucceeded(
      generation_two, steady(5.0), colmag_ros::steadyDurationFromSeconds(3.0)).front().accepted());
  assert(generations.handleActive(generation_two, steady(5.1)).accepted());
  assert(generations.handleDone(
      generation_one, result(colmag_ros::GoalTerminalState::kSucceeded, 0, 1)).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredStaleGeneration);
  assert(generations.requestCancel(generation_one, steady(6.0)).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredStaleGeneration);
  assert(generations.snapshot().goal_generation == generation_two);
  assert(generations.snapshot().state == colmag_ros::GoalLifecycleState::kGoalActive);

  assert(generations.handleDone(
      generation_two, result(colmag_ros::GoalTerminalState::kSucceeded, 0, 2)).accepted());
  assert(generations.verifyFinalState(
      generation_two, finalSample(generation_two, 3), finalPolicy()).state ==
         colmag_ros::GoalLifecycleState::kFinalStateVerified);
  const colmag_ros::LifecycleTransition prepared_three =
      generations.beginGoal(identity("generation-three"));
  assert(prepared_three.accepted());
  assert(prepared_three.goal_generation > generation_two);
  assert(prepared_three.event_sequence > prepared_two.event_sequence);
  assert(generations.handleJointStateFailure(
      generation_two, "missing_joint_name:fr3_joint7").disposition ==
         colmag_ros::TransitionDisposition::kIgnoredStaleGeneration);
  assert(generations.snapshot().goal_generation == prepared_three.goal_generation);
  assert(generations.snapshot().state == colmag_ros::GoalLifecycleState::kGoalPrepared);

  // Duplicate success cannot begin final verification twice.
  colmag_ros::GoalLifecycle duplicate_success;
  const std::uint64_t duplicate_generation = startFinalVerification(&duplicate_success, 5);
  assert(duplicate_success.handleDone(
      duplicate_generation, result(colmag_ros::GoalTerminalState::kSucceeded, 0, 5)).disposition ==
         colmag_ros::TransitionDisposition::kInvalidTransition);
  assert(duplicate_success.snapshot().final_verification_start_count == 1);

  // A transport send failure is a unique terminal and owns no action goal.
  colmag_ros::GoalLifecycle send_failure;
  const std::uint64_t failed_generation = send_failure.beginGoal(identity("send-failure")).goal_generation;
  assert(send_failure.beginSend(failed_generation).accepted());
  colmag_ros::LifecycleTransition failed = send_failure.markSendFailed(failed_generation, -999);
  assert(failed.accepted() && failed.terminal);
  assert(failed.state == colmag_ros::GoalLifecycleState::kSendFailed);
  assert(!send_failure.snapshot().owned_active_goal);
  assert(send_failure.snapshot().terminal_transition_count == 1);

  const auto assert_restart_required = [](colmag_ros::GoalLifecycle* lifecycle) {
    const colmag_ros::LifecycleTransition retry =
        lifecycle->beginGoal(identity("must-not-rearm"));
    assert(!retry.accepted());
    assert(retry.reason == "goal_lifecycle_requires_restart");
  };
  assert_restart_required(&send_failure);

  colmag_ros::GoalLifecycle missing_joint_after_send;
  const std::uint64_t missing_joint_after_send_generation =
      startActiveGoal(&missing_joint_after_send, "missing-joint-after-send");
  const colmag_ros::LifecycleTransition missing_joint_after_send_failure =
      missing_joint_after_send.handleJointStateFailure(
          missing_joint_after_send_generation, "missing_joint_name:fr3_joint7");
  assert(missing_joint_after_send_failure.accepted());
  assert(missing_joint_after_send_failure.terminal);
  assert(missing_joint_after_send_failure.state ==
         colmag_ros::GoalLifecycleState::kJointStateFailed);
  assert(missing_joint_after_send_failure.reason ==
         "post_send_joint_state_failure:missing_joint_name:fr3_joint7");
  assert(missing_joint_after_send_failure.cancel_owned_goal);
  assert_restart_required(&missing_joint_after_send);
  const colmag_ros::LifecycleSnapshot missing_joint_terminal =
      missing_joint_after_send.snapshot();
  assert(missing_joint_terminal.terminal_transition_count == 1);
  assert(missing_joint_after_send.verifyFinalState(
      missing_joint_after_send_generation,
      finalSample(missing_joint_after_send_generation, 2), finalPolicy()).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
  assert(missing_joint_after_send.snapshot().state ==
         colmag_ros::GoalLifecycleState::kJointStateFailed);
  const colmag_ros::LifecycleTransition repeated_joint_failure =
      missing_joint_after_send.handleJointStateFailure(
          missing_joint_after_send_generation, "non_finite_joint_position");
  assert(repeated_joint_failure.disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
  assert(repeated_joint_failure.event_sequence == missing_joint_terminal.event_sequence);
  assert(repeated_joint_failure.state == colmag_ros::GoalLifecycleState::kJointStateFailed);
  assert(missing_joint_after_send.handleActive(
      missing_joint_after_send_generation + 1, steady(6.0)).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredStaleGeneration);

  colmag_ros::OneShotMotionGate unlimited_after_joint_failure(true, 0);
  unlimited_after_joint_failure.recordSendAttempt();
  assert(unlimited_after_joint_failure.canSend(&reason));
  assert(unlimited_after_joint_failure.sendAttempts() == 1);
  assert_restart_required(&missing_joint_after_send);

  colmag_ros::GoalLifecycle missing_position_after_send;
  const std::uint64_t missing_position_generation =
      startFinalVerification(&missing_position_after_send, 1);
  const colmag_ros::LifecycleTransition missing_position_failure =
      missing_position_after_send.handleJointStateFailure(
          missing_position_generation, "missing_joint_position:fr3_joint7");
  assert(missing_position_failure.accepted() && missing_position_failure.terminal);
  assert(missing_position_failure.state == colmag_ros::GoalLifecycleState::kJointStateFailed);
  assert(!missing_position_failure.cancel_owned_goal);
  assert_restart_required(&missing_position_after_send);

  colmag_ros::GoalLifecycle non_finite_position_after_send;
  const std::uint64_t non_finite_position_generation =
      startActiveGoal(&non_finite_position_after_send, "non-finite-position-after-send");
  const colmag_ros::LifecycleTransition non_finite_position_failure =
      non_finite_position_after_send.handleJointStateFailure(
          non_finite_position_generation, "non_finite_joint_position");
  assert(non_finite_position_failure.accepted() && non_finite_position_failure.terminal);
  assert(non_finite_position_failure.state == colmag_ros::GoalLifecycleState::kJointStateFailed);
  assert(non_finite_position_failure.cancel_owned_goal);
  assert_restart_required(&non_finite_position_after_send);

  colmag_ros::GoalLifecycle pre_send_joint_state;
  const colmag_ros::LifecycleTransition initial_joint_failure =
      pre_send_joint_state.handleJointStateFailure(0, "missing_joint_name:fr3_joint7");
  assert(initial_joint_failure.disposition ==
         colmag_ros::TransitionDisposition::kInvalidTransition);
  assert(initial_joint_failure.reason == "joint_state_failure_before_motion_ownership");
  const colmag_ros::LifecycleTransition pre_send_prepared =
      pre_send_joint_state.beginGoal(identity("pre-send-joint-state"));
  assert(pre_send_prepared.accepted());
  const colmag_ros::LifecycleTransition prepared_joint_failure =
      pre_send_joint_state.handleJointStateFailure(
          pre_send_prepared.goal_generation, "missing_joint_position:fr3_joint7");
  assert(prepared_joint_failure.disposition ==
         colmag_ros::TransitionDisposition::kInvalidTransition);
  assert(!pre_send_joint_state.snapshot().terminal);
  assert(pre_send_joint_state.beginSend(pre_send_prepared.goal_generation).accepted());

  colmag_ros::GoalLifecycle rejected;
  const std::uint64_t rejected_generation = startActiveGoal(&rejected, "rejected");
  assert(rejected.handleDone(
      rejected_generation, result(colmag_ros::GoalTerminalState::kRejected, -1, 1)).state ==
         colmag_ros::GoalLifecycleState::kGoalRejected);
  assert_restart_required(&rejected);

  colmag_ros::GoalLifecycle aborted;
  const std::uint64_t aborted_generation = startActiveGoal(&aborted, "aborted");
  assert(aborted.handleDone(
      aborted_generation, result(colmag_ros::GoalTerminalState::kAborted, -4, 1)).state ==
         colmag_ros::GoalLifecycleState::kGoalAborted);
  assert_restart_required(&aborted);

  colmag_ros::GoalLifecycle unexpected_preempt;
  const std::uint64_t preempted_generation =
      startActiveGoal(&unexpected_preempt, "unexpected-preempt");
  assert(unexpected_preempt.handleDone(
      preempted_generation, result(colmag_ros::GoalTerminalState::kPreempted, 0, 1)).state ==
         colmag_ros::GoalLifecycleState::kGoalPreempted);
  assert_restart_required(&unexpected_preempt);

  assert_restart_required(&timeout);
  assert_restart_required(&cancel);

  // Final verification requires a post-result process receive sequence. Header
  // time alone can never prove freshness.
  colmag_ros::GoalLifecycle freshness;
  const std::uint64_t freshness_generation = startFinalVerification(&freshness, 10);
  assert(freshness.verifyFinalState(
      freshness_generation, finalSample(freshness_generation, 10, 10.2), finalPolicy()).disposition ==
         colmag_ros::TransitionDisposition::kInvalidTransition);
  colmag_ros::LifecycleTransition fresh = freshness.verifyFinalState(
      freshness_generation, finalSample(freshness_generation, 11), finalPolicy());
  assert(fresh.accepted() && fresh.terminal);
  assert(fresh.state == colmag_ros::GoalLifecycleState::kFinalStateVerified);

  // A closed multi-joint primitive reuses the same lifecycle owner while
  // requiring every configured final position to return to its start value.
  colmag_ros::GoalIdentity hexagon_identity = identity("closed-hexagon");
  hexagon_identity.final_joint_names = finalPolicy().configured_joint_names;
  hexagon_identity.final_joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.6, -0.7};
  colmag_ros::GoalLifecycle hexagon_bad_final;
  colmag_ros::LifecycleTransition prepared = hexagon_bad_final.beginGoal(hexagon_identity);
  assert(prepared.accepted());
  assert(hexagon_bad_final.beginSend(prepared.goal_generation).accepted());
  assert(hexagon_bad_final.markSendSucceeded(
      prepared.goal_generation, steady(5.0),
      colmag_ros::steadyDurationFromSeconds(8.0)).front().accepted());
  assert(hexagon_bad_final.handleActive(prepared.goal_generation, steady(5.1)).accepted());
  assert(hexagon_bad_final.handleDone(
      prepared.goal_generation,
      result(colmag_ros::GoalTerminalState::kSucceeded, 0, 10)).accepted());
  colmag_ros::FinalStateSample wrong_hexagon = finalSample(prepared.goal_generation, 11);
  wrong_hexagon.positions = hexagon_identity.final_joint_positions;
  wrong_hexagon.positions[5] += 0.02;
  const colmag_ros::LifecycleTransition wrong_hexagon_result =
      hexagon_bad_final.verifyFinalState(
          prepared.goal_generation, wrong_hexagon, finalPolicy());
  assert(wrong_hexagon_result.state ==
         colmag_ros::GoalLifecycleState::kFinalStateVerifyFailed);
  assert(wrong_hexagon_result.reason == "final_joint_error_above_tolerance");
  assert_restart_required(&hexagon_bad_final);

  colmag_ros::GoalLifecycle hexagon_good_final;
  prepared = hexagon_good_final.beginGoal(hexagon_identity);
  assert(hexagon_good_final.beginSend(prepared.goal_generation).accepted());
  assert(hexagon_good_final.markSendSucceeded(
      prepared.goal_generation, steady(5.0),
      colmag_ros::steadyDurationFromSeconds(8.0)).front().accepted());
  assert(hexagon_good_final.handleActive(prepared.goal_generation, steady(5.1)).accepted());
  assert(hexagon_good_final.handleDone(
      prepared.goal_generation,
      result(colmag_ros::GoalTerminalState::kSucceeded, 0, 10)).accepted());
  colmag_ros::FinalStateSample closed_hexagon = finalSample(prepared.goal_generation, 11);
  closed_hexagon.positions = hexagon_identity.final_joint_positions;
  const colmag_ros::LifecycleTransition closed_hexagon_result =
      hexagon_good_final.verifyFinalState(
          prepared.goal_generation, closed_hexagon, finalPolicy());
  assert(closed_hexagon_result.state == colmag_ros::GoalLifecycleState::kFinalStateVerified);
  assert(hexagon_good_final.snapshot().final_joint_error == 0.0);

  colmag_ros::GoalLifecycle final_before_deadline;
  const std::uint64_t final_before_generation = startFinalVerification(&final_before_deadline, 10);
  assert(final_before_deadline.verifyFinalState(
      final_before_generation,
      finalSample(final_before_generation, 11, 10.1, 6.999),
      finalPolicy()).state == colmag_ros::GoalLifecycleState::kFinalStateVerified);

  for (const double receive_time : {7.0, 7.1})
  {
    colmag_ros::GoalLifecycle expired_final;
    const std::uint64_t generation = startFinalVerification(&expired_final, 10);
    const colmag_ros::LifecycleTransition expired = expired_final.verifyFinalState(
        generation, finalSample(generation, 11, 10.1, receive_time), finalPolicy());
    assert(expired.accepted());
    assert(expired.state == colmag_ros::GoalLifecycleState::kFinalStateVerifyFailed);
    assert(expired.reason == "final_state_timeout");
    assert(expired_final.snapshot().terminal_transition_count == 1);
  }

  for (int iteration = 0; iteration < 100; ++iteration)
  {
    colmag_ros::GoalLifecycle concurrent_final;
    const std::uint64_t generation = startFinalVerification(&concurrent_final, 10);
    std::atomic<bool> go(false);
    colmag_ros::LifecycleTransition sample_race;
    colmag_ros::LifecycleTransition scheduler_race;
    std::thread sample_thread([&]() {
      while (!go.load()) {}
      sample_race = concurrent_final.verifyFinalState(
          generation, finalSample(generation, 11, 10.1, 7.0), finalPolicy());
    });
    std::thread scheduler_thread([&]() {
      while (!go.load()) {}
      scheduler_race = concurrent_final.handleFinalStateTimeout(generation, steady(7.0));
    });
    go.store(true);
    sample_thread.join();
    scheduler_thread.join();
    assert(static_cast<int>(sample_race.accepted()) +
           static_cast<int>(scheduler_race.accepted()) == 1);
    assert(concurrent_final.snapshot().state ==
           colmag_ros::GoalLifecycleState::kFinalStateVerifyFailed);
    assert(concurrent_final.snapshot().terminal_transition_count == 1);
  }

  colmag_ros::GoalLifecycle future_sample;
  const std::uint64_t future_generation = startFinalVerification(&future_sample, 1);
  assert(future_sample.verifyFinalState(
      future_generation, finalSample(future_generation, 2, 10.3), finalPolicy()).state ==
         colmag_ros::GoalLifecycleState::kFinalStateVerifyFailed);

  colmag_ros::GoalLifecycle stale_sample;
  const std::uint64_t stale_generation = startFinalVerification(&stale_sample, 1);
  assert(stale_sample.verifyFinalState(
      stale_generation, finalSample(stale_generation, 2, 9.0), finalPolicy()).state ==
         colmag_ros::GoalLifecycleState::kFinalStateVerifyFailed);

  colmag_ros::GoalLifecycle zero_stamp;
  const std::uint64_t zero_generation = startFinalVerification(&zero_stamp, 1);
  assert(zero_stamp.verifyFinalState(
      zero_generation, finalSample(zero_generation, 2, 0.0), finalPolicy()).state ==
         colmag_ros::GoalLifecycleState::kFinalStateVerifyFailed);

  colmag_ros::GoalLifecycle ros_time_rollback;
  const std::uint64_t rollback_generation = startFinalVerification(&ros_time_rollback, 1);
  colmag_ros::FinalStateSample rollback = finalSample(rollback_generation, 2);
  rollback.current_ros_time_sec = 9.8;
  assert(ros_time_rollback.verifyFinalState(
      rollback_generation, rollback, finalPolicy()).state ==
         colmag_ros::GoalLifecycleState::kFinalStateVerifyFailed);

  colmag_ros::GoalLifecycle small_ros_time_rollback;
  const std::uint64_t small_rollback_generation = startFinalVerification(
      &small_ros_time_rollback, 1);
  colmag_ros::FinalStateSample small_rollback = finalSample(small_rollback_generation, 2);
  small_rollback.current_ros_time_sec = 9.99;
  small_rollback.header_stamp_sec = 9.98;
  const colmag_ros::LifecycleTransition rollback_result =
      small_ros_time_rollback.verifyFinalState(
          small_rollback_generation, small_rollback, finalPolicy());
  assert(rollback_result.state == colmag_ros::GoalLifecycleState::kFinalStateVerifyFailed);
  assert(rollback_result.reason == "final_state_ros_time_moved_backwards");

  colmag_ros::GoalLifecycle wrong_generation;
  const std::uint64_t current_generation = startFinalVerification(&wrong_generation, 1);
  assert(wrong_generation.verifyFinalState(
      current_generation + 1, finalSample(current_generation + 1, 2), finalPolicy()).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredStaleGeneration);
  assert(wrong_generation.snapshot().state == colmag_ros::GoalLifecycleState::kFinalStateVerifying);

  colmag_ros::GoalLifecycle missing_joint;
  const std::uint64_t missing_generation = startFinalVerification(&missing_joint, 1);
  colmag_ros::FinalStateSample missing = finalSample(missing_generation, 2);
  missing.names.pop_back();
  missing.positions.pop_back();
  assert(missing_joint.verifyFinalState(missing_generation, missing, finalPolicy()).state ==
         colmag_ros::GoalLifecycleState::kFinalStateVerifyFailed);
  assert_restart_required(&missing_joint);

  colmag_ros::GoalLifecycle duplicate_joint;
  const std::uint64_t duplicate_joint_generation = startFinalVerification(&duplicate_joint, 1);
  colmag_ros::FinalStateSample duplicate = finalSample(duplicate_joint_generation, 2);
  duplicate.names[0] = "fr3_joint7";
  assert(duplicate_joint.verifyFinalState(
      duplicate_joint_generation, duplicate, finalPolicy()).state ==
         colmag_ros::GoalLifecycleState::kFinalStateVerifyFailed);
  assert_restart_required(&duplicate_joint);

  colmag_ros::GoalLifecycle final_timeout;
  const std::uint64_t final_timeout_generation = startFinalVerification(&final_timeout, 3);
  assert(final_timeout.handleFinalStateTimeout(final_timeout_generation, steady(7.1)).accepted());
  assert(final_timeout.verifyFinalState(
      final_timeout_generation, finalSample(final_timeout_generation, 4), finalPolicy()).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
  assert_restart_required(&final_timeout);

  // Shutdown is idempotent and owns at most one bounded cancel side effect.
  colmag_ros::GoalLifecycle shutdown_empty;
  assert(shutdown_empty.beginShutdown(steady(1.0)).accepted());
  assert(shutdown_empty.beginShutdown(steady(1.1)).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
  assert(!shutdown_empty.beginGoal(identity("after-shutdown")).accepted());

  colmag_ros::GoalLifecycle shutdown_active;
  const std::uint64_t shutdown_generation = startActiveGoal(&shutdown_active, "shutdown-active");
  colmag_ros::LifecycleTransition shutdown = shutdown_active.beginShutdown(steady(6.0));
  assert(shutdown.accepted() && shutdown.cancel_owned_goal);
  assert(shutdown.state == colmag_ros::GoalLifecycleState::kShutdownCancelled);
  const colmag_ros::LifecycleTransition repeated_shutdown = shutdown_active.beginShutdown(steady(6.1));
  assert(repeated_shutdown.disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
  assert(!repeated_shutdown.cancel_owned_goal);
  assert(shutdown_active.handleDone(
      shutdown_generation, result(colmag_ros::GoalTerminalState::kPreempted, 0, 1)).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
  assert(shutdown_active.handleActive(shutdown_generation, steady(6.2)).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
  assert(shutdown_active.handleTimeout(shutdown_generation, steady(9.0)).disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
  assert(shutdown_active.snapshot().terminal_transition_count == 1);

  for (int iteration = 0; iteration < 100; ++iteration)
  {
    colmag_ros::GoalLifecycle shutdown_race;
    const std::uint64_t generation = startActiveGoal(&shutdown_race, "shutdown-race");
    std::atomic<bool> go(false);
    colmag_ros::LifecycleTransition shutdown_transition;
    colmag_ros::LifecycleTransition done_transition;
    std::thread shutdown_thread([&]() {
      while (!go.load()) {}
      shutdown_transition = shutdown_race.beginShutdown(steady(6.0));
    });
    std::thread done_thread([&]() {
      while (!go.load()) {}
      done_transition = shutdown_race.handleDone(
          generation, result(colmag_ros::GoalTerminalState::kPreempted, 0, 1, 6.0));
    });
    go.store(true);
    shutdown_thread.join();
    done_thread.join();
    assert(shutdown_transition.accepted());
    assert(done_transition.accepted() ||
           done_transition.disposition ==
               colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
    assert(shutdown_race.snapshot().terminal_transition_count == 1);
    assert(shutdown_race.snapshot().shutting_down);
  }

  // Shutdown linearizes against active/timeout/final callbacks. Once its guard
  // wins, later callbacks are ignored; if a deadline terminal wins first,
  // shutdown preserves it and only records the irreversible guard.
  for (int iteration = 0; iteration < 100; ++iteration)
  {
    colmag_ros::GoalLifecycle shutdown_timeout_race;
    const std::uint64_t generation = startActiveGoal(
        &shutdown_timeout_race, "shutdown-timeout-race");
    std::atomic<bool> go(false);
    colmag_ros::LifecycleTransition shutdown_transition;
    colmag_ros::LifecycleTransition timeout_transition;
    std::thread shutdown_thread([&]() {
      while (!go.load()) {}
      shutdown_transition = shutdown_timeout_race.beginShutdown(steady(8.0));
    });
    std::thread timeout_thread([&]() {
      while (!go.load()) {}
      timeout_transition = shutdown_timeout_race.handleTimeout(generation, steady(8.0));
    });
    go.store(true);
    shutdown_thread.join();
    timeout_thread.join();
    assert(shutdown_transition.accepted());
    assert(timeout_transition.accepted() ||
           timeout_transition.disposition ==
               colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
    assert(shutdown_timeout_race.snapshot().state ==
           colmag_ros::GoalLifecycleState::kGoalTimeout);
    assert(shutdown_timeout_race.snapshot().terminal_transition_count == 1);
    assert(shutdown_timeout_race.snapshot().shutting_down);
  }

  for (int iteration = 0; iteration < 100; ++iteration)
  {
    colmag_ros::GoalLifecycle shutdown_final_race;
    const std::uint64_t generation = startFinalVerification(&shutdown_final_race, 10);
    std::atomic<bool> go(false);
    colmag_ros::LifecycleTransition shutdown_transition;
    colmag_ros::LifecycleTransition sample_transition;
    std::thread shutdown_thread([&]() {
      while (!go.load()) {}
      shutdown_transition = shutdown_final_race.beginShutdown(steady(6.1));
    });
    std::thread sample_thread([&]() {
      while (!go.load()) {}
      sample_transition = shutdown_final_race.verifyFinalState(
          generation, finalSample(generation, 11), finalPolicy());
    });
    go.store(true);
    shutdown_thread.join();
    sample_thread.join();
    assert(shutdown_transition.accepted());
    assert(sample_transition.accepted() ||
           sample_transition.disposition ==
               colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
    assert(shutdown_final_race.snapshot().terminal_transition_count == 1);
    assert(shutdown_final_race.snapshot().shutting_down);
  }

  colmag_ros::GoalLifecycle shutdown_during_send;
  const std::uint64_t shutdown_during_send_generation =
      shutdown_during_send.beginGoal(identity("shutdown-during-send")).goal_generation;
  assert(shutdown_during_send.beginSend(shutdown_during_send_generation).accepted());
  const colmag_ros::LifecycleTransition shutdown_sending =
      shutdown_during_send.beginShutdown(steady(5.0));
  assert(shutdown_sending.accepted());
  assert(!shutdown_sending.cancel_owned_goal);
  const std::vector<colmag_ros::LifecycleTransition> transport_returned =
      shutdown_during_send.markSendSucceeded(
          shutdown_during_send_generation, steady(5.1),
          colmag_ros::steadyDurationFromSeconds(3.0));
  assert(transport_returned.size() == 1);
  assert(transport_returned.front().cancel_owned_goal);
  assert(transport_returned.front().disposition ==
         colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
  assert(shutdown_during_send.snapshot().state ==
         colmag_ros::GoalLifecycleState::kShutdownCancelled);
  assert(shutdown_during_send.snapshot().terminal_transition_count == 1);

  // A send lease keeps the client alive while shutdown marks transport
  // closing. closeAndDetach waits with its mutex released, then transfers the
  // unique owner exactly once.
  {
    std::atomic<int> cancel_count(0);
    std::atomic<int> destruction_count(0);
    colmag_ros::OneShotMotionGate send_one_shot(true, 1);
    colmag_ros::GoalLifecycle send_lifecycle;
    const std::uint64_t generation =
        send_lifecycle.beginGoal(identity("transport-send-shutdown")).goal_generation;
    assert(send_lifecycle.beginSend(generation).accepted());
    send_one_shot.recordSendAttempt();
    colmag_ros::TransportClientOwner<FakeTransportClient> transport;
    transport.install(std::unique_ptr<FakeTransportClient>(
        new FakeTransportClient(&cancel_count, &destruction_count)));
    colmag_ros::TransportClientOwner<FakeTransportClient>::Lease send =
        transport.acquire();
    assert(send);
    assert(send_lifecycle.handleActive(generation, steady(4.9)).disposition ==
           colmag_ros::TransitionDisposition::kDeferred);
    const colmag_ros::LifecycleTransition shutdown_transition =
        send_lifecycle.beginShutdown(steady(5.0));
    assert(shutdown_transition.accepted());
    assert(!shutdown_transition.cancel_owned_goal);

    std::mutex state_mutex;
    bool detach_returned = false;
    std::unique_ptr<FakeTransportClient> detached;
    std::thread shutdown_thread([&]() {
      detached = transport.closeAndDetach();
      std::lock_guard<std::mutex> lock(state_mutex);
      detach_returned = true;
    });
    while (!transport.closing())
    {
      std::this_thread::yield();
    }
    assert(!transport.acquire());
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      assert(!detach_returned);
    }
    const std::vector<colmag_ros::LifecycleTransition> send_returned =
        send_lifecycle.markSendSucceeded(
            generation, steady(5.1), colmag_ros::steadyDurationFromSeconds(3.0));
    assert(send_returned.size() == 1);
    assert(send_returned.front().cancel_owned_goal);
    send->cancelGoal();
    send = colmag_ros::TransportClientOwner<FakeTransportClient>::Lease();
    shutdown_thread.join();
    assert(detached);
    assert(detach_returned);
    detached.reset();
    assert(destruction_count.load() == 1);
    assert(cancel_count.load() == 1);
    assert(send_lifecycle.snapshot().terminal_transition_count == 1);
    assert(send_one_shot.sendAttempts() == 1);
    std::string one_shot_reason;
    assert(!send_one_shot.canSend(&one_shot_reason));
    assert(!send_lifecycle.beginShutdown(steady(5.2)).cancel_owned_goal);
    assert(!transport.closeAndDetach());
  }

  // Teardown may join a callback thread only after the transport mutex is
  // released. The late callback observes closing, obtains/releases the gate
  // mutex, and cannot perform a second cancel or lifecycle transition.
  {
    std::atomic<int> cancel_count(0);
    std::atomic<int> destruction_count(0);
    std::atomic<bool> callback_saw_closed_transport(false);
    std::mutex callback_mutex;
    std::condition_variable callback_ready_condition;
    std::condition_variable callback_release_condition;
    bool callback_ready = false;
    bool callback_released = false;

    colmag_ros::GoalLifecycle teardown_lifecycle;
    const std::uint64_t generation = startActiveGoal(
        &teardown_lifecycle, "teardown-lock-order");
    colmag_ros::TransportClientOwner<FakeTransportClient> transport;
    transport.install(std::unique_ptr<FakeTransportClient>(
        new FakeTransportClient(&cancel_count, &destruction_count)));
    {
      colmag_ros::TransportClientOwner<FakeTransportClient>::Lease client =
          transport.acquire();
      assert(client);
      client->startCallback([&]() {
        {
          std::lock_guard<std::mutex> lock(callback_mutex);
          callback_ready = true;
        }
        callback_ready_condition.notify_one();
        {
          std::unique_lock<std::mutex> lock(callback_mutex);
          callback_release_condition.wait(lock, [&]() { return callback_released; });
        }
        assert(teardown_lifecycle.handleDone(
            generation,
            result(colmag_ros::GoalTerminalState::kPreempted, 0, 1)).disposition ==
               colmag_ros::TransitionDisposition::kIgnoredAlreadyTerminal);
        colmag_ros::TransportClientOwner<FakeTransportClient>::Lease late =
            transport.acquire();
        callback_saw_closed_transport.store(!late);
      });
    }
    {
      std::unique_lock<std::mutex> lock(callback_mutex);
      callback_ready_condition.wait(lock, [&]() { return callback_ready; });
    }

    const colmag_ros::LifecycleTransition shutdown_transition =
        teardown_lifecycle.beginShutdown(steady(6.0));
    assert(shutdown_transition.accepted());
    assert(shutdown_transition.cancel_owned_goal);
    std::unique_ptr<FakeTransportClient> detached = transport.closeAndDetach();
    assert(detached);
    detached->cancelGoal();
    {
      std::lock_guard<std::mutex> lock(callback_mutex);
      callback_released = true;
    }
    callback_release_condition.notify_one();
    detached.reset();

    assert(callback_saw_closed_transport.load());
    assert(cancel_count.load() == 1);
    assert(destruction_count.load() == 1);
    assert(teardown_lifecycle.snapshot().terminal_transition_count == 1);
    const colmag_ros::LifecycleTransition repeated =
        teardown_lifecycle.beginShutdown(steady(6.1));
    assert(!repeated.cancel_owned_goal);
    assert(!transport.closeAndDetach());
  }

  colmag_ros::GoalLifecycle status_lifecycle;
  const std::uint64_t status_generation = startActiveGoal(&status_lifecycle, "status-owner");
  const colmag_ros::LifecycleStatusProjection active_diagnostic =
      colmag_ros::projectLifecycleStatus(
          status_lifecycle.snapshot(), "REJECTED", "invalid_json", "malformed-command",
          "malformed-command", "diagnostic-source");
  assert(active_diagnostic.state == "GOAL_ACTIVE");
  assert(active_diagnostic.goal_generation == status_generation);
  assert(active_diagnostic.command_id == "status-owner");
  assert(active_diagnostic.diagnostic_code == "invalid_json");
  assert(active_diagnostic.diagnostic_command_id == "malformed-command");
  assert(status_lifecycle.handleDone(
      status_generation,
      result(colmag_ros::GoalTerminalState::kAborted, -4, 1)).accepted());
  const colmag_ros::LifecycleStatusProjection terminal_diagnostic =
      colmag_ros::projectLifecycleStatus(
          status_lifecycle.snapshot(), "REJECTED", "command_adapter_session_mismatch",
          "old-session-command", "old-session-command", "diagnostic-source");
  assert(terminal_diagnostic.state == "GOAL_ABORTED");
  assert(terminal_diagnostic.command_id == "status-owner");
  assert(terminal_diagnostic.terminal);

  assert(std::string(colmag_ros::goalLifecycleStateName(
             colmag_ros::GoalLifecycleState::kGoalRejected)) == "GOAL_REJECTED");
  return 0;
}
