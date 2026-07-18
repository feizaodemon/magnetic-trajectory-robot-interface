#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace colmag_ros
{

using SteadyClock = std::chrono::steady_clock;
using SteadyTimePoint = SteadyClock::time_point;
using SteadyDuration = SteadyClock::duration;

SteadyDuration steadyDurationFromSeconds(double seconds);
std::string newAdapterSessionId();
bool validateRuntimeDuration(const std::string& parameter,
                             double value_sec,
                             std::string* reason);

struct CommandMetadata
{
  std::string command_id;
  std::string target_adapter_session_id;
  bool has_issued_at = false;
  double issued_at_sec = 0.0;
  bool has_schema_version = false;
  int schema_version = 0;
};

struct CommandGuardDecision
{
  bool allowed = false;
  bool compatibility_warning = false;
  std::string reason;
};

class CommandReplayGuard
{
public:
  CommandReplayGuard(double max_age_sec,
                     double future_tolerance_sec,
                     std::size_t cache_size,
                     int supported_schema_version,
                     const std::string& adapter_session_id = "");

  CommandGuardDecision evaluate(const CommandMetadata& metadata,
                                double now_sec,
                                bool hardware_active,
                                bool motion_command = true);

private:
  void remember(const std::string& command_id);

  double max_age_sec_;
  double future_tolerance_sec_;
  std::size_t cache_size_;
  int supported_schema_version_;
  std::string adapter_session_id_;
  bool have_last_accepted_ros_time_ = false;
  double last_accepted_ros_time_sec_ = 0.0;
  std::deque<std::string> recent_ids_;
  std::unordered_set<std::string> recent_id_set_;
};

class OneShotMotionGate
{
public:
  OneShotMotionGate(bool enabled, std::size_t maximum_motion_goals);

  bool canSend(std::string* reason) const;
  void recordSendAttempt();
  std::size_t sendAttempts() const;

private:
  bool enabled_;
  std::size_t maximum_motion_goals_;
  std::size_t send_attempts_ = 0;
};

class RosTimeRollbackGuard
{
public:
  bool evaluate(double now_sec, const std::string& rollback_reason, std::string* reason);

private:
  std::mutex mutex_;
  bool have_last_time_ = false;
  double last_time_sec_ = 0.0;
};

template <typename Client>
class TransportClientOwner
{
public:
  class Lease
  {
  public:
    Lease() = default;
    Lease(const Lease&) = delete;
    Lease& operator=(const Lease&) = delete;
    Lease(Lease&& other) noexcept
      : owner_(other.owner_), client_(other.client_)
    {
      other.owner_ = nullptr;
      other.client_ = nullptr;
    }
    Lease& operator=(Lease&& other) noexcept
    {
      if (this != &other)
      {
        release();
        owner_ = other.owner_;
        client_ = other.client_;
        other.owner_ = nullptr;
        other.client_ = nullptr;
      }
      return *this;
    }
    ~Lease() { release(); }

    explicit operator bool() const { return client_ != nullptr; }
    Client* get() const { return client_; }
    Client* operator->() const { return client_; }

  private:
    friend class TransportClientOwner<Client>;
    Lease(TransportClientOwner<Client>* owner, Client* client)
      : owner_(owner), client_(client)
    {
    }
    void release()
    {
      if (owner_)
      {
        owner_->releaseOperation();
        owner_ = nullptr;
        client_ = nullptr;
      }
    }

    TransportClientOwner<Client>* owner_ = nullptr;
    Client* client_ = nullptr;
  };

  void install(std::unique_ptr<Client> client)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    client_ = std::move(client);
  }

  Lease acquire()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closing_ || !client_)
    {
      return Lease();
    }
    ++active_operations_;
    return Lease(this, client_.get());
  }

  std::unique_ptr<Client> closeAndDetach()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    closing_ = true;
    operations_finished_.wait(lock, [this]() { return active_operations_ == 0; });
    return std::move(client_);
  }

  bool closing() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return closing_;
  }

private:
  void releaseOperation()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    --active_operations_;
    if (closing_ && active_operations_ == 0)
    {
      operations_finished_.notify_all();
    }
  }

  mutable std::mutex mutex_;
  std::condition_variable operations_finished_;
  std::unique_ptr<Client> client_;
  std::size_t active_operations_ = 0;
  bool closing_ = false;
};

enum class GoalLifecycleState
{
  kReady,
  kGoalPrepared,
  kGoalSending,
  kGoalSent,
  kGoalActive,
  kFinalStateVerifying,
  kGoalRejected,
  kGoalAborted,
  kGoalPreempted,
  kGoalTimeout,
  kFinalStateVerified,
  kFinalStateVerifyFailed,
  kSoftwareCancelRequested,
  kSoftwareCancelComplete,
  kSendFailed,
  kJointStateFailed,
  kShutdownCancelled,
};

enum class GoalTerminalState
{
  kSucceeded,
  kRejected,
  kAborted,
  kPreempted,
};

enum class TransitionDisposition
{
  kAccepted,
  kDeferred,
  kIgnoredStaleGeneration,
  kIgnoredAlreadyTerminal,
  kInvalidTransition,
};

struct GoalIdentity
{
  std::string command_id;
  std::string source_id;
  std::string primitive;
  std::string target_joint;
  double target_position = 0.0;
  std::vector<std::string> final_joint_names;
  std::vector<double> final_joint_positions;
};

struct GoalResultEvent
{
  GoalTerminalState terminal_state = GoalTerminalState::kAborted;
  int result_error_code = 0;
  SteadyTimePoint result_receive_steady_time;
  double result_ros_time_sec = 0.0;
  SteadyDuration final_state_timeout;
  std::uint64_t joint_state_sequence_baseline = 0;
};

struct FinalStateSample
{
  std::uint64_t goal_generation = 0;
  std::uint64_t receive_sequence = 0;
  SteadyTimePoint receive_steady_time;
  double header_stamp_sec = 0.0;
  double current_ros_time_sec = 0.0;
  std::vector<std::string> names;
  std::vector<double> positions;
};

struct FinalStatePolicy
{
  std::vector<std::string> configured_joint_names;
  double max_age_sec = 0.0;
  double future_tolerance_sec = 0.0;
  double error_tolerance = 0.0;
};

struct LifecycleTransition
{
  TransitionDisposition disposition = TransitionDisposition::kInvalidTransition;
  std::uint64_t goal_generation = 0;
  std::uint64_t event_sequence = 0;
  GoalLifecycleState state = GoalLifecycleState::kReady;
  bool terminal = false;
  bool cancel_owned_goal = false;
  std::string reason;

  bool accepted() const { return disposition == TransitionDisposition::kAccepted; }
};

struct LifecycleSnapshot
{
  GoalLifecycleState state = GoalLifecycleState::kReady;
  std::uint64_t goal_generation = 0;
  std::uint64_t event_sequence = 0;
  GoalIdentity identity;
  bool terminal = false;
  bool owned_active_goal = false;
  bool awaiting_final_verification = false;
  bool shutting_down = false;
  SteadyTimePoint execution_deadline;
  SteadyTimePoint final_state_deadline;
  double final_joint_error = 0.0;
  int result_error_code = 0;
  std::size_t terminal_transition_count = 0;
  std::size_t final_verification_start_count = 0;
};

struct LifecycleStatusProjection
{
  std::string state;
  std::string command_id;
  std::string source_id;
  std::string diagnostic_code;
  std::string diagnostic_command_id;
  GoalLifecycleState lifecycle_state = GoalLifecycleState::kReady;
  std::uint64_t goal_generation = 0;
  std::uint64_t lifecycle_event_sequence = 0;
  bool terminal = false;
};

LifecycleStatusProjection projectLifecycleStatus(
    const LifecycleSnapshot& lifecycle,
    const std::string& requested_state,
    const std::string& diagnostic_code,
    const std::string& diagnostic_command_id,
    const std::string& fallback_command_id,
    const std::string& fallback_source_id);

const char* goalLifecycleStateName(GoalLifecycleState state);
const char* transitionDispositionName(TransitionDisposition disposition);

class GoalLifecycle
{
public:
  LifecycleTransition beginGoal(const GoalIdentity& identity);
  LifecycleTransition beginSend(std::uint64_t generation);
  std::vector<LifecycleTransition> markSendSucceeded(std::uint64_t generation,
                                                     SteadyTimePoint now,
                                                     SteadyDuration execution_timeout);
  LifecycleTransition markSendFailed(std::uint64_t generation, int error_code);
  LifecycleTransition handleJointStateFailure(std::uint64_t generation,
                                              const std::string& detail);
  LifecycleTransition handleActive(std::uint64_t generation, SteadyTimePoint now);
  LifecycleTransition handleDone(std::uint64_t generation,
                                 const GoalResultEvent& result);
  LifecycleTransition requestCancel(std::uint64_t generation, SteadyTimePoint now);
  LifecycleTransition handleTimeout(std::uint64_t generation, SteadyTimePoint now);
  LifecycleTransition verifyFinalState(std::uint64_t generation,
                                       const FinalStateSample& sample,
                                       const FinalStatePolicy& policy);
  LifecycleTransition handleFinalStateTimeout(std::uint64_t generation, SteadyTimePoint now);
  LifecycleTransition beginShutdown(SteadyTimePoint now);

  LifecycleSnapshot snapshot() const;

private:
  LifecycleTransition transitionLocked(TransitionDisposition disposition,
                                       const std::string& reason = "",
                                       bool cancel_owned_goal = false);
  LifecycleTransition acceptedTransitionLocked(const std::string& reason = "",
                                                bool cancel_owned_goal = false);
  LifecycleTransition terminalTransitionLocked(GoalLifecycleState state,
                                                const std::string& reason,
                                                bool cancel_owned_goal = false);
  LifecycleTransition applyActiveLocked(SteadyTimePoint now);
  LifecycleTransition applyDoneLocked(const GoalResultEvent& result);
  LifecycleTransition executionTimeoutIfReachedLocked(SteadyTimePoint now);
  LifecycleTransition finalTimeoutIfReachedLocked(SteadyTimePoint now);
  LifecycleTransition finalFailureLocked(const std::string& reason);
  LifecycleSnapshot snapshotLocked() const;

  mutable std::mutex mutex_;
  GoalLifecycleState state_ = GoalLifecycleState::kReady;
  std::uint64_t generation_ = 0;
  std::uint64_t event_sequence_ = 0;
  GoalIdentity identity_;
  bool terminal_ = false;
  bool owned_active_goal_ = false;
  bool awaiting_final_verification_ = false;
  bool transport_established_ = false;
  bool cancel_dispatched_ = false;
  bool done_received_ = false;
  bool deferred_active_ = false;
  bool deferred_done_ = false;
  bool shutting_down_ = false;
  SteadyTimePoint deferred_active_time_;
  GoalResultEvent deferred_result_;
  SteadyTimePoint execution_deadline_;
  SteadyTimePoint final_state_deadline_;
  SteadyTimePoint result_receive_steady_time_;
  bool has_execution_deadline_ = false;
  bool has_final_state_deadline_ = false;
  double result_ros_time_sec_ = 0.0;
  std::uint64_t joint_state_sequence_baseline_ = 0;
  double final_joint_error_ = 0.0;
  int result_error_code_ = 0;
  std::size_t terminal_transition_count_ = 0;
  std::size_t final_verification_start_count_ = 0;
};

}  // namespace colmag_ros
