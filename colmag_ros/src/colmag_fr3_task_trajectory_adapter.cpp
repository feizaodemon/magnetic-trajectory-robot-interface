#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <ros/steady_timer.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <urdf/model.h>

#include "colmag_ros/fr3_hardware_safety.h"
#include "colmag_ros/fr3_hexagon_trajectory.h"
#include "colmag_ros/fr3_joint_excursion_trajectory.h"
#include "colmag_ros/fr3_trajectory_safety.h"
#include "colmag_ros/real_fr3_command_admission.h"

namespace colmag_ros
{
namespace
{
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;
typedef TransportClientOwner<TrajectoryClient> TrajectoryClientOwner;
struct PreparedMotionTrajectory
{
  bool valid = false;
  std::string reason;
  GoalIdentity identity;
  std::vector<JointTrajectorySample> samples;
};
bool readBool(const boost::property_tree::ptree& tree, const std::string& key, bool fallback)
{
  try
  {
    return tree.get<bool>(key);
  }
  catch (const boost::property_tree::ptree_error&)
  {
    return fallback;
  }
}

std::string readString(const boost::property_tree::ptree& tree,
                       const std::string& key,
                       const std::string& fallback = "")
{
  try
  {
    return tree.get<std::string>(key);
  }
  catch (const boost::property_tree::ptree_error&)
  {
    return fallback;
  }
}

bool readDouble(const boost::property_tree::ptree& tree, const std::string& key, double* value)
{
  try
  {
    *value = tree.get<double>(key);
    return true;
  }
  catch (const boost::property_tree::ptree_error&)
  {
    return false;
  }
}

bool readInt(const boost::property_tree::ptree& tree, const std::string& key, int* value)
{
  try
  {
    *value = tree.get<int>(key);
    return true;
  }
  catch (const boost::property_tree::ptree_error&)
  {
    return false;
  }
}

std::vector<std::string> defaultJointNames(const std::string& arm_id)
{
  std::vector<std::string> names;
  for (int index = 1; index <= 7; ++index)
  {
    names.push_back(arm_id + "_joint" + std::to_string(index));
  }
  return names;
}

std::string jsonEscape(const std::string& value)
{
  std::ostringstream out;
  for (const char ch : value)
  {
    switch (ch)
    {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

std::string actionResultReason(int error_code)
{
  switch (error_code)
  {
    case control_msgs::FollowJointTrajectoryResult::SUCCESSFUL: return "successful";
    case control_msgs::FollowJointTrajectoryResult::INVALID_GOAL: return "invalid_goal";
    case control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS: return "invalid_joints";
    case control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP: return "old_header_timestamp";
    case control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED: return "path_tolerance_violated";
    case control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED: return "goal_tolerance_violated";
    default: return "unknown_result_error";
  }
}

}  // namespace

class ColmagFr3TaskTrajectoryAdapter
{
public:
  ColmagFr3TaskTrajectoryAdapter(ros::NodeHandle root_nh, ros::NodeHandle private_nh)
    : root_nh_(root_nh), private_nh_(private_nh)
  {
    loadParameters();
    std::string parameter_reason;
    if (!validateRuntimeParameters(&parameter_reason))
    {
      throw std::invalid_argument(parameter_reason);
    }
    adapter_session_id_ = newAdapterSessionId();
    initializeSafetyOwners();

    current_joint_positions_.assign(joint_names_.size(), 0.0);
    state_pub_ = private_nh_.advertise<std_msgs::String>(state_topic_, 10, true);
    adapter_session_pub_ = root_nh_.advertise<std_msgs::String>(adapter_session_topic_, 1, true);
    std_msgs::String session_message;
    session_message.data = adapter_session_id_;
    adapter_session_pub_.publish(session_message);
    command_sub_ = root_nh_.subscribe(command_topic_, 10,
                                      &ColmagFr3TaskTrajectoryAdapter::commandCallback, this);
    joint_state_sub_ = root_nh_.subscribe(joint_state_topic_, 10,
                                          &ColmagFr3TaskTrajectoryAdapter::jointStateCallback, this);
    if (!required_control_mode_.empty())
    {
      mode_sub_ = root_nh_.subscribe(control_mode_topic_, 1,
                                     &ColmagFr3TaskTrajectoryAdapter::controlModeCallback, this);
    }
    lifecycle_timer_ = root_nh_.createSteadyTimer(
        ros::WallDuration(watchdog_period_sec_),
        &ColmagFr3TaskTrajectoryAdapter::lifecycleTimer, this);
    if (send_goals_ && hardware_execution_enabled_ && hardware_profile_ == "fr3-hardware-active")
    {
      trajectory_client_owner_.install(
          std::unique_ptr<TrajectoryClient>(new TrajectoryClient(action_name_, true)));
  }
  publishStatus("READY", "NO_OP", "initialized",
                  send_goals_ ? "hardware_gates_pending" : "dry_run_default_no_motion");
  }
  ~ColmagFr3TaskTrajectoryAdapter() noexcept { shutdown(); }

  void shutdown() noexcept
  {
    const LifecycleTransition transition = goal_lifecycle_.beginShutdown(SteadyClock::now());
    try
    {
      publishLifecycleTransition(transition, "shutdown");
    }
    catch (const std::exception& exc)
    {
      ROS_ERROR_STREAM("Shutdown lifecycle publication failed: " << exc.what());
    }
    try
    {
      lifecycle_timer_.stop();
      command_sub_.shutdown();
      joint_state_sub_.shutdown();
      mode_sub_.shutdown();
    }
    catch (const std::exception& exc)
    {
      ROS_ERROR_STREAM("Adapter wiring shutdown failed: " << exc.what());
    }
    std::unique_ptr<TrajectoryClient> trajectory_client =
        trajectory_client_owner_.closeAndDetach();
    cancelOwnedGoalOnClient(trajectory_client.get(), transition, "shutdown");
    try
    {
      trajectory_client.reset();
    }
    catch (const std::exception& exc)
    {
      ROS_ERROR_STREAM("ActionClient teardown failed: " << exc.what());
    }
  }
private:
  void loadParameters()
  {
    private_nh_.param<std::string>("command_topic", command_topic_, "/colmag/task_command");
    private_nh_.param<std::string>("state_topic", state_topic_, "/colmag/fr3_task_trajectory_adapter_state");
    private_nh_.param<std::string>("adapter_session_topic", adapter_session_topic_,
                                   "/colmag/fr3_adapter_session");
    private_nh_.param<std::string>("controller_name", controller_name_,
                                   "position_joint_trajectory_controller");
    private_nh_.param<std::string>("follow_joint_trajectory_action", action_name_,
                                   "/position_joint_trajectory_controller/follow_joint_trajectory");
    private_nh_.param<std::string>("joint_state_topic", joint_state_topic_,
                                   "/franka_state_controller/joint_states");
    private_nh_.param<std::string>("arm_id", arm_id_, "fr3");
    private_nh_.param<std::string>("target_joint_name", target_joint_name_, arm_id_ + "_joint1");
    private_nh_.param<std::string>("controller_joint_names_param", controller_joint_names_param_,
                                   "/" + controller_name_ + "/joints");
    private_nh_.param<std::string>("robot_description_param", robot_description_param_,
                                   "/robot_description");
    private_nh_.param<std::string>("hardware_profile", hardware_profile_, "dry-run");
    private_nh_.param<std::string>("control_mode_topic", control_mode_topic_, "");
    private_nh_.param<std::string>("required_control_mode", required_control_mode_, "");
    private_nh_.param("send_goals", send_goals_, false);
    private_nh_.param("hardware_execution_enabled", hardware_execution_enabled_, false);
    private_nh_.param("require_confirmed", require_confirmed_, true);
    private_nh_.param("require_accepted", require_accepted_, true);
    private_nh_.param("reject_gazebo_only_commands", reject_gazebo_only_commands_, true);
    private_nh_.param("reject_controls_real_robot_true", reject_controls_real_robot_true_, true);
    private_nh_.param("joint_state_max_age_sec", joint_state_max_age_sec_, 0.5);
    private_nh_.param("joint_state_future_tolerance_sec", joint_state_future_tolerance_sec_, 0.05);
    private_nh_.param("joint_nudge_delta_rad", joint_nudge_delta_rad_, 0.10);
    if (!private_nh_.hasParam("joint_nudge_delta_rad"))
    {
      private_nh_.param("move_left_joint_delta_rad", joint_nudge_delta_rad_, 0.10);
    }
    private_nh_.param("max_abs_joint_delta_rad", max_abs_joint_delta_rad_, 0.12);
    private_nh_.param("goal_duration_sec", goal_duration_sec_, 3.0);
    private_nh_.param("trajectory_start_delay_sec", trajectory_start_delay_sec_, 0.2);
    private_nh_.param("max_velocity_rad_s", max_velocity_rad_s_, 0.12);
    private_nh_.param("max_acceleration_rad_s2", max_acceleration_rad_s2_, 0.18);
    private_nh_.param("sample_period_sec", sample_period_sec_, 0.10);
    private_nh_.param<std::string>("hexagon_primary_joint_name",
                                   hexagon_primary_joint_name_, arm_id_ + "_joint1");
    private_nh_.param<std::string>("hexagon_secondary_joint_name",
                                   hexagon_secondary_joint_name_, arm_id_ + "_joint6");
    private_nh_.param("hexagon_primary_offset_rad", hexagon_primary_offset_rad_, 0.08);
    private_nh_.param("hexagon_secondary_offset_rad", hexagon_secondary_offset_rad_, 0.10);
    private_nh_.param("hexagon_minimum_edge_duration_sec",
                      hexagon_minimum_edge_duration_sec_, 1.0);
    private_nh_.param("joint_limit_margin_rad", joint_limit_margin_rad_, 0.02);
    private_nh_.param("action_server_wait_sec", action_server_wait_sec_, 0.5);
    private_nh_.param("execution_timeout_sec", execution_timeout_sec_, 15.0);
    private_nh_.param("final_state_timeout_sec", final_state_timeout_sec_, 1.0);
    private_nh_.param("watchdog_period_sec", watchdog_period_sec_, 0.05);
    private_nh_.param("final_joint_error_tolerance_rad", final_joint_error_tolerance_rad_, 0.01);
    private_nh_.param("one_shot_hardware_mode", one_shot_hardware_mode_, true);
    private_nh_.param("max_motion_goals_per_process", max_motion_goals_per_process_, 1);
    private_nh_.param("command_max_age_sec", command_max_age_sec_, 2.0);
    private_nh_.param("command_future_tolerance_sec", command_future_tolerance_sec_, 0.1);
    private_nh_.param("recent_command_cache_size", recent_command_cache_size_, 128);
    private_nh_.param("supported_schema_version", supported_schema_version_, 1);

    if (!private_nh_.getParam("allowed_tasks", allowed_tasks_) || allowed_tasks_.empty())
    {
      allowed_tasks_ = {"MOVE_LEFT", "HEXAGON_TRAJECTORY", "MOVE_RIGHT", "STOP_OR_CANCEL"};
    }
    for (std::string& task : allowed_tasks_)
    {
      task = normalizeRealFr3Task(task);
    }
    if (!private_nh_.getParam("joint_names", joint_names_) || joint_names_.empty())
    {
      joint_names_ = defaultJointNames(arm_id_);
    }
  }
  void initializeSafetyOwners()
  {
    const std::size_t cache_size = recent_command_cache_size_ > 0 ?
                                   static_cast<std::size_t>(recent_command_cache_size_) : 0U;
    const std::size_t max_goals = max_motion_goals_per_process_ > 0 ?
                                  static_cast<std::size_t>(max_motion_goals_per_process_) : 0U;
    command_guard_.reset(new CommandReplayGuard(command_max_age_sec_, command_future_tolerance_sec_,
                                                cache_size, supported_schema_version_,
                                                adapter_session_id_));
    one_shot_gate_.reset(new OneShotMotionGate(one_shot_hardware_mode_, max_goals));
  }
  bool hardwareActive() const
  {
    return send_goals_ && hardware_execution_enabled_ &&
           hardware_profile_ == "fr3-hardware-active";
  }
  bool taskModeAllowsAdmission() const
  {
    return required_control_mode_.empty() ||
           (mode_observed_ && current_control_mode_ == required_control_mode_);
  }
  void controlModeCallback(const std_msgs::StringConstPtr& message)
  {
    const bool previously_allowed = taskModeAllowsAdmission();
    const std::string selected = message->data == "TASK" || message->data == "TELEOP" ?
                                     message->data : std::string();
    current_control_mode_ = selected;
    mode_observed_ = !selected.empty();
    if (previously_allowed && !taskModeAllowsAdmission())
    {
      relinquishOwnedGoalForMode();
    }
    publishStatus(taskModeAllowsAdmission() ? "TASK_MODE" : "TASK_BLOCKED",
                  "NO_OP", "control_mode",
                  mode_observed_ ? "mode_state_updated" : "invalid_control_mode");
  }
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& message)
  {
    std::vector<double> positions;
    positions.reserve(joint_names_.size());
    for (const std::string& joint_name : joint_names_)
    {
      const std::vector<std::string>::const_iterator found =
          std::find(message->name.begin(), message->name.end(), joint_name);
      if (found == message->name.end())
      {
        handleJointStateMismatch("missing_joint_name:" + joint_name);
        return;
      }
      const std::size_t source_index = static_cast<std::size_t>(found - message->name.begin());
      if (source_index >= message->position.size())
      {
        handleJointStateMismatch("missing_joint_position:" + joint_name);
        return;
      }
      positions.push_back(message->position[source_index]);
    }
    if (!finiteVector(positions))
    {
      handleJointStateMismatch("non_finite_joint_position");
      return;
    }

    const std::uint64_t generation = goal_lifecycle_.snapshot().goal_generation;
    const SteadyTimePoint receive_time = SteadyClock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_observed_joint_names_ = message->name;
      current_joint_positions_ = positions;
      last_joint_state_stamp_ = message->header.stamp;
      last_joint_state_receive_steady_time_ = receive_time;
      ++joint_state_receive_sequence_;
      have_joint_state_ = true;
      joint_name_mismatch_reported_ = false;
    }
    tryFinalStateVerification(generation);
  }
  void reportJointStateMismatchOnce(const std::string& reason)
  {
    bool should_publish = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!joint_name_mismatch_reported_)
      {
        joint_name_mismatch_reported_ = true;
        should_publish = true;
      }
    }
    if (should_publish)
    {
      publishStatus("WAITING", "NO_OP", "joint_state_mismatch", reason);
    }
  }
  void handleJointStateMismatch(const std::string& reason)
  {
    reportJointStateMismatchOnce(reason);
    const LifecycleSnapshot snapshot = goal_lifecycle_.snapshot();
    const LifecycleTransition transition = goal_lifecycle_.handleJointStateFailure(
        snapshot.goal_generation, reason);
    publishLifecycleTransition(transition, "joint_state_failure");
    cancelOwnedGoal(transition, "joint_state_failure");
  }
  CommandMetadata commandMetadata(const boost::property_tree::ptree& payload) const
  {
    CommandMetadata metadata;
    metadata.command_id = readString(payload, "command_id", "");
    metadata.target_adapter_session_id = readString(payload, "target_adapter_session_id", "");
    metadata.has_issued_at = readDouble(payload, "issued_at", &metadata.issued_at_sec);
    metadata.has_schema_version = readInt(payload, "schema_version", &metadata.schema_version);
    return metadata;
  }
  void rememberCommandContext(const CommandMetadata& metadata,
                              const std::string& source_id)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_command_id_ = metadata.command_id;
    last_source_id_ = source_id;
  }
  RealFr3AdmissionDecision admissionDecision(const boost::property_tree::ptree& payload)
  {
    RealFr3AdmissionInput input;
    input.task = readString(payload, "task", "NO_OP");
    input.confirmed = readBool(payload, "confirmed", false);
    input.accepted = readBool(payload, "accepted", false);
    input.gazebo_only = readBool(payload, "gazebo_only", false);
    input.controls_real_robot = readBool(payload, "upstream_controls_real_robot",
                                         readBool(payload, "controls_real_robot", false));
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_upstream_controls_real_robot_ = input.controls_real_robot;
    }

    RealFr3AdmissionPolicy policy;
    policy.require_confirmed = require_confirmed_;
    policy.require_accepted = require_accepted_;
    policy.allowed_tasks = allowed_tasks_;
    policy.reject_gazebo_only_commands = reject_gazebo_only_commands_;
    policy.reject_controls_real_robot_true = reject_controls_real_robot_true_;
    return evaluateRealFr3CommandAdmission(input, policy);
  }
  void commandCallback(const std_msgs::StringConstPtr& message)
  {
    if (!taskModeAllowsAdmission())
    {
      publishStatus("REJECTED", "NO_OP", "control_mode_rejected",
                    mode_observed_ ? "task_mode_not_selected" :
                                     "control_mode_not_observed");
      return;
    }
    boost::property_tree::ptree payload;
    std::istringstream input(message->data);
    try
    {
      boost::property_tree::read_json(input, payload);
    }
    catch (const boost::property_tree::json_parser_error& exc)
    {
      publishStatus("REJECTED", "NO_OP", "invalid_json", exc.what());
      return;
    }

    const CommandMetadata metadata = commandMetadata(payload);
    const std::string source_id = readString(payload, "source_id", "");
    rememberCommandContext(metadata, source_id);
    const RealFr3AdmissionDecision admission = admissionDecision(payload);
    if (!admission.allowed)
    {
      publishStatus("REJECTED", admission.normalized_task, "admission_rejected",
                    realFr3AdmissionReasonCode(admission.reason));
      return;
    }

    const bool motion_command = admission.normalized_task != "STOP_OR_CANCEL";
    const CommandGuardDecision guard = command_guard_->evaluate(
        metadata, ros::Time::now().toSec(), hardwareActive(), motion_command);
    if (!guard.allowed)
    {
      publishStatus("REJECTED", admission.normalized_task, "command_guard_rejected", guard.reason);
      return;
    }
    if (admission.normalized_task == "STOP_OR_CANCEL")
    {
      stopOwnedGoal();
      return;
    }
    if (admission.normalized_task != "MOVE_LEFT" &&
        admission.normalized_task != "HEXAGON_TRAJECTORY" &&
        admission.normalized_task != "MOVE_RIGHT")
    {
      publishStatus("REJECTED", admission.normalized_task, "goal_suppressed",
                    "unsupported_task_no_real_goal");
      return;
    }
    if (!send_goals_)
    {
      publishStatus("DRY_RUN_ACCEPTED", admission.normalized_task, "goal_suppressed",
                    guard.compatibility_warning ? guard.reason : "send_goals_false");
      return;
    }
    if (!hardware_execution_enabled_)
    {
      publishStatus("REJECTED", admission.normalized_task, "hardware_gate_rejected",
                    "hardware_execution_disabled");
      return;
    }
    if (hardware_profile_ != "fr3-hardware-active")
    {
      publishStatus("REJECTED", admission.normalized_task, "hardware_gate_rejected",
                    "active_hardware_profile_required");
      return;
    }
    sendMotionGoal(admission.normalized_task, metadata, source_id);
  }
  bool jointStateSnapshot(std::vector<double>* positions,
                          std::vector<std::string>* observed_names,
                          ros::Time* stamp,
                          std::string* reason)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!have_joint_state_)
      {
        *reason = "missing_joint_states";
        return false;
      }
      *positions = current_joint_positions_;
      *observed_names = last_observed_joint_names_;
      *stamp = last_joint_state_stamp_;
    }
    if (stamp->isZero())
    {
      *reason = "zero_joint_state_stamp";
      return false;
    }
    const ros::Time now = ros::Time::now();
    if (!joint_ros_time_guard_.evaluate(
            now.toSec(), "joint_state_ros_time_moved_backwards", reason))
    {
      return false;
    }
    const double age_sec = (now - *stamp).toSec();
    if (!std::isfinite(age_sec) || age_sec > joint_state_max_age_sec_)
    {
      *reason = "stale_joint_states";
      return false;
    }
    if (age_sec < -joint_state_future_tolerance_sec_)
    {
      *reason = "future_joint_state_stamp";
      return false;
    }
    reason->clear();
    return true;
  }
  bool controllerJointContract(const std::vector<std::string>& observed_names,
                               std::string* reason) const
  {
    std::vector<std::string> controller_names;
    if (!root_nh_.getParam(controller_joint_names_param_, controller_names))
    {
      *reason = "controller_joint_names_unavailable";
      return false;
    }
    return validateJointNameContract(joint_names_, controller_names, observed_names, reason);
  }
  bool targetJointLimit(JointPositionLimit* limit, std::string* reason) const
  {
    std::string robot_description;
    if (!root_nh_.getParam(robot_description_param_, robot_description) || robot_description.empty())
    {
      *reason = "robot_description_unavailable";
      return false;
    }
    urdf::Model model;
    if (!model.initString(robot_description))
    {
      *reason = "robot_description_parse_failed";
      return false;
    }
    const urdf::JointConstSharedPtr joint = model.getJoint(target_joint_name_);
    if (!joint || !joint->limits)
    {
      *reason = "target_joint_urdf_limit_unavailable";
      return false;
    }
    limit->lower = joint->limits->lower;
    limit->upper = joint->limits->upper;
    reason->clear();
    return true;
  }
  bool configuredJointLimits(std::vector<JointPositionLimit>* limits,
                             std::string* reason) const
  {
    std::string robot_description;
    if (!root_nh_.getParam(robot_description_param_, robot_description) ||
        robot_description.empty())
    {
      *reason = "hexagon_joint_missing";
      return false;
    }
    urdf::Model model;
    if (!model.initString(robot_description))
    {
      *reason = "hexagon_trajectory_invalid";
      return false;
    }
    limits->clear();
    for (const std::string& joint_name : joint_names_)
    {
      const urdf::JointConstSharedPtr joint = model.getJoint(joint_name);
      if (!joint || !joint->limits)
      {
        *reason = "hexagon_joint_missing";
        limits->clear();
        return false;
      }
      JointPositionLimit limit;
      limit.lower = joint->limits->lower;
      limit.upper = joint->limits->upper;
      limits->push_back(limit);
    }
    reason->clear();
    return true;
  }
  JointExcursionTrajectory prepareJointExcursionTrajectory(
      const std::vector<double>& positions, double signed_excursion, std::string* reason)
  {
    JointPositionLimit limit;
    if (!targetJointLimit(&limit, reason))
    {
      return JointExcursionTrajectory();
    }

    JointExcursionTrajectoryRequest request;
    request.joint_names = joint_names_;
    request.current_positions = positions;
    request.target_joint_name = target_joint_name_;
    request.signed_excursion_rad = signed_excursion;
    request.max_abs_excursion_rad = max_abs_joint_delta_rad_;
    request.minimum_leg_duration_sec = goal_duration_sec_;
    request.max_velocity_rad_s = max_velocity_rad_s_;
    request.max_acceleration_rad_s2 = max_acceleration_rad_s2_;
    request.sample_period_sec = sample_period_sec_;
    request.target_limit = limit;
    request.joint_limit_margin_rad = joint_limit_margin_rad_;
    JointExcursionTrajectory trajectory = generateJointExcursionTrajectory(request);
    *reason = trajectory.reason;
    return trajectory;
  }
  PreparedMotionTrajectory prepareMotionTrajectory(const std::string& task,
                                                     const CommandMetadata& metadata,
                                                     const std::string& source_id)
  {
    PreparedMotionTrajectory prepared;
    std::vector<double> positions;
    std::vector<std::string> observed_names;
    ros::Time stamp;
    if (!jointStateSnapshot(&positions, &observed_names, &stamp, &prepared.reason) ||
        !controllerJointContract(observed_names, &prepared.reason))
    {
      return prepared;
    }
    prepared.identity.command_id = metadata.command_id;
    prepared.identity.source_id = source_id;
    prepared.identity.primitive = task;
    if (task == "MOVE_LEFT" || task == "MOVE_RIGHT")
    {
      const double signed_excursion = task == "MOVE_LEFT" ?
                                      joint_nudge_delta_rad_ : -joint_nudge_delta_rad_;
      const JointExcursionTrajectory trajectory =
          prepareJointExcursionTrajectory(positions, signed_excursion, &prepared.reason);
      if (!trajectory.valid) return prepared;
      prepared.identity.target_joint = target_joint_name_;
      prepared.identity.target_position = trajectory.peak_position_rad;
      prepared.identity.final_joint_names = joint_names_;
      prepared.identity.final_joint_positions = positions;
      prepared.samples = trajectory.samples;
      prepared.valid = true;
      return prepared;
    }

    HexagonTrajectoryRequest request;
    request.geometry.joint_names = joint_names_;
    request.geometry.current_positions = positions;
    request.geometry.primary_joint_name = hexagon_primary_joint_name_;
    request.geometry.secondary_joint_name = hexagon_secondary_joint_name_;
    request.geometry.primary_offset_rad = hexagon_primary_offset_rad_;
    request.geometry.secondary_offset_rad = hexagon_secondary_offset_rad_;
    request.geometry.max_abs_offset_rad = max_abs_joint_delta_rad_;
    if (!configuredJointLimits(&request.joint_limits, &prepared.reason)) return prepared;
    request.joint_limit_margin_rad = joint_limit_margin_rad_;
    request.minimum_edge_duration_sec = hexagon_minimum_edge_duration_sec_;
    request.max_velocity_rad_s = max_velocity_rad_s_;
    request.max_acceleration_rad_s2 = max_acceleration_rad_s2_;
    request.sample_period_sec = sample_period_sec_;
    const HexagonTrajectory trajectory = generateHexagonTrajectory(request);
    prepared.reason = trajectory.reason;
    if (!trajectory.valid) return prepared;
    prepared.identity.final_joint_names = joint_names_;
    prepared.identity.final_joint_positions = positions;
    prepared.samples = trajectory.samples;
    prepared.valid = true;
    return prepared;
  }
  control_msgs::FollowJointTrajectoryGoal rosGoal(
      const std::vector<JointTrajectorySample>& samples) const
  {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(trajectory_start_delay_sec_);
    goal.trajectory.joint_names = joint_names_;
    for (const JointTrajectorySample& sample : samples)
    {
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = sample.positions;
      point.velocities = sample.velocities;
      point.accelerations = sample.accelerations;
      point.time_from_start = ros::Duration(sample.time_from_start_sec);
      goal.trajectory.points.push_back(point);
    }
    return goal;
  }
  bool validateRuntimeParameters(std::string* reason) const
  {
    if (!required_control_mode_.empty() &&
        (required_control_mode_ != "TASK" || control_mode_topic_.empty()))
    {
      *reason = "invalid_control_mode_configuration";
      return false;
    }
    if (max_motion_goals_per_process_ < 0)
    {
      *reason = "negative_max_motion_goals_per_process";
      return false;
    }
    const std::vector<std::pair<std::string, double> > bounded = {
      {"action_server_wait_sec", action_server_wait_sec_},
      {"execution_timeout_sec", execution_timeout_sec_},
      {"final_state_timeout_sec", final_state_timeout_sec_},
      {"trajectory_start_delay_sec", trajectory_start_delay_sec_},
      {"joint_state_max_age_sec", joint_state_max_age_sec_},
      {"command_max_age_sec", command_max_age_sec_},
      {"watchdog_period_sec", watchdog_period_sec_},
    };
    for (const std::pair<std::string, double>& parameter : bounded)
    {
      if (!validateRuntimeDuration(parameter.first, parameter.second, reason))
      {
        return false;
      }
    }
    const std::vector<double> positive = {
      joint_state_future_tolerance_sec_, goal_duration_sec_, max_velocity_rad_s_,
      max_acceleration_rad_s2_, sample_period_sec_, final_joint_error_tolerance_rad_,
      hexagon_primary_offset_rad_, hexagon_secondary_offset_rad_,
      hexagon_minimum_edge_duration_sec_};
    if (!finiteVector(positive) ||
        std::any_of(positive.begin(), positive.end(), [](double value) { return value <= 0.0; }))
    {
      *reason = "invalid_non_positive_runtime_parameter";
      return false;
    }
    reason->clear();
    return true;
  }
  void publishLifecycleTransition(const LifecycleTransition& transition,
                                  const std::string& phase,
                                  const std::string& reason = "",
                                  const std::string& action_server_status = "not_checked") const
  {
    if (!transition.accepted())
    {
      ROS_DEBUG_STREAM("Ignored lifecycle event: disposition="
                       << transitionDispositionName(transition.disposition)
                       << " generation=" << transition.goal_generation
                       << " reason=" << transition.reason);
      return;
    }
    std::lock_guard<std::mutex> publish_lock(status_publish_mutex_);
    LifecycleSnapshot lifecycle = goal_lifecycle_.snapshot();
    if (transition.event_sequence <= last_published_lifecycle_event_sequence_ ||
        lifecycle.event_sequence != transition.event_sequence)
    {
      return;
    }
    lifecycle.state = transition.state;
    lifecycle.terminal = transition.terminal;
    std_msgs::String message;
    const std::string primitive = lifecycle.identity.primitive.empty() ?
                                  "MOVE_LEFT" : lifecycle.identity.primitive;
    message.data = statusPayload(
        goalLifecycleStateName(transition.state), primitive, phase,
        reason.empty() ? transition.reason : reason, action_server_status, lifecycle);
    state_pub_.publish(message);
    last_published_lifecycle_event_sequence_ = transition.event_sequence;
  }
  void publishLifecycleTransitions(const std::vector<LifecycleTransition>& transitions,
                                   const std::string& phase,
                                   const std::string& action_server_status) const
  {
    if (transitions.empty())
    {
      return;
    }
    std::lock_guard<std::mutex> publish_lock(status_publish_mutex_);
    LifecycleSnapshot current = goal_lifecycle_.snapshot();
    if (current.event_sequence != transitions.back().event_sequence)
    {
      return;
    }
    for (const LifecycleTransition& transition : transitions)
    {
      if (!transition.accepted() ||
          transition.event_sequence <= last_published_lifecycle_event_sequence_ ||
          transition.goal_generation != current.goal_generation)
      {
        continue;
      }
      LifecycleSnapshot lifecycle = current;
      lifecycle.state = transition.state;
      lifecycle.event_sequence = transition.event_sequence;
      lifecycle.terminal = transition.terminal;
      std_msgs::String message;
      const std::string primitive = lifecycle.identity.primitive.empty() ?
                                    "MOVE_LEFT" : lifecycle.identity.primitive;
      message.data = statusPayload(
          goalLifecycleStateName(transition.state), primitive, phase,
          transition.reason, action_server_status, lifecycle);
      state_pub_.publish(message);
      last_published_lifecycle_event_sequence_ = transition.event_sequence;
    }
  }
  void cancelOwnedGoalOnClient(TrajectoryClient* client,
                               const LifecycleTransition& transition,
                               const std::string& context) noexcept
  {
    if (!transition.cancel_owned_goal || !client)
    {
      return;
    }
    try
    {
      client->cancelGoal();
    }
    catch (const std::exception& exc)
    {
      ROS_ERROR_STREAM("Owned goal cancel failed during " << context << ": " << exc.what());
    }
  }
  void cancelOwnedGoal(const LifecycleTransition& transition,
                       const std::string& context) noexcept
  {
    TrajectoryClientOwner::Lease client = trajectory_client_owner_.acquire();
    cancelOwnedGoalOnClient(client.get(), transition, context);
  }
  void sendMotionGoal(const std::string& task,
                      const CommandMetadata& metadata,
                      const std::string& source_id)
  {
    std::string reason;
    if (!validateRuntimeParameters(&reason))
    {
      publishStatus("REJECTED", task, "goal_suppressed", reason);
      return;
    }
    const LifecycleSnapshot lifecycle = goal_lifecycle_.snapshot();
    if (lifecycle.goal_generation != 0 && !lifecycle.terminal)
    {
      reason = "goal_lifecycle_busy";
    }
    if (reason.empty())
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      one_shot_gate_->canSend(&reason);
    }
    if (!reason.empty())
    {
      publishStatus("REJECTED", task, "goal_suppressed", reason);
      return;
    }
    const PreparedMotionTrajectory trajectory =
        prepareMotionTrajectory(task, metadata, source_id);
    reason = trajectory.reason;
    if (!trajectory.valid)
    {
      publishStatus("REJECTED", task, "trajectory_rejected", reason);
      return;
    }
    bool action_server_available = false;
    {
      TrajectoryClientOwner::Lease client = trajectory_client_owner_.acquire();
      if (!client)
      {
        publishStatus("REJECTED", task, "goal_suppressed",
                      "action_client_not_initialized");
        return;
      }
      action_server_available =
          client->waitForServer(ros::Duration(action_server_wait_sec_));
    }
    if (!action_server_available)
    {
      publishStatus("REJECTED", task, "goal_suppressed",
                    "action_server_unavailable", "unavailable");
      return;
    }

    const LifecycleTransition prepared = goal_lifecycle_.beginGoal(trajectory.identity);
    publishLifecycleTransition(prepared, "trajectory_ready");
    if (!prepared.accepted())
    {
      publishStatus("REJECTED", task, "goal_suppressed", prepared.reason);
      return;
    }

    const control_msgs::FollowJointTrajectoryGoal goal = rosGoal(trajectory.samples);
    std::vector<LifecycleTransition> transitions;
    LifecycleTransition failed;
    std::string send_exception;
    {
      TrajectoryClientOwner::Lease client = trajectory_client_owner_.acquire();
      const LifecycleTransition sending = goal_lifecycle_.beginSend(prepared.goal_generation);
      if (!sending.accepted() || !client)
      {
        return;
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        one_shot_gate_->recordSendAttempt();
      }
      try
      {
        // Noetic actionlib 1.14.3 creates its goal state before publish and
        // returns the handle immediately after publish. For this adapter's
        // remote, non-latched controller path, roscpp 1.17.4 serializes before
        // queue insertion and performs no throwing work after insertion. Thus
        // a caller-visible exception here is pre-send for the supported path;
        // intraprocess action servers are outside this production contract.
        const std::uint64_t generation = prepared.goal_generation;
        client->sendGoal(
            goal,
            [this, generation](const actionlib::SimpleClientGoalState& state,
                               const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
              actionDone(generation, state, result);
            },
            [this, generation]() { actionActive(generation); });
      }
      catch (const std::exception& exc)
      {
        send_exception = exc.what();
        failed = goal_lifecycle_.markSendFailed(prepared.goal_generation, -999);
      }
      if (send_exception.empty())
      {
        transitions = goal_lifecycle_.markSendSucceeded(
            prepared.goal_generation, SteadyClock::now(),
            steadyDurationFromSeconds(execution_timeout_sec_));
        for (const LifecycleTransition& transition : transitions)
        {
          cancelOwnedGoalOnClient(
              client.get(), transition, "shutdown_after_transport_send");
        }
      }
    }
    if (!send_exception.empty())
    {
      publishLifecycleTransition(failed, "send_failed", "send_goal_exception", "unavailable");
      ROS_ERROR_STREAM("FollowJointTrajectory sendGoal failed: " << send_exception);
      return;
    }
    publishLifecycleTransitions(transitions, "action_transport", "available");
  }
  void actionActive(std::uint64_t generation)
  {
    const LifecycleTransition transition = goal_lifecycle_.handleActive(
        generation, SteadyClock::now());
    publishLifecycleTransition(transition, "action_transport");
    cancelOwnedGoal(transition, "late_active_timeout");
  }
  void actionDone(std::uint64_t generation,
                  const actionlib::SimpleClientGoalState& action_state,
                  const control_msgs::FollowJointTrajectoryResultConstPtr& result)
  {
    const int error_code = result ? result->error_code : -999;
    GoalTerminalState terminal_state = GoalTerminalState::kAborted;
    if (action_state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      terminal_state = GoalTerminalState::kSucceeded;
    }
    else if (action_state == actionlib::SimpleClientGoalState::REJECTED)
    {
      terminal_state = GoalTerminalState::kRejected;
    }
    else if (action_state == actionlib::SimpleClientGoalState::PREEMPTED)
    {
      terminal_state = GoalTerminalState::kPreempted;
    }

    GoalResultEvent event;
    event.terminal_state = terminal_state;
    event.result_error_code = error_code;
    event.result_receive_steady_time = SteadyClock::now();
    event.result_ros_time_sec = ros::Time::now().toSec();
    event.final_state_timeout = steadyDurationFromSeconds(final_state_timeout_sec_);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      event.joint_state_sequence_baseline = joint_state_receive_sequence_;
    }
    const LifecycleTransition transition = goal_lifecycle_.handleDone(generation, event);
    const std::string reason = transition.state == GoalLifecycleState::kFinalStateVerifying ?
                               "action_success_requires_post_result_joint_state" :
                               actionResultReason(error_code);
    publishLifecycleTransition(transition, "action_terminal", reason,
                               action_state.toString());
    cancelOwnedGoal(transition, "late_done_timeout");
  }
  void stopOwnedGoal()
  {
    LifecycleTransition transition;
    const LifecycleSnapshot snapshot = goal_lifecycle_.snapshot();
    transition = goal_lifecycle_.requestCancel(snapshot.goal_generation, SteadyClock::now());
    if (transition.accepted())
    {
      publishLifecycleTransition(transition, "software_cancel");
    }
    else
    {
      publishStatus(transition.state == GoalLifecycleState::kFinalStateVerifying ?
                    "FINAL_STATE_VERIFYING" : "NO_OWNED_ACTIVE_GOAL", "STOP_OR_CANCEL",
                    "software_cancel", "no_owned_active_goal");
    }
    cancelOwnedGoal(transition, "software_cancel");
  }
  void relinquishOwnedGoalForMode()
  {
    const LifecycleSnapshot snapshot = goal_lifecycle_.snapshot();
    const LifecycleTransition transition =
        goal_lifecycle_.requestCancel(snapshot.goal_generation, SteadyClock::now());
    if (transition.accepted())
    {
      publishLifecycleTransition(transition, "control_mode_transition",
                                 "owned_goal_relinquish_requested");
    }
    cancelOwnedGoal(transition, "control_mode_transition");
  }
  void lifecycleTimer(const ros::SteadyTimerEvent&)
  {
    LifecycleTransition execution_timeout;
    LifecycleTransition final_timeout;
    const LifecycleSnapshot snapshot = goal_lifecycle_.snapshot();
    if (snapshot.goal_generation != 0)
    {
      const SteadyTimePoint now = SteadyClock::now();
      execution_timeout = goal_lifecycle_.handleTimeout(snapshot.goal_generation, now);
      publishLifecycleTransition(execution_timeout, "execution_timeout",
                                 "owned_goal_cancel_requested_after_timeout");
      final_timeout = goal_lifecycle_.handleFinalStateTimeout(snapshot.goal_generation, now);
      publishLifecycleTransition(final_timeout, "final_state_timeout");
    }
    cancelOwnedGoal(execution_timeout, "execution_timeout");
  }
  void tryFinalStateVerification(std::uint64_t generation)
  {
    FinalStateSample sample;
    sample.goal_generation = generation;
    sample.current_ros_time_sec = ros::Time::now().toSec();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!have_joint_state_)
      {
        return;
      }
      sample.receive_sequence = joint_state_receive_sequence_;
      sample.receive_steady_time = last_joint_state_receive_steady_time_;
      sample.header_stamp_sec = last_joint_state_stamp_.toSec();
      sample.names = last_observed_joint_names_;
      sample.positions = current_joint_positions_;
    }

    FinalStatePolicy policy;
    policy.configured_joint_names = joint_names_;
    policy.max_age_sec = joint_state_max_age_sec_;
    policy.future_tolerance_sec = joint_state_future_tolerance_sec_;
    policy.error_tolerance = final_joint_error_tolerance_rad_;
    const LifecycleTransition transition =
        goal_lifecycle_.verifyFinalState(generation, sample, policy);
    if (transition.accepted())
    {
      publishLifecycleTransition(transition, "final_state_verification");
    }
  }
  std::string stringArray(const std::vector<std::string>& values) const
  {
    std::ostringstream out;
    out << "[";
    for (std::size_t index = 0; index < values.size(); ++index)
    {
      if (index > 0)
      {
        out << ",";
      }
      out << "\"" << jsonEscape(values[index]) << "\"";
    }
    out << "]";
    return out.str();
  }
  std::string statusPayload(const std::string& state,
                            const std::string& task,
                            const std::string& phase,
                            const std::string& reason,
                            const std::string& action_server_status,
                            const LifecycleSnapshot& lifecycle) const
  {
    std::vector<std::string> observed_names;
    bool upstream_controls = false;
    bool quota_available = false;
    std::size_t send_attempts = 0;
    std::uint64_t joint_state_sequence = 0;
    std::string command_id;
    std::string source_id;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      observed_names = last_observed_joint_names_;
      upstream_controls = last_upstream_controls_real_robot_;
      send_attempts = one_shot_gate_->sendAttempts();
      std::string quota_reason;
      quota_available = one_shot_gate_->canSend(&quota_reason);
      joint_state_sequence = joint_state_receive_sequence_;
      command_id = last_command_id_;
      source_id = last_source_id_;
    }
    const LifecycleStatusProjection status = projectLifecycleStatus(
        lifecycle, state, phase, command_id, command_id, source_id);
    const bool adapter_controls = hardwareActive();
    const bool lifecycle_ready =
        (lifecycle.goal_generation == 0 && lifecycle.state == GoalLifecycleState::kReady &&
         !lifecycle.terminal) ||
        (lifecycle.terminal && lifecycle.state == GoalLifecycleState::kFinalStateVerified);
    const bool ready_for_fresh_motion_goal =
        adapter_controls && !lifecycle.shutting_down && lifecycle_ready && quota_available;
    std::ostringstream out;
    out << "{"
        << "\"action_name\":\"" << jsonEscape(action_name_) << "\","
        << "\"action_result_error_code\":" << lifecycle.result_error_code << ","
        << "\"action_server_status\":\"" << jsonEscape(action_server_status) << "\","
        << "\"adapter_session_id\":\"" << jsonEscape(adapter_session_id_) << "\","
        << "\"adapter_session_topic\":\"" << jsonEscape(adapter_session_topic_) << "\","
        << "\"adapter_controls_real_robot\":" << (adapter_controls ? "true" : "false") << ","
        << "\"allowed_tasks\":" << stringArray(allowed_tasks_) << ","
        << "\"command_id\":\"" << jsonEscape(status.command_id) << "\","
        << "\"command_topic\":\"" << jsonEscape(command_topic_) << "\","
        << "\"controls_real_robot\":" << (adapter_controls ? "true" : "false") << ","
        << "\"dry_run\":" << (adapter_controls ? "false" : "true") << ","
        << "\"execution_owner\":\"fr3_task_trajectory_adapter\","
        << "\"expected_joint_names\":" << stringArray(joint_names_) << ","
        << "\"diagnostic_code\":\"" << jsonEscape(status.diagnostic_code) << "\","
        << "\"diagnostic_command_id\":\"" << jsonEscape(status.diagnostic_command_id) << "\","
        << "\"goal_generation\":" << status.goal_generation << ","
        << "\"final_joint_error_rad\":";
    if (std::isfinite(lifecycle.final_joint_error))
    {
      out << lifecycle.final_joint_error;
    }
    else
    {
      out << "null";
    }
    out << ","
        << "\"hardware_execution_enabled\":" << (hardware_execution_enabled_ ? "true" : "false") << ","
        << "\"hardware_profile\":\"" << jsonEscape(hardware_profile_) << "\","
        << "\"joint_state_receive_sequence\":" << joint_state_sequence << ","
        << "\"joint_state_topic\":\"" << jsonEscape(joint_state_topic_) << "\","
        << "\"lifecycle_event_sequence\":"
        << status.lifecycle_event_sequence << ","
        << "\"lifecycle_state\":\""
        << goalLifecycleStateName(status.lifecycle_state) << "\","
        << "\"max_motion_goals_per_process\":" << max_motion_goals_per_process_ << ","
        << "\"motion_goal_quota_unlimited\":"
        << (max_motion_goals_per_process_ == 0 ? "true" : "false") << ","
        << "\"observed_joint_names\":" << stringArray(observed_names) << ","
        << "\"one_shot_hardware_mode\":" << (one_shot_hardware_mode_ ? "true" : "false") << ","
        << "\"motion_goal_send_attempts\":" << send_attempts << ","
        << "\"phase\":\"" << jsonEscape(phase) << "\","
        << "\"reason\":\"" << jsonEscape(reason) << "\","
        << "\"ready_for_fresh_motion_goal\":"
        << (ready_for_fresh_motion_goal ? "true" : "false") << ","
        << "\"robot_family\":\"FR3\","
        << "\"selected_primitive\":\""
        << jsonEscape(task == "STOP_OR_CANCEL" || task == "NO_OP" ? "" : task) << "\","
        << "\"send_goals\":" << (send_goals_ ? "true" : "false") << ","
        << "\"source_id\":\"" << jsonEscape(status.source_id) << "\","
        << "\"state\":\"" << jsonEscape(status.state) << "\","
        << "\"terminal\":" << (status.terminal ? "true" : "false") << ","
        << "\"target_joint_name\":\"" << jsonEscape(target_joint_name_) << "\","
        << "\"task\":\"" << jsonEscape(task) << "\","
        << "\"upstream_controls_real_robot\":" << (upstream_controls ? "true" : "false") << ","
        << "\"timestamp\":" << ros::Time::now().toSec()
        << "}";
    return out.str();
  }
  void publishStatus(const std::string& state,
                     const std::string& task,
                     const std::string& phase,
                     const std::string& reason,
                     const std::string& action_server_status = "not_checked") const
  {
    std::lock_guard<std::mutex> publish_lock(status_publish_mutex_);
    const LifecycleSnapshot lifecycle = goal_lifecycle_.snapshot();
    std_msgs::String message;
    message.data = statusPayload(state, task, phase, reason, action_server_status, lifecycle);
    state_pub_.publish(message);
  }
  ros::NodeHandle root_nh_, private_nh_;
  ros::Subscriber command_sub_, joint_state_sub_, mode_sub_;
  ros::Publisher state_pub_, adapter_session_pub_;
  ros::SteadyTimer lifecycle_timer_;
  std::unique_ptr<CommandReplayGuard> command_guard_;
  std::unique_ptr<OneShotMotionGate> one_shot_gate_;
  mutable std::mutex state_mutex_, status_publish_mutex_;
  GoalLifecycle goal_lifecycle_;
  RosTimeRollbackGuard joint_ros_time_guard_;
  std::vector<std::string> allowed_tasks_, joint_names_;
  std::vector<double> current_joint_positions_;
  std::vector<std::string> last_observed_joint_names_;
  ros::Time last_joint_state_stamp_;
  bool have_joint_state_ = false;
  bool joint_name_mismatch_reported_ = false;
  bool last_upstream_controls_real_robot_ = false;
  std::uint64_t joint_state_receive_sequence_ = 0;
  SteadyTimePoint last_joint_state_receive_steady_time_;
  mutable std::uint64_t last_published_lifecycle_event_sequence_ = 0;
  std::string last_command_id_, last_source_id_;
  std::string command_topic_, state_topic_, adapter_session_topic_, adapter_session_id_;
  std::string controller_name_, action_name_;
  std::string joint_state_topic_, arm_id_, target_joint_name_;
  std::string hexagon_primary_joint_name_, hexagon_secondary_joint_name_;
  std::string controller_joint_names_param_, robot_description_param_, hardware_profile_;
  std::string control_mode_topic_, required_control_mode_, current_control_mode_;
  bool mode_observed_ = false;
  bool send_goals_ = false;
  bool hardware_execution_enabled_ = false;
  bool require_confirmed_ = true;
  bool require_accepted_ = true;
  bool reject_gazebo_only_commands_ = true;
  bool reject_controls_real_robot_true_ = true;
  bool one_shot_hardware_mode_ = true;
  int max_motion_goals_per_process_ = 1;
  int recent_command_cache_size_ = 128;
  int supported_schema_version_ = 1;
  double joint_state_max_age_sec_ = 0.5;
  double joint_state_future_tolerance_sec_ = 0.05;
  double joint_nudge_delta_rad_ = 0.10;
  double max_abs_joint_delta_rad_ = 0.12;
  double goal_duration_sec_ = 3.0;
  double trajectory_start_delay_sec_ = 0.2;
  double max_velocity_rad_s_ = 0.12;
  double max_acceleration_rad_s2_ = 0.18;
  double sample_period_sec_ = 0.10;
  double hexagon_primary_offset_rad_ = 0.08;
  double hexagon_secondary_offset_rad_ = 0.10;
  double hexagon_minimum_edge_duration_sec_ = 1.0;
  double joint_limit_margin_rad_ = 0.02;
  double action_server_wait_sec_ = 0.5;
  double execution_timeout_sec_ = 15.0;
  double final_state_timeout_sec_ = 1.0;
  double watchdog_period_sec_ = 0.05;
  double final_joint_error_tolerance_rad_ = 0.01;
  double command_max_age_sec_ = 2.0;
  double command_future_tolerance_sec_ = 0.1;
  TrajectoryClientOwner trajectory_client_owner_;
};
}  // namespace colmag_ros
int main(int argc, char** argv)
{
  ros::init(argc, argv, "colmag_fr3_task_trajectory_adapter");
  colmag_ros::ColmagFr3TaskTrajectoryAdapter adapter(ros::NodeHandle(), ros::NodeHandle("~"));
  ros::spin();
  adapter.shutdown();
  return 0;
}
