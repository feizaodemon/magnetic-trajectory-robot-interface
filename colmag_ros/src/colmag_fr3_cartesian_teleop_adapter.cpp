#include "colmag_ros/fr3_cartesian_teleop_core.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace colmag_ros
{
namespace
{

constexpr const char* kTaskMode = "TASK";
constexpr const char* kTeleopMode = "TELEOP";
constexpr const char* kActiveHardwareProfile = "fr3-hardware-active";
constexpr const char* kMagneticBoardInput = "magnetic_board";
constexpr const char* kNormalizedCartesianInput = "normalized_cartesian";

std::string normalizedMode(const std::string& value)
{
  const std::string::size_type first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
  {
    return std::string();
  }
  const std::string::size_type last = value.find_last_not_of(" \t\r\n");
  std::string result = value.substr(first, last - first + 1);
  for (char& character : result)
  {
    character = static_cast<char>(
        std::toupper(static_cast<unsigned char>(character)));
  }
  if (result != kTaskMode && result != kTeleopMode)
  {
    return std::string();
  }
  return result;
}

template <std::size_t N>
bool readDoubleArray(ros::NodeHandle* node,
                     const std::string& name,
                     std::array<double, N>* output)
{
  if (!node->hasParam(name))
  {
    return true;
  }
  std::vector<double> values;
  if (!node->getParam(name, values) || values.size() != N)
  {
    return false;
  }
  for (std::size_t index = 0; index < N; ++index)
  {
    (*output)[index] = values[index];
  }
  return true;
}

std::string jsonEscape(const std::string& value)
{
  std::string result;
  result.reserve(value.size());
  for (char character : value)
  {
    if (character == '\\' || character == '"')
    {
      result.push_back('\\');
    }
    result.push_back(character);
  }
  return result;
}

}  // namespace

class Fr3CartesianTeleopAdapter
{
public:
  Fr3CartesianTeleopAdapter()
    : private_node_("~")
  {
    loadParameters();
    core_.reset(new Fr3CartesianTeleopCore(core_config_));
    std::string reason;
    if (!core_->validateConfiguration(&reason))
    {
      throw std::runtime_error("invalid Cartesian teleop configuration: " + reason);
    }

    command_publisher_ = node_.advertise<trajectory_msgs::JointTrajectory>(
        command_topic_, 1, false);
    state_publisher_ = node_.advertise<std_msgs::String>(state_topic_, 10, true);
    if (input_mode_ == kMagneticBoardInput)
    {
      input_subscriber_ = node_.subscribe(
          trajectory_topic_, 20,
          &Fr3CartesianTeleopAdapter::handleTrajectory, this);
    }
    else
    {
      input_subscriber_ = node_.subscribe(
          normalized_input_topic_, 20,
          &Fr3CartesianTeleopAdapter::handleNormalizedInput, this);
    }
    joint_state_subscriber_ = node_.subscribe(
        joint_state_topic_, 20,
        &Fr3CartesianTeleopAdapter::handleJointState, this);
    mode_subscriber_ = node_.subscribe(
        control_mode_topic_, 1,
        &Fr3CartesianTeleopAdapter::handleMode, this);
    timer_ = node_.createTimer(
        ros::Duration(command_period_sec_),
        &Fr3CartesianTeleopAdapter::tick, this);

    publishState("WAITING_FOR_MODE", "mode_state_not_observed", false);
    ROS_INFO_STREAM("colmag_fr3_cartesian_teleop_adapter started; hardware_active="
                    << (hardwareGateOpen() ? "true" : "false")
                    << " input_mode=" << input_mode_
                    << " command_topic=" << command_topic_);
  }

private:
  void loadParameters()
  {
    private_node_.param<std::string>("trajectory_topic", trajectory_topic_,
                                     "/colmag/trajectory_2d");
    private_node_.param<std::string>("normalized_input_topic", normalized_input_topic_,
                                     "/colmag/teleop/cartesian_input");
    private_node_.param<std::string>("input_mode", input_mode_, kMagneticBoardInput);
    private_node_.param<std::string>("joint_state_topic", joint_state_topic_,
                                     "/franka_state_controller/joint_states");
    private_node_.param<std::string>("control_mode_topic", control_mode_topic_,
                                     "/colmag/control_mode");
    private_node_.param<std::string>("command_topic", command_topic_,
                                     "/position_joint_trajectory_controller/command");
    private_node_.param<std::string>("state_topic", state_topic_,
                                     "/colmag/fr3_cartesian_teleop_state");
    private_node_.param<std::string>("arm_id", arm_id_, "fr3");
    private_node_.param("allow_real_robot", allow_real_robot_, false);
    private_node_.param("send_commands", send_commands_, false);
    private_node_.param("calibration_valid", calibration_valid_, false);
    private_node_.param("hardware_bringup_enabled", hardware_bringup_enabled_, false);
    private_node_.param("simulation_commands_enabled", simulation_commands_enabled_, false);
    private_node_.param<std::string>("hardware_profile", hardware_profile_, "dry-run");
    private_node_.param("input_freshness_timeout_sec", input_timeout_sec_, 0.25);
    private_node_.param("joint_state_freshness_timeout_sec", joint_timeout_sec_, 0.25);
    private_node_.param("joint_state_future_tolerance_sec", joint_future_tolerance_sec_, 0.05);
    private_node_.param("command_period_sec", command_period_sec_, 1.0 / 30.0);
    private_node_.param("command_time_from_start_sec", command_time_sec_, 0.10);

    private_node_.param("input_x_min", core_config_.input_x_min, core_config_.input_x_min);
    private_node_.param("input_x_max", core_config_.input_x_max, core_config_.input_x_max);
    private_node_.param("input_y_min", core_config_.input_y_min, core_config_.input_y_min);
    private_node_.param("input_y_max", core_config_.input_y_max, core_config_.input_y_max);
    private_node_.param("input_z_min", core_config_.input_z_min, core_config_.input_z_min);
    private_node_.param("input_z_max", core_config_.input_z_max, core_config_.input_z_max);
    private_node_.param("swap_xy", core_config_.swap_xy, core_config_.swap_xy);
    private_node_.param("map_x_sign", core_config_.map_x_sign, core_config_.map_x_sign);
    private_node_.param("map_y_sign", core_config_.map_y_sign, core_config_.map_y_sign);
    private_node_.param("map_z", core_config_.map_z, core_config_.map_z);
    private_node_.param("height_gain", core_config_.height_gain, core_config_.height_gain);
    private_node_.param("target_smoothing_tau_sec", core_config_.target_smoothing_tau_sec,
                        core_config_.target_smoothing_tau_sec);
    private_node_.param("max_linear_speed_m_s", core_config_.max_linear_speed_m_s,
                        core_config_.max_linear_speed_m_s);
    private_node_.param("max_linear_acceleration_m_s2",
                        core_config_.max_linear_acceleration_m_s2,
                        core_config_.max_linear_acceleration_m_s2);
    private_node_.param("max_linear_jerk_m_s3", core_config_.max_linear_jerk_m_s3,
                        core_config_.max_linear_jerk_m_s3);
    private_node_.param("s_curve_min_duration_sec", core_config_.s_curve_min_duration_sec,
                        core_config_.s_curve_min_duration_sec);
    private_node_.param("ik_max_iterations", core_config_.ik_max_iterations,
                        core_config_.ik_max_iterations);
    private_node_.param("dls_damping", core_config_.dls_damping, core_config_.dls_damping);
    private_node_.param("jacobian_epsilon", core_config_.jacobian_epsilon,
                        core_config_.jacobian_epsilon);
    private_node_.param("ik_position_tolerance_m", core_config_.ik_position_tolerance_m,
                        core_config_.ik_position_tolerance_m);
    private_node_.param("ik_orientation_tolerance_rad",
                        core_config_.ik_orientation_tolerance_rad,
                        core_config_.ik_orientation_tolerance_rad);
    private_node_.param("ik_position_step_limit_m", core_config_.ik_position_step_limit_m,
                        core_config_.ik_position_step_limit_m);
    private_node_.param("ik_orientation_step_limit_rad",
                        core_config_.ik_orientation_step_limit_rad,
                        core_config_.ik_orientation_step_limit_rad);
    private_node_.param("max_joint_step_rad", core_config_.max_joint_step_rad,
                        core_config_.max_joint_step_rad);
    private_node_.param("joint_velocity_filter_alpha",
                        core_config_.joint_velocity_filter_alpha,
                        core_config_.joint_velocity_filter_alpha);
    private_node_.param("joint_limit_margin_rad", core_config_.joint_limit_margin_rad,
                        core_config_.joint_limit_margin_rad);

    if (!readDoubleArray(&private_node_, "workspace_min", &core_config_.workspace_min) ||
        !readDoubleArray(&private_node_, "workspace_max", &core_config_.workspace_max) ||
        !readDoubleArray(&private_node_, "joint_lower", &core_config_.joint_lower) ||
        !readDoubleArray(&private_node_, "joint_upper", &core_config_.joint_upper))
    {
      throw std::runtime_error("workspace/joint limit parameters must be numeric fixed-size arrays");
    }
    if (input_mode_ != kMagneticBoardInput &&
        input_mode_ != kNormalizedCartesianInput)
    {
      throw std::runtime_error("input_mode must be magnetic_board or normalized_cartesian");
    }
    if (simulation_commands_enabled_ &&
        (hardware_bringup_enabled_ || allow_real_robot_ || send_commands_ ||
         calibration_valid_ || hardware_profile_ == kActiveHardwareProfile))
    {
      throw std::runtime_error(
          "simulation command mode cannot be combined with real-hardware gates");
    }
    if (!std::isfinite(input_timeout_sec_) || input_timeout_sec_ <= 0.0 ||
        !std::isfinite(joint_timeout_sec_) || joint_timeout_sec_ <= 0.0 ||
        !std::isfinite(joint_future_tolerance_sec_) || joint_future_tolerance_sec_ < 0.0 ||
        !std::isfinite(command_period_sec_) || command_period_sec_ <= 0.0 ||
        !std::isfinite(command_time_sec_) || command_time_sec_ <= command_period_sec_)
    {
      throw std::runtime_error("invalid freshness or command timing parameter");
    }
    joint_names_.clear();
    for (int index = 1; index <= 7; ++index)
    {
      joint_names_.push_back(arm_id_ + "_joint" + std::to_string(index));
    }
  }

  void handleMode(const std_msgs::String::ConstPtr& message)
  {
    const std::string selected = normalizedMode(message->data);
    if (selected.empty())
    {
      control_mode_.clear();
      mode_observed_ = false;
      input_after_mode_change_ = false;
      joint_after_mode_change_ = false;
      core_->reset();
      publishState("PAUSED", "invalid_control_mode", false);
      return;
    }
    if (!mode_observed_ || selected != control_mode_)
    {
      control_mode_ = selected;
      mode_observed_ = true;
      input_after_mode_change_ = false;
      joint_after_mode_change_ = false;
      core_->reset();
      publishState(selected == kTeleopMode ? "WAITING_FOR_INPUT" : "TASK_MODE",
                   selected == kTeleopMode ? "new_input_required" : "teleop_not_owner",
                   false);
    }
  }

  void handleTrajectory(const std_msgs::String::ConstPtr& message)
  {
    MagneticBoardSample parsed;
    long long sequence_id = 0;
    long long sample_index = 0;
    try
    {
      std::istringstream stream(message->data);
      boost::property_tree::ptree payload;
      boost::property_tree::read_json(stream, payload);
      const double timestamp = payload.get<double>("timestamp");
      sequence_id = payload.get<long long>("sequence_id");
      sample_index = payload.get<long long>("sample_index");
      parsed.x = payload.get<double>("x");
      parsed.y = payload.get<double>("y");
      parsed.z = payload.get<double>("z");
      parsed.interaction_engaged = payload.get<bool>("valid");
      if (!std::isfinite(timestamp) || !std::isfinite(parsed.x) ||
          !std::isfinite(parsed.y) || !std::isfinite(parsed.z))
      {
        throw std::runtime_error("non_finite_input");
      }
    }
    catch (const std::exception& exception)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Rejected trajectory_2d payload: " << exception.what());
      publishState("PAUSED", "invalid_magnetic_input", false);
      return;
    }
    if (have_input_identity_ &&
        (sequence_id < last_sequence_id_ ||
         (sequence_id == last_sequence_id_ && sample_index <= last_sample_index_)))
    {
      publishState("PAUSED", "non_monotonic_magnetic_input", false);
      return;
    }
    latest_sample_ = parsed;
    last_sequence_id_ = sequence_id;
    last_sample_index_ = sample_index;
    have_input_identity_ = true;
    have_input_ = true;
    last_input_receive_time_ = ros::WallTime::now();
    if (mode_observed_ && control_mode_ == kTeleopMode)
    {
      input_after_mode_change_ = true;
    }
    if (!parsed.interaction_engaged)
    {
      core_->reset();
      publishState("PAUSED", "interaction_not_engaged", false);
    }
  }

  void handleNormalizedInput(const std_msgs::String::ConstPtr& message)
  {
    NormalizedCartesianInput parsed;
    long long sequence_id = 0;
    try
    {
      std::istringstream stream(message->data);
      boost::property_tree::ptree payload;
      boost::property_tree::read_json(stream, payload);
      const double timestamp = payload.get<double>("timestamp");
      sequence_id = payload.get<long long>("sequence_id");
      parsed.direction[0] = payload.get<double>("x");
      parsed.direction[1] = payload.get<double>("y");
      parsed.direction[2] = payload.get<double>("z");
      parsed.interaction_engaged = payload.get<bool>("engaged");
      if (!std::isfinite(timestamp) ||
          !std::all_of(parsed.direction.begin(), parsed.direction.end(),
                       [](double value) { return std::isfinite(value); }))
      {
        throw std::runtime_error("non_finite_input");
      }
    }
    catch (const std::exception& exception)
    {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Rejected normalized Cartesian input: " << exception.what());
      publishState("PAUSED", "invalid_normalized_input", false);
      return;
    }
    if (have_input_identity_ && sequence_id <= last_sequence_id_)
    {
      publishState("PAUSED", "non_monotonic_normalized_input", false);
      return;
    }
    latest_normalized_input_ = parsed;
    last_sequence_id_ = sequence_id;
    last_sample_index_ = 0;
    have_input_identity_ = true;
    have_input_ = true;
    last_input_receive_time_ = ros::WallTime::now();
    if (mode_observed_ && control_mode_ == kTeleopMode)
    {
      input_after_mode_change_ = true;
    }
    if (!parsed.interaction_engaged)
    {
      core_->reset();
      publishState("PAUSED", "interaction_not_engaged", false);
    }
  }

  void handleJointState(const sensor_msgs::JointState::ConstPtr& message)
  {
    Fr3JointVector ordered{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    for (std::size_t required = 0; required < joint_names_.size(); ++required)
    {
      const std::vector<std::string>::const_iterator found =
          std::find(message->name.begin(), message->name.end(), joint_names_[required]);
      if (found == message->name.end())
      {
        publishState("PAUSED", "joint_state_names_mismatch", false);
        return;
      }
      const std::size_t index = static_cast<std::size_t>(found - message->name.begin());
      if (index >= message->position.size() || !std::isfinite(message->position[index]))
      {
        publishState("PAUSED", "invalid_joint_state", false);
        return;
      }
      ordered[required] = message->position[index];
    }
    latest_joints_ = ordered;
    latest_joint_stamp_ = message->header.stamp;
    last_joint_receive_time_ = ros::WallTime::now();
    have_joint_state_ = true;
    if (mode_observed_ && control_mode_ == kTeleopMode)
    {
      joint_after_mode_change_ = true;
    }
  }

  bool hardwareGateOpen() const
  {
    return hardware_bringup_enabled_ && allow_real_robot_ && send_commands_ &&
           calibration_valid_ &&
           hardware_profile_ == kActiveHardwareProfile;
  }

  bool commandGateOpen() const
  {
    return simulation_commands_enabled_ || hardwareGateOpen();
  }

  bool inputsFresh(std::string* reason, double* input_age, double* joint_age) const
  {
    const ros::WallTime wall_now = ros::WallTime::now();
    if (!have_input_)
    {
      *reason = input_mode_ == kMagneticBoardInput ?
                    "magnetic_input_missing" : "cartesian_input_missing";
      return false;
    }
    *input_age = (wall_now - last_input_receive_time_).toSec();
    if (!std::isfinite(*input_age) || *input_age > input_timeout_sec_)
    {
      *reason = input_mode_ == kMagneticBoardInput ?
                    "stale_magnetic_input" : "stale_cartesian_input";
      return false;
    }
    if (!joint_after_mode_change_)
    {
      *reason = "new_joint_state_required";
      return false;
    }
    if (!have_joint_state_ || latest_joint_stamp_.isZero())
    {
      *reason = "joint_state_missing";
      return false;
    }
    *joint_age = (wall_now - last_joint_receive_time_).toSec();
    const double header_age = (ros::Time::now() - latest_joint_stamp_).toSec();
    if (!std::isfinite(*joint_age) || *joint_age > joint_timeout_sec_ ||
        !std::isfinite(header_age) || header_age > joint_timeout_sec_ ||
        header_age < -joint_future_tolerance_sec_)
    {
      *reason = "stale_joint_state";
      return false;
    }
    reason->clear();
    return true;
  }

  void tick(const ros::TimerEvent&)
  {
    if (!mode_observed_ || control_mode_ != kTeleopMode)
    {
      core_->reset();
      publishState(mode_observed_ ? "TASK_MODE" : "WAITING_FOR_MODE",
                   mode_observed_ ? "teleop_not_owner" : "mode_state_not_observed",
                   false);
      return;
    }
    if (!input_after_mode_change_)
    {
      core_->reset();
      publishState("WAITING_FOR_INPUT", "new_input_required", false);
      return;
    }
    double input_age = 0.0;
    double joint_age = 0.0;
    std::string freshness_reason;
    if (!inputsFresh(&freshness_reason, &input_age, &joint_age))
    {
      core_->reset();
      publishState("PAUSED", freshness_reason, false, input_age, joint_age);
      return;
    }
    const Fr3CartesianTeleopStep result =
        input_mode_ == kMagneticBoardInput ?
            core_->step(latest_sample_, latest_joints_, command_period_sec_) :
            core_->stepNormalizedCartesian(
                latest_normalized_input_, latest_joints_, command_period_sec_);
    if (!result.accepted)
    {
      publishState("REJECTED", result.reason, false, input_age, joint_age);
      return;
    }
    if (!commandGateOpen())
    {
      publishState("DRY_RUN_TARGET", "execution_gates_closed", false,
                   input_age, joint_age);
      return;
    }

    trajectory_msgs::JointTrajectory command;
    command.header.stamp = ros::Time::now();
    command.joint_names = joint_names_;
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.assign(result.joint_positions.begin(), result.joint_positions.end());
    point.velocities.assign(result.joint_velocities.begin(), result.joint_velocities.end());
    point.time_from_start = ros::Duration(command_time_sec_);
    command.points.push_back(point);
    command_publisher_.publish(command);
    publishState("COMMAND_PUBLISHED", "bounded_cartesian_target", true,
                 input_age, joint_age);
  }

  void publishState(const std::string& state,
                    const std::string& reason,
                    bool command_published,
                    double input_age = -1.0,
                    double joint_age = -1.0)
  {
    std::ostringstream payload;
    payload << std::fixed << std::setprecision(4)
            << "{\"state\":\"" << jsonEscape(state)
            << "\",\"reason\":\"" << jsonEscape(reason)
            << "\",\"control_mode\":\"" << jsonEscape(control_mode_)
            << "\",\"input_mode\":\"" << jsonEscape(input_mode_)
            << "\",\"hardware_active\":" << (hardwareGateOpen() ? "true" : "false")
            << ",\"simulation_commands_enabled\":"
            << (simulation_commands_enabled_ ? "true" : "false")
            << ",\"command_published\":" << (command_published ? "true" : "false")
            << ",\"input_age_sec\":" << input_age
            << ",\"joint_state_age_sec\":" << joint_age << "}";
    std_msgs::String message;
    message.data = payload.str();
    state_publisher_.publish(message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher command_publisher_;
  ros::Publisher state_publisher_;
  ros::Subscriber input_subscriber_;
  ros::Subscriber joint_state_subscriber_;
  ros::Subscriber mode_subscriber_;
  ros::Timer timer_;

  Fr3CartesianTeleopConfig core_config_;
  std::unique_ptr<Fr3CartesianTeleopCore> core_;
  std::string trajectory_topic_;
  std::string normalized_input_topic_;
  std::string input_mode_;
  std::string joint_state_topic_;
  std::string control_mode_topic_;
  std::string command_topic_;
  std::string state_topic_;
  std::string arm_id_;
  std::string hardware_profile_;
  std::vector<std::string> joint_names_;
  bool allow_real_robot_ = false;
  bool send_commands_ = false;
  bool calibration_valid_ = false;
  bool hardware_bringup_enabled_ = false;
  bool simulation_commands_enabled_ = false;
  double input_timeout_sec_ = 0.25;
  double joint_timeout_sec_ = 0.25;
  double joint_future_tolerance_sec_ = 0.05;
  double command_period_sec_ = 1.0 / 30.0;
  double command_time_sec_ = 0.10;

  std::string control_mode_;
  bool mode_observed_ = false;
  bool input_after_mode_change_ = false;
  bool joint_after_mode_change_ = false;
  MagneticBoardSample latest_sample_;
  NormalizedCartesianInput latest_normalized_input_;
  Fr3JointVector latest_joints_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ros::Time latest_joint_stamp_;
  ros::WallTime last_input_receive_time_;
  ros::WallTime last_joint_receive_time_;
  bool have_input_ = false;
  bool have_joint_state_ = false;
  bool have_input_identity_ = false;
  long long last_sequence_id_ = 0;
  long long last_sample_index_ = 0;
};

}  // namespace colmag_ros

int main(int argc, char** argv)
{
  ros::init(argc, argv, "colmag_fr3_cartesian_teleop_adapter");
  try
  {
    colmag_ros::Fr3CartesianTeleopAdapter adapter;
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL_STREAM("Failed to start Cartesian teleop adapter: " << exception.what());
    return 2;
  }
  return 0;
}
