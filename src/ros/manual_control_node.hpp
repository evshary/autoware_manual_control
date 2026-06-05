#ifndef _MANUAL_CONTROL_NODE_HPP_
#define _MANUAL_CONTROL_NODE_HPP_

#include "common/types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/manual_operator_heartbeat.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
using LocalizationInitializationState =
    autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
using autoware_control_msgs::msg::Control;
using autoware_vehicle_msgs::msg::GearCommand;

using autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat;
using autoware_vehicle_msgs::msg::GearReport;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::PoseWithCovarianceStamped;

namespace autoware::manual_control {

// Commands go to /external/selected/control_cmd, the gate input both LOCAL and
// REMOTE forward, so one publish path drives in either AD-API operation mode.
class ManualControlNode : public rclcpp::Node {
public:
  ManualControlNode()
      : Node("ManualControl",
             rclcpp::NodeOptions()
                 .allow_undeclared_parameters(true)
                 .automatically_declare_parameters_from_overrides(true)) {
    // Drive mode this operator commands (local | remote); STOP is the idle state.
    std::string mode_str;
    this->get_parameter_or<std::string>("operator_mode", mode_str, "remote");
    operator_mode_ = parse_operator_mode(mode_str);

    pub_control_command_ = this->create_publisher<Control>(
        "/external/selected/control_cmd", rclcpp::QoS(1).transient_local());
    pub_gear_cmd_ =
        this->create_publisher<GearCommand>("/external/selected/gear_cmd", 1);
    pub_initialpose_ = this->create_publisher<PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(1));

    // Heartbeat the operator's mode so Autoware keeps it available.
    pub_heartbeat_ = this->create_publisher<ManualOperatorHeartbeat>(
        operator_mode_ == OperationModeState::LOCAL ? "/external/local/heartbeat"
                                                    : "/external/remote/heartbeat",
        1);
    heartbeat_timer_ = this->create_wall_timer(100ms, [this]() {
      ManualOperatorHeartbeat msg;
      msg.stamp = this->get_clock()->now();
      msg.ready = true;
      pub_heartbeat_->publish(msg);
    });

    cli_stop_ = this->create_client<ChangeOperationMode>(
        "/api/operation_mode/change_to_stop");
    cli_drive_ = this->create_client<ChangeOperationMode>(
        operator_mode_ == OperationModeState::LOCAL
            ? "/api/operation_mode/change_to_local"
            : "/api/operation_mode/change_to_remote");
    // Entering the drive mode self-engages via this service; no separate engage step.
    cli_enable_control_ = this->create_client<ChangeOperationMode>(
        "/api/operation_mode/enable_autoware_control");

    sub_velocity_ = this->create_subscription<VelocityReport>(
        "/vehicle/status/velocity_status", 1,
        std::bind(&ManualControlNode::onVelocity, this, _1));
    sub_gear_ = this->create_subscription<GearReport>(
        "/vehicle/status/gear_status", 10,
        std::bind(&ManualControlNode::onGear, this, _1));
    sub_steering_ = this->create_subscription<SteeringReport>(
        "/vehicle/status/steering_status", 1,
        std::bind(&ManualControlNode::onSteering, this, _1));

    rclcpp::QoS qos_state(1);
    qos_state.transient_local().reliable();
    sub_op_mode_ = this->create_subscription<OperationModeState>(
        "/api/operation_mode/state", qos_state,
        std::bind(&ManualControlNode::onOperationModeState, this, _1));
    sub_loc_state_ = this->create_subscription<LocalizationInitializationState>(
        "/api/localization/initialization_state", qos_state,
        [this](const LocalizationInitializationState::ConstSharedPtr msg) {
          loc_initialized_ =
              (msg->state == LocalizationInitializationState::INITIALIZED);
        });

    init_parameters();
  }

  std::string operationModeName() const { return mode_name(current_op_mode_); }

  // Re-publishing /initialpose re-jumps the vehicle, so skip if already localised.
  bool is_localized() const { return loc_initialized_; }

  // Toggle between STOP and the operator's drive mode (entering drive self-engages).
  void toggle_operation_mode() {
    if (current_op_mode_ == operator_mode_) {
      request_operation_mode(OperationModeState::STOP);
    } else if (avail_drive_) {
      request_operation_mode(operator_mode_);
    } else {
      set_info_message(mode_name(operator_mode_) + " mode unavailable");
    }
  }

  VehicleState get_vehicle_state() const {
    VehicleState s;
    s.velocity = current_velocity_;
    s.steer_angle = current_steer_angle_;
    switch (current_gear_type_) {
    case GearReport::PARK:
      s.gear = Gear::PARK;
      break;
    case GearReport::REVERSE:
      s.gear = Gear::REVERSE;
      break;
    case GearReport::DRIVE:
      s.gear = Gear::DRIVE;
      break;
    case GearReport::LOW:
      s.gear = Gear::LOW;
      break;
    default:
      s.gear = Gear::NONE;
      break;
    }
    s.is_engaged = control_enabled_;
    return s;
  }

  void publish_command(const ControlCommand &cmd) {
    Control msg;
    msg.stamp = this->get_clock()->now();
    msg.longitudinal.velocity = cmd.velocity;
    msg.longitudinal.acceleration = cmd.acceleration;
    msg.lateral.steering_tire_angle = cmd.steer_angle;
    pub_control_command_->publish(msg);
    publish_gear(target_gear_);
  }

  void publish_gear(Gear gear) {
    GearCommand msg;
    msg.stamp = this->get_clock()->now();
    switch (gear) {
    case Gear::PARK:
      msg.command = GearCommand::PARK;
      break;
    case Gear::REVERSE:
      msg.command = GearCommand::REVERSE;
      break;
    case Gear::DRIVE:
      msg.command = GearCommand::DRIVE;
      break;
    case Gear::LOW:
      msg.command = GearCommand::LOW;
      break;
    default:
      msg.command = GearCommand::NONE;
      break;
    }
    pub_gear_cmd_->publish(msg);
  }

  void set_target_gear(Gear gear) { target_gear_ = gear; }

  void reset_initial_pose() {
    if (preset_names_.empty())
      return;

    current_preset_index_ = (current_preset_index_ + 1) % preset_names_.size();
    std::string current_name = preset_names_[current_preset_index_];

    std::vector<double> pose_data;
    std::string param_name = "init_pose.presets." + current_name;
    if (has_parameter(param_name)) {
      pose_data = get_parameter(param_name).as_double_array();
    } else {
      pose_data = {0.0, 0.0, 0.0, 0.0};
    }

    if (pose_data.size() < 4)
      return;

    PoseWithCovarianceStamped pose;
    pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = pose_data[0];
    pose.pose.pose.position.y = pose_data[1];
    pose.pose.pose.position.z = pose_data[2];
    double yaw = pose_data[3];
    pose.pose.pose.orientation.z = sin(yaw * 0.5);
    pose.pose.pose.orientation.w = cos(yaw * 0.5);

    for (size_t i = 0; i < 36; ++i)
      pose.pose.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;
    pub_initialpose_->publish(pose);

    set_info_message("[ManualControl]: Preset: " + current_name);
  }

  std::string get_info_message() { return info_message_; }

  void set_info_message(const std::string &msg) {
    info_message_ = msg;
    info_message_time_ = std::chrono::steady_clock::now();
  }

private:
  static uint8_t parse_operator_mode(const std::string &s) {
    return s == "local" ? OperationModeState::LOCAL : OperationModeState::REMOTE;
  }

  static std::string mode_name(uint8_t mode) {
    switch (mode) {
    case OperationModeState::STOP:
      return "STOP";
    case OperationModeState::AUTONOMOUS:
      return "AUTONOMOUS";
    case OperationModeState::LOCAL:
      return "LOCAL";
    case OperationModeState::REMOTE:
      return "REMOTE";
    default:
      return "UNKNOWN";
    }
  }

  // One request per call, never a retry loop; outcome shown on the info line.
  void request_operation_mode(uint8_t mode) {
    auto client = (mode == OperationModeState::STOP) ? cli_stop_ : cli_drive_;
    const std::string name = mode_name(mode);
    if (!client || !client->service_is_ready()) {
      set_info_message("Operation mode " + name + ": service unavailable");
      return;
    }
    auto req = std::make_shared<ChangeOperationMode::Request>();
    client->async_send_request(
        req, [this, name](rclcpp::Client<ChangeOperationMode>::SharedFuture fut) {
          auto res = fut.get();
          if (res && res->status.success)
            set_info_message("Operation mode -> " + name);
          else
            set_info_message("Operation mode " + name + " failed: " +
                             (res ? res->status.message : "no response"));
        });
  }

  void init_parameters() {
    if (!this->has_parameter("init_pose.presets.names")) {
      this->declare_parameter("init_pose.presets.names",
                              std::vector<std::string>{"origin"});
    }
    if (!this->has_parameter("init_pose.presets.origin")) {
      this->declare_parameter("init_pose.presets.origin",
                              std::vector<double>{0.0, 0.0, 0.0, 0.0});
    }
    preset_names_ =
        this->get_parameter("init_pose.presets.names").as_string_array();
  }

  void onVelocity(const VelocityReport::ConstSharedPtr msg) {
    current_velocity_ = msg->longitudinal_velocity;
  }
  void onGear(const GearReport::ConstSharedPtr msg) {
    current_gear_type_ = msg->report;
  }
  void onSteering(const SteeringReport::ConstSharedPtr msg) {
    current_steer_angle_ = msg->steering_tire_angle;
  }

  void onOperationModeState(const OperationModeState::ConstSharedPtr msg) {
    current_op_mode_ = msg->mode;
    avail_drive_ = (operator_mode_ == OperationModeState::LOCAL)
                       ? msg->is_local_mode_available
                       : msg->is_remote_mode_available;
    control_enabled_ = msg->is_autoware_control_enabled;

    // Self-engage once in the drive mode but control is not yet enabled.
    if (msg->mode == operator_mode_ && !msg->is_in_transition && !control_enabled_)
      enable_autoware_control();
  }

  void enable_autoware_control() {
    auto now = std::chrono::steady_clock::now();
    if (now - last_enable_request_ < 1s)
      return;
    if (!cli_enable_control_ || !cli_enable_control_->service_is_ready())
      return;
    last_enable_request_ = now;
    auto req = std::make_shared<ChangeOperationMode::Request>();
    cli_enable_control_->async_send_request(req);
  }

  rclcpp::Publisher<Control>::SharedPtr pub_control_command_;
  rclcpp::Publisher<GearCommand>::SharedPtr pub_gear_cmd_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;
  rclcpp::Publisher<ManualOperatorHeartbeat>::SharedPtr pub_heartbeat_;

  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_;
  rclcpp::Subscription<GearReport>::SharedPtr sub_gear_;
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_;

  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_stop_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_drive_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_enable_control_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_op_mode_;
  rclcpp::Subscription<LocalizationInitializationState>::SharedPtr sub_loc_state_;
  bool loc_initialized_ = false;
  uint8_t current_op_mode_ = OperationModeState::UNKNOWN;
  bool avail_drive_ = false;
  uint8_t operator_mode_ = OperationModeState::REMOTE;
  bool control_enabled_ = false;
  std::chrono::steady_clock::time_point last_enable_request_;

  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  uint8_t current_gear_type_ = GearReport::PARK;
  double current_velocity_ = 0.0;
  double current_steer_angle_ = 0.0;

  std::vector<std::string> preset_names_;
  int current_preset_index_ = -1;

  Gear target_gear_ = Gear::PARK;

  std::string info_message_;
  std::chrono::steady_clock::time_point info_message_time_;
};

} // namespace autoware::manual_control

#endif /*_MANUAL_CONTROL_NODE_HPP_*/
