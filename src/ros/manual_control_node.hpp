#ifndef _MANUAL_CONTROL_NODE_HPP_
#define _MANUAL_CONTROL_NODE_HPP_

#include "common/types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>

#include <autoware_adapi_v1_msgs/msg/manual_operator_heartbeat.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

using tier4_control_msgs::msg::GateMode;
using EngageSrv = tier4_external_api_msgs::srv::Engage;
using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_control_msgs::msg::Control;
using autoware_vehicle_msgs::msg::GearCommand;

using autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat;
using autoware_vehicle_msgs::msg::Engage;
using autoware_vehicle_msgs::msg::GearReport;
using autoware_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::PoseWithCovarianceStamped;

namespace autoware::manual_control {

// LOCAL drives the gate directly (/control/command/control_cmd, no heartbeat);
// REMOTE feeds external_cmd_selector and heartbeats /external/remote/heartbeat
// so vehicle_cmd_gate stays live.
enum class OperatorRole { LOCAL, REMOTE };

class ManualControlNode : public rclcpp::Node {
public:
  explicit ManualControlNode(OperatorRole role = OperatorRole::LOCAL)
      : Node("ManualControl",
             rclcpp::NodeOptions()
                 .allow_undeclared_parameters(true)
                 .automatically_declare_parameters_from_overrides(true)),
        role_(role) {
    // Publishers
    pub_gate_mode_ = this->create_publisher<GateMode>(
        "/control/gate_mode_cmd", rclcpp::QoS(1).transient_local());
    client_engage_ = this->create_client<EngageSrv>(
        "/api/autoware/set/engage", rmw_qos_profile_services_default);

    const std::string control_cmd_topic =
        (role == OperatorRole::REMOTE) ? "/external/selected/control_cmd"
                                       : "/control/command/control_cmd";
    pub_control_command_ = this->create_publisher<Control>(
        control_cmd_topic, rclcpp::QoS(1).transient_local());
    pub_gear_cmd_ =
        this->create_publisher<GearCommand>("/external/selected/gear_cmd", 1);
    pub_initialpose_ = this->create_publisher<PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(1));

    // heartbeat (REMOTE only)
    if (role == OperatorRole::REMOTE) {
      pub_heartbeat_ = this->create_publisher<ManualOperatorHeartbeat>(
          "/external/remote/heartbeat", 1);
      heartbeat_timer_ = this->create_wall_timer(100ms, [this]() {
        ManualOperatorHeartbeat msg;
        msg.stamp = this->get_clock()->now();
        msg.ready = true;
        pub_heartbeat_->publish(msg);
      });
    }

    // Subscribers
    sub_gate_mode_ = this->create_subscription<GateMode>(
        "/control/current_gate_mode", 10,
        std::bind(&ManualControlNode::onGateMode, this, _1));
    sub_engage_ = this->create_subscription<Engage>(
        "/api/autoware/get/engage", 10,
        std::bind(&ManualControlNode::onEngageStatus, this, _1));
    sub_velocity_ = this->create_subscription<VelocityReport>(
        "/vehicle/status/velocity_status", 1,
        std::bind(&ManualControlNode::onVelocity, this, _1));
    sub_gear_ = this->create_subscription<GearReport>(
        "/vehicle/status/gear_status", 10,
        std::bind(&ManualControlNode::onGear, this, _1));

    // REMOTE operator only: complete the standard Autoware engage handshake.
    // change_to_remote sets the target but does not engage; watch
    // operation_mode/state and call enable_autoware_control when the target
    // is REMOTE but control isn't enabled yet, rate-limited to 2s.
    if (role_ == OperatorRole::REMOTE) {
      enable_client_ = this->create_client<ChangeOperationMode>(
          "/api/operation_mode/enable_autoware_control");
      rclcpp::QoS qos_state(1);
      qos_state.transient_local().reliable();
      sub_op_mode_ = this->create_subscription<OperationModeState>(
          "/api/operation_mode/state", qos_state,
          std::bind(&ManualControlNode::onOperationModeState, this, _1));
    }

    init_parameters();

    // Timer for state monitoring (Auto-Engage / Auto-Mode-Switch)
    monitor_timer_ = this->create_wall_timer(
        1s, std::bind(&ManualControlNode::monitor_state, this));

    if (should_start_external()) {
      pending_external_request_ = true;
    }
  }

  // --- API for Main Loop ---

  VehicleState get_vehicle_state() const {
    VehicleState s;
    s.velocity = current_velocity_;
    // Map ROS gear to Enum
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
    s.is_engaged = current_engage_;
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

  bool toggle_manual_control() {
    bool switch_to_external = (gate_mode_ != GateMode::EXTERNAL);
    if (gate_mode_ == GateMode::EXTERNAL) {
      pub_gate_mode_->publish(
          tier4_control_msgs::build<GateMode>().data(GateMode::AUTO));
    } else {
      pub_gate_mode_->publish(
          tier4_control_msgs::build<GateMode>().data(GateMode::EXTERNAL));
      auto req = std::make_shared<EngageSrv::Request>();
      req->engage = true;
      if (client_engage_->service_is_ready()) {
        client_engage_->async_send_request(req);
      }
    }
    return switch_to_external;
  }

  void set_target_gear(Gear gear) { target_gear_ = gear; }

  void force_external_mode() {
    pending_external_request_ = true;
    gate_mode_ = GateMode::EXTERNAL; // Optimistic update

    // Sync gear to avoid resetting to PARK
    switch (current_gear_type_) {
    case GearReport::DRIVE:
      target_gear_ = Gear::DRIVE;
      break;
    case GearReport::REVERSE:
      target_gear_ = Gear::REVERSE;
      break;
    case GearReport::LOW:
      target_gear_ = Gear::LOW;
      break;
    case GearReport::PARK:
    default:
      target_gear_ = Gear::PARK;
      break;
    }

    pub_gate_mode_->publish(
        tier4_control_msgs::build<GateMode>().data(GateMode::EXTERNAL));
    auto req = std::make_shared<EngageSrv::Request>();
    req->engage = true;
    if (client_engage_->service_is_ready()) {
      client_engage_->async_send_request(req);
    }
  }

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

  bool should_start_external() const {
    return this->get_parameter("start_as_external").as_bool();
  }

  std::string get_info_message() { return info_message_; }

  void set_info_message(const std::string &msg) {
    info_message_ = msg;
    info_message_time_ = std::chrono::steady_clock::now();
  }

private:
  void init_parameters() {
    if (!this->has_parameter("start_as_external")) {
      this->declare_parameter("start_as_external", false);
    }
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

  void onGateMode(const GateMode::ConstSharedPtr msg) {
    gate_mode_ = msg->data;
  }
  void onEngageStatus(const Engage::ConstSharedPtr msg) {
    current_engage_ = msg->engage;
  }
  void onVelocity(const VelocityReport::ConstSharedPtr msg) {
    current_velocity_ = msg->longitudinal_velocity;
  }
  void onGear(const GearReport::ConstSharedPtr msg) {
    current_gear_type_ = msg->report;
  }

  void onOperationModeState(const OperationModeState::ConstSharedPtr msg) {
    if (msg->mode != OperationModeState::REMOTE) return;
    if (msg->is_autoware_control_enabled) return;
    if (msg->is_in_transition) return;

    const auto now = std::chrono::steady_clock::now();
    if (now - last_enable_attempt_ < std::chrono::seconds(2)) return;
    last_enable_attempt_ = now;

    if (!enable_client_ ||
        !enable_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN(this->get_logger(),
          "enable_autoware_control: service not yet available");
      return;
    }
    auto req = std::make_shared<ChangeOperationMode::Request>();
    enable_client_->async_send_request(req,
        [this](rclcpp::Client<ChangeOperationMode>::SharedFuture fut) {
          auto res = fut.get();
          if (res && res->status.success) {
            RCLCPP_INFO(this->get_logger(),
                "enable_autoware_control: success");
          } else {
            RCLCPP_WARN(this->get_logger(),
                "enable_autoware_control: failed (%s)",
                res ? res->status.message.c_str() : "null response");
          }
        });
  }

  void monitor_state() {
    // 1. Handle Pending External Mode Request (Startup or user intent)
    if (pending_external_request_) {
      if (gate_mode_ != GateMode::EXTERNAL) {
        // Retry publishing External Mode
        pub_gate_mode_->publish(
            tier4_control_msgs::build<GateMode>().data(GateMode::EXTERNAL));

        // Also queue engage
        auto req = std::make_shared<EngageSrv::Request>();
        req->engage = true;
        if (client_engage_->service_is_ready()) {
          client_engage_->async_send_request(req);
        }
      } else {
        // We achieved External Mode
        pending_external_request_ = false;
      }
    }

    // 2. Maintain Engage State (if we are in External)
    if (gate_mode_ == GateMode::EXTERNAL && !current_engage_) {
      auto req = std::make_shared<EngageSrv::Request>();
      req->engage = true;
      if (client_engage_->service_is_ready())
        client_engage_->async_send_request(req);
    }
  }

  const OperatorRole role_;

  rclcpp::Publisher<GateMode>::SharedPtr pub_gate_mode_;
  rclcpp::Client<EngageSrv>::SharedPtr client_engage_;
  rclcpp::Publisher<Control>::SharedPtr pub_control_command_;
  rclcpp::Publisher<GearCommand>::SharedPtr pub_gear_cmd_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;
  rclcpp::Publisher<ManualOperatorHeartbeat>::SharedPtr pub_heartbeat_; // REMOTE only

  rclcpp::Subscription<GateMode>::SharedPtr sub_gate_mode_;
  rclcpp::Subscription<Engage>::SharedPtr sub_engage_;
  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_;
  rclcpp::Subscription<GearReport>::SharedPtr sub_gear_;

  // REMOTE only — completes the engage handshake (enable_autoware_control).
  rclcpp::Client<ChangeOperationMode>::SharedPtr enable_client_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_op_mode_;
  std::chrono::steady_clock::time_point last_enable_attempt_ =
      std::chrono::steady_clock::now() - std::chrono::seconds(60);

  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_; // REMOTE only

  // Internal State cache
  uint8_t gate_mode_ = GateMode::AUTO;
  bool current_engage_ = false;
  uint8_t current_gear_type_ = GearReport::PARK;
  double current_velocity_ = 0.0;

  std::vector<std::string> preset_names_;
  int current_preset_index_ = -1;

  Gear target_gear_ = Gear::PARK;

  std::string info_message_;
  std::chrono::steady_clock::time_point info_message_time_;

  bool pending_external_request_ = false;
};

} // namespace autoware::manual_control

#endif /*_MANUAL_CONTROL_NODE_HPP_*/
