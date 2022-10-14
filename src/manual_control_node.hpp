#ifndef _MANUAL_CONTROL_NODE_HPP_
#define _MANUAL_CONTROL_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>

#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

using tier4_control_msgs::msg::GateMode;
using EngageSrv = tier4_external_api_msgs::srv::Engage;
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;

using autoware_auto_vehicle_msgs::msg::Engage;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using autoware_auto_vehicle_msgs::msg::GearReport;


class ManualControlNode : public rclcpp::Node
{
  public:
    ManualControlNode(): Node("ManualControl")
    {
      // init variables
      gear_type_ = GearCommand::DRIVE;
      steering_tire_angle_ = 0;
      target_velocity_ = 0;

      // init handler
      pub_gate_mode_ = this->create_publisher<GateMode>(
        "/control/gate_mode_cmd", rclcpp::QoS(1));
      client_engage_ = this->create_client<EngageSrv>(
        "/api/autoware/set/engage", rmw_qos_profile_services_default);
      pub_control_command_ = this->create_publisher<AckermannControlCommand>(
        "/external/selected/control_cmd", rclcpp::QoS(1));
      pub_gear_cmd_ = this->create_publisher<GearCommand>(
        "/external/selected/gear_cmd", 1);

      sub_gate_mode_ = this->create_subscription<GateMode>(
        "/control/current_gate_mode", 10, std::bind(&ManualControlNode::onGateMode, this, _1));
      sub_engage_ = this->create_subscription<Engage>(
        "/api/autoware/get/engage", 10, std::bind(&ManualControlNode::onEngageStatus, this, _1));
      sub_velocity_ = this->create_subscription<VelocityReport>(
        "/vehicle/status/velocity_status", 1, std::bind(&ManualControlNode::onVelocity, this, _1));
      sub_gear_ = this->create_subscription<GearReport>(
        "/vehicle/status/gear_status", 10, std::bind(&ManualControlNode::onGear, this, _1));
      
      // 30 Hz
      timer_ = this->create_wall_timer(33ms, std::bind(&ManualControlNode::publish_cmd, this));
    }
    void toggle_manual_control()
    {
      if (gate_mode_ == GateMode::EXTERNAL) {
        // Set GateMode to auto
        pub_gate_mode_->publish(tier4_control_msgs::build<GateMode>().data(GateMode::AUTO));
      } else {
        // Set GateMode to external
        pub_gate_mode_->publish(tier4_control_msgs::build<GateMode>().data(GateMode::EXTERNAL));
        // Engage
        auto req = std::make_shared<EngageSrv::Request>();
        req->engage = true;
        if (!client_engage_->service_is_ready()) {
          RCLCPP_INFO(this->get_logger(), "client is unavailable");
          return;
        }
        client_engage_->async_send_request(req);
      }
    }
    void update_gear_cmd(uint8_t type)
    {
      gear_type_ = type;
    }
    void update_control_cmd(double velocity, double angle)
    {
      target_velocity_ = velocity;
      steering_tire_angle_ = angle;
    }
    std::string get_status()
    {
      std::string status = "Engage:";
      status += (current_engage_)? "Ready":"Not Ready";
      status += "\tGate Mode:";
      switch (gate_mode_) {
        case GateMode::AUTO:
          status += "Auto";
          break;
        case GateMode::EXTERNAL:
          status += "External";
          break;
        default:
          status += "Unknown";
          break;
      }
      status += "\tGear:";
      switch (gear_type_) {
        case GearReport::PARK:
          status += "P";
          break;
        case GearReport::REVERSE:
          status += "R";
          break;
        case GearReport::DRIVE:
          status += "D";
          break;
        case GearReport::LOW:
          status += "L";
          break;
      }
      return status;
    }
  private:
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
    void publish_cmd()
    {
      AckermannControlCommand ackermann;
      {
        ackermann.lateral.steering_tire_angle = steering_tire_angle_;
        ackermann.longitudinal.speed = 0;
        double acceleration = std::clamp((target_velocity_ - current_velocity_) * 0.5, -1.0, 1.0);
        ackermann.longitudinal.acceleration = acceleration;
      }
      GearCommand gear_cmd;
      {
        gear_cmd.command = gear_type_;
      }
      pub_control_command_->publish(ackermann);
      pub_gear_cmd_->publish(gear_cmd);
    }

    rclcpp::Publisher<GateMode>::SharedPtr pub_gate_mode_;
    rclcpp::Client<EngageSrv>::SharedPtr client_engage_;
    rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_control_command_;
    rclcpp::Publisher<GearCommand>::SharedPtr pub_gear_cmd_;

    rclcpp::Subscription<GateMode>::SharedPtr sub_gate_mode_;
    rclcpp::Subscription<Engage>::SharedPtr sub_engage_;
    rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_;
    rclcpp::Subscription<GearReport>::SharedPtr sub_gear_;

    rclcpp::TimerBase::SharedPtr timer_;

    uint8_t gear_type_;
    double steering_tire_angle_;
    double target_velocity_;
    // status
    uint8_t gate_mode_;
    bool current_engage_;
    uint8_t current_gear_type_;
    double current_velocity_;
};

#endif /*_MANUAL_CONTROL_NODE_HPP_*/
