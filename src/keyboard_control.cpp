#include <iostream>
#include <thread>
#include <unistd.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>

#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>

#define MAX_STEER_ANGLE 0.3925 // 22.5 * (PI / 180)

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
      acceleration_ = 0;
      steering_tire_angle_ = 0;

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
    void update_control_cmd(double acceleration, double angle)
    {
      acceleration_ = acceleration;
      steering_tire_angle_ = angle;
    }
    void print_status()
    {
      std::cout << "Engage:";
      if (current_engage_) {
        std::cout << "Ready";
      } else {
        std::cout << "Not Ready";
      }
      std::cout << "\t";
      std::cout << "Gate Mode:";
      switch (gate_mode_) {
        case GateMode::AUTO:
          std::cout << "Auto";
          break;
        case GateMode::EXTERNAL:
          std::cout << "External";
          break;
        default:
          std::cout << "Unknown";
          break;
      }
      std::cout << "\t";
      std::cout << "Gear:";
      switch (gear_type_) {
        case GearReport::PARK:
          std::cout << "P";
          break;
        case GearReport::REVERSE:
          std::cout << "R";
          break;
        case GearReport::DRIVE:
          std::cout << "D";
          break;
        case GearReport::LOW:
          std::cout << "L";
          break;
      }
      std::cout << std::endl;
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
        ackermann.longitudinal.acceleration = acceleration_;
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
    double acceleration_;
    double steering_tire_angle_;
    // status
    uint8_t gate_mode_;
    bool current_engage_;
    uint8_t current_gear_type_;
    double current_velocity_;
};

class TerminalReader
{
  public:
    TerminalReader()
    {
      tcgetattr(STDIN_FILENO, &origin_settings_);
      // new terminal settings
      new_settings_ = origin_settings_;
      new_settings_.c_lflag &= ~(ICANON | ECHO | ECHOE); // Non-canonical mode
      new_settings_.c_cc[VTIME] = 0;    // No timeout_
      new_settings_.c_cc[VMIN] = 1;     // Return while reading 1 char
      // timeout_ value
      timeout_.tv_sec = 1;
      timeout_.tv_usec = 0;
    }
    void configure_termnial()
    {
      tcsetattr(STDIN_FILENO, TCSADRAIN, &new_settings_);
    }
    void restore_terminal()
    {
      tcsetattr(STDIN_FILENO, TCSADRAIN, &origin_settings_);
    }
    int read_key()
    {
      int ch = 0;

      configure_termnial();
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(STDIN_FILENO, &fds);
      select(STDIN_FILENO+1, &fds, NULL, NULL, &timeout_);
      if (FD_ISSET(STDIN_FILENO, &fds)) {
        ch = getchar();
      }
      restore_terminal();

      return ch;
    }
  private:
    struct termios origin_settings_;
    struct termios new_settings_;
    struct timeval timeout_;
};

int g_thread_state;  // 1 means running, 0 means stop

void read_keyboard(std::shared_ptr<ManualControlNode> node)
{
  TerminalReader t_reader;
  float acceleration = 0;
  float angle = 0;

  while (g_thread_state) {
    int ch = t_reader.read_key();
    if (ch != 0) {
      if (ch == 'z') {
        node->toggle_manual_control();
      } else if (ch == 'x') {
        node->update_gear_cmd(GearCommand::DRIVE);
      } else if (ch == 'c') {
        node->update_gear_cmd(GearCommand::REVERSE);
      } else if (ch == 'v') {
        node->update_gear_cmd(GearCommand::PARK);
      } else if (ch == 's') {
        node->print_status();
      } else {
        if (ch == 'i') {
          acceleration = std::clamp(acceleration + 0.1, 0.0, 1.0);
        } else if (ch == 'j') {
          angle = std::clamp(angle + 0.02, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
        } else if (ch == 'l') {
          angle = std::clamp(angle - 0.02, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
        } else if (ch == 'k') {
          angle = 0;
        } else {
          continue;
        }
        std::cout << "angle:" << angle * 180 / M_PI << "\tacceleration:" << acceleration << std::endl;
        node->update_control_cmd(acceleration, angle);
      }
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManualControlNode>();
  // Run keyboard thread
  g_thread_state = 1;
  std::thread keyboard_thread(read_keyboard, node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  // Stop keyboard thread
  g_thread_state = 0; 
  keyboard_thread.join();
  return 0;
}