#include <iostream>
#include <thread>
#include <unistd.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>

using namespace std::chrono_literals;

using tier4_control_msgs::msg::GateMode;
using EngageSrv = tier4_external_api_msgs::srv::Engage;
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;

class ManualControlNode : public rclcpp::Node
{
  public:
    ManualControlNode(): Node("ManualControl")
    {
      // init variables
      external_ = false;
      gear_type_ = GearCommand::DRIVE;
      acceleration_ = 0;
      steering_tire_angle_ = 0;

      // init handler
      pub_gate_mode_ = this->create_publisher<GateMode>("/control/gate_mode_cmd", rclcpp::QoS(1));
      client_engage_ = this->create_client<EngageSrv>("/api/autoware/set/engage", rmw_qos_profile_services_default);
      pub_control_command_ = this->create_publisher<AckermannControlCommand>("/external/selected/control_cmd", rclcpp::QoS(1));
      pub_gear_cmd_ = this->create_publisher<GearCommand>("/external/selected/gear_cmd", 1);

      // 30 Hz
      timer_ = this->create_wall_timer(33ms, std::bind(&ManualControlNode::publish_cmd, this));
    }
    void toggle_manual_control()
    {
      if (external_) {
        // Set GateMode to auto
        pub_gate_mode_->publish(tier4_control_msgs::build<GateMode>().data(GateMode::AUTO));
        external_ = false;
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
        external_ = true;
      }
    }
    void update_gear_cmd(uint8_t type)
    {
      gear_type_ = type;
    }
    void update_control_cmd(float acceleration, float angle)
    {
      acceleration_ = acceleration;
      steering_tire_angle_ = angle;
    }
  private:
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

    rclcpp::TimerBase::SharedPtr timer_;

    bool external_;
    uint8_t gear_type_;
    float acceleration_;
    float steering_tire_angle_;
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
      } else if (ch == 'i') {
        acceleration = std::clamp(acceleration + 0.1, 0.0, 1.0);
      } else if (ch == 'j') {
        angle = std::clamp(angle + 0.1, -22.5, 22.5);
      } else if (ch == 'l') {
        angle = std::clamp(angle - 0.1, -22.5, 22.5);
      } else if (ch == 'k') {
        acceleration = 0;
      }
      std::cout << angle << " " << acceleration << std::endl;
      node->update_control_cmd(acceleration, angle);
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