#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <rclcpp/rclcpp.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>

using tier4_control_msgs::msg::GateMode;
using EngageSrv = tier4_external_api_msgs::srv::Engage;

class ManualControlNode : public rclcpp::Node
{
  public:
    ManualControlNode(): Node("ManualControl")
    {
      pub_gate_mode_ = this->create_publisher<GateMode>("/control/gate_mode_cmd", rclcpp::QoS(1));
      client_engage_ = this->create_client<EngageSrv>("/api/autoware/set/engage", rmw_qos_profile_services_default);
    }
    void update_status() {
      // enable GateMode
      pub_gate_mode_->publish(tier4_control_msgs::build<GateMode>().data(GateMode::EXTERNAL));
      // Enage
      auto req = std::make_shared<EngageSrv::Request>();
      req->engage = true;
      if (!client_engage_->service_is_ready()) {
        RCLCPP_INFO(this->get_logger(), "client is unavailable");
        return;
      }
      client_engage_->async_send_request(req, []([[maybe_unsed]] rclcpp::Client<EngageSrv>::SharedFuture result) {});
    }
  private:
    rclcpp::Publisher<GateMode>::SharedPtr pub_gate_mode_;
    rclcpp::Client<EngageSrv>::SharedPtr client_engage_;
};

class TerminalSettings
{
  public:
    TerminalSettings() {
      tcgetattr(STDIN_FILENO, &origin_settings);
    }
    void enable_read_keyboard() {
      new_settings = origin_settings;
      new_settings.c_lflag &= ~ICANON; // Non-canonical mode
      new_settings.c_cc[VTIME] = 0;    // No timeout
      new_settings.c_cc[VMIN] = 1;     // Return while reading 1 char
      tcsetattr(STDIN_FILENO, TCSADRAIN, &new_settings);
    }
    void restore_terminal() {
      tcsetattr(STDIN_FILENO, TCSADRAIN, &origin_settings);
    }
  private:
    struct termios origin_settings;
    struct termios new_settings;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManualControlNode>();
  TerminalSettings t_settings;
  while (rclcpp::ok()) {
    try {
      node->update_status();
      rclcpp::spin_some(node);
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_ERROR(node->get_logger(), "Unexpectedly failed with %s", e.what());
    }
  }
  rclcpp::shutdown();
  t_settings.restore_terminal();
  return 0;
}