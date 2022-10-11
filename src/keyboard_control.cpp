#include <iostream>
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManualControlNode>();
  while (rclcpp::ok()) {
    try {
      node->update_status();
      rclcpp::spin_some(node);
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_ERROR(node->get_logger(), "Unexpectedly failed with %s", e.what());
    }
  }
  rclcpp::shutdown();
  return 0;
}