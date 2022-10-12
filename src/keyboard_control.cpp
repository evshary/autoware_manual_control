#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>
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
    void update_status()
    {
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

class TerminalReader
{
  public:
    TerminalReader()
    {
      tcgetattr(STDIN_FILENO, &origin_settings);
      // new terminal settings
      new_settings = origin_settings;
      new_settings.c_lflag &= ~ICANON; // Non-canonical mode
      new_settings.c_cc[VTIME] = 0;    // No timeout
      new_settings.c_cc[VMIN] = 1;     // Return while reading 1 char
      // timeout value
      timeout.tv_sec = 1;
      timeout.tv_usec = 0;
    }
    void configure_termnial()
    {
      tcsetattr(STDIN_FILENO, TCSADRAIN, &new_settings);
    }
    void restore_terminal()
    {
      tcsetattr(STDIN_FILENO, TCSADRAIN, &origin_settings);
    }
    int read_key()
    {
      int ch = 0;

      configure_termnial();
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(STDIN_FILENO, &fds);
      select(STDIN_FILENO+1, &fds, NULL, NULL, &timeout);
      if (FD_ISSET(STDIN_FILENO, &fds)) {
        ch = getchar();
      }
      restore_terminal();

      return ch;
    }
  private:
    struct termios origin_settings;
    struct termios new_settings;
    struct timeval timeout;
};

int g_thread_state;  // 1 means running, 0 means stop

void* read_keyboard(void *data)
{
  TerminalReader t_reader;
  while (g_thread_state) {
    int ch = t_reader.read_key();
    if (ch != 0) {
      std::cout << ch << std::endl;
    }
  }
  return NULL;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManualControlNode>();
  // Run keyboard thread
  pthread_t keyboard_thread;
  g_thread_state = 1;
  pthread_create(&keyboard_thread, NULL, read_keyboard, NULL);
  rclcpp::spin(node);
  rclcpp::shutdown();
  // Stop keyboard thread
  g_thread_state = 0; 
  pthread_join(keyboard_thread, NULL);
  return 0;
}