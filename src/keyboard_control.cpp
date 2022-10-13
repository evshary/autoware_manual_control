#include <iostream>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "manual_control_node.hpp"
#include "terminal_reader.hpp"

#define MAX_STEER_ANGLE  0.3925 // 22.5 * (PI / 180)
#define STEP_STEER_ANGLE 0.0174 // 1 * (PI / 180)
#define MAX_SPEED        27.78  // 100 km/hr = 27.78 m/s
#define STEP_SPEED       1.389  // 5 km/hr = 1.389 m/s

int g_thread_state;  // 1 means running, 0 means stop

void read_keyboard(std::shared_ptr<ManualControlNode> node)
{
  TerminalReader t_reader;
  double velocity = 0;  // m/s
  double angle = 0;     // radian

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
        std::cout << node->get_status() << std::endl;
      } else {
        if (ch == 'u') {
          velocity = std::clamp(velocity + STEP_SPEED, 0.0, MAX_SPEED);
        } else if (ch == 'o') {
          velocity = std::clamp(velocity - STEP_SPEED, 0.0, MAX_SPEED);
        } else if (ch == 'i') {
          velocity = 0.0;
        } else if (ch == 'j') {
          angle = std::clamp(angle + STEP_STEER_ANGLE, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
        } else if (ch == 'l') {
          angle = std::clamp(angle - STEP_STEER_ANGLE, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
        } else if (ch == 'k') {
          angle = 0;
        } else {
          continue;
        }
        std::cout << "angle(deg):" << angle * 180 / M_PI << "\tvelocity(km/hr):" << velocity * 3600 / 1000 << std::endl;
        node->update_control_cmd(velocity, angle);
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
