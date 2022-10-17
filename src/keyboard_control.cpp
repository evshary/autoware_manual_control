#include <iostream>
#include <thread>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "manual_control_node.hpp"
#include "terminal_reader.hpp"

#define MAX_STEER_ANGLE  0.3925 // 22.5 * (PI / 180)
#define STEP_STEER_ANGLE 0.0174 // 1 * (PI / 180)
#define MAX_SPEED        27.78  // 100 km/hr = 27.78 m/s
#define STEP_SPEED       1.389  // 5 km/hr = 1.389 m/s

void print_help()
{
  std::cout << "------------------------------------" << std::endl;
  std::cout << "| Different Mode:                  |" << std::endl;
  std::cout << "|   z: Toggle auto & external mode |" << std::endl;
  std::cout << "|   x: GateMode => Drive           |" << std::endl;
  std::cout << "|   c: GateMode => Reverse         |" << std::endl;
  std::cout << "|   v: GateMode => Park            |" << std::endl;
  std::cout << "|   s: View current mode           |" << std::endl;
  std::cout << "| Speed:                           |" << std::endl;
  std::cout << "|   u: Increase speed              |" << std::endl;
  std::cout << "|   i: Set speed to 0              |" << std::endl;
  std::cout << "|   o: Decrease speed              |" << std::endl;
  std::cout << "| Steering Angle                   |" << std::endl;
  std::cout << "|   j: Left turn                   |" << std::endl;
  std::cout << "|   k: Set angle to 0              |" << std::endl;
  std::cout << "|   l: Right turn                  |" << std::endl;
  std::cout << "------------------------------------" << std::endl;
}

int g_thread_state;  // 1 means running, 0 means stop

void read_keyboard(std::shared_ptr<ManualControlNode> node)
{
  TerminalReader t_reader;
  double velocity = 0;  // m/s
  double angle = 0;     // radian

  t_reader.configure_termnial();
  print_help();
  while (g_thread_state) {
    int ch = t_reader.read_key();
    if (ch != 0) {
      if (ch == 'z') {
        std::string new_mode = (node->toggle_manual_control())? "EXTERNAL":"AUTO";
        std::cout << "Toggle to " << new_mode << std::endl;
      } else if (ch == 'x') {
        node->update_gear_cmd(GearCommand::DRIVE);
        std::cout << "Switch to DRIVE mode" << std::endl;
      } else if (ch == 'c') {
        node->update_gear_cmd(GearCommand::REVERSE);
        std::cout << "Switch to REVERSE mode" << std::endl;
      } else if (ch == 'v') {
        node->update_gear_cmd(GearCommand::PARK);
        std::cout << "Switch to PARK mode" << std::endl;
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
          print_help();
          continue;
        }
        std::cout << "angle(deg):" << angle * 180 / M_PI << "\tvelocity(km/hr):" << velocity * 3600 / 1000 << std::endl;
        node->update_control_cmd(velocity, angle);
      }
    }
  }
  t_reader.restore_terminal();
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
