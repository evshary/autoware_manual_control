// Standalone keyboard teleop entry point.

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "core/control_runtime.hpp"
#include "core/drive_mode_factory.hpp"
#include "core/register_modes.hpp"
#include "input/input_system.hpp"
#include "ros/manual_control_node.hpp"
#include "ui/console_ui.hpp"

using namespace autoware::manual_control;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ManualControlNode>();
  register_all_modes(DriveModeFactory::instance(), *node);
  activate_modes_from_config(DriveModeFactory::instance(), *node);

  InputSystem input_system;
  ConsoleUI ui(input_system);
  ui.init();

  RuntimeConfig cfg = RuntimeConfig::load(*node);

  // Spin first so DDS discovers peers and the latched localization state lands.
  for (int i = 0; i < 20 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!node->is_localized())
    node->reset_initial_pose();

  run_control_runtime(node, input_system, ui, cfg);

  rclcpp::shutdown();
  return 0;
}
