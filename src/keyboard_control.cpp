// Standalone keyboard teleop entry point.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "core/control_runtime.hpp"
#include "core/drive_mode_factory.hpp"
#include "core/params_loader.hpp"
#include "input/input_system.hpp"
#include "ros/manual_control_node.hpp"
#include "ui/console_ui.hpp"

using namespace autoware::manual_control;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ManualControlNode>();
  register_default_modes(DriveModeFactory::instance(), *node);

  InputSystem input_system;
  ConsoleUI ui(input_system);
  ui.init();

  RuntimeConfig cfg = load_runtime_config(*node);
  run_control_runtime(node, input_system, ui, cfg);

  rclcpp::shutdown();
  return 0;
}
