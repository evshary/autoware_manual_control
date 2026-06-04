// Standalone keyboard teleop entry point.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "common/types.hpp"
#include "core/control_runtime.hpp"
#include "core/drive_mode_factory.hpp"
#include "core/mode_manager.hpp"
#include "input/input_system.hpp"
#include "modes/cruise_mode.hpp"
#include "modes/physics_mode.hpp"
#include "modes/stop_mode.hpp"
#include "ros/manual_control_node.hpp"
#include "ui/console_ui.hpp"

using namespace autoware::manual_control;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto &factory = DriveModeFactory::instance();
  factory.registerMode(ModeType::PHYSICS,
                       []() { return std::make_unique<PhysicsDriveMode>(); });
  factory.registerMode(ModeType::CRUISE,
                       []() { return std::make_unique<CruiseDriveMode>(); });
  factory.registerMode(ModeType::STOP,
                       []() { return std::make_unique<StopDriveMode>(); });

  auto node = std::make_shared<ManualControlNode>();
  InputSystem input_system;
  ConsoleUI ui(input_system);

  ui.init();

  RuntimeConfig cfg;
  run_control_runtime(node, input_system, ui, cfg);

  rclcpp::shutdown();
  return 0;
}
