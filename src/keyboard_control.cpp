// Standalone keyboard teleop entry point.

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "core/control_runtime.hpp"
#include "core/drive_mode_factory.hpp"
#include "core/register_modes.hpp"
#include "io/intent/keyboard.hpp"
#include "io/telemetry/console.hpp"
#include "ros/manual_control_node.hpp"

using namespace autoware::manual_control;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ManualControlNode>();
  register_all_modes(DriveModeFactory::instance(), *node);
  activate_modes_from_config(DriveModeFactory::instance(), *node);

  KeyboardIntent keyboard;
  ConsoleTelemetry console;
  console.init();

  // Live WASD echo (UPPER=held, lower=tapped, '.'=idle).
  console.set_extra_line([&keyboard] {
    auto glyph = [](KeyHold h, char c) {
      return h == KeyHold::Held     ? c
             : h == KeyHold::Tapped ? static_cast<char>(c + 32)
                                    : '.';
    };
    return std::string("Keys: [") + glyph(keyboard.w_state(), 'W') +
           glyph(keyboard.a_state(), 'A') + glyph(keyboard.s_state(), 'S') +
           glyph(keyboard.d_state(), 'D') + "]";
  });

  RuntimeConfig cfg = RuntimeConfig::load(*node);

  // Spin first so DDS discovers peers and the latched localization state lands.
  for (int i = 0; i < 20 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!node->is_localized())
    node->reset_initial_pose();

  run_control_runtime(node, keyboard, console, cfg);

  rclcpp::shutdown();
  return 0;
}
