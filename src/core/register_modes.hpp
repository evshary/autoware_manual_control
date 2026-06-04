#ifndef TELEOP_REGISTER_MODES_HPP
#define TELEOP_REGISTER_MODES_HPP

// Compiles in the available drive modes and activates the config-selected subset.

#include "core/drive_mode_factory.hpp"
#include "core/param_utils.hpp"
#include "modes/cruise_mode.hpp"
#include "modes/physics_mode.hpp"
#include "modes/stop_mode.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace autoware::manual_control {

// Params load once here and are captured, so createMode stays node-free.
template <typename Mode>
inline void register_mode(DriveModeFactory &factory, rclcpp::Node &node) {
  auto params = Mode::loadParams(node);
  factory.registerAvailable(Mode::kName,
                            [params] { return std::make_unique<Mode>(params); });
}

inline void register_all_modes(DriveModeFactory &factory, rclcpp::Node &node) {
  register_mode<StopDriveMode>(factory, node);
  register_mode<PhysicsDriveMode>(factory, node);
  register_mode<CruiseDriveMode>(factory, node);
}

// Active subset + cycle order from config; must include "stop".
inline void activate_modes_from_config(DriveModeFactory &factory, rclcpp::Node &node) {
  auto names = load_param<std::vector<std::string>>(
      node, "modes", std::vector<std::string>{"stop", "physics", "cruise"});
  factory.setActiveOrder(names);
}

} // namespace autoware::manual_control

#endif // TELEOP_REGISTER_MODES_HPP
