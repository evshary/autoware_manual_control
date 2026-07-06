#ifndef TELEOP_REGISTER_MODES_HPP
#define TELEOP_REGISTER_MODES_HPP

// Compiles in the available drive modes and activates the config-selected subset.

#include "core/drive_mode_factory.hpp"
#include "core/parameter_reader.hpp"
#include "modes/cruise_mode.hpp"
#include "modes/physics_mode.hpp"
#include "modes/stop_mode.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::manual_control
{

// Params load once here and are captured, so createMode stays node-free.
template<typename Mode>
inline void register_mode(DriveModeFactory & factory, const ParameterReader & reader)
{
  auto params = Mode::loadParams(reader);
  factory.registerAvailable(
    Mode::kName,
    [params] {return std::make_unique<Mode>(params);});
}

inline void register_all_modes(DriveModeFactory & factory, const ParameterReader & reader)
{
  register_mode<StopDriveMode>(factory, reader);
  register_mode<PhysicsDriveMode>(factory, reader);
  register_mode<CruiseDriveMode>(factory, reader);
}

// Active subset + cycle order from config; must include "stop".
inline void activate_modes_from_config(DriveModeFactory & factory, const ParameterReader & reader)
{
  auto names = reader.read<std::vector<std::string>>(
    "modes", std::vector<std::string>{"stop", "physics", "cruise"});
  factory.setActiveOrder(names);
}

} // namespace autoware::manual_control

#endif // TELEOP_REGISTER_MODES_HPP
