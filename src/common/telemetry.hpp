#ifndef TELEOP_TELEMETRY_HPP
#define TELEOP_TELEMETRY_HPP

#include "common/types.hpp"

#include <string>

namespace autoware::manual_control {

// Per-tick control snapshot for the telemetry sink.
struct Telemetry {
  std::string mode;
  std::string mode_status;
  std::string operation_mode; // live Autoware operation mode
  VehicleState vehicle;
  ControlCommand command;
  ShiftState shift_state = ShiftState::IDLE;
  Gear pending_gear = Gear::NONE;
  std::string info;
};

} // namespace autoware::manual_control

#endif // TELEOP_TELEMETRY_HPP
