#ifndef TELEOP_TELEMETRY_HPP
#define TELEOP_TELEMETRY_HPP

#include "common/types.hpp"

#include <string>

namespace autoware::manual_control {

// Per-tick control snapshot for the telemetry sink.
struct Telemetry {
  std::string mode;
  std::string mode_status;
  VehicleState vehicle;   // measured
  ControlCommand command; // commanded this tick
  ShiftState shift_state = ShiftState::IDLE;
  Gear pending_gear = Gear::NONE;
  std::string info;
};

} // namespace autoware::manual_control

#endif // TELEOP_TELEMETRY_HPP
