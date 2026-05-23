#ifndef TELEOP_TELEMETRY_HPP
#define TELEOP_TELEMETRY_HPP

#include "common/types.hpp"

#include <string>

namespace autoware::manual_control {

// Per-tick control snapshot, consumed by a TelemetrySink. Transport-agnostic:
// medium-specific fields (keyboard key highlight, network client id) are not
// here — each sink reads those from its own input.
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
