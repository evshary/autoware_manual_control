#ifndef TELEOP_TYPES_HPP
#define TELEOP_TYPES_HPP

#include <cstdint>

namespace autoware::manual_control {

// Shared atoms; the port payloads Intent and Telemetry live in their own headers.

// Mirrors Autoware gear constants.
enum class Gear : uint8_t {
  NONE = 0,
  PARK = 1,
  REVERSE = 2,
  NEUTRAL = 3,
  DRIVE = 4,
  LOW = 5
};

enum class ShiftState { IDLE, STOPPING, SHIFTING };

struct VehicleState {
  float velocity = 0.0f; // m/s
  Gear gear = Gear::PARK;
  bool is_engaged = false;
  float steer_angle = 0.0f; // rad
};

struct ControlCommand {
  float velocity = 0.0f;     // m/s
  float acceleration = 0.0f; // m/s^2
  float steer_angle = 0.0f;  // rad
  Gear gear_cmd = Gear::NONE;
};

} // namespace autoware::manual_control

#endif // TELEOP_TYPES_HPP
