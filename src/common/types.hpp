#ifndef TELEOP_COMMON_HPP
#define TELEOP_COMMON_HPP

#include <cstdint>
#include <string>

// Enum for Gear (Aligned with Autoware constants usually)
enum class Gear : uint8_t {
  NONE = 0,
  PARK = 1,
  REVERSE = 2,
  NEUTRAL = 3,
  DRIVE = 4,
  LOW = 5
};

enum class ModeType { STOP, PHYSICS, CRUISE };

// Shift Safety State
enum class ShiftState { IDLE, STOPPING, SHIFTING };

// Data structure for Vehicle Status (Feedback from ROS)
struct VehicleState {
  float velocity = 0.0f; // m/s
  Gear gear = Gear::PARK;
  bool is_engaged = false;
  float steer_angle = 0.0f; // rad
};

// Data structure for processed Input (Semantic)
struct InputState {
  float throttle = 0.0f; // 0.0 to 1.0
  bool throttle_hold = false;
  float brake = 0.0f; // 0.0 to 1.0
  bool brake_hold = false;
  int steer_dir = 0; // -1 (Right), 0, 1 (Left)

  bool shift_drive = false;
  bool shift_reverse = false;
  bool shift_park = false;

  bool toggle_auto = false;
  bool emergency_stop = false;
  bool reset_pose = false;
  bool switch_mode = false;

  bool quit = false;
};

// Data structure for Control Command (Output)
struct ControlCommand {
  float velocity = 0.0f;     // m/s
  float acceleration = 0.0f; // m/s^2
  float steer_angle = 0.0f;  // rad
  Gear gear_cmd = Gear::NONE;
};

#endif // TELEOP_COMMON_HPP
