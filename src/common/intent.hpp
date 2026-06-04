#ifndef TELEOP_INTENT_HPP
#define TELEOP_INTENT_HPP

namespace autoware::manual_control {

// Decoded, transport-agnostic operator command (the IntentSource payload).
struct Intent {
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

} // namespace autoware::manual_control

#endif // TELEOP_INTENT_HPP
