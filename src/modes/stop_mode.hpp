#ifndef TELEOP_MODES_STOP_MODE_HPP
#define TELEOP_MODES_STOP_MODE_HPP

#include "core/drive_mode.hpp"
#include <string>

namespace autoware::manual_control {

// ==========================================
// Stop Mode
// ==========================================
class StopDriveMode : public DriveMode {
public:
  ControlCommand update(float /*dt*/, const InputState & /*input*/,
                        const VehicleState & /*vehicle_state*/) override {
    ControlCommand cmd;
    cmd.velocity = 0.0f;
    cmd.acceleration = -10.0f; // Strong brake
    cmd.steer_angle = 0.0f;    // Center for safety
    return cmd;
  }
  std::string getName() const override { return "STOP"; }
};

} // namespace autoware::manual_control

#endif // TELEOP_MODES_STOP_MODE_HPP
