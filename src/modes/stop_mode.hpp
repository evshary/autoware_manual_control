#ifndef TELEOP_MODES_STOP_MODE_HPP
#define TELEOP_MODES_STOP_MODE_HPP

#include "core/drive_mode.hpp"
#include <string>

namespace autoware::manual_control {

class StopDriveMode : public DriveMode {
public:
  struct Params {
    float brake_accel = -10.0f; // m/s^2
  };

  explicit StopDriveMode(const Params &params) : params_(params) {}

  ControlCommand update(float /*dt*/, const InputState & /*input*/,
                        const VehicleState & /*vehicle_state*/) override {
    ControlCommand cmd;
    cmd.velocity = 0.0f;
    cmd.acceleration = params_.brake_accel;
    cmd.steer_angle = 0.0f;
    return cmd;
  }
  std::string getName() const override { return "STOP"; }

private:
  Params params_;
};

} // namespace autoware::manual_control

#endif // TELEOP_MODES_STOP_MODE_HPP
