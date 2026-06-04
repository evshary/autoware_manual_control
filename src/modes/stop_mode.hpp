#ifndef TELEOP_MODES_STOP_MODE_HPP
#define TELEOP_MODES_STOP_MODE_HPP

#include "core/drive_mode.hpp"
#include "core/param_utils.hpp"
#include <string>

namespace autoware::manual_control {

class StopDriveMode : public DriveMode {
public:
  struct Params {
    float brake_accel = -10.0f; // m/s^2
  };

  explicit StopDriveMode(const Params &params) : params_(params) {}

  static constexpr const char *kName = "stop";
  static Params loadParams(rclcpp::Node &node) {
    Params p;
    p.brake_accel = load_float(node, "stop.brake_accel", p.brake_accel);
    return p;
  }

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
