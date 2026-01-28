#ifndef TELEOP_DRIVE_MODE_HPP
#define TELEOP_DRIVE_MODE_HPP

#include "common/types.hpp"
#include <string>

namespace autoware::manual_control {

// Abstract Base Class for Driving Strategies
class DriveMode {
public:
  virtual ~DriveMode() = default;

  // Called when this mode becomes active
  virtual void onEnter(const VehicleState & /*current_state*/) {}

  // Called when this mode is deactivated
  virtual void onExit() {}

  // Main control loop
  virtual ControlCommand update(float dt, const InputState &input,
                                const VehicleState &vehicle_state) = 0;

  // UI Helpers
  virtual std::string getName() const = 0;
  virtual std::string getStatusString() const { return ""; }
};

} // namespace autoware::manual_control

#endif // TELEOP_DRIVE_MODE_HPP
