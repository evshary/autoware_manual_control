#ifndef TELEOP_DRIVE_MODE_HPP
#define TELEOP_DRIVE_MODE_HPP

#include "common/intent.hpp"
#include "common/types.hpp"
#include <string>

namespace autoware::manual_control {

class DriveMode {
public:
  virtual ~DriveMode() = default;

  virtual void onEnter(const VehicleState & /*current_state*/) {}
  virtual void onExit() {}

  virtual ControlCommand update(float dt, const Intent &intent,
                                const VehicleState &vehicle_state) = 0;

  virtual std::string getName() const = 0;
  virtual std::string getStatusString() const { return ""; }
};

} // namespace autoware::manual_control

#endif // TELEOP_DRIVE_MODE_HPP
