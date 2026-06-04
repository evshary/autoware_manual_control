#ifndef TELEOP_MODE_MANAGER_HPP
#define TELEOP_MODE_MANAGER_HPP

#include "common/intent.hpp"
#include "common/types.hpp"
#include "core/drive_mode.hpp"
#include "core/drive_mode_factory.hpp"
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace autoware::manual_control {

class ModeManager {
public:
  ModeManager() {
    // Modes must be registered + activated on the factory before use.
    switchMode("stop", {});
  }

  void reinit(const VehicleState &state) { switchMode(current_type_, state); }

  void update(float dt, const Intent &intent,
              const VehicleState &vehicle_state) {

    if (intent.emergency_stop) {
      if (current_type_ == "stop") {
        switchMode(previous_type_, vehicle_state);
      } else {
        switchMode("stop", vehicle_state);
      }
    }

    if (intent.switch_mode) {
      const auto &modes = DriveModeFactory::instance().activeOrder();
      auto it = std::find(modes.begin(), modes.end(), current_type_);
      size_t index = (it != modes.end()) ? std::distance(modes.begin(), it) : 0;
      size_t next_index = (index + 1) % modes.size();
      switchMode(modes[next_index], vehicle_state);
    }

    if (!active_mode_)
      return;

    last_cmd_ = active_mode_->update(dt, intent, vehicle_state);
  }

  ControlCommand getCommand() const { return last_cmd_; }

  std::string getCurrentModeName() const {
    return active_mode_ ? active_mode_->getName() : "NONE";
  }

  std::string getStatusString() const {
    return active_mode_ ? active_mode_->getStatusString() : "";
  }

private:
  void switchMode(const std::string &type, const VehicleState &state) {
    if (active_mode_) {
      active_mode_->onExit();
    }

    previous_type_ = current_type_;
    current_type_ = type;

    active_mode_ = DriveModeFactory::instance().createMode(type);

    if (active_mode_) {
      active_mode_->onEnter(state);
    }
  }

  std::unique_ptr<DriveMode> active_mode_;
  std::string current_type_ = "stop";
  std::string previous_type_ = "stop";
  ControlCommand last_cmd_;
};

} // namespace autoware::manual_control

#endif // TELEOP_MODE_MANAGER_HPP
