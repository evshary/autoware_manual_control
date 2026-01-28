#ifndef TELEOP_MODE_MANAGER_HPP
#define TELEOP_MODE_MANAGER_HPP

#include "common/types.hpp"
#include "core/drive_mode.hpp"
#include "core/drive_mode_factory.hpp"
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::manual_control {

class ModeManager {
public:
  ModeManager() {
    // Default start - assumes modes are registered beforehand or strictly
    // handles nullptr We defer the initial switch or handle it gracefully if
    // factory returns null
    switchMode(ModeType::STOP, {});
  }

  void reinit(const VehicleState &state) { switchMode(current_type_, state); }

  void update(float dt, const InputState &input,
              const VehicleState &vehicle_state) {

    // 1. Global Overrides (Emergency Stop)
    if (input.emergency_stop) {
      if (current_type_ == ModeType::STOP) {
        // Reinstate previous
        switchMode(previous_type_, vehicle_state);
      } else {
        switchMode(ModeType::STOP, vehicle_state);
      }
    }

    // 2. Explicit Mode Switch
    if (input.switch_mode) {
      // Cycle to next available mode from Factory
      auto modes = DriveModeFactory::instance().getAvailableModes();
      auto it = std::find(modes.begin(), modes.end(), current_type_);
      size_t index = 0;
      if (it != modes.end()) {
        index = std::distance(modes.begin(), it);
      }
      size_t next_index = (index + 1) % modes.size();

      // Removed the logic that skips STOP mode
      switchMode(modes[next_index], vehicle_state);
    }

    if (!active_mode_)
      return;

    // 3. Compute Command
    last_cmd_ = active_mode_->update(dt, input, vehicle_state);
  }

  ControlCommand getCommand() const { return last_cmd_; }

  std::string getCurrentModeName() const {
    return active_mode_ ? active_mode_->getName() : "NONE";
  }

  std::string getStatusString() const {
    return active_mode_ ? active_mode_->getStatusString() : "";
  }

  ModeType getCurrentModeType() const { return current_type_; }

private:
  void switchMode(ModeType type, const VehicleState &state) {
    if (active_mode_) {
      active_mode_->onExit();
    }

    previous_type_ = current_type_;
    current_type_ = type;

    active_mode_ = DriveModeFactory::instance().createMode(type);

    // Should we fallback if null?
    if (!active_mode_ && type != ModeType::STOP) {
      // TODO: Handle failure to create mode
    }

    if (active_mode_) {
      active_mode_->onEnter(state);
    }
  }

  std::unique_ptr<DriveMode> active_mode_;
  ModeType current_type_ = ModeType::STOP;
  ModeType previous_type_ = ModeType::STOP;
  ControlCommand last_cmd_;
};

} // namespace autoware::manual_control

#endif // TELEOP_MODE_MANAGER_HPP
