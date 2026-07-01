#pragma once

#include "core/drive_mode.hpp"
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::manual_control {

// Modes compiled in (registerAvailable) are a library capability; the active
// subset + cycle order (setActiveOrder) is a per-deployment config choice.
class DriveModeFactory {
public:
  using Creator = std::function<std::unique_ptr<DriveMode>()>;

  static DriveModeFactory &instance();

  void registerAvailable(const std::string &name, Creator creator);
  bool isAvailable(const std::string &name) const;
  std::vector<std::string> availableNames() const;

  void setActiveOrder(const std::vector<std::string> &order); // validates; requires "stop"
  const std::vector<std::string> &activeOrder() const;

  // Also raised by the rclcpp transport, which rejects a typeless empty `modes: []`
  // at node construction -- before setActiveOrder can run -- and translates it here.
  static constexpr const char *kEmptyModesError =
      "[modes] 'modes' is empty; need at least \"stop\"";

  std::unique_ptr<DriveMode> createMode(const std::string &name) const;

private:
  DriveModeFactory() = default;
  std::map<std::string, Creator> available_;
  std::vector<std::string> active_order_;
};

} // namespace autoware::manual_control
