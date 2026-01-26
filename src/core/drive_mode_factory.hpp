#pragma once

#include "common/types.hpp"
#include "core/drive_mode.hpp"
#include <functional>
#include <map>
#include <memory>
#include <vector>

namespace autoware::manual_control {

class DriveModeFactory {
public:
  using Creator = std::function<std::unique_ptr<DriveMode>()>;

  static DriveModeFactory &instance();

  void registerMode(ModeType type, Creator creator);
  std::unique_ptr<DriveMode> createMode(ModeType type) const;
  std::vector<ModeType> getAvailableModes() const;

private:
  DriveModeFactory() = default;
  std::map<ModeType, Creator> creators_;
};

} // namespace autoware::manual_control
