#include "core/drive_mode_factory.hpp"

namespace autoware::manual_control {

DriveModeFactory &DriveModeFactory::instance() {
  static DriveModeFactory instance;
  return instance;
}

void DriveModeFactory::registerMode(ModeType type, Creator creator) {
  creators_[type] = creator;
}

std::unique_ptr<DriveMode> DriveModeFactory::createMode(ModeType type) const {
  auto it = creators_.find(type);
  if (it != creators_.end()) {
    return it->second();
  }
  return nullptr;
}

std::vector<ModeType> DriveModeFactory::getAvailableModes() const {
  std::vector<ModeType> modes;
  for (const auto &pair : creators_) {
    modes.push_back(pair.first);
  }
  return modes;
}

} // namespace autoware::manual_control
