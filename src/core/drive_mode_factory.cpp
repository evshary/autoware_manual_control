#include "core/drive_mode_factory.hpp"
#include <algorithm>
#include <stdexcept>

namespace autoware::manual_control {

DriveModeFactory &DriveModeFactory::instance() {
  static DriveModeFactory inst;
  return inst;
}

void DriveModeFactory::registerAvailable(const std::string &name, Creator c) {
  available_[name] = std::move(c);
}

bool DriveModeFactory::isAvailable(const std::string &name) const {
  return available_.count(name) > 0;
}

std::vector<std::string> DriveModeFactory::availableNames() const {
  std::vector<std::string> v;
  for (const auto &kv : available_) {
    v.push_back(kv.first);
  }
  return v;
}

void DriveModeFactory::setActiveOrder(const std::vector<std::string> &order) {
  if (order.empty()) {
    throw std::runtime_error("[modes] 'modes' is empty; need at least \"stop\"");
  }
  if (std::find(order.begin(), order.end(), "stop") == order.end()) {
    throw std::runtime_error(
        "[modes] 'modes' must include \"stop\" (initial/emergency mode)");
  }
  for (const auto &n : order) {
    if (!isAvailable(n)) {
      std::string avail;
      for (const auto &a : availableNames()) {
        avail += (avail.empty() ? "" : ", ") + a;
      }
      throw std::runtime_error("[modes] unknown drive mode '" + n +
                               "'; available: " + avail);
    }
  }
  active_order_ = order;
}

const std::vector<std::string> &DriveModeFactory::activeOrder() const {
  return active_order_;
}

std::unique_ptr<DriveMode> DriveModeFactory::createMode(const std::string &name) const {
  auto it = available_.find(name);
  return it != available_.end() ? it->second() : nullptr;
}

} // namespace autoware::manual_control
