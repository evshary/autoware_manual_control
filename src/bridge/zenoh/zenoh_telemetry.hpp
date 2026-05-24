#ifndef TELEOP_ZENOH_TELEMETRY_HPP
#define TELEOP_ZENOH_TELEMETRY_HPP

// Network TelemetrySink: serializes each Telemetry snapshot to JSON on the
// vehicle's telemetry key. The network-specific fields come from ZenohInput.

#include <zenoh.hxx>

#include <nlohmann/json.hpp>

#include "bridge/zenoh/zenoh_input.hpp"
#include "common/types.hpp"
#include "core/telemetry.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace autoware::manual_control {

class ZenohTelemetry {
public:
  ZenohTelemetry(std::shared_ptr<zenoh::Session> session,
                 const ZenohInput &input,
                 const std::string &telemetry_key)
      : session_(std::move(session)), input_(input) {
    pub_ = std::make_unique<zenoh::Publisher>(
        session_->declare_publisher(telemetry_key));
  }

  // TelemetrySink contract.
  void publish(const Telemetry &t) {
    nlohmann::json j;
    j["mode"] = t.mode;
    j["mode_status"] = t.mode_status;
    j["velocity"] = t.vehicle.velocity;
    j["gear"] = gear_str(t.vehicle.gear);
    j["steer_angle"] = t.vehicle.steer_angle;
    j["target_velocity"] = t.command.velocity;
    j["target_acceleration"] = t.command.acceleration;
    j["target_steer"] = t.command.steer_angle;
    j["shift_state"] = shift_state_str(t.shift_state);
    j["pending_gear"] =
        (t.shift_state != ShiftState::IDLE) ? gear_str(t.pending_gear) : "";
    j["info"] = t.info;
    // from ZenohInput
    j["active_client_id"] = input_.active_client_id();
    j["watchdog_tripped"] = input_.watchdog_tripped();
    // Wall-clock ms at publish time — lets the UI compute one-way latency
    // (Date.now() - timestamp) without an extra round-trip.
    j["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
    pub_->put(zenoh::Bytes(j.dump()));
  }

private:
  static std::string gear_str(Gear g) {
    switch (g) {
    case Gear::DRIVE:
      return "DRIVE";
    case Gear::REVERSE:
      return "REVERSE";
    case Gear::PARK:
      return "PARK";
    case Gear::LOW:
      return "LOW";
    case Gear::NEUTRAL:
      return "NEUTRAL";
    default:
      return "NONE";
    }
  }
  static std::string shift_state_str(ShiftState s) {
    switch (s) {
    case ShiftState::STOPPING:
      return "STOPPING";
    case ShiftState::SHIFTING:
      return "SHIFTING";
    default:
      return "IDLE";
    }
  }

  std::shared_ptr<zenoh::Session> session_;
  const ZenohInput &input_;
  std::unique_ptr<zenoh::Publisher> pub_;
};

} // namespace autoware::manual_control

#endif // TELEOP_ZENOH_TELEMETRY_HPP
