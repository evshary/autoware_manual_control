#ifndef TELEOP_IO_TELEMETRY_ZENOH_HPP
#define TELEOP_IO_TELEMETRY_ZENOH_HPP

#include <zenoh.hxx>

#include <nlohmann/json.hpp>

#include "common/telemetry.hpp"
#include "common/types.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace autoware::manual_control
{

// Network TelemetrySink: serializes each Telemetry to JSON on the telemetry key.
class ZenohTelemetry
{
public:
  ZenohTelemetry(
    std::shared_ptr<zenoh::Session> session,
    const std::string & telemetry_key)
  : session_(std::move(session))
  {
    pub_ = std::make_unique<zenoh::Publisher>(
      session_->declare_publisher(telemetry_key));
  }

  // Injects extra JSON fields at publish time (source-derived, wired at the root).
  void set_aux(std::function<void(nlohmann::json &)> f) {aux_ = std::move(f);}

  void publish(const Telemetry & t)
  {
    nlohmann::json j;
    j["mode"] = t.mode;
    j["mode_status"] = t.mode_status;
    j["operation_mode"] = t.operation_mode;
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
    if (aux_) {
      aux_(j);
    }
    j["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count();
    try {
      pub_->put(zenoh::Bytes(j.dump()));
    } catch (const zenoh::ZException & e) {
      // Rate-limit: a persistent zenoh fault must not spam the control-rate loop.
      static std::atomic_flag warned = ATOMIC_FLAG_INIT;
      if (!warned.test_and_set()) {
        std::fprintf(
          stderr,
          "[ZenohTelemetry] publish failed (%s); will not warn again\n",
          e.what());
      }
    }
  }

private:
  static std::string gear_str(Gear g)
  {
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
  static std::string shift_state_str(ShiftState s)
  {
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
  std::unique_ptr<zenoh::Publisher> pub_;
  std::function<void(nlohmann::json &)> aux_;
};

} // namespace autoware::manual_control

#endif // TELEOP_IO_TELEMETRY_ZENOH_HPP
