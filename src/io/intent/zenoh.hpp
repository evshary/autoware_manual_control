#ifndef TELEOP_IO_INTENT_ZENOH_HPP
#define TELEOP_IO_INTENT_ZENOH_HPP

#include <zenoh.hxx>

#include <nlohmann/json.hpp>

#include "common/intent.hpp"
#include "common/types.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>

namespace autoware::manual_control {

struct ZenohIntentConfig {
  std::string intent_key = "manual_control/v1/intent";
  float arrival_timeout_s = 0.5f;
};

// Network IntentSource: decodes intent messages; deadman-brakes on stale input.
class ZenohIntent {
public:
  ZenohIntent(std::shared_ptr<zenoh::Session> session,
              const ZenohIntentConfig &cfg)
      : cfg_(cfg), session_(std::move(session)) {
    last_arrival_ = std::chrono::steady_clock::now();
    subscriber_ = std::make_unique<zenoh::Subscriber<void>>(
        session_->declare_subscriber(
            zenoh::KeyExpr(cfg_.intent_key),
            [this](zenoh::Sample &sample) { on_intent(sample); }, []() {}));
  }

  Intent update() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();

    const bool fresh =
        have_intent_ && secs(now - last_arrival_) <= cfg_.arrival_timeout_s;
    watchdog_tripped_.store(!fresh);

    if (!fresh)
      return safe_state();

    Intent s;
    s.throttle = clamp01(latest_.throttle);
    s.throttle_hold = s.throttle > 0.0f;
    s.brake = clamp01(latest_.brake);
    s.brake_hold = s.brake > 0.0f;
    s.steer_dir = (latest_.steer > 0.0) ? 1 : (latest_.steer < 0.0 ? -1 : 0);

    if (latest_.gear != applied_gear_) {
      s.shift_drive = (latest_.gear == Gear::DRIVE);
      s.shift_reverse = (latest_.gear == Gear::REVERSE);
      s.shift_park = (latest_.gear == Gear::PARK);
      applied_gear_ = latest_.gear;
    }

    s.emergency_stop = step(applied_estop_, latest_.estop);
    s.switch_mode = step(applied_mode_cycle_, latest_.mode_cycle);
    s.toggle_auto = step(applied_toggle_auto_, latest_.toggle_auto);
    s.reset_pose = step(applied_reset_pose_, latest_.reset_pose);
    return s;
  }

  std::string active_client_id() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_.client_id;
  }
  bool watchdog_tripped() const { return watchdog_tripped_.load(); }

private:
  struct Wire {
    float throttle = 0.0f;
    float brake = 0.0f;
    double steer = 0.0;
    Gear gear = Gear::PARK;
    int64_t estop = 0;
    int64_t mode_cycle = 0;
    int64_t toggle_auto = 0;
    int64_t reset_pose = 0;
    std::string client_id;
  };

  void on_intent(zenoh::Sample &sample) {
    nlohmann::json j;
    try {
      j = nlohmann::json::parse(sample.get_payload().as_string());
    } catch (...) {
      warn_once("unparseable JSON");
      return;
    }

    if (!j.contains("client_id") || !j["client_id"].is_string()) {
      warn_once("missing or invalid client_id");
      return;
    }
    auto num_ok = [&](const char *k) {
      return !j.contains(k) || j[k].is_number();
    };
    if (!num_ok("throttle") || !num_ok("brake") || !num_ok("steer") ||
        !num_ok("estop") || !num_ok("mode_cycle") || !num_ok("toggle_auto") ||
        !num_ok("reset_pose")) {
      warn_once("non-numeric value in numeric field");
      return;
    }
    if (j.contains("gear")) {
      if (!j["gear"].is_string()) {
        warn_once("non-string gear");
        return;
      }
      const std::string &g = j["gear"].get_ref<const std::string &>();
      if (g != "DRIVE" && g != "REVERSE" && g != "PARK") {
        warn_once("unknown gear value");
        return;
      }
    }

    Wire in;
    in.throttle = j.value("throttle", 0.0);
    in.brake = j.value("brake", 0.0);
    in.steer = j.value("steer", 0.0);
    in.gear = gear_from(j.value("gear", std::string("PARK")));
    in.estop = j.value("estop", int64_t{0});
    in.mode_cycle = j.value("mode_cycle", int64_t{0});
    in.toggle_auto = j.value("toggle_auto", int64_t{0});
    in.reset_pose = j.value("reset_pose", int64_t{0});
    in.client_id = j.value("client_id", std::string{});

    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = in;
    have_intent_ = true;
    last_arrival_ = std::chrono::steady_clock::now();
  }

  static void warn_once(const char *reason) {
    static bool warned = false;
    if (!warned) {
      warned = true;
      std::fprintf(stderr,
                   "[ZenohIntent] rejecting malformed intent (%s); "
                   "will not warn again\n",
                   reason);
    }
  }

  static float secs(std::chrono::steady_clock::duration d) {
    return std::chrono::duration<float>(d).count();
  }
  static float clamp01(float v) {
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
  }
  static Gear gear_from(const std::string &g) {
    if (g == "DRIVE")
      return Gear::DRIVE;
    if (g == "REVERSE")
      return Gear::REVERSE;
    return Gear::PARK;
  }
  // Monotonic one-shot counter edge-detector: true when `wire` rises above
  // `applied` (which it then adopts). Adopting even a lower `wire` re-baselines
  // a reconnected operator whose counter restarted, instead of stalling forever.
  static bool step(int64_t &applied, int64_t wire) {
    bool fired = wire > applied;
    applied = wire;
    return fired;
  }
  static Intent safe_state() {
    Intent s;
    s.brake = 1.0f;
    return s;
  }

  ZenohIntentConfig cfg_;
  std::shared_ptr<zenoh::Session> session_;
  std::unique_ptr<zenoh::Subscriber<void>> subscriber_;

  mutable std::mutex mutex_;
  Wire latest_;
  bool have_intent_ = false;
  std::chrono::steady_clock::time_point last_arrival_;

  Gear applied_gear_ = Gear::PARK;
  int64_t applied_estop_ = 0;
  int64_t applied_mode_cycle_ = 0;
  int64_t applied_toggle_auto_ = 0;
  int64_t applied_reset_pose_ = 0;

  std::atomic<bool> watchdog_tripped_{true};
};

} // namespace autoware::manual_control

#endif // TELEOP_IO_INTENT_ZENOH_HPP
