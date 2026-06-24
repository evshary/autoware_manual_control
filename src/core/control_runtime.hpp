#ifndef TELEOP_CONTROL_RUNTIME_HPP
#define TELEOP_CONTROL_RUNTIME_HPP

// Shared 60Hz control loop, generic over an intent source and a telemetry sink.

#include "common/intent.hpp"
#include "common/telemetry.hpp"
#include "common/types.hpp"
#include "core/mode_manager.hpp"
#include "core/parameter_reader.hpp"
#include "core/autoware_gateway.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

namespace autoware::manual_control
{

struct RuntimeConfig
{
  double rate_hz = 60.0;
  float shift_stop_tolerance = 0.05f; // m/s — speed below which a shift proceeds
  float shift_brake_accel = -10.0f;   // m/s^2 — override accel while shifting

  // Loads its own params.
  static RuntimeConfig load(const ParameterReader & reader)
  {
    RuntimeConfig cfg;
    cfg.rate_hz = reader.read<double>("control_rate_hz", cfg.rate_hz);
    cfg.shift_stop_tolerance =
      reader.read<float>("shift_stop_tolerance", cfg.shift_stop_tolerance);
    cfg.shift_brake_accel =
      reader.read<float>("shift_brake_accel", cfg.shift_brake_accel);
    return cfg;
  }
};

// IntentSource: Intent update(). TelemetrySink: void publish(Telemetry). On
// stale input, update() returns a braking Intent (throttle 0, brake 1) so the
// loop needs no special case.
template<typename IntentSource, typename TelemetrySink>
void run_control_runtime(
  std::shared_ptr<AutowareGateway> gateway,
  IntentSource & source, TelemetrySink & sink,
  const RuntimeConfig & cfg)
{
  ModeManager mode_manager;

  auto last_time = std::chrono::steady_clock::now();

  ShiftState shift_state = ShiftState::IDLE;
  Gear pending_gear = Gear::PARK;

  const auto loop_duration = std::chrono::microseconds(static_cast<int>(1000000.0 / cfg.rate_hz));

  bool running = true;
  while (gateway->ok() && running) {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<float> dt_duration = now - last_time;
    last_time = now;
    float dt = dt_duration.count();
    if (dt > 0.1f) {
      dt = 0.1f;
    }

    Intent intent = source.update();

    if (intent.quit) {
      running = false;
    }

    if (intent.toggle_auto) {
      gateway->toggle_operation_mode();
    }

    if (intent.reset_pose) {
      gateway->reset_initial_pose();
    }

    VehicleState vehicle_state = gateway->get_vehicle_state();

    // Shift request: brake to a stop, shift, then resume (stop-wait-shift).
    if (intent.shift_drive && vehicle_state.gear != Gear::DRIVE) {
      pending_gear = Gear::DRIVE;
      shift_state = ShiftState::STOPPING;
    }
    if (intent.shift_reverse && vehicle_state.gear != Gear::REVERSE) {
      pending_gear = Gear::REVERSE;
      shift_state = ShiftState::STOPPING;
    }
    if (intent.shift_park && vehicle_state.gear != Gear::PARK) {
      pending_gear = Gear::PARK;
      shift_state = ShiftState::STOPPING;
    }

    bool override_control = false;
    ControlCommand override_cmd;

    if (shift_state == ShiftState::STOPPING) {
      override_control = true;
      override_cmd.velocity = 0.0f;
      override_cmd.acceleration = cfg.shift_brake_accel;
      override_cmd.steer_angle = vehicle_state.steer_angle;

      if (std::abs(vehicle_state.velocity) < cfg.shift_stop_tolerance) {
        gateway->set_target_gear(pending_gear);
        shift_state = ShiftState::SHIFTING;
      }
    } else if (shift_state == ShiftState::SHIFTING) {
      override_control = true;
      override_cmd.velocity = 0.0f;
      override_cmd.acceleration = cfg.shift_brake_accel;
      override_cmd.steer_angle = vehicle_state.steer_angle;

      if (vehicle_state.gear == pending_gear) {
        mode_manager.reinit(vehicle_state);
        shift_state = ShiftState::IDLE;
        override_control = false;
      }
    }

    mode_manager.update(dt, intent, vehicle_state);
    ControlCommand cmd = mode_manager.getCommand();

    if (override_control) {
      cmd = override_cmd;
    }

    gateway->publish_command(cmd);

    Telemetry t;
    t.mode = mode_manager.getCurrentModeName();
    t.mode_status = mode_manager.getStatusString();
    t.vehicle = vehicle_state;
    t.command = cmd;
    t.shift_state = shift_state;
    t.pending_gear = pending_gear;
    t.info = gateway->get_info_message();
    t.operation_mode = gateway->operationModeName();
    sink.publish(t);

    gateway->spin_some();
    std::this_thread::sleep_for(loop_duration);
  }
}

} // namespace autoware::manual_control

#endif // TELEOP_CONTROL_RUNTIME_HPP
