#ifndef TELEOP_CONTROL_RUNTIME_HPP
#define TELEOP_CONTROL_RUNTIME_HPP

// Shared 60Hz control loop, generic over an input source and a telemetry sink.

#include "common/types.hpp"
#include "core/mode_manager.hpp"
#include "core/param_utils.hpp"
#include "core/telemetry.hpp"
#include "ros/manual_control_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace autoware::manual_control {

struct RuntimeConfig {
  double rate_hz = 60.0;
  float shift_stop_tolerance = 0.05f; // m/s — speed below which a shift proceeds
  float shift_brake_accel = -10.0f;   // m/s^2 — override accel while shifting

  // Loads its own params (mirrors a mode's loadParams).
  static RuntimeConfig load(rclcpp::Node &node) {
    RuntimeConfig cfg;
    cfg.rate_hz = load_double(node, "control_rate_hz", cfg.rate_hz);
    cfg.shift_stop_tolerance =
        load_float(node, "shift_stop_tolerance", cfg.shift_stop_tolerance);
    cfg.shift_brake_accel =
        load_float(node, "shift_brake_accel", cfg.shift_brake_accel);
    return cfg;
  }
};

// InputSource: InputState update(). TelemetrySink: void publish(Telemetry). On
// stale input, update() returns a braking InputState (throttle 0, brake 1) so
// the loop needs no special case.
template <typename InputSource, typename TelemetrySink>
void run_control_runtime(std::shared_ptr<ManualControlNode> node,
                         InputSource &input, TelemetrySink &sink,
                         const RuntimeConfig &cfg) {
  ModeManager mode_manager;

  rclcpp::Rate rate(cfg.rate_hz);
  auto last_time = std::chrono::steady_clock::now();

  ShiftState shift_state = ShiftState::IDLE;
  Gear pending_gear = Gear::PARK;

  bool running = true;
  while (rclcpp::ok() && running) {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<float> dt_duration = now - last_time;
    last_time = now;
    float dt = dt_duration.count();
    if (dt > 0.1f)
      dt = 0.1f;

    InputState input_state = input.update();

    if (input_state.quit)
      running = false;

    if (input_state.toggle_auto) {
      node->toggle_operation_mode();
    }

    if (input_state.reset_pose) {
      node->reset_initial_pose();
    }

    VehicleState vehicle_state = node->get_vehicle_state();

    // Shift request: brake to a stop, shift, then resume (stop-wait-shift).
    if (input_state.shift_drive && vehicle_state.gear != Gear::DRIVE) {
      pending_gear = Gear::DRIVE;
      shift_state = ShiftState::STOPPING;
    }
    if (input_state.shift_reverse && vehicle_state.gear != Gear::REVERSE) {
      pending_gear = Gear::REVERSE;
      shift_state = ShiftState::STOPPING;
    }
    if (input_state.shift_park && vehicle_state.gear != Gear::PARK) {
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
        node->set_target_gear(pending_gear);
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

    mode_manager.update(dt, input_state, vehicle_state);
    ControlCommand cmd = mode_manager.getCommand();

    if (override_control) {
      cmd = override_cmd;
    }

    node->publish_command(cmd);

    Telemetry t;
    t.mode = mode_manager.getCurrentModeName();
    t.mode_status = mode_manager.getStatusString();
    t.vehicle = vehicle_state;
    t.command = cmd;
    t.shift_state = shift_state;
    t.pending_gear = pending_gear;
    t.info = node->get_info_message();
    t.operation_mode = node->operationModeName();
    sink.publish(t);

    rclcpp::spin_some(node);
    rate.sleep();
  }
}

} // namespace autoware::manual_control

#endif // TELEOP_CONTROL_RUNTIME_HPP
