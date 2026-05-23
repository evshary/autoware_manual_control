#ifndef TELEOP_CONTROL_RUNTIME_HPP
#define TELEOP_CONTROL_RUNTIME_HPP

// Shared 60Hz control loop, parameterized on an input source and a telemetry
// sink so the keyboard and Zenoh entries reuse one copy of the loop.

#include "common/types.hpp"
#include "core/mode_manager.hpp"
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
};

// InputSource and TelemetrySink are duck-typed:
//   InputSource:   InputState update();
//   TelemetrySink: void publish(const Telemetry&);
// On loss of fresh input the InputSource must return a safe InputState
// (throttle 0, brake 1, steer 0) so the loop brakes without a special case.
template <typename InputSource, typename TelemetrySink>
void run_control_runtime(std::shared_ptr<ManualControlNode> node,
                         InputSource &input, TelemetrySink &sink,
                         const RuntimeConfig &cfg) {
  ModeManager mode_manager;

  if (node->should_start_external()) {
    node->force_external_mode();
  }

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
      dt = 0.1f; // Cap max dt

    // --- Input Phase ---
    InputState input_state = input.update();

    if (input_state.quit)
      running = false;

    // --- Logic Phase ---
    if (input_state.toggle_auto) {
      node->toggle_manual_control();
    }

    if (input_state.reset_pose) {
      node->reset_initial_pose();
    }

    VehicleState vehicle_state = node->get_vehicle_state();

    // Shift Request Handling (Stop-Wait-Shift)
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

    // State Machine
    if (shift_state == ShiftState::STOPPING) {
      override_control = true;
      override_cmd.velocity = 0.0f;
      override_cmd.acceleration = -10.0f; // Max Brake
      override_cmd.steer_angle = vehicle_state.steer_angle;

      if (std::abs(vehicle_state.velocity) < cfg.shift_stop_tolerance) {
        node->set_target_gear(pending_gear);
        shift_state = ShiftState::SHIFTING;
      }
    } else if (shift_state == ShiftState::SHIFTING) {
      override_control = true;
      override_cmd.velocity = 0.0f;
      override_cmd.acceleration = -10.0f; // Hold Brake
      override_cmd.steer_angle = vehicle_state.steer_angle;

      if (vehicle_state.gear == pending_gear) {
        mode_manager.reinit(vehicle_state);
        shift_state = ShiftState::IDLE;
        override_control = false;
      }
    }

    // Update Mode Manager
    mode_manager.update(dt, input_state, vehicle_state);
    ControlCommand cmd = mode_manager.getCommand();

    if (override_control) {
      cmd = override_cmd;
    }

    // --- Output Phase ---
    node->publish_command(cmd);

    // --- Telemetry Phase ---
    Telemetry t;
    t.mode = mode_manager.getCurrentModeName();
    t.mode_status = mode_manager.getStatusString();
    t.vehicle = vehicle_state;
    t.command = cmd;
    t.shift_state = shift_state;
    t.pending_gear = pending_gear;
    t.info = node->get_info_message();
    sink.publish(t);

    rclcpp::spin_some(node);
    rate.sleep();
  }
}

} // namespace autoware::manual_control

#endif // TELEOP_CONTROL_RUNTIME_HPP
