#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "common/types.hpp"
#include "core/drive_mode_factory.hpp"
#include "core/mode_manager.hpp"
#include "input/input_system.hpp"
#include "modes/cruise_mode.hpp"
#include "modes/physics_mode.hpp"
#include "modes/stop_mode.hpp"
#include "ros/manual_control_node.hpp"
#include "ui/console_ui.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // 1. Initialize Components
  // Register Modes
  auto &factory = autoware::manual_control::DriveModeFactory::instance();
  factory.registerMode(ModeType::PHYSICS, []() {
    return std::make_unique<autoware::manual_control::PhysicsDriveMode>();
  });
  factory.registerMode(ModeType::CRUISE, []() {
    return std::make_unique<autoware::manual_control::CruiseDriveMode>();
  });
  factory.registerMode(ModeType::STOP, []() {
    return std::make_unique<autoware::manual_control::StopDriveMode>();
  });

  auto node = std::make_shared<autoware::manual_control::ManualControlNode>();
  autoware::manual_control::InputSystem input_system;
  autoware::manual_control::ModeManager mode_manager;
  autoware::manual_control::ConsoleUI ui;

  // 2. Setup
  ui.init();

  // Check Environment for Auto-External
  const char *sim_env = std::getenv("SCENARIO_SIMULATION");
  if (sim_env && std::string(sim_env) == "false") {
    node->force_external_mode();
  }

  // 3. Main Loop
  rclcpp::Rate rate(60);
  auto last_time = std::chrono::steady_clock::now();

  // Shift Safety State
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
    InputState input = input_system.update();

    if (input.quit)
      running = false;

    // --- Logic Phase ---

    if (input.toggle_auto) {
      node->toggle_manual_control();
      // Reset input state on toggle
      input_system.reset();
    }

    if (input.reset_pose) {
      node->reset_initial_pose();
    }

    // Get Vehicle State early for logic checks
    VehicleState vehicle_state = node->get_vehicle_state();

    // Shift Request Handling (Stop-Wait-Shift)
    if (input.shift_drive && vehicle_state.gear != Gear::DRIVE) {
      pending_gear = Gear::DRIVE;
      shift_state = ShiftState::STOPPING;
    }
    if (input.shift_reverse && vehicle_state.gear != Gear::REVERSE) {
      pending_gear = Gear::REVERSE;
      shift_state = ShiftState::STOPPING;
    }
    if (input.shift_park && vehicle_state.gear != Gear::PARK) {
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

      // Wait for Stop (0.05 m/s tolerance)
      if (std::abs(vehicle_state.velocity) < 0.05f) {
        node->set_target_gear(pending_gear);
        shift_state = ShiftState::SHIFTING;
      }
    } else if (shift_state == ShiftState::SHIFTING) {
      override_control = true;
      override_cmd.velocity = 0.0f;
      override_cmd.acceleration = -10.0f; // Hold Brake
      override_cmd.steer_angle = vehicle_state.steer_angle;

      // Wait for Gear Confirmation
      if (vehicle_state.gear == pending_gear) {
        mode_manager.reinit(vehicle_state);
        shift_state = ShiftState::IDLE;
        override_control = false;
      }
    }

    // Get Vehicle State (Already retrieved above)

    // Update Mode Manager
    mode_manager.update(dt, input, vehicle_state);
    ControlCommand cmd = mode_manager.getCommand();

    if (override_control) {
      cmd = override_cmd;
    }

    // --- Output Phase ---
    node->publish_command(cmd);

    // --- UI Phase ---
    ui.refresh(input_system, mode_manager, vehicle_state, cmd, shift_state,
               pending_gear);

    // ROS Spin
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
