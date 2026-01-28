#ifndef TELEOP_INPUT_SYSTEM_HPP
#define TELEOP_INPUT_SYSTEM_HPP

#include "common/types.hpp"
#include "input/keyboard_reader.hpp"
#include <chrono>

namespace autoware::manual_control {

// Helper class for key hold detection
class KeyState {
public:
  void update() {
    auto now = std::chrono::steady_clock::now();
    auto diff =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_press);
    // OS repeat usually starts after ~200-500ms. If we get rapid events, we
    // assume holding.
    if (diff.count() < 100) {
      is_holding = true;
    }
    last_press = now;
  }

  bool is_active() const {
    auto now = std::chrono::steady_clock::now();
    auto diff =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_press);
    int timeout = is_holding ? 100 : 450;
    return diff.count() < timeout;
  }

  void reset() {
    is_holding = false;
    // Set time to past to effectively de-activate
    last_press = std::chrono::steady_clock::now() - std::chrono::seconds(10);
  }

  // Checks and maintains lifecycle
  bool check_and_maintain() {
    bool active = is_active();
    if (!active) {
      if (is_holding)
        reset();
      is_holding = false;
    }
    return active;
  }

  bool is_holding_state() const { return is_holding; }

private:
  std::chrono::steady_clock::time_point last_press;
  bool is_holding{false};
};

class InputSystem {
public:
  InputSystem() = default;

  InputState update() {
    InputState state;

    // reset triggers
    state.switch_mode = false;
    state.toggle_auto = false;
    state.emergency_stop = false;
    state.reset_pose = false;
    state.shift_drive = false;
    state.shift_reverse = false;
    state.shift_park = false;
    state.quit = false;

    // Process raw key events
    int ch;
    while ((ch = reader_.readKey()) > 0) {
      switch (ch) {
      case 'w':
      case 'W':
        key_w_.update();
        break;
      case 's':
      case 'S':
        key_s_.update();
        break;
      case 'a':
      case 'A':
        key_a_.update();
        break;
      case 'd':
      case 'D':
        key_d_.update();
        break;
      case ' ':
        state.emergency_stop = true;
        break;
      case 'm':
      case 'M':
        state.switch_mode = true;
        break;
      case 'z':
      case 'Z':
        state.toggle_auto = true;
        break;
      case 'r':
      case 'R':
        state.reset_pose = true;
        break;
      case 'x':
      case 'X':
        state.shift_drive = true;
        break;
      case 'c':
      case 'C':
        state.shift_reverse = true;
        break;
      case 'v':
      case 'V':
        state.shift_park = true;
        break;
      case 'q':
      case 'Q':
        state.quit = true;
        break;
      }
    }

    // Update continuous states
    bool w = key_w_.check_and_maintain();
    bool s = key_s_.check_and_maintain();
    bool a = key_a_.check_and_maintain();
    bool d = key_d_.check_and_maintain();

    state.throttle = w ? 1.0f : 0.0f;
    state.throttle_hold = key_w_.is_holding_state();
    state.brake = s ? 1.0f : 0.0f;
    state.brake_hold = key_s_.is_holding_state();
    state.steer_dir = (a ? 1 : 0) + (d ? -1 : 0); // Left is +

    return state;
  }

  // Allow resetting internal states (e.g. after mode switch)
  void reset() {
    key_w_.reset();
    key_s_.reset();
    key_a_.reset();
    key_d_.reset();
  }

  // Expose key hold feedback for UI
  bool isHoldingW() const { return key_w_.is_holding_state(); }
  bool isHoldingS() const { return key_s_.is_holding_state(); }
  bool isHoldingA() const { return key_a_.is_holding_state(); }
  bool isHoldingD() const { return key_d_.is_holding_state(); }

  bool isActiveW() const { return key_w_.is_active(); }
  bool isActiveS() const { return key_s_.is_active(); }
  bool isActiveA() const { return key_a_.is_active(); }
  bool isActiveD() const { return key_d_.is_active(); }

private:
  KeyboardReader reader_;
  KeyState key_w_, key_s_, key_a_, key_d_;
};

} // namespace autoware::manual_control

#endif // TELEOP_INPUT_SYSTEM_HPP
