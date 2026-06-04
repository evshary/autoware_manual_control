#ifndef TELEOP_IO_INTENT_KEYBOARD_HPP
#define TELEOP_IO_INTENT_KEYBOARD_HPP

#include "common/intent.hpp"

#include <cctype>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace autoware::manual_control {

// Raw non-blocking terminal reader: RAII over termios (restores on exit).
class KeyboardReader {
public:
  KeyboardReader() {
    tcgetattr(STDIN_FILENO, &oldt_);
    newt_ = oldt_;
    newt_.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt_);
    oldf_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf_ | O_NONBLOCK);
  }
  ~KeyboardReader() {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
    fcntl(STDIN_FILENO, F_SETFL, oldf_);
  }
  int readKey() {
    char ch;
    if (read(STDIN_FILENO, &ch, 1) > 0)
      return ch;
    return 0;
  }

private:
  struct termios oldt_, newt_;
  int oldf_;
};

// Key hold detection: tells a tap from a held key off the OS key-repeat stream.
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
    last_press = std::chrono::steady_clock::now() - std::chrono::seconds(10);
  }

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

// Per-key echo state: idle, a quick tap, or a sustained hold.
enum class KeyHold { Idle, Tapped, Held };

// Keyboard IntentSource: maps keys to an Intent each tick; writes no stdout.
class KeyboardIntent {
public:
  Intent update() {
    Intent intent;

    intent.switch_mode = false;
    intent.toggle_auto = false;
    intent.emergency_stop = false;
    intent.reset_pose = false;
    intent.shift_drive = false;
    intent.shift_reverse = false;
    intent.shift_park = false;
    intent.quit = false;

    int ch;
    while ((ch = reader_.readKey()) > 0) {
      switch (std::tolower(static_cast<unsigned char>(ch))) {
      case 'w':
        key_w_.update();
        break;
      case 's':
        key_s_.update();
        break;
      case 'a':
        key_a_.update();
        break;
      case 'd':
        key_d_.update();
        break;
      case ' ':
        intent.emergency_stop = true;
        break;
      case 'm':
        intent.switch_mode = true;
        break;
      case 'z':
        intent.toggle_auto = true;
        break;
      case 'r':
        intent.reset_pose = true;
        break;
      case 'x':
        intent.shift_drive = true;
        break;
      case 'c':
        intent.shift_reverse = true;
        break;
      case 'v':
        intent.shift_park = true;
        break;
      case 'q':
        intent.quit = true;
        break;
      }
    }

    bool w = key_w_.check_and_maintain();
    bool s = key_s_.check_and_maintain();
    bool a = key_a_.check_and_maintain();
    bool d = key_d_.check_and_maintain();

    intent.throttle = w ? 1.0f : 0.0f;
    intent.throttle_hold = key_w_.is_holding_state();
    intent.brake = s ? 1.0f : 0.0f;
    intent.brake_hold = key_s_.is_holding_state();
    intent.steer_dir = (a ? 1 : 0) + (d ? -1 : 0); // Left is +

    // Drop held continuous inputs on the operation-mode toggle so the new mode
    // starts from a fresh release.
    if (intent.toggle_auto) {
      key_w_.reset();
      key_s_.reset();
      key_a_.reset();
      key_d_.reset();
    }

    return intent;
  }

  // Per-key hold level for an input-feedback view.
  KeyHold w_state() const { return level(key_w_); }
  KeyHold a_state() const { return level(key_a_); }
  KeyHold s_state() const { return level(key_s_); }
  KeyHold d_state() const { return level(key_d_); }

private:
  static KeyHold level(const KeyState &k) {
    return k.is_holding_state() ? KeyHold::Held
           : k.is_active()     ? KeyHold::Tapped
                               : KeyHold::Idle;
  }

  KeyboardReader reader_;
  KeyState key_w_, key_s_, key_a_, key_d_;
};

} // namespace autoware::manual_control

#endif // TELEOP_IO_INTENT_KEYBOARD_HPP
