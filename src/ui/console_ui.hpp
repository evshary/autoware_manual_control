
#ifndef TELEOP_CONSOLE_UI_HPP
#define TELEOP_CONSOLE_UI_HPP

#include "common/types.hpp"
#include "core/drive_mode_factory.hpp"
#include "core/telemetry.hpp"
#include "input/input_system.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>

namespace autoware::manual_control {

// Keyboard TelemetrySink: renders Telemetry to the terminal. The key
// highlight is read from the InputSystem injected at construction.
class ConsoleUI {
public:
  explicit ConsoleUI(const InputSystem &input) : input_(input) {}

  void init() {
    std::cout << "\033[2J\033[1;1H"; // Clear screen
    printHeader();
  }

  void publish(const Telemetry &t) {
    // Only refresh at reasonable rate (~10Hz) to avoid flickering
    static int frame = 0;
    if (frame++ % 6 != 0)
      return;

    std::cout
        << "\033[u"; // Restore cursor to start of status line (below header)
    std::cout << "\033[J"; // Clear everything below

    // 1. Info Message (Persistent/Top)
    if (!t.info.empty()) {
      std::cout << t.info << "\n";
    }

    // 2. Status Line
    // Helper to stringify gear
    auto gearToString = [](Gear g) -> std::string {
      switch (g) {
      case Gear::PARK:
        return "P";
      case Gear::REVERSE:
        return "R";
      case Gear::DRIVE:
        return "D";
      case Gear::LOW:
        return "L";
      default:
        return "N";
      }
    };

    std::string gear_display = gearToString(t.vehicle.gear);

    // If shifting, show transition
    if (t.shift_state != ShiftState::IDLE) {
      gear_display += "->" + gearToString(t.pending_gear);
    }

    std::cout << "[" << t.mode << "] "
              << "Gear: " << gear_display << " | ";

    // Real Speed & Set Speed (Command) & Steer
    std::cout << "Real: " << std::fixed << std::setprecision(1)
              << std::abs(t.vehicle.velocity * 3.6) << " km/hr | ";

    std::cout << "Set: " << (t.command.velocity * 3.6) << " km/hr | ";
    std::cout << "Steer: " << std::setprecision(2) << t.command.steer_angle
              << " rad";

    // Extra Info (Mode specific)
    if (!t.mode_status.empty()) {
      std::cout << " | " << t.mode_status;
    }

    std::cout << " | "; // Separator for Keys

    // Inputs (Visual: Upper=Hold, Lower=Active, .=None)
    auto getKeyChar = [](bool active, bool holding, char c) -> char {
      if (holding)
        return c; // Uppercase
      if (active)
        return (char)(c + 32); // Lowercase
      return '.';
    };

    std::cout << "[";
    std::cout << getKeyChar(input_.isActiveW(), input_.isHoldingW(), 'W');
    std::cout << getKeyChar(input_.isActiveA(), input_.isHoldingA(), 'A');
    std::cout << getKeyChar(input_.isActiveS(), input_.isHoldingS(), 'S');
    std::cout << getKeyChar(input_.isActiveD(), input_.isHoldingD(), 'D');
    std::cout << "]";

    std::cout << std::flush;
  }

private:
  void printHeader() {
    std::cout << "========================================" << std::endl;
    std::cout << "   Autoware Teleop                      " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  [W] Throttle  [S] Brake               " << std::endl;
    std::cout << "  [A] Left      [D] Right               " << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "  [Z] Auto/Local (Toggle)               " << std::endl;
    std::cout << "  [X] Drive  [C] Reverse  [V] Park      " << std::endl;
    std::cout << "  [SPACE] Emergency Stop / Resume       " << std::endl;
    std::cout << "  [R] Reset Initial Pose                " << std::endl;

    std::cout << "  [M] Switch Mode (";
    auto modes = autoware::manual_control::DriveModeFactory::instance()
                     .getAvailableModes();
    for (size_t i = 0; i < modes.size(); ++i) {
      std::string modeName =
          autoware::manual_control::DriveModeFactory::instance()
              .createMode(modes[i])
              ->getName();
      std::cout << modeName;
      if (i < modes.size() - 1)
        std::cout << "/";
    }
    std::cout << ")      " << std::endl;

    std::cout << "  [Q] Quit                              " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "\033[s"; // Save Cursor Position
  }

  const InputSystem &input_;
};

} // namespace autoware::manual_control

#endif // TELEOP_CONSOLE_UI_HPP
