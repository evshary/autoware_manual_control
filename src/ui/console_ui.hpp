
#ifndef TELEOP_CONSOLE_UI_HPP
#define TELEOP_CONSOLE_UI_HPP

#include "common/types.hpp"
#include "core/drive_mode_factory.hpp"
#include "core/mode_manager.hpp"
#include "input/input_system.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>

namespace autoware::manual_control {

class ConsoleUI {
public:
  void init() {
    std::cout << "\033[2J\033[1;1H"; // Clear screen
    printHeader();
  }

  void refresh(const InputSystem &input, const ModeManager &manager,
               const VehicleState &state, const ControlCommand &cmd,
               ShiftState shiftState, Gear pendingGear) {
    // Only refresh at reasonable rate (~10Hz) to avoid flickering
    static int frame = 0;
    if (frame++ % 6 != 0)
      return;

    std::cout << "\r\033[K"; // Clear line

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

    std::string gear_display = gearToString(state.gear);

    // If shifting, show transition
    if (shiftState != ShiftState::IDLE) {
      // Blinking arrow? or just static ->
      gear_display += "->" + gearToString(pendingGear);
    }

    std::cout << "[" << manager.getCurrentModeName() << "] "
              << "Gear: " << gear_display << " | ";

    // 2. Real Speed & Set Speed (Command) & Steer
    std::cout << "Real: " << std::fixed << std::setprecision(1)
              << std::abs(state.velocity * 3.6) << " km/hr | ";

    std::cout << "Set: " << (cmd.velocity * 3.6) << " km/hr | ";
    std::cout << "Steer: " << std::setprecision(2) << cmd.steer_angle << " rad";

    // 3. Extra Info (Mode specific)
    std::string status = manager.getStatusString();
    if (!status.empty()) {
      std::cout << " | " << status;
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
    std::cout << getKeyChar(input.isActiveW(), input.isHoldingW(), 'W');
    std::cout << getKeyChar(input.isActiveA(), input.isHoldingA(), 'A');
    std::cout << getKeyChar(input.isActiveS(), input.isHoldingS(), 'S');
    std::cout << getKeyChar(input.isActiveD(), input.isHoldingD(), 'D');
    std::cout << "]";

    std::cout << std::flush;
  }

private:
  void printHeader() {
    std::cout << "========================================" << std::endl;
    std::cout << "   Autoware Teleop (Refactored)         " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  [W] Throttle  [S] Brake               " << std::endl;
    std::cout << "  [A] Left      [D] Right               " << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "  [Z] Auto/Local (Toggle)               " << std::endl;
    std::cout << "  [X] Drive  [C] Reverse  [V] Park      " << std::endl;
    std::cout << "  [SPACE] Emergency Stop                " << std::endl;
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
  }
};

} // namespace autoware::manual_control

#endif // TELEOP_CONSOLE_UI_HPP
