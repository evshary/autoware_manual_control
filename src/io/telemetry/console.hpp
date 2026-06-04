#ifndef TELEOP_IO_TELEMETRY_CONSOLE_HPP
#define TELEOP_IO_TELEMETRY_CONSOLE_HPP

#include "common/telemetry.hpp"
#include "common/types.hpp"
#include "core/drive_mode_factory.hpp"
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <utility>

namespace autoware::manual_control {

// Console TelemetrySink: renders Telemetry to the terminal; sole stdout writer.
class ConsoleTelemetry {
public:
  void init() {
    std::cout << "\033[2J\033[1;1H"; // Clear screen
    printHeader();
  }

  // Optional extra status line, rendered each tick; console stays agnostic to its content.
  void set_extra_line(std::function<std::string()> provider) {
    extra_line_ = std::move(provider);
  }

  void publish(const Telemetry &t) {
    // Only refresh at reasonable rate (~10Hz) to avoid flickering
    static int frame = 0;
    if (frame++ % 6 != 0)
      return;

    std::cout << "\033[u";  // restore cursor
    std::cout << "\033[J";  // clear below

    if (!t.info.empty()) {
      std::cout << t.info << "\n";
    }

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

    if (t.shift_state != ShiftState::IDLE) {
      gear_display += "->" + gearToString(t.pending_gear);
    }

    std::cout << "[" << t.mode << "] "
              << "Op:" << (t.operation_mode.empty() ? "?" : t.operation_mode)
              << " | "
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

    if (extra_line_) {
      std::cout << "\n" << extra_line_();
    }

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
    std::cout << "  [Z] Toggle STOP / drive mode          " << std::endl;
    std::cout << "  [X] Drive  [C] Reverse  [V] Park      " << std::endl;
    std::cout << "  [SPACE] Emergency Stop / Resume       " << std::endl;
    std::cout << "  [R] Reset Initial Pose                " << std::endl;

    std::cout << "  [M] Switch Mode (";
    const auto &modes = DriveModeFactory::instance().activeOrder();
    for (size_t i = 0; i < modes.size(); ++i) {
      std::cout << modes[i];
      if (i < modes.size() - 1)
        std::cout << "/";
    }
    std::cout << ")      " << std::endl;

    std::cout << "  [Q] Quit                              " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "\033[s"; // Save Cursor Position
  }

  std::function<std::string()> extra_line_;
};

} // namespace autoware::manual_control

#endif // TELEOP_IO_TELEMETRY_CONSOLE_HPP
