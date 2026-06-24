// Standalone keyboard teleop entry point.

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "core/control_runtime.hpp"
#include "core/drive_mode_factory.hpp"
#include "core/register_modes.hpp"
#include "core/autoware_gateway.hpp"
#include "core/parameter_reader.hpp"
#include "io/intent/keyboard.hpp"
#include "io/telemetry/console.hpp"

using namespace autoware::manual_control;

int main(int argc, char * argv[])
{
  AutowareGateway::init_system(argc, argv);

  auto gateway = std::make_shared<AutowareGateway>();
  auto reader = gateway->make_parameter_reader();

  register_all_modes(DriveModeFactory::instance(), *reader);
  activate_modes_from_config(DriveModeFactory::instance(), *reader);

  KeyboardIntent keyboard;
  ConsoleTelemetry console;
  console.init();

  // Live WASD echo (UPPER=held, lower=tapped, '.'=idle).
  console.set_extra_line(
    [&keyboard] {
      auto glyph = [](KeyHold h, char c) {
        return h == KeyHold::Held ? c :
        h == KeyHold::Tapped ? static_cast<char>(c + 32) :
        '.';
      };
      return std::string("Keys: [") + glyph(keyboard.w_state(), 'W') +
      glyph(keyboard.a_state(), 'A') + glyph(keyboard.s_state(), 'S') +
      glyph(keyboard.d_state(), 'D') + "]";
    });

  RuntimeConfig cfg = RuntimeConfig::load(*reader);

  // Spin first so DDS discovers peers and the latched localization state lands.
  for (int i = 0; i < 20 && AutowareGateway::ok_system(); ++i) {
    gateway->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!gateway->is_localized()) {
    gateway->reset_initial_pose();
  }

  run_control_runtime(gateway, keyboard, console, cfg);

  AutowareGateway::shutdown_system();
  return 0;
}
