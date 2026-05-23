// Zenoh-driven teleop entry point.

#include <cstdio>
#include <memory>
#include <string>

// zenoh.hxx must precede rclcpp to avoid macro/template clashes.
#include <zenoh.hxx>

#include <rclcpp/rclcpp.hpp>

#include "bridge/zenoh/zenoh_input.hpp"
#include "bridge/zenoh/zenoh_telemetry.hpp"
#include "common/types.hpp"
#include "core/control_runtime.hpp"
#include "core/drive_mode_factory.hpp"
#include "modes/cruise_mode.hpp"
#include "modes/physics_mode.hpp"
#include "modes/stop_mode.hpp"
#include "ros/manual_control_node.hpp"

using namespace autoware::manual_control;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Register Modes
  auto &factory = DriveModeFactory::instance();
  factory.registerMode(ModeType::PHYSICS,
                       []() { return std::make_unique<PhysicsDriveMode>(); });
  factory.registerMode(ModeType::CRUISE,
                       []() { return std::make_unique<CruiseDriveMode>(); });
  factory.registerMode(ModeType::STOP,
                       []() { return std::make_unique<StopDriveMode>(); });

  auto node = std::make_shared<ManualControlNode>(OperatorRole::REMOTE);

  auto declare = [&](const char *name, const rclcpp::ParameterValue &def) {
    if (!node->has_parameter(name))
      node->declare_parameter(name, def);
  };
  declare("zenoh_config", rclcpp::ParameterValue(std::string("")));
  declare("vehicle", rclcpp::ParameterValue(std::string("v1")));
  declare("arrival_timeout_ms", rclcpp::ParameterValue(500.0));
  declare("control_rate_hz", rclcpp::ParameterValue(60.0));

  // params-file values may arrive typed as int; read uniformly as double.
  auto as_double = [&](const char *name) -> double {
    auto p = node->get_parameter(name);
    return (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
               ? static_cast<double>(p.as_int())
               : p.as_double();
  };

  const std::string zenoh_cfg = node->get_parameter("zenoh_config").as_string();
  const std::string vehicle = node->get_parameter("vehicle").as_string();
  const double rate_hz = as_double("control_rate_hz");

  zenoh::Config zconf = zenoh_cfg.empty()
                            ? zenoh::Config::create_default()
                            : zenoh::Config::from_file(zenoh_cfg.c_str());
  auto session = std::make_shared<zenoh::Session>(
      zenoh::Session::open(std::move(zconf)));

  ZenohInputConfig in_cfg;
  in_cfg.intent_key = "manual_control/" + vehicle + "/intent";
  in_cfg.arrival_timeout_s =
      static_cast<float>(as_double("arrival_timeout_ms") / 1000.0);
  ZenohInput input(session, in_cfg);

  ZenohTelemetry telemetry(session, input,
                           "manual_control/" + vehicle + "/telemetry");

  RuntimeConfig cfg;
  cfg.rate_hz = rate_hz;

  std::fprintf(stderr, "[remote_control] vehicle=%s rate=%.0fHz intent=%s\n",
               vehicle.c_str(), rate_hz, in_cfg.intent_key.c_str());

  run_control_runtime(node, input, telemetry, cfg);

  rclcpp::shutdown();
  return 0;
}
