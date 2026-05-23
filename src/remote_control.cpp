// Zenoh-driven teleop entry point.

#include <cstdio>
#include <memory>
#include <string>

// zenoh.hxx must precede rclcpp to avoid macro/template clashes.
#include <zenoh.hxx>

#include <rclcpp/rclcpp.hpp>

#include "bridge/zenoh/zenoh_input.hpp"
#include "bridge/zenoh/zenoh_telemetry.hpp"
#include "core/control_runtime.hpp"
#include "core/drive_mode_factory.hpp"
#include "core/params_loader.hpp"
#include "ros/manual_control_node.hpp"

using namespace autoware::manual_control;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ManualControlNode>(OperatorRole::REMOTE);
  register_default_modes(DriveModeFactory::instance(), *node);

  const std::string zenoh_cfg =
      load_param<std::string>(*node, "zenoh_config", std::string{});
  const std::string vehicle =
      load_param<std::string>(*node, "vehicle", std::string{"v1"});
  const float arrival_timeout_ms =
      load_float(*node, "arrival_timeout_ms", 500.0f);

  zenoh::Config zconf = zenoh_cfg.empty()
                            ? zenoh::Config::create_default()
                            : zenoh::Config::from_file(zenoh_cfg.c_str());
  auto session = std::make_shared<zenoh::Session>(
      zenoh::Session::open(std::move(zconf)));

  ZenohInputConfig in_cfg;
  in_cfg.intent_key = "manual_control/" + vehicle + "/intent";
  in_cfg.arrival_timeout_s = arrival_timeout_ms / 1000.0f;
  ZenohInput input(session, in_cfg);

  ZenohTelemetry telemetry(session, input,
                           "manual_control/" + vehicle + "/telemetry");

  RuntimeConfig cfg = load_runtime_config(*node);

  std::fprintf(stderr, "[remote_control] vehicle=%s rate=%.0fHz intent=%s\n",
               vehicle.c_str(), cfg.rate_hz, in_cfg.intent_key.c_str());

  run_control_runtime(node, input, telemetry, cfg);

  rclcpp::shutdown();
  return 0;
}
