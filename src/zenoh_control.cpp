// Zenoh-driven teleop entry point.

#include <cstdio>
#include <memory>
#include <string>

// zenoh.hxx must precede rclcpp to avoid macro/template clashes.
#include <zenoh.hxx>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "core/control_runtime.hpp"
#include "core/drive_mode_factory.hpp"
#include "core/param_utils.hpp"
#include "core/register_modes.hpp"
#include "io/intent/zenoh.hpp"
#include "io/telemetry/zenoh.hpp"
#include "ros/manual_control_node.hpp"

using namespace autoware::manual_control;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ManualControlNode>();
  register_all_modes(DriveModeFactory::instance(), *node);
  activate_modes_from_config(DriveModeFactory::instance(), *node);

  const std::string zenoh_cfg =
      load_param<std::string>(*node, "zenoh_config", std::string{});
  const std::string scope =
      load_param<std::string>(*node, "scope", std::string{"v1"});
  const float arrival_timeout_ms =
      load_float(*node, "arrival_timeout_ms", 500.0f);

  zenoh::Config zconf = zenoh_cfg.empty()
                            ? zenoh::Config::create_default()
                            : zenoh::Config::from_file(zenoh_cfg.c_str());
  auto session =
      std::make_shared<zenoh::Session>(zenoh::Session::open(std::move(zconf)));

  ZenohIntentConfig in_cfg;
  in_cfg.intent_key = "manual_control/" + scope + "/intent";
  in_cfg.arrival_timeout_s = arrival_timeout_ms / 1000.0f;
  ZenohIntent intent(session, in_cfg);

  ZenohTelemetry telemetry(session, "manual_control/" + scope + "/telemetry");
  telemetry.set_aux([&intent](nlohmann::json &j) {
    j["watchdog_tripped"] = intent.watchdog_tripped();
  });

  RuntimeConfig cfg = RuntimeConfig::load(*node);

  std::fprintf(stderr, "[zenoh_control] scope=%s rate=%.0fHz intent=%s\n",
               scope.c_str(), cfg.rate_hz, in_cfg.intent_key.c_str());

  // No startup pose seed: seeding would teleport the vehicle and break self-localization.
  run_control_runtime(node, intent, telemetry, cfg);

  rclcpp::shutdown();
  return 0;
}
