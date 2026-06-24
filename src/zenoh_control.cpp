// Zenoh-driven teleop entry point.

#include <cstdio>
#include <memory>
#include <string>

// zenoh.hxx must be the first include: it defines macros that clash with later
// headers if they are seen first.
#include <zenoh.hxx>

#include <nlohmann/json.hpp>

#include "core/control_runtime.hpp"
#include "core/drive_mode_factory.hpp"
#include "core/register_modes.hpp"
#include "core/autoware_gateway.hpp"
#include "core/parameter_reader.hpp"
#include "io/intent/zenoh.hpp"
#include "io/telemetry/zenoh.hpp"

using namespace autoware::manual_control;

int main(int argc, char * argv[])
{
  AutowareGateway::init_system(argc, argv);

  auto gateway = std::make_shared<AutowareGateway>();
  auto reader = gateway->make_parameter_reader();

  register_all_modes(DriveModeFactory::instance(), *reader);
  activate_modes_from_config(DriveModeFactory::instance(), *reader);

  const std::string zenoh_cfg =
    reader->read<std::string>("zenoh_config", std::string{});
  const std::string scope =
    reader->read<std::string>("scope", std::string{"v1"});
  const float arrival_timeout_ms =
    reader->read<float>("arrival_timeout_ms", 500.0f);

  zenoh::Config zconf = zenoh_cfg.empty() ?
    zenoh::Config::create_default() :
    zenoh::Config::from_file(zenoh_cfg.c_str());
  auto session =
    std::make_shared<zenoh::Session>(zenoh::Session::open(std::move(zconf)));

  ZenohIntentConfig in_cfg;
  in_cfg.intent_key = "manual_control/" + scope + "/intent";
  in_cfg.arrival_timeout_s = arrival_timeout_ms / 1000.0f;
  ZenohIntent intent(session, in_cfg);

  ZenohTelemetry telemetry(session, "manual_control/" + scope + "/telemetry");
  telemetry.set_aux(
    [&intent](nlohmann::json & j) {
      j["watchdog_tripped"] = intent.watchdog_tripped();
    });

  RuntimeConfig cfg = RuntimeConfig::load(*reader);

  std::fprintf(
    stderr, "[zenoh_control] scope=%s rate=%.0fHz intent=%s\n",
    scope.c_str(), cfg.rate_hz, in_cfg.intent_key.c_str());

  // No startup pose seed: seeding would teleport the vehicle and break self-localization.
  run_control_runtime(gateway, intent, telemetry, cfg);

  AutowareGateway::shutdown_system();
  return 0;
}
