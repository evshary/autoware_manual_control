#ifndef TELEOP_TRANSPORT_ZENOH_CLI_PARAMS_HPP
#define TELEOP_TRANSPORT_ZENOH_CLI_PARAMS_HPP

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::manual_control::zenoh_params
{

// Immutable view of the parameters resolved at startup: the contents of the
// `--config <yaml>` file, a plain nested YAML document, flattened to dotted
// names (physics: { max_speed: x } -> "physics.max_speed"). A scalar is a
// one-element vector; a sequence keeps its elements. No ROS conventions: no
// node/parameter envelope to strip, no env overlay -- a pure load-time lookup.
class ParamMap
{
public:
  ParamMap() = default;
  explicit ParamMap(std::map<std::string, std::vector<std::string>> params)
  : params_(std::move(params)) {}

  // The values for a dotted name, or nullptr if it was not in the file.
  const std::vector<std::string> * lookup(const std::string & name) const;

private:
  std::map<std::string, std::vector<std::string>> params_;
};

// Parses argv once before any reader is built (extracts --config <path>).
void parse(int argc, char * argv[]);

// The startup-parsed map; lifetime spans the program.
const ParamMap & loaded();

} // namespace autoware::manual_control::zenoh_params

#endif // TELEOP_TRANSPORT_ZENOH_CLI_PARAMS_HPP
