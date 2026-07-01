// Contract gate for the native (zenoh) ParameterReader: the libyaml DOM flattening
// and the read<T> contract the teleop owns -- dotted keys, type fallback, empty and
// present-but-empty semantics. The config is plain nested YAML (no ros__parameters
// envelope); the binary takes it via --config.

#include "core/parameter_reader.hpp"
#include "transport/zenoh/cli_params.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <unistd.h>

namespace autoware::manual_control
{
// Transport-internal factory (defined in transport/zenoh/parameter_reader.cpp).
std::unique_ptr<ParameterReader> make_zenoh_parameter_reader();
} // namespace autoware::manual_control

using namespace autoware::manual_control;

namespace
{

// Writes a neutral config to a temp path, parses it via --config, and returns a
// reader bound to the resulting map.
std::unique_ptr<ParameterReader> reader_for(const std::string & yaml)
{
  char tmpl[] = "/tmp/teleop_param_XXXXXX";
  int fd = mkstemp(tmpl);
  if (fd >= 0) {close(fd);}
  const std::string path(tmpl);
  std::ofstream(path) << yaml;
  char arg0[] = "test";
  char arg1[] = "--config";
  std::vector<char> p(path.begin(), path.end());
  p.push_back('\0');
  char * argv[] = {arg0, arg1, p.data()};
  zenoh_params::parse(3, argv);
  return make_zenoh_parameter_reader();
}

// A reader over no config file at all (everything falls back).
std::unique_ptr<ParameterReader> reader_no_file()
{
  char arg0[] = "test";
  char * argv[] = {arg0};
  zenoh_params::parse(1, argv);
  return make_zenoh_parameter_reader();
}

const char * kFullConfig =
  R"YAML(
operator_mode: remote
scope: v1
arrival_timeout_ms: 500.0
zenoh_config: ""
init_pose:
  presets:
    names: ["origin", "intersection"]
    origin: [0.0, 0.0, 0.0, 0.0]
modes: ["stop", "physics", "cruise"]
physics:
  max_speed: 27.78            # m/s
)YAML";

} // namespace

// 1. A quoted value with a trailing comment yields the bare value.
TEST(ParamReader, QuotedValueWithTrailingComment) {
  auto r = reader_for("key: \"value\"  # note\n");
  EXPECT_EQ(r->read<std::string>("key", std::string{"DEF"}), "value");
}

// 2. A '#' inside quotes is literal, not a comment.
TEST(ParamReader, HashInsideQuotesIsLiteral) {
  auto r = reader_for("key: \"a#b\"\n");
  EXPECT_EQ(r->read<std::string>("key", std::string{"DEF"}), "a#b");
}

// 3. Nested mapping flattens to a dotted key.
TEST(ParamReader, NestedDottedKey) {
  auto r = reader_for(kFullConfig);
  EXPECT_FLOAT_EQ(r->read<float>("physics.max_speed", -1.0f), 27.78f);
}

// 4. Inline string sequence.
TEST(ParamReader, InlineStringSequence) {
  auto r = reader_for(kFullConfig);
  auto modes = r->read<std::vector<std::string>>("modes", {});
  ASSERT_EQ(modes.size(), 3u);
  EXPECT_EQ(modes[0], "stop");
  EXPECT_EQ(modes[1], "physics");
  EXPECT_EQ(modes[2], "cruise");
}

// 5. Inline float sequence (deeply nested key).
TEST(ParamReader, InlineFloatSequence) {
  auto r = reader_for(kFullConfig);
  auto origin = r->read<std::vector<std::string>>("init_pose.presets.origin", {});
  ASSERT_EQ(origin.size(), 4u);
  EXPECT_EQ(origin[0], "0.0");
  EXPECT_EQ(origin[3], "0.0");
}

// 6. Empty-string scalar. The read<string> contract treats an empty value as
//    "use the default", so with default "" the result is "" (empty is the
//    sentinel).
TEST(ParamReader, EmptyStringScalar) {
  auto r = reader_for(kFullConfig);
  EXPECT_EQ(r->read<std::string>("zenoh_config", std::string{}), "");
}

// 7. Full-line and trailing comments / blank lines are ignored.
TEST(ParamReader, CommentsAndBlankLinesIgnored) {
  auto r = reader_for(
    "# a full-line comment\n"
    "\n"
    "scope: v1   # trailing comment\n");
  EXPECT_EQ(r->read<std::string>("scope", std::string{"DEF"}), "v1");
}

// 8. Sibling sections coexist: each flattens to its own dotted prefix, neither
//    bleeds into the other.
TEST(ParamReader, SiblingSectionsFlattenIndependently) {
  auto r = reader_for(
    "physics:\n"
    "  max_speed: 10.0\n"
    "cruise:\n"
    "  max_speed: 20.0\n");
  EXPECT_FLOAT_EQ(r->read<float>("physics.max_speed", -1.0f), 10.0f);
  EXPECT_FLOAT_EQ(r->read<float>("cruise.max_speed", -1.0f), 20.0f);
}

// 9. Type-mismatch fallback: a non-numeric value falls back to the default.
TEST(ParamReader, TypeMismatchFallsBack) {
  auto r = reader_for("physics:\n  max_speed: abc\n");
  EXPECT_FLOAT_EQ(r->read<float>("physics.max_speed", 9.0f), 9.0f);
}

// 10. Missing / unreadable file: every read returns its default, no crash.
TEST(ParamReader, MissingFileAllDefaults) {
  char arg0[] = "test";
  char arg1[] = "--config";
  char arg2[] = "/no/such/config.yaml";
  char * argv[] = {arg0, arg1, arg2};
  zenoh_params::parse(3, argv);
  auto r = make_zenoh_parameter_reader();
  EXPECT_EQ(r->read<std::string>("scope", std::string{"DEF"}), "DEF");
  EXPECT_FLOAT_EQ(r->read<float>("physics.max_speed", 1.5f), 1.5f);

  auto r2 = reader_no_file();
  EXPECT_EQ(r2->read<std::string>("scope", std::string{"DEF"}), "DEF");
}

// 11. Present-but-empty semantics. A string sequence present as `[]` wins over the
//     default (so an empty `modes:` reaches the factory and aborts there); an absent
//     key falls back. A numeric sequence instead treats present-empty as unset.
TEST(ParamReader, PresentEmptySequence) {
  auto present = reader_for("modes: []\n");
  EXPECT_TRUE(present->read<std::vector<std::string>>("modes", {"stop"}).empty());

  auto absent = reader_for("scope: v1\n");
  auto def = absent->read<std::vector<std::string>>("modes", {"stop"});
  ASSERT_EQ(def.size(), 1u);
  EXPECT_EQ(def[0], "stop");

  auto pose = reader_for("start: []\n");
  EXPECT_EQ(pose->read<std::vector<double>>("start", {1.0, 2.0}).size(), 2u);
}
