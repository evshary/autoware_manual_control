// Byte-perfect verification gate for the native_zenoh CDR codec (rosidl's own
// rosidl_typesupport_fastrtps_cpp cdr_serialize / cdr_deserialize, driven
// through the Cdr-encapsulation helper this transport publishes with).
//
// The codec bodies are rosidl-generated, so the wire bytes are correct by
// construction. What this gate certifies is OUR additions around them:
//  (a) tools/stage_idl.py's namespace rewrite yields a codec whose bytes match
//      rmw across EVERY field SHAPE, not only the shapes the ten production
//      messages happen to use today;
//  (b) the src/transport/zenoh/cdr.hpp Cdr helper (encapsulation + buffer + length)
//      is rmw-identical in both directions;
//  (c) the decode direction is rmw-identical -- fed EXTERNAL reference bytes,
//      not just our own re-read.
//
// Three oracles, each independent of the others:
//  1. rmw_cyclonedds goldens (test/capture_golden.py, rclpy serialize_message
//     under RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -- the exact XCDR1/CDR_LE wire
//     DDS puts on the network). Every fixture asserts BOTH
//     encode(value) == golden AND decode(golden) parses back to value, and the
//     length is asserted EXACT (no trailing-slack window a wrong final member
//     could hide in). Regenerate with capture_golden.py after an Autoware msg
//     bump; capture_golden.py --check diffs the live wire against these.
//  2. The ros2/test_interface_files test_msgs corpus (vendored .idl resolved by
//     CMake; stable fixtures). Run through the SAME stage_idl -> rosidl codec ->
//     cdr.hpp path the production codec uses, so stage_idl is proven on every
//     shape: all int widths signed/unsigned, float32/64, bool, byte, char,
//     unbounded + bounded + empty strings, fixed arrays incl. nested and the
//     trailing alignment_check member, bounded + unbounded sequences, single and
//     multi-level nesting, constants, defaults, empty. (WStrings is excluded:
//     rmw_cyclonedds encodes wstring UTF-32 while fastrtps uses UTF-16, an
//     upstream incompatibility; no production message uses wstring.)
//  3. @foxglove/cdr byte vectors (MIT; see test/third_party/foxglove_cdr) run
//     through cdr.hpp directly. These come from a DIFFERENT implementation than
//     rmw, so a bug shared by rmw and our codec cannot pass both oracles, and
//     CdrReader.test.ts gives byte -> value (a decoder oracle independent of our
//     own encoder).

#include "transport/zenoh/cdr.hpp"

#include "autoware_manual_control/msg/control.hpp"
#include "autoware_manual_control/msg/gear_command.hpp"
#include "autoware_manual_control/msg/gear_report.hpp"
#include "autoware_manual_control/msg/localization_initialization_state.hpp"
#include "autoware_manual_control/msg/manual_operator_heartbeat.hpp"
#include "autoware_manual_control/msg/operation_mode_state.hpp"
#include "autoware_manual_control/msg/pose_with_covariance_stamped.hpp"
#include "autoware_manual_control/msg/steering_report.hpp"
#include "autoware_manual_control/msg/velocity_report.hpp"
#include "autoware_manual_control/srv/change_operation_mode.hpp"

#include "autoware_manual_control/msg/detail/control__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/gear_command__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/gear_report__rosidl_typesupport_fastrtps_cpp.hpp"
#include \
  "autoware_manual_control/msg/detail/localization_initialization_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include \
  "autoware_manual_control/msg/detail/manual_operator_heartbeat__rosidl_typesupport_fastrtps_cpp.hpp"
#include \
  "autoware_manual_control/msg/detail/operation_mode_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include \
  "autoware_manual_control/msg/detail/pose_with_covariance_stamped__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/steering_report__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/velocity_report__rosidl_typesupport_fastrtps_cpp.hpp"
#include \
  "autoware_manual_control/srv/detail/change_operation_mode__rosidl_typesupport_fastrtps_cpp.hpp"

// test_msgs shape-coverage fixtures (built only under BUILD_TESTING).
#include "autoware_manual_control/msg/arrays.hpp"
#include "autoware_manual_control/msg/basic_types.hpp"
#include "autoware_manual_control/msg/bounded_sequences.hpp"
#include "autoware_manual_control/msg/constants.hpp"
#include "autoware_manual_control/msg/defaults.hpp"
#include "autoware_manual_control/msg/empty.hpp"
#include "autoware_manual_control/msg/multi_nested.hpp"
#include "autoware_manual_control/msg/nested.hpp"
#include "autoware_manual_control/msg/strings.hpp"
#include "autoware_manual_control/msg/unbounded_sequences.hpp"

#include "autoware_manual_control/msg/detail/arrays__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/basic_types__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/bounded_sequences__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/constants__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/defaults__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/empty__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/multi_nested__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/nested__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/strings__rosidl_typesupport_fastrtps_cpp.hpp"
#include \
  "autoware_manual_control/msg/detail/unbounded_sequences__rosidl_typesupport_fastrtps_cpp.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

using namespace autoware::manual_control;
using namespace autoware_manual_control::msg;
using namespace autoware_manual_control::srv;
// Codec lives in a nested typesupport namespace per IDL module; bring both in.
using autoware_manual_control::msg::typesupport_fastrtps_cpp::cdr_deserialize;
using autoware_manual_control::msg::typesupport_fastrtps_cpp::cdr_serialize;
using autoware_manual_control::srv::typesupport_fastrtps_cpp::cdr_deserialize;
using autoware_manual_control::srv::typesupport_fastrtps_cpp::cdr_serialize;

// capture_golden.py --check parses these literals; keep them as GOLDEN(name, hex).
#define GOLDEN(name, ...) (std::string(__VA_ARGS__))

namespace
{

template<typename Msg> std::vector<uint8_t> encode(const Msg & m)
{
  cdr::CdrWriter w;
  cdr_serialize(m, w.cdr());
  return w.bytes();
}

template<typename Msg> Msg decode(const std::vector<uint8_t> & bytes)
{
  cdr::CdrReader r(bytes);
  Msg out;
  cdr_deserialize(r.cdr(), out);
  return out;
}

std::vector<uint8_t> from_hex(const std::string & h)
{
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < h.size(); i += 2) {
    out.push_back(static_cast<uint8_t>(std::stoul(h.substr(i, 2), nullptr, 16)));
  }
  return out;
}

std::string to_hex(const std::vector<uint8_t> & b)
{
  static const char * d = "0123456789abcdef";
  std::string s;
  for (uint8_t c : b) {
    s += d[c >> 4];
    s += d[c & 0xf];
  }
  return s;
}

// Length-exact byte assert: the encoder output must equal the rmw reference
// byte-for-byte, same length. No trailing-slack window -- a wrong final-member
// value (the reason test_msgs carries a trailing alignment_check field) cannot
// hide past the encoder's last byte.
void expect_golden(const std::vector<uint8_t> & got, const std::string & ref_hex)
{
  auto ref = from_hex(ref_hex);
  EXPECT_EQ(got.size(), ref.size()) << "encoder length != rmw length";
  EXPECT_EQ(to_hex(got), to_hex(ref));
}

// Two-directional gate for a value with a working operator== (i.e. no NaN): the
// encoder must reproduce the rmw golden exactly, AND the rmw golden must decode
// back to the same value (external decoder oracle -- bytes from rmw, not ours).
template<typename Msg>
void expect_golden_bidi(const Msg & m, const std::string & ref_hex)
{
  expect_golden(encode(m), ref_hex);
  EXPECT_TRUE(decode<Msg>(from_hex(ref_hex)) == m)
    << "rmw golden did not decode back to the source value";
}

} // namespace

// Oracle 1a -- production wire contract: two-directional, length-exact.
// The literal control messages; SAFETY-CRITICAL control + gear first.

TEST(Production, ControlCommand_SafetyCritical) {
  Control c;
  c.stamp.sec = 1;
  c.stamp.nanosec = 2;
  c.lateral.steering_tire_angle = 0.5f;
  c.longitudinal.velocity = 3.0f;
  c.longitudinal.acceleration = 1.25f;
  expect_golden_bidi(
    c, GOLDEN(
      "ControlCommand_SafetyCritical",
      "0001000001000000020000000000000000000000000000000000000000000000"
      "000000000000003f000000000000000000000000000000000000000000000000"
      "000040400000a03f000000000000"));
}

// Byte-pin the safety-critical float edges: NaN / +inf / -inf / large negative.
// NaN breaks operator==, so assert bytes + bit patterns directly.
TEST(Production, ControlCommand_EdgeFloats) {
  Control c;
  c.lateral.steering_tire_angle = std::numeric_limits<float>::quiet_NaN();
  c.longitudinal.velocity = std::numeric_limits<float>::infinity();
  c.longitudinal.acceleration = -std::numeric_limits<float>::infinity();
  c.longitudinal.jerk = -3.4e38f;
  expect_golden(
    encode(c),
    GOLDEN(
      "ControlCommand_EdgeFloats",
      "0001000000000000000000000000000000000000000000000000000000000000"
      "000000000000c07f000000000000000000000000000000000000000000000000"
      "0000807f000080ff9ec97fff0000"));
  Control out = decode<Control>(encode(c));
  EXPECT_TRUE(std::isnan(out.lateral.steering_tire_angle));
  EXPECT_TRUE(std::isinf(out.longitudinal.velocity));
  EXPECT_GT(out.longitudinal.velocity, 0.0f);
  EXPECT_TRUE(std::isinf(out.longitudinal.acceleration));
  EXPECT_LT(out.longitudinal.acceleration, 0.0f);
  EXPECT_FLOAT_EQ(out.longitudinal.jerk, -3.4e38f);
}

TEST(Production, GearCommand_SafetyCritical) {
  GearCommand g;
  g.stamp.sec = 5;
  g.stamp.nanosec = 6;
  g.command = 22; // PARK
  expect_golden_bidi(
    g, GOLDEN(
      "GearCommand_SafetyCritical",
      "00010000050000000600000016"));
}

// Enum boundaries (0 and 255 / max-uint8) for the gear command field.
TEST(Production, GearCommand_EnumZero) {
  GearCommand g;
  g.command = 0; // NONE
  expect_golden_bidi(
    g, GOLDEN(
      "GearCommand_EnumZero",
      "00010000000000000000000000"));
}

TEST(Production, GearCommand_EnumMax) {
  GearCommand g;
  g.command = 255;
  expect_golden_bidi(
    g, GOLDEN(
      "GearCommand_EnumMax",
      "000100000000000000000000ff"));
}

TEST(Production, ManualOperatorHeartbeat) {
  ManualOperatorHeartbeat h;
  h.stamp.sec = 21;
  h.stamp.nanosec = 22;
  h.ready = true;
  expect_golden_bidi(
    h, GOLDEN(
      "ManualOperatorHeartbeat",
      "00010000150000001600000001"));
}

TEST(Production, PoseWithCovarianceStamped_InitialPose) {
  PoseWithCovarianceStamped p;
  p.header.stamp.sec = 7;
  p.header.stamp.nanosec = 8;
  p.header.frame_id = "map";
  p.pose.pose.position.x = 1.5;
  p.pose.pose.position.y = -2.25;
  p.pose.pose.position.z = 0.0;
  double yaw = 1.0;
  p.pose.pose.orientation.z = std::sin(yaw * 0.5);
  p.pose.pose.orientation.w = std::cos(yaw * 0.5);
  for (size_t i = 0; i < 36; ++i) {
    p.pose.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;
  }
  expect_golden_bidi(
    p, GOLDEN(
      "PoseWithCovarianceStamped_InitialPose",
      "000100000700000008000000040000006d617000000000000000f83f00000000"
      "000002c0000000000000000000000000000000000000000000000000f0054b74"
      "e8aede3f507d5b062815ec3f9a9999999999b93f000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "000000009a9999999999b93f0000000000000000000000000000000000000000"
      "000000000000000000000000000000000000000000000000000000009a999999"
      "9999b93f00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000009a9999999999b93f00000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000009a9999999999b93f000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "000000009a9999999999b93f"));
}

TEST(Production, OperationModeState) {
  OperationModeState o;
  o.stamp.sec = 17;
  o.stamp.nanosec = 18;
  o.mode = 2; // AUTONOMOUS
  o.is_autoware_control_enabled = true;
  o.is_remote_mode_available = true;
  expect_golden_bidi(
    o, GOLDEN(
      "OperationModeState",
      "00010000110000001200000002010000000001"));
}

TEST(Production, OperationModeState_EnumMax) {
  OperationModeState o;
  o.mode = 255;
  expect_golden_bidi(
    o, GOLDEN(
      "OperationModeState_EnumMax",
      "000100000000000000000000ff000000000000"));
}

TEST(Production, LocalizationInitializationState) {
  LocalizationInitializationState l;
  l.stamp.sec = 19;
  l.stamp.nanosec = 20;
  l.state = 3; // INITIALIZED
  expect_golden_bidi(
    l, GOLDEN(
      "LocalizationInitializationState",
      "0001000013000000140000000300"));
}

TEST(Production, VelocityReport) {
  VelocityReport v;
  v.header.stamp.sec = 13;
  v.header.stamp.nanosec = 14;
  v.header.frame_id = "base_link";
  v.longitudinal_velocity = 2.0f;
  v.lateral_velocity = 0.5f;
  v.heading_rate = -0.25f;
  expect_golden_bidi(
    v, GOLDEN(
      "VelocityReport",
      "000100000d0000000e0000000a000000626173655f6c696e6b00000000000040"
      "0000003f000080be"));
}

TEST(Production, GearReport) {
  GearReport g;
  g.stamp.sec = 9;
  g.stamp.nanosec = 10;
  g.report = 2;
  expect_golden_bidi(
    g, GOLDEN(
      "GearReport",
      "00010000090000000a00000002"));
}

TEST(Production, SteeringReport) {
  SteeringReport s;
  s.stamp.sec = 15;
  s.stamp.nanosec = 16;
  s.steering_tire_angle = 0.25f;
  expect_golden_bidi(
    s, GOLDEN(
      "SteeringReport",
      "000100000f000000100000000000803e"));
}

TEST(Production, ChangeOperationMode_Request_Empty) {
  ChangeOperationMode_Request req; // Empty srv: a single padding member.
  expect_golden_bidi(
    req, GOLDEN(
      "ChangeOperationMode_Request_Empty",
      "0001000000"));
}

TEST(Production, ChangeOperationMode_Response) {
  ChangeOperationMode_Response resp;
  resp.status.success = true;
  resp.status.code = 60001;
  resp.status.message = "no effect";
  expect_golden_bidi(
    resp, GOLDEN(
      "ChangeOperationMode_Response",
      "00010000010061ea0a0000006e6f2065666665637400"));
}

// Oracle 2 -- test_msgs full-shape coverage. Same stage_idl -> codec -> cdr.hpp
// path as production. Each fixture is two-directional + length-exact; the
// nesting-heavy ones additionally re-encode the decoded message to prove the
// decode consumed the external rmw bytes correctly (a decode that mis-parsed
// would re-encode differently). Field values mirror test/capture_golden.py.

namespace
{

// All int widths signed/unsigned at their extremes + float/bool/byte/char.
BasicTypes make_basic_types()
{
  BasicTypes b;
  b.bool_value = true;
  b.byte_value = 255;
  b.char_value = 65;
  b.float32_value = -1.5f;
  b.float64_value = 2.5;
  b.int8_value = -128;
  b.uint8_value = 200;
  b.int16_value = -32768;
  b.uint16_value = 65535;
  b.int32_value = -2147483648;
  b.uint32_value = 4294967295u;
  b.int64_value = INT64_MIN;
  b.uint64_value = UINT64_MAX;
  return b;
}

Arrays make_arrays()
{
  Arrays a;
  a.bool_values = {true, false, true};
  a.byte_values = {0, 127, 255};
  a.char_values = {1, 2, 3};
  a.float32_values = {1.5f, 0.0f, -1.5f};
  a.float64_values = {2.5, 0.0, -2.5};
  a.int8_values = {-128, 0, 127};
  a.uint8_values = {0, 1, 255};
  a.int16_values = {-32768, 0, 32767};
  a.uint16_values = {0, 1, 65535};
  a.int32_values = {-2147483648, 0, 2147483647};
  a.uint32_values = {0, 1, 4294967295u};
  a.int64_values = {INT64_MIN, 0, INT64_MAX};
  a.uint64_values = {0, 1, UINT64_MAX};
  a.string_values = {"a", "", "ccc"};
  // nested struct arrays default-constructed (defaults applied by the codec).
  a.alignment_check = 0x0a0b0c0d; // trailing member after the array fields
  return a;
}

UnboundedSequences make_unbounded()
{
  UnboundedSequences u;
  u.bool_values = {true, false};
  u.int32_values = {-1, 0, 2147483647};
  u.float64_values = {1.5, -2.5};
  u.string_values = {"x", "", "zzz"};
  u.uint8_values = {1, 2, 3, 4};
  u.alignment_check = 0x11223344;
  return u;
}

UnboundedSequences make_unbounded_empty()
{
  UnboundedSequences u; // all sequences empty
  u.alignment_check = 0x55667788;
  return u;
}

BoundedSequences make_bounded()
{
  BoundedSequences b;
  b.bool_values = {true, true, false};
  b.int32_values = {7, 8};
  b.float64_values = {-9.5};
  b.string_values = {"p", "qq"};
  b.uint8_values = {9, 8};
  b.alignment_check = 0x0d0c0b0a;
  return b;
}

} // namespace

TEST(Shapes, BasicTypes) {
  expect_golden_bidi(
    make_basic_types(),
    GOLDEN(
      "TM_BasicTypes",
      "0001000001ff41000000c0bf000000000000044080c80080ffff000000000080"
      "ffffffff0000000000000080ffffffffffffffff"));
}

TEST(Shapes, BasicTypes_Zero) {
  expect_golden_bidi(
    BasicTypes(),
    GOLDEN(
      "TM_BasicTypes_Zero",
      "0001000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000"));
}

TEST(Shapes, Strings) {
  // Unbounded + bounded <=22 + empty strings. The struct default-constructs the
  // *_default members to their IDL @default text; clear them so every shape is
  // pinned explicitly (and matches the rclpy-built golden, which starts empty).
  Strings s;
  s.string_value = "hello";
  s.bounded_string_value = "bnd";
  s.string_value_default1 = "";
  s.string_value_default2 = "";
  s.string_value_default3 = "";
  s.string_value_default4 = "";
  s.string_value_default5 = "";
  s.bounded_string_value_default1 = "";
  s.bounded_string_value_default2 = "";
  s.bounded_string_value_default3 = "";
  s.bounded_string_value_default4 = "";
  s.bounded_string_value_default5 = "";
  expect_golden_bidi(
    s, GOLDEN(
      "TM_Strings",
      "000100000600000068656c6c6f00000001000000000000000100000000000000"
      "01000000000000000100000000000000010000000000000004000000626e6400"
      "0100000000000000010000000000000001000000000000000100000000000000"
      "0100000000"));
}

TEST(Shapes, Arrays) {
  Arrays a = make_arrays();
  std::string golden = GOLDEN(
    "TM_Arrays",
    "00010000010001007fff0102030000000000c03f000000000000c0bf00000000"
    "00000440000000000000000000000000000004c080007f0001ff00800000ff7f"
    "00000100ffff00000000008000000000ffffff7f0000000001000000ffffffff"
    "0000000000000000000000800000000000000000ffffffffffffff7f00000000"
    "000000000100000000000000ffffffffffffffff020000006100000001000000"
    "0000000004000000636363000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000000000000001"
    "326400000000903f00000000000000000000f23fcec818fcd0070000d08affff"
    "60ea000000a69dfdffffffff80f0fa0200000000013264000000903f00000000"
    "0000f23fcec818fcd0070000d08affff60ea000000a69dfdffffffff80f0fa02"
    "00000000013264000000903f000000000000f23fcec818fcd0070000d08affff"
    "60ea000000a69dfdffffffff80f0fa02000000000001000001ff00017f000000"
    "0000903f00000000000090bf6f1283c0ca21094000000000000000006f1283c0"
    "ca2109c0007f800001ff0000ff7f008000000100ffff000000000000ffffff7f"
    "000000800000000001000000ffffffff000000000000000000000000ffffffff"
    "ffffff7f000000000000008000000000000000000100000000000000ffffffff"
    "ffffffff01000000000000000a0000006d61782076616c75650000000a000000"
    "6d696e2076616c75650000000d0c0b0a");
  expect_golden(encode(a), golden);
  Arrays out = decode<Arrays>(from_hex(golden));
  EXPECT_EQ(out.alignment_check, 0x0a0b0c0d); // trailing member byte-pinned
  EXPECT_EQ(out.string_values[2], "ccc");
  EXPECT_EQ(out.int64_values[2], INT64_MAX);
  EXPECT_TRUE(encode(out) == from_hex(golden)); // decode consumed bytes exactly
}

TEST(Shapes, UnboundedSequences) {
  UnboundedSequences u = make_unbounded();
  std::string golden = GOLDEN(
    "TM_UnboundedSequences",
    "0001000002000000010000000000000000000000000000000200000000000000"
    "0000f83f00000000000004c00000000004000000010203040000000000000000"
    "03000000ffffffff00000000ffffff7f00000000000000000000000003000000"
    "02000000780000000100000000000000040000007a7a7a000000000000000000"
    "000000000300000000010000030000000001ff000300000000017f0003000000"
    "0000903f00000000000090bf03000000000000006f1283c0ca21094000000000"
    "000000006f1283c0ca2109c003000000007f8000030000000001ff0003000000"
    "0000ff7f008000000300000000000100ffff00000300000000000000ffffff7f"
    "00000080030000000000000001000000ffffffff030000000000000000000000"
    "00000000ffffffffffffff7f0000000000000080030000000000000000000000"
    "000000000100000000000000ffffffffffffffff030000000100000000000000"
    "0a0000006d61782076616c75650000000a0000006d696e2076616c7565000000"
    "44332211");
  expect_golden(encode(u), golden);
  UnboundedSequences out = decode<UnboundedSequences>(from_hex(golden));
  EXPECT_EQ(out.alignment_check, 0x11223344);
  EXPECT_EQ(out.string_values.size(), 3u);
  EXPECT_EQ(out.string_values[1], "");
  EXPECT_TRUE(encode(out) == from_hex(golden));
}

TEST(Shapes, UnboundedSequences_Empty) {
  UnboundedSequences u = make_unbounded_empty();
  std::string golden = GOLDEN(
    "TM_UnboundedSequences_Empty",
    "0001000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000000000000000"
    "00000000000000000300000000010000030000000001ff000300000000017f00"
    "030000000000903f00000000000090bf030000006f1283c0ca21094000000000"
    "000000006f1283c0ca2109c003000000007f8000030000000001ff0003000000"
    "0000ff7f008000000300000000000100ffff00000300000000000000ffffff7f"
    "00000080030000000000000001000000ffffffff030000000000000000000000"
    "00000000ffffffffffffff7f0000000000000080030000000000000000000000"
    "000000000100000000000000ffffffffffffffff030000000100000000000000"
    "0a0000006d61782076616c75650000000a0000006d696e2076616c7565000000"
    "88776655");
  expect_golden(encode(u), golden);
  UnboundedSequences out = decode<UnboundedSequences>(from_hex(golden));
  EXPECT_EQ(out.alignment_check, 0x55667788);
  EXPECT_TRUE(out.bool_values.empty());
  EXPECT_TRUE(encode(out) == from_hex(golden));
}

TEST(Shapes, BoundedSequences) {
  BoundedSequences b = make_bounded();
  std::string golden = GOLDEN(
    "TM_BoundedSequences",
    "0001000003000000010100000000000000000000000000000100000000000000"
    "000023c000000000020000000908000000000000000000000200000007000000"
    "0800000000000000000000000000000002000000020000007000000003000000"
    "717100000000000000000000000000000300000000010000030000000001ff00"
    "0300000000017f00030000000000903f00000000000090bf030000006f1283c0"
    "ca21094000000000000000006f1283c0ca2109c003000000007f800003000000"
    "0001ff00030000000000ff7f008000000300000000000100ffff000003000000"
    "00000000ffffff7f00000080030000000000000001000000ffffffff03000000"
    "000000000000000000000000ffffffffffffff7f000000000000008003000000"
    "0000000000000000000000000100000000000000ffffffffffffffff03000000"
    "01000000000000000a0000006d61782076616c75650000000a0000006d696e20"
    "76616c75650000000a0b0c0d");
  expect_golden(encode(b), golden);
  BoundedSequences out = decode<BoundedSequences>(from_hex(golden));
  EXPECT_EQ(out.alignment_check, 0x0d0c0b0a);
  EXPECT_EQ(out.int32_values.size(), 2u);
  EXPECT_TRUE(encode(out) == from_hex(golden));
}

TEST(Shapes, Nested) {
  Nested n;
  n.basic_types_value = make_basic_types();
  expect_golden_bidi(
    n, GOLDEN(
      "TM_Nested",
      "0001000001ff41000000c0bf000000000000044080c80080ffff000000000080"
      "ffffffff0000000000000080ffffffffffffffff"));
}

TEST(Shapes, MultiNested) {
  MultiNested m;
  for (size_t i = 0; i < 3; ++i) {
    m.array_of_arrays[i] = make_arrays();
    m.array_of_bounded_sequences[i] = make_bounded();
    m.array_of_unbounded_sequences[i] = make_unbounded();
  }
  m.bounded_sequence_of_arrays = {make_arrays()};
  m.unbounded_sequence_of_unbounded_sequences = {make_unbounded(),
    make_unbounded_empty()};
  // The deepest nesting (array-of / seq-of struct): byte-pinned to its rmw
  // golden in both directions, the same oracle the other shapes use.
  expect_golden_bidi(
    m, GOLDEN(
      "TM_MultiNested",
      "00010000010001007fff0102030000000000c03f000000000000c0bf00000000"
      "00000440000000000000000000000000000004c080007f0001ff00800000ff7f"
      "00000100ffff00000000008000000000ffffff7f0000000001000000ffffffff"
      "0000000000000000000000800000000000000000ffffffffffffff7f00000000"
      "000000000100000000000000ffffffffffffffff020000006100000001000000"
      "0000000004000000636363000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000001"
      "326400000000903f00000000000000000000f23fcec818fcd0070000d08affff"
      "60ea000000a69dfdffffffff80f0fa0200000000013264000000903f00000000"
      "0000f23fcec818fcd0070000d08affff60ea000000a69dfdffffffff80f0fa02"
      "00000000013264000000903f000000000000f23fcec818fcd0070000d08affff"
      "60ea000000a69dfdffffffff80f0fa02000000000001000001ff00017f000000"
      "0000903f00000000000090bf6f1283c0ca21094000000000000000006f1283c0"
      "ca2109c0007f800001ff0000ff7f008000000100ffff000000000000ffffff7f"
      "000000800000000001000000ffffffff000000000000000000000000ffffffff"
      "ffffff7f000000000000008000000000000000000100000000000000ffffffff"
      "ffffffff01000000000000000a0000006d61782076616c75650000000a000000"
      "6d696e2076616c75650000000d0c0b0a010001007fff0102030000000000c03f"
      "000000000000c0bf000000000000000000000440000000000000000000000000"
      "000004c080007f0001ff00800000ff7f00000100ffff00000000008000000000"
      "ffffff7f0000000001000000ffffffff00000000000000000000008000000000"
      "00000000ffffffffffffff7f00000000000000000100000000000000ffffffff"
      "ffffffff02000000610000000100000000000000040000006363630000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000001326400000000903f0000000000000000"
      "0000f23fcec818fcd0070000d08affff60ea000000a69dfdffffffff80f0fa02"
      "00000000013264000000903f000000000000f23fcec818fcd0070000d08affff"
      "60ea000000a69dfdffffffff80f0fa0200000000013264000000903f00000000"
      "0000f23fcec818fcd0070000d08affff60ea000000a69dfdffffffff80f0fa02"
      "000000000001000001ff00017f0000000000903f00000000000090bf6f1283c0"
      "ca21094000000000000000006f1283c0ca2109c0007f800001ff0000ff7f0080"
      "00000100ffff000000000000ffffff7f000000800000000001000000ffffffff"
      "000000000000000000000000ffffffffffffff7f000000000000008000000000"
      "000000000100000000000000ffffffffffffffff01000000000000000a000000"
      "6d61782076616c75650000000a0000006d696e2076616c75650000000d0c0b0a"
      "010001007fff0102030000000000c03f000000000000c0bf0000000000000000"
      "00000440000000000000000000000000000004c080007f0001ff00800000ff7f"
      "00000100ffff00000000008000000000ffffff7f0000000001000000ffffffff"
      "0000000000000000000000800000000000000000ffffffffffffff7f00000000"
      "000000000100000000000000ffffffffffffffff020000006100000001000000"
      "0000000004000000636363000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000001"
      "326400000000903f00000000000000000000f23fcec818fcd0070000d08affff"
      "60ea000000a69dfdffffffff80f0fa0200000000013264000000903f00000000"
      "0000f23fcec818fcd0070000d08affff60ea000000a69dfdffffffff80f0fa02"
      "00000000013264000000903f000000000000f23fcec818fcd0070000d08affff"
      "60ea000000a69dfdffffffff80f0fa02000000000001000001ff00017f000000"
      "0000903f00000000000090bf6f1283c0ca21094000000000000000006f1283c0"
      "ca2109c0007f800001ff0000ff7f008000000100ffff000000000000ffffff7f"
      "000000800000000001000000ffffffff000000000000000000000000ffffffff"
      "ffffff7f000000000000008000000000000000000100000000000000ffffffff"
      "ffffffff01000000000000000a0000006d61782076616c75650000000a000000"
      "6d696e2076616c75650000000d0c0b0a03000000010100000000000000000000"
      "00000000010000000000000000000000000023c0000000000200000009080000"
      "0000000000000000020000000700000008000000000000000000000000000000"
      "0200000002000000700000000300000071710000000000000000000000000000"
      "0300000000010000030000000001ff000300000000017f00030000000000903f"
      "00000000000090bf030000006f1283c0ca21094000000000000000006f1283c0"
      "ca2109c003000000007f8000030000000001ff00030000000000ff7f00800000"
      "0300000000000100ffff00000300000000000000ffffff7f0000008003000000"
      "0000000001000000ffffffff03000000000000000000000000000000ffffffff"
      "ffffff7f00000000000000800300000000000000000000000000000001000000"
      "00000000ffffffffffffffff0300000001000000000000000a0000006d617820"
      "76616c75650000000a0000006d696e2076616c75650000000a0b0c0d03000000"
      "010100000000000000000000000000000100000000000000000023c000000000"
      "0200000009080000000000000000000002000000070000000800000000000000"
      "0000000000000000020000000200000070000000030000007171000000000000"
      "00000000000000000300000000010000030000000001ff000300000000017f00"
      "030000000000903f00000000000090bf030000006f1283c0ca21094000000000"
      "000000006f1283c0ca2109c003000000007f8000030000000001ff0003000000"
      "0000ff7f008000000300000000000100ffff00000300000000000000ffffff7f"
      "00000080030000000000000001000000ffffffff030000000000000000000000"
      "00000000ffffffffffffff7f0000000000000080030000000000000000000000"
      "000000000100000000000000ffffffffffffffff030000000100000000000000"
      "0a0000006d61782076616c75650000000a0000006d696e2076616c7565000000"
      "0a0b0c0d03000000010100000000000000000000000000000100000000000000"
      "000023c000000000020000000908000000000000000000000200000007000000"
      "0800000000000000000000000000000002000000020000007000000003000000"
      "717100000000000000000000000000000300000000010000030000000001ff00"
      "0300000000017f00030000000000903f00000000000090bf030000006f1283c0"
      "ca21094000000000000000006f1283c0ca2109c003000000007f800003000000"
      "0001ff00030000000000ff7f008000000300000000000100ffff000003000000"
      "00000000ffffff7f00000080030000000000000001000000ffffffff03000000"
      "000000000000000000000000ffffffffffffff7f000000000000008003000000"
      "0000000000000000000000000100000000000000ffffffffffffffff03000000"
      "01000000000000000a0000006d61782076616c75650000000a0000006d696e20"
      "76616c75650000000a0b0c0d0200000001000000000000000000000000000000"
      "02000000000000000000f83f00000000000004c0000000000400000001020304"
      "000000000000000003000000ffffffff00000000ffffff7f0000000000000000"
      "000000000300000002000000780000000100000000000000040000007a7a7a00"
      "0000000000000000000000000300000000010000030000000001ff0003000000"
      "00017f00030000000000903f00000000000090bf03000000000000006f1283c0"
      "ca21094000000000000000006f1283c0ca2109c003000000007f800003000000"
      "0001ff00030000000000ff7f008000000300000000000100ffff000003000000"
      "00000000ffffff7f00000080030000000000000001000000ffffffff03000000"
      "000000000000000000000000ffffffffffffff7f000000000000008003000000"
      "0000000000000000000000000100000000000000ffffffffffffffff03000000"
      "01000000000000000a0000006d61782076616c75650000000a0000006d696e20"
      "76616c7565000000443322110200000001000000000000000000000000000000"
      "02000000000000000000f83f00000000000004c0000000000400000001020304"
      "000000000000000003000000ffffffff00000000ffffff7f0000000000000000"
      "000000000300000002000000780000000100000000000000040000007a7a7a00"
      "0000000000000000000000000300000000010000030000000001ff0003000000"
      "00017f00030000000000903f00000000000090bf03000000000000006f1283c0"
      "ca21094000000000000000006f1283c0ca2109c003000000007f800003000000"
      "0001ff00030000000000ff7f008000000300000000000100ffff000003000000"
      "00000000ffffff7f00000080030000000000000001000000ffffffff03000000"
      "000000000000000000000000ffffffffffffff7f000000000000008003000000"
      "0000000000000000000000000100000000000000ffffffffffffffff03000000"
      "01000000000000000a0000006d61782076616c75650000000a0000006d696e20"
      "76616c7565000000443322110200000001000000000000000000000000000000"
      "02000000000000000000f83f00000000000004c0000000000400000001020304"
      "000000000000000003000000ffffffff00000000ffffff7f0000000000000000"
      "000000000300000002000000780000000100000000000000040000007a7a7a00"
      "0000000000000000000000000300000000010000030000000001ff0003000000"
      "00017f00030000000000903f00000000000090bf03000000000000006f1283c0"
      "ca21094000000000000000006f1283c0ca2109c003000000007f800003000000"
      "0001ff00030000000000ff7f008000000300000000000100ffff000003000000"
      "00000000ffffff7f00000080030000000000000001000000ffffffff03000000"
      "000000000000000000000000ffffffffffffff7f000000000000008003000000"
      "0000000000000000000000000100000000000000ffffffffffffffff03000000"
      "01000000000000000a0000006d61782076616c75650000000a0000006d696e20"
      "76616c75650000004433221101000000010001007fff0102030000000000c03f"
      "000000000000c0bf000000000000000000000440000000000000000000000000"
      "000004c080007f0001ff00800000ff7f00000100ffff00000000008000000000"
      "ffffff7f0000000001000000ffffffff00000000000000000000008000000000"
      "00000000ffffffffffffff7f00000000000000000100000000000000ffffffff"
      "ffffffff02000000610000000100000000000000040000006363630000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000001326400000000903f0000000000000000"
      "0000f23fcec818fcd0070000d08affff60ea000000a69dfdffffffff80f0fa02"
      "00000000013264000000903f000000000000f23fcec818fcd0070000d08affff"
      "60ea000000a69dfdffffffff80f0fa0200000000013264000000903f00000000"
      "0000f23fcec818fcd0070000d08affff60ea000000a69dfdffffffff80f0fa02"
      "000000000001000001ff00017f0000000000903f00000000000090bf6f1283c0"
      "ca21094000000000000000006f1283c0ca2109c0007f800001ff0000ff7f0080"
      "00000100ffff000000000000ffffff7f000000800000000001000000ffffffff"
      "000000000000000000000000ffffffffffffff7f000000000000008000000000"
      "000000000100000000000000ffffffffffffffff01000000000000000a000000"
      "6d61782076616c75650000000a0000006d696e2076616c75650000000d0c0b0a"
      "0000000000000000000000000000000002000000020000000100000000000000"
      "000000000000000002000000000000000000f83f00000000000004c000000000"
      "0400000001020304000000000000000003000000ffffffff00000000ffffff7f"
      "0000000000000000000000000300000002000000780000000100000000000000"
      "040000007a7a7a00000000000000000000000000030000000001000003000000"
      "0001ff000300000000017f00030000000000903f00000000000090bf03000000"
      "000000006f1283c0ca21094000000000000000006f1283c0ca2109c003000000"
      "007f8000030000000001ff00030000000000ff7f008000000300000000000100"
      "ffff00000300000000000000ffffff7f00000080030000000000000001000000"
      "ffffffff03000000000000000000000000000000ffffffffffffff7f00000000"
      "00000080030000000000000000000000000000000100000000000000ffffffff"
      "ffffffff0300000001000000000000000a0000006d61782076616c7565000000"
      "0a0000006d696e2076616c756500000044332211000000000000000000000000"
      "0000000000000000000000000000000000000000000000000000000000000000"
      "0000000000000000000000000000000000000000000000000300000000010000"
      "030000000001ff000300000000017f00030000000000903f00000000000090bf"
      "030000006f1283c0ca21094000000000000000006f1283c0ca2109c003000000"
      "007f8000030000000001ff00030000000000ff7f008000000300000000000100"
      "ffff00000300000000000000ffffff7f00000080030000000000000001000000"
      "ffffffff03000000000000000000000000000000ffffffffffffff7f00000000"
      "00000080030000000000000000000000000000000100000000000000ffffffff"
      "ffffffff0300000001000000000000000a0000006d61782076616c7565000000"
      "0a0000006d696e2076616c756500000088776655"));
}

TEST(Shapes, Constants) {
  // Constants-only message: the placeholder member serialises as one byte.
  expect_golden_bidi(
    Constants(), GOLDEN(
      "TM_Constants",
      "0001000000"));
}

TEST(Shapes, Defaults) {
  // The codec applies the IDL @default values to a default-constructed message.
  expect_golden_bidi(
    Defaults(),
    GOLDEN(
      "TM_Defaults",
      "00010000013264000000903f000000000000f23fcec818fcd0070000d08affff"
      "60ea000000a69dfdffffffff80f0fa0200000000"));
}

TEST(Shapes, Empty) {
  expect_golden_bidi(
    Empty(), GOLDEN(
      "TM_Empty",
      "0001000000"));
}

// Oracle 3 -- @foxglove/cdr byte vectors (MIT; test/third_party/foxglove_cdr).
// A CDR implementation independent of rmw, so a bug shared by rmw and our codec
// cannot pass both. Lifted verbatim from CdrWriter.test.ts / CdrReader.test.ts
// and run through cdr.hpp directly (raw field serialise/deserialise via cdr()).
// CdrReader.test.ts is byte -> value: a decoder oracle from a foreign producer.

namespace
{
// tf2_msgs/TFMessage: sequence-of-struct-with-strings, the shape closest to the
// nested control messages. Identical hex in CdrWriter.test.ts and
// CdrReader.test.ts (CDR_LE, 00 01 00 00).
const char * kFoxgloveTf2 =
  "0001000001000000cce0d158f08cf9060a000000626173655f6c696e6b000000"
  "060000007261646172000000ae47e17a14ae0e40000000000000000000000000"
  "0000000000000000000000000000000000000000000000000000000000000000"
  "0000f03f";
} // namespace

TEST(Foxglove, TFMessage_Encode) {
  // Writer oracle: build the message field-by-field, expect Foxglove's bytes.
  cdr::CdrWriter w;
  auto & c = w.cdr();
  c.serialize(static_cast<uint32_t>(1)); // sequence length
  c.serialize(static_cast<uint32_t>(1490149580)); // sec
  c.serialize(static_cast<uint32_t>(117017840));   // nsec
  c.serialize(std::string("base_link"));
  c.serialize(std::string("radar"));
  c.serialize(3.835); // translation x
  for (int i = 0; i < 5; ++i) {
    c.serialize(0.0); // y, z, qx, qy, qz
  }
  c.serialize(1.0);   // qw
  EXPECT_EQ(to_hex(w.bytes()), to_hex(from_hex(kFoxgloveTf2)));
}

TEST(Foxglove, TFMessage_Decode) {
  // Decoder oracle: foreign-produced bytes -> values through cdr.hpp.
  cdr::CdrReader r(from_hex(kFoxgloveTf2));
  auto & c = r.cdr();
  uint32_t seqlen, sec, nsec;
  c.deserialize(seqlen);
  c.deserialize(sec);
  c.deserialize(nsec);
  std::string frame, child;
  c.deserialize(frame);
  c.deserialize(child);
  double x;
  c.deserialize(x);
  EXPECT_EQ(seqlen, 1u);
  EXPECT_EQ(sec, 1490149580u);
  EXPECT_EQ(nsec, 117017840u);
  EXPECT_EQ(frame, "base_link");
  EXPECT_EQ(child, "radar");
  EXPECT_DOUBLE_EQ(x, 3.835);
}

TEST(Foxglove, Utf8String) {
  // "é" is 2 UTF-8 bytes; CDR length includes the null terminator (3).
  const char * kEAcute = "0001000003000000c3a900";
  cdr::CdrWriter w;
  w.cdr().serialize(std::string("\xc3\xa9"));
  EXPECT_EQ(to_hex(w.bytes()), to_hex(from_hex(kEAcute)));

  cdr::CdrReader r(from_hex(kEAcute));
  std::string s;
  r.cdr().deserialize(s);
  EXPECT_EQ(s, std::string("\xc3\xa9"));
}

TEST(Foxglove, ParameterEvent_Decode) {
  // rcl_interfaces/ParameterEvent: time + string + sequence + empty string;
  // a decoder oracle for the empty-string / sequence-of-struct shapes.
  const char * kParamEvent =
    "00010000a9b71561a570ea01110000002f5f726f7332636c695f333738333633"
    "00000000010000000d0000007573655f73696d5f74696d650001000000000000"
    "0000000000000000000000000100000000000000000000000000000000000000"
    "00000000000000000000000000000000";
  cdr::CdrReader r(from_hex(kParamEvent));
  auto & c = r.cdr();
  uint32_t sec, nsec, nnew;
  c.deserialize(sec);
  c.deserialize(nsec);
  std::string node, pname;
  c.deserialize(node);
  c.deserialize(nnew);
  c.deserialize(pname);
  uint8_t ptype;
  c.deserialize(ptype);
  EXPECT_EQ(sec, 1628813225u);
  EXPECT_EQ(nsec, 32141477u);
  EXPECT_EQ(node, "/_ros2cli_378363");
  EXPECT_EQ(nnew, 1u);
  EXPECT_EQ(pname, "use_sim_time");
  EXPECT_EQ(ptype, 1u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
