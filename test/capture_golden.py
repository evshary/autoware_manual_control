#!/usr/bin/env python3
"""
Capture the golden-byte references for test_cdr.cpp from real rmw_cyclonedds.

The native_zenoh codec must be byte-identical to the DDS-CDR wire that
rmw_cyclonedds puts on the network. rclpy's serialize_message() runs the active
rmw's typesupport, so with RMW_IMPLEMENTATION=rmw_cyclonedds_cpp it emits exactly
that wire. This serialises:

  - every production control/telemetry message with the deterministic field
    values the GoldenByte tests use (the literal wire contract), and
  - the ros2/test_interface_files test_msgs corpus (BasicTypes / Strings /
    Arrays / Bounded+UnboundedSequences / Nested / MultiNested / Constants /
    Defaults / Empty / WStrings), which exercises every field SHAPE rosidl was
    designed to test -- so the codec is certified on every shape, not only the
    shapes today's production messages happen to use.

The C++ gate asserts encode(value) == these bytes (encoder oracle) AND
decode(these bytes) == value (external decoder oracle). Run this after an
Autoware msg bump to refresh the production goldens; the test_msgs goldens are
stable fixtures.

Usage (with ROS + Autoware sourced):
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 test/capture_golden.py
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 test/capture_golden.py --check
"""
import math
import os
import re
import sys

os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

from autoware_adapi_v1_msgs.msg import (  # noqa: E402
    LocalizationInitializationState,
    ManualOperatorHeartbeat,
    OperationModeState,
)
from autoware_adapi_v1_msgs.srv import ChangeOperationMode  # noqa: E402
from autoware_control_msgs.msg import Control  # noqa: E402
from autoware_vehicle_msgs.msg import (  # noqa: E402
    GearCommand,
    GearReport,
    SteeringReport,
    VelocityReport,
)
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from test_msgs.msg import (  # noqa: E402
    Arrays,
    BasicTypes,
    BoundedSequences,
    Constants,
    Defaults,
    Empty,
    MultiNested,
    Nested,
    Strings,
    UnboundedSequences,
)


def hex_of(msg):
    return serialize_message(msg).hex()


# Shape-coverage fixtures: mirror the C++ fixtures in test_cdr.cpp exactly.
def basic_types():
    b = BasicTypes()
    b.bool_value = True
    b.byte_value = bytes([255])
    b.char_value = 65
    b.float32_value = -1.5
    b.float64_value = 2.5
    b.int8_value = -128
    b.uint8_value = 200
    b.int16_value = -32768
    b.uint16_value = 65535
    b.int32_value = -2147483648
    b.uint32_value = 4294967295
    b.int64_value = -9223372036854775808
    b.uint64_value = 18446744073709551615
    return b


def basic_types_zero():
    return BasicTypes()


def strings_fixture():
    s = Strings()
    s.string_value = 'hello'           # unbounded
    s.bounded_string_value = 'bnd'     # bounded <=22
    for f in ('string_value_default1', 'string_value_default2',
              'string_value_default3', 'string_value_default4',
              'string_value_default5', 'bounded_string_value_default1',
              'bounded_string_value_default2', 'bounded_string_value_default3',
              'bounded_string_value_default4', 'bounded_string_value_default5'):
        setattr(s, f, '')              # empty string in every other slot
    return s


def arrays_fixture():
    a = Arrays()
    a.bool_values = [True, False, True]
    a.byte_values = [bytes([0]), bytes([127]), bytes([255])]
    a.char_values = [1, 2, 3]
    a.float32_values = [1.5, 0.0, -1.5]
    a.float64_values = [2.5, 0.0, -2.5]
    a.int8_values = [-128, 0, 127]
    a.uint8_values = [0, 1, 255]
    a.int16_values = [-32768, 0, 32767]
    a.uint16_values = [0, 1, 65535]
    a.int32_values = [-2147483648, 0, 2147483647]
    a.uint32_values = [0, 1, 4294967295]
    a.int64_values = [-9223372036854775808, 0, 9223372036854775807]
    a.uint64_values = [0, 1, 18446744073709551615]
    a.string_values = ['a', '', 'ccc']
    for i in range(3):
        a.basic_types_values[i] = basic_types_zero()
        a.constants_values[i] = Constants()
        a.defaults_values[i] = Defaults()
    a.alignment_check = 0x0a0b0c0d
    return a


def unbounded_sequences():
    u = UnboundedSequences()
    u.bool_values = [True, False]
    u.int32_values = [-1, 0, 2147483647]
    u.float64_values = [1.5, -2.5]
    u.string_values = ['x', '', 'zzz']
    u.uint8_values = bytes([1, 2, 3, 4])
    u.alignment_check = 0x11223344
    return u


def unbounded_empty():
    u = UnboundedSequences()
    u.alignment_check = 0x55667788
    return u


def bounded_sequences():
    b = BoundedSequences()
    b.bool_values = [True, True, False]
    b.int32_values = [7, 8]
    b.float64_values = [-9.5]
    b.string_values = ['p', 'qq']
    b.uint8_values = bytes([9, 8])
    b.alignment_check = 0x0d0c0b0a
    return b


def nested_fixture():
    n = Nested()
    n.basic_types_value = basic_types()
    return n


def multi_nested():
    m = MultiNested()
    for i in range(3):
        m.array_of_arrays[i] = arrays_fixture()
        m.array_of_bounded_sequences[i] = bounded_sequences()
        m.array_of_unbounded_sequences[i] = unbounded_sequences()
    m.bounded_sequence_of_arrays = [arrays_fixture()]
    m.unbounded_sequence_of_unbounded_sequences = [unbounded_sequences(),
                                                   unbounded_empty()]
    return m


def constants_fixture():
    # The placeholder member is stripped from the rclpy struct (constants-only
    # message); both sides leave it at its default 0, so default-construct.
    return Constants()


def defaults_fixture():
    return Defaults()  # rosidl applies the @default values


def empty_fixture():
    return Empty()


# Production wire contract: the literal control/telemetry messages.
def production():
    cases = {}

    c = Control()
    c.stamp.sec = 1
    c.stamp.nanosec = 2
    c.lateral.steering_tire_angle = 0.5
    c.longitudinal.velocity = 3.0
    c.longitudinal.acceleration = 1.25
    cases['ControlCommand_SafetyCritical'] = c

    # Edge floats: NaN / +inf / -inf / large / negative on the control message.
    ce = Control()
    ce.lateral.steering_tire_angle = float('nan')
    ce.longitudinal.velocity = float('inf')
    ce.longitudinal.acceleration = float('-inf')
    ce.longitudinal.jerk = -3.4e38
    cases['ControlCommand_EdgeFloats'] = ce

    g = GearCommand()
    g.stamp.sec = 5
    g.stamp.nanosec = 6
    g.command = 22
    cases['GearCommand_SafetyCritical'] = g

    g0 = GearCommand()
    g0.command = 0  # NONE -- enum boundary low
    cases['GearCommand_EnumZero'] = g0

    g255 = GearCommand()
    g255.command = 255  # enum boundary high (max uint8)
    cases['GearCommand_EnumMax'] = g255

    h = ManualOperatorHeartbeat()
    h.stamp.sec = 21
    h.stamp.nanosec = 22
    h.ready = True
    cases['ManualOperatorHeartbeat'] = h

    p = PoseWithCovarianceStamped()
    p.header.stamp.sec = 7
    p.header.stamp.nanosec = 8
    p.header.frame_id = 'map'
    p.pose.pose.position.x = 1.5
    p.pose.pose.position.y = -2.25
    p.pose.pose.position.z = 0.0
    yaw = 1.0
    p.pose.pose.orientation.z = math.sin(yaw * 0.5)
    p.pose.pose.orientation.w = math.cos(yaw * 0.5)
    p.pose.covariance = [0.1 if i % 7 == 0 else 0.0 for i in range(36)]
    cases['PoseWithCovarianceStamped_InitialPose'] = p

    o = OperationModeState()
    o.stamp.sec = 17
    o.stamp.nanosec = 18
    o.mode = 2
    o.is_autoware_control_enabled = True
    o.is_remote_mode_available = True
    cases['OperationModeState'] = o

    o255 = OperationModeState()
    o255.mode = 255  # enum boundary high
    cases['OperationModeState_EnumMax'] = o255

    li = LocalizationInitializationState()
    li.stamp.sec = 19
    li.stamp.nanosec = 20
    li.state = 3
    cases['LocalizationInitializationState'] = li

    v = VelocityReport()
    v.header.stamp.sec = 13
    v.header.stamp.nanosec = 14
    v.header.frame_id = 'base_link'
    v.longitudinal_velocity = 2.0
    v.lateral_velocity = 0.5
    v.heading_rate = -0.25
    cases['VelocityReport'] = v

    gr = GearReport()
    gr.stamp.sec = 9
    gr.stamp.nanosec = 10
    gr.report = 2
    cases['GearReport'] = gr

    s = SteeringReport()
    s.stamp.sec = 15
    s.stamp.nanosec = 16
    s.steering_tire_angle = 0.25
    cases['SteeringReport'] = s

    cases['ChangeOperationMode_Request_Empty'] = ChangeOperationMode.Request()

    resp = ChangeOperationMode.Response()
    resp.status.success = True
    resp.status.code = 60001
    resp.status.message = 'no effect'
    cases['ChangeOperationMode_Response'] = resp

    return cases


def shapes():
    return {
        'TM_BasicTypes': basic_types(),
        'TM_BasicTypes_Zero': basic_types_zero(),
        'TM_Strings': strings_fixture(),
        'TM_Arrays': arrays_fixture(),
        'TM_UnboundedSequences': unbounded_sequences(),
        'TM_UnboundedSequences_Empty': unbounded_empty(),
        'TM_BoundedSequences': bounded_sequences(),
        'TM_Nested': nested_fixture(),
        'TM_MultiNested': multi_nested(),
        'TM_Constants': constants_fixture(),
        'TM_Defaults': defaults_fixture(),
        'TM_Empty': empty_fixture(),
    }


def all_cases():
    cases = {}
    cases.update(production())
    cases.update(shapes())
    return cases


def parse_committed_goldens():
    """Read the GOLDEN("name", "hex") entries committed in test_cdr.cpp."""
    here = os.path.dirname(os.path.abspath(__file__))
    text = open(os.path.join(here, 'test_cdr.cpp')).read()
    pat = re.compile(r'GOLDEN\(\s*"([^"]+)"\s*,\s*((?:\s*"[0-9a-fA-F]*")+)\s*\)')
    out = {}
    for name, blob in pat.findall(text):
        out[name] = ''.join(re.findall(r'"([0-9a-fA-F]*)"', blob))
    return out


def check():
    """Re-capture from live rmw and diff against the committed goldens.

    Catches a silent Autoware msg-layout drift in CI instead of going unnoticed.
    Every committed byte-pinned golden is re-serialised from the live rmw and
    compared; a name with no producing fixture is itself reported as drift.
    """
    live = {name: hex_of(msg) for name, msg in all_cases().items()}
    committed = parse_committed_goldens()
    drift = []
    for name, committed_hex in committed.items():
        if name not in live:
            drift.append('%s: committed but no fixture produces it' % name)
        elif live[name] != committed_hex:
            drift.append('%s:\n  committed %s\n  live      %s'
                         % (name, committed_hex, live[name]))
    if drift:
        sys.stderr.write('golden drift detected:\n' + '\n'.join(drift) + '\n')
        return 1
    print('goldens up to date (%d byte-pinned cases)' % len(committed))
    return 0


def main():
    if '--check' in sys.argv[1:]:
        sys.exit(check())
    for name, msg in all_cases().items():
        print('%-40s %s' % (name, hex_of(msg)))


if __name__ == '__main__':
    main()
