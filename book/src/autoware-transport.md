# Autoware Transport

The *Autoware transport* is how the node talks to the vehicle: it publishes control, gear and initial-pose, subscribes vehicle and operation-mode state, and calls the AD-API operation-mode services. Two transports implement the same `AutowareGateway` interface (`src/core/autoware_gateway.hpp`), and exactly one is compiled in, chosen by the CMake cache variable `TELEOP_AUTOWARE_TRANSPORT` (`rclcpp` | `native_zenoh`, default `rclcpp`).

The gateway interface both transports satisfy is small — everything above it (the control loop, the modes, the I/O) is identical across transports:

- `publish_command`, `set_target_gear`, `get_vehicle_state`, `toggle_operation_mode`, `reset_initial_pose`, `is_localized`, `operationModeName`, `get_info_message`, `spin_some`, `ok`
- the static `init_system` / `shutdown_system` / `ok_system`
- `make_parameter_reader`

## `rclcpp` — native ROS 2

The default. `src/transport/rclcpp/autoware_gateway.cpp` wraps a `ManualControlNode` (`src/transport/rclcpp/manual_control_node.hpp`), an `rclcpp::Node` named `ManualControl`. This is the build you drop into an Autoware workspace.

It uses the standard Autoware interface:

- **Publishers**
  - `/external/selected/control_cmd` — `autoware_control_msgs/Control`, transient-local QoS
  - `/external/selected/gear_cmd` — `autoware_vehicle_msgs/GearCommand`
  - `/initialpose` — `geometry_msgs/PoseWithCovarianceStamped`
  - `/external/local/heartbeat` or `/external/remote/heartbeat` (per `operator_mode`) — a 10 Hz operator heartbeat, `autoware_adapi_v1_msgs/ManualOperatorHeartbeat`
- **Subscribers**
  - `/vehicle/status/velocity_status`
  - `/vehicle/status/steering_status`
  - `/vehicle/status/gear_status`
  - `/api/operation_mode/state`
  - `/api/localization/initialization_state`
- **Services** — three clients:
  - `/api/operation_mode/enable_autoware_control`
  - `/api/operation_mode/change_to_stop`
  - `/api/operation_mode/change_to_local` *or* `/api/operation_mode/change_to_remote` (a single build creates whichever `operator_mode` selects, not both)

`/external/selected/control_cmd` is the input both the LOCAL and REMOTE operation-mode gates forward, so one publish path drives the vehicle in either mode. Entering the driving operation mode calls `change_to_local` / `change_to_remote`; once that mode is active the node auto-calls `enable_autoware_control` from the mode-state callback — a single press with no separate engage key (self-engage).

Build dependencies are the usual ROS ones declared in `CMakeLists.txt`:

- `rclcpp`
- `autoware_control_msgs`
- `autoware_vehicle_msgs`
- `autoware_adapi_v1_msgs`
- `geometry_msgs`

## `native_zenoh` — ROS-free

`src/transport/zenoh/autoware_gateway.cpp` implements the *same* interface and the *same* Autoware semantics, but over Zenoh and with no ROS in the build or the binary. It opens a `zenoh::Session` and declares publishers/subscribers on scope-prefixed keys that mirror the ROS topics one-for-one (`<scope>/vehicle/status/velocity_status`, `<scope>/external/selected/control_cmd`, `<scope>/api/operation_mode/state`, and so on, where `<scope>` is the `scope` parameter, default `v1`). The operation-mode services become Zenoh queries (`session.get(...)` on `<scope>/api/operation_mode/...`). The same self-engage, stop-wait-shift, heartbeat and initial-pose-preset logic runs.

What makes it ROS-free is that the messages are serialised by a **vendored codec** rather than by a ROS runtime.

### The regenerated CDR codec

Autoware's DDS expects messages in DDS-CDR. The native transport produces exactly those bytes by compiling in the `rosidl_typesupport_fastrtps_cpp` output for the message contract — the very code `rmw_fastrtps` would call — committed under `src/transport/zenoh/generated/` together with the rosidl runtime headers it includes. The gateway calls the generated `cdr_serialize` / `cdr_deserialize` directly:

- `src/transport/zenoh/cdr.hpp` provides thin `CdrWriter` / `CdrReader` wrappers around Fast-CDR, fixed to `eprosima::fastcdr::Cdr::DDS_CDR` mode and writing/reading the 4-byte CDR encapsulation header — the framing DDS puts in front of every sample.
- `src/transport/zenoh/typesupport_stub.cpp` supplies inert definitions for the registration-tail symbols the generator always emits (the rmw type-discovery path). The native transport never calls that path — only `cdr_serialize` / `cdr_deserialize` — so the stubs exist only to satisfy the linker, keeping the binary free of `librosidl_runtime` and `librmw`.

The result is a binary whose only runtime library is Zenoh's `libzenohc` (Fast-CDR and libyaml are linked in statically), yet whose wire bytes are correct **by construction** — not a hand-written re-implementation but the same `rosidl_typesupport_fastrtps_cpp` serialiser `rmw_fastrtps` itself runs. CI's golden gate captures real-rmw wire bytes (via `rmw_cyclonedds_cpp`) to guard against upstream message drift; see [Testing & CI](testing.md).

### The wire contract

The exact set of messages that cross the wire is enumerated in two places that must agree:

- `TELEOP_MSG_IDL` in `tools/regen/CMakeLists.txt` lists *which* `.idl` are regenerated (their *content* comes from the installed Autoware/ROS packages at regen time).
- `TELEOP_PROD_MSG_STEMS` in `CMakeLists.txt` is the allow-list of codec stems compiled into the shipped binary. It is keyed as an allow-list, not a denylist, so a production message is never silently dropped; any *other* vendored codec (the test-shape corpus) is routed to the test target instead of the binary.

The two lists correspond one-for-one:

| Package | Message | Codec stem |
| :-- | :-- | :-- |
| `builtin_interfaces` | `Time` | `time` |
| `std_msgs` | `Header` | `header` |
| `geometry_msgs` | `Point` | `point` |
| `geometry_msgs` | `Quaternion` | `quaternion` |
| `geometry_msgs` | `Pose` | `pose` |
| `geometry_msgs` | `PoseWithCovariance` | `pose_with_covariance` |
| `geometry_msgs` | `PoseWithCovarianceStamped` | `pose_with_covariance_stamped` |
| `autoware_control_msgs` | `Lateral` | `lateral` |
| `autoware_control_msgs` | `Longitudinal` | `longitudinal` |
| `autoware_control_msgs` | `Control` | `control` |
| `autoware_vehicle_msgs` | `GearCommand` | `gear_command` |
| `autoware_vehicle_msgs` | `GearReport` | `gear_report` |
| `autoware_vehicle_msgs` | `SteeringReport` | `steering_report` |
| `autoware_vehicle_msgs` | `VelocityReport` | `velocity_report` |
| `autoware_adapi_v1_msgs` | `LocalizationInitializationState` | `localization_initialization_state` |
| `autoware_adapi_v1_msgs` | `ManualOperatorHeartbeat` | `manual_operator_heartbeat` |
| `autoware_adapi_v1_msgs` | `OperationModeState` | `operation_mode_state` |
| `autoware_adapi_v1_msgs` | `ResponseStatus` | `response_status` |
| `autoware_adapi_v1_msgs` | `ChangeOperationMode` (srv) | `change_operation_mode` |

Regenerating the codec after an upstream Autoware message change is the only step that needs a ROS environment — see [Building](building.md#regenerating-the-codec) — and CI's golden-drift guard turns red when it is due.

`native_zenoh` is what makes the operator side ROS-free, and combined with Zenoh operator I/O it is the basis for [cross-machine remote driving](cross-machine.md).
