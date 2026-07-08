# Introduction

Autoware Manual Control is a modular manual-driving node for [Autoware](https://github.com/autowarefoundation/autoware.universe). A human or a program supplies driving intent — throttle, brake, steer, gear, engage, mode — and the node turns it into the control, gear and operation-mode commands an Autoware vehicle expects, while feeding live telemetry back. It ships ready to drive from a keyboard, and it is built so the *same* control logic can be driven from a remote client over [Zenoh](https://zenoh.io/) and can talk to Autoware either over native ROS 2 or with no ROS at all.

Here it is in motion — the containerized quickstart from [Running Locally](running-locally.md), driving the planning simulator from the keyboard:

<video controls preload="metadata" style="width: 100%" src="videos/teleop-local.mp4"></video>

The whole design rests on keeping two concerns apart.

## The two axes

There are two questions you answer independently when you run this node.

**1. Operator I/O — where does intent come from, and where does telemetry go?**

This is the human/client side. It is chosen at build time by which entry point you compile:

- **keyboard** — `KeyboardIntent` (`src/io/intent/keyboard.hpp`) reads `WASD` and the action keys from a raw terminal and produces an `Intent` each tick. It is the sole stdin reader.
- **console** — `ConsoleTelemetry` (`src/io/telemetry/console.hpp`) renders each `Telemetry` to the terminal. It is the sole stdout writer. Keyboard and console are the two halves of the *local terminal operator*: they are compiled together into the `keyboard_control` executable by the CMake option `TELEOP_WITH_KEYBOARD` (default `ON`).
- **zenoh** — `ZenohIntent` (`src/io/intent/zenoh.hpp`) and `ZenohTelemetry` (`src/io/telemetry/zenoh.hpp`) exchange the same `Intent` and `Telemetry` as JSON over Zenoh keys, so any networked client can drive and observe. They are compiled into the `zenoh_control` executable by `TELEOP_WITH_ZENOH` (default `OFF`).

**2. Autoware transport — how does the node talk to Autoware?**

This is the vehicle side. It is chosen by the CMake cache variable `TELEOP_AUTOWARE_TRANSPORT` (`CMakeLists.txt` defines it with allowed values `rclcpp | native_zenoh`, default `rclcpp`):

- **`rclcpp`** — native ROS 2. The node publishes control/gear/initial-pose, subscribes vehicle and operation-mode state, and calls the AD-API operation-mode services through `rclcpp`. Build it into any Autoware workspace as an ordinary ROS package.
- **`native_zenoh`** — ROS-free. The node speaks Autoware's DDS/CDR wire format *directly* over Zenoh. There is no `rclcpp`, no message packages, no rosidl tooling in the build; the binary's only runtime library is Zenoh's `libzenohc`, with Fast-CDR and libyaml linked in statically. The wire bytes are correct by construction — the codec *is* the `rosidl_typesupport_fastrtps_cpp` serialiser Autoware itself uses (see [Autoware Transport](autoware-transport.md)).

Zenoh therefore appears on **both** axes, but they are genuinely different uses and you select them separately:

- Zenoh as *operator I/O* carries `Intent`/`Telemetry` as **JSON** on `manual_control/<scope>/…` keys — a convenient, human-readable contract for a UI.
- Zenoh as *Autoware transport* carries Autoware messages as **DDS-CDR** on `<scope>/…` keys — the exact bytes Autoware's DDS expects.

You can use one, the other, both, or neither-via-Zenoh.

## The matrix

The two axes are orthogonal. Each of the two I/O entry points links against either Autoware Transport, so the buildable configurations are their cartesian product: two executables × two transports = four binaries.

| Operator I/O \ Transport | `rclcpp` (ROS) | `native_zenoh` (ROS-free) |
| :-- | :-- | :-- |
| **`keyboard_control`** (keyboard intent + console telemetry) | ✅ keyboard demo | ✅ local terminal, no ROS |
| **`zenoh_control`** (zenoh JSON intent + telemetry) | ✅ UI over Zenoh, Autoware over ROS | ✅ cross-machine remote driving, fully ROS-free |

Each row is one entry point. `keyboard_control` bundles the keyboard intent source and the console telemetry sink — the two halves of the local terminal operator, never selected apart — while `zenoh_control` carries both halves over Zenoh. They are paired in one row because they compile as a unit, yet each half is independently transport-agnostic, which is why either executable links cleanly against either transport. The two I/O build flags select which executable(s) build, and `TELEOP_AUTOWARE_TRANSPORT` decides which transport they link against.

Three cells are the ones worth naming:

- **`keyboard_control` × `rclcpp`** — the getting-started demo. The default build; `ros2 run autoware_manual_control keyboard_control`.
- **`zenoh_control` × `rclcpp`** — drive from your own client over Zenoh while the node integrates with Autoware over ROS. The everyday integration path: your UI publishes JSON intent, the node drives a normal ROS Autoware.
- **`zenoh_control` × `native_zenoh`** — operator I/O over Zenoh *and* Autoware transport over Zenoh. No ROS on the operator side at all: suitable for [cross-machine remote driving](cross-machine.md) over the Internet.

The `zenoh_control` rows in practice — any client that publishes the JSON intent can drive, here a web fleet console ([zenoh_autoware_fms](https://github.com/evshary/zenoh_autoware_fms)) spawning one teleop per vehicle:

<video controls preload="metadata" style="width: 100%" src="videos/fleet-console.mp4"></video>

## The seam that makes this work

The four cells are not four code paths. They exist because operator I/O and Autoware transport meet at exactly one narrow, *link-time* seam, and neither side knows the other's concrete type.

- The control loop, `run_control_runtime` (`src/core/control_runtime.hpp`), is a function template over an `IntentSource` (anything with `Intent update()`) and a `TelemetrySink` (anything with `void publish(const Telemetry &)`). It calls only those two methods, plus an `AutowareGateway`. It contains no `#include` of any I/O or transport implementation.
- The vehicle side hides behind `AutowareGateway` (`src/core/autoware_gateway.hpp`), a Pimpl class whose `Impl` is defined *only* in the selected transport's `.cpp` (`src/transport/rclcpp/autoware_gateway.cpp` or `src/transport/zenoh/autoware_gateway.cpp`). The header pulls in neither `rclcpp` nor Zenoh. CMake compiles exactly one of the two `autoware_gateway.cpp` files into the target, so the transport is decided by *which object file is linked* — not by a runtime switch or a config flag.
- Parameters cross the same way. `ParameterReader` (`src/core/parameter_reader.hpp`) is a transport-blind Pimpl; its `read<T>` is explicitly instantiated once in the linked transport's `parameter_reader.cpp`, so the rclcpp build reads ROS parameters and the native build reads a YAML file, behind one interface.

Because the loop and the gateway interface are fixed, an entry point is just a *composition root*: `keyboard_control.cpp` wires `KeyboardIntent` + `ConsoleTelemetry` into the loop; `zenoh_control.cpp` wires `ZenohIntent` + `ZenohTelemetry`. Add an I/O and you write one composition root; add a transport and you write one `autoware_gateway.cpp` (and a `parameter_reader.cpp`). Nothing in `core/` or `modes/` changes. That is what the rest of this book builds on — see [Architecture](architecture.md) for the layout and the extension points.
