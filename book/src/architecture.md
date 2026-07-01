# Architecture

The node is a small, component-based system. A generic control loop drives the vehicle through a transport-blind gateway, taking intent from an *intent source* and pushing telemetry to a *telemetry sink*. The I/O and the transport are both swapped without touching the loop or the modes, because they meet only at a link-time seam.

## Directory layout

```text
src/
├── common/                   # Shared data, no behaviour
│   ├── types.hpp             #   Gear, ShiftState, VehicleState, ControlCommand
│   ├── intent.hpp            #   Intent  — the IntentSource payload
│   └── telemetry.hpp         #   Telemetry — the TelemetrySink payload
├── core/                     # Transport- and I/O-blind machinery
│   ├── control_runtime.hpp   #   run_control_runtime<IntentSource, TelemetrySink> + RuntimeConfig
│   ├── mode_manager.hpp      #   ModeManager: holds the active DriveMode
│   ├── drive_mode.hpp        #   DriveMode interface (the strategy)
│   ├── drive_mode_factory.*  #   DriveModeFactory: register/activate modes by name
│   ├── register_modes.hpp    #   register_all_modes + activate_modes_from_config
│   ├── autoware_gateway.hpp  #   AutowareGateway: the vehicle-side interface (Pimpl)
│   └── parameter_reader.hpp  #   ParameterReader: transport-blind params (Pimpl)
├── modes/                    # DriveMode strategies
│   └── stop_mode.hpp  physics_mode.hpp  cruise_mode.hpp
├── io/                       # Operator I/O adapters
│   ├── intent/keyboard.hpp   zenoh.hpp      #   IntentSource: Intent update()
│   └── telemetry/console.hpp zenoh.hpp      #   TelemetrySink: void publish(const Telemetry&)
├── transport/                # Autoware-side implementations (exactly one is linked)
│   ├── rclcpp/   autoware_gateway.cpp  parameter_reader.cpp  manual_control_node.hpp
│   └── zenoh/    autoware_gateway.cpp  parameter_reader.cpp  cdr.hpp  cli_params.hpp
│                 typesupport_stub.cpp  generated/   # the vendored CDR codec
├── keyboard_control.cpp      # Composition root: KeyboardIntent + ConsoleTelemetry
└── zenoh_control.cpp         # Composition root: ZenohIntent + ZenohTelemetry
```

## The control loop

`run_control_runtime` (`src/core/control_runtime.hpp`) is a function template:

```cpp
template <typename IntentSource, typename TelemetrySink>
void run_control_runtime(std::shared_ptr<AutowareGateway> gateway,
                         IntentSource& source, TelemetrySink& sink,
                         const RuntimeConfig& cfg);
```

Each tick it calls `source.update()` for an `Intent`, reads `gateway->get_vehicle_state()`, runs the gear-shift state machine and the active mode via `ModeManager`, publishes the resulting `ControlCommand` with `gateway->publish_command()`, and pushes a `Telemetry` to `sink.publish()`. It includes none of the I/O or transport headers — it knows only the duck-typed `update()` / `publish()` and the `AutowareGateway` interface. A stale `IntentSource` simply returns a braking `Intent`, so the loop needs no special case for disconnection.

## The seam

[Introduction § The seam](introduction.md#the-seam-that-makes-this-work) explains *why* the [matrix](introduction.md#the-matrix) is a build-time rather than a runtime choice; in terms of this layout, three mechanisms keep `core/` and `modes/` free of any concrete I/O or transport:

1. **I/O — duck-typed template parameters.** `run_control_runtime` names no base class; the composition root supplies the concrete `IntentSource` / `TelemetrySink` types and the loop is instantiated for them.
2. **Transport — a Pimpl gateway.** `AutowareGateway` (`src/core/autoware_gateway.hpp`) declares the interface; its `struct Impl` and the matching `static init_system` / `ok_system` / `shutdown_system` are defined only in the one `transport/*/autoware_gateway.cpp` that CMake links. The header pulls in neither `rclcpp` nor Zenoh.
3. **Parameters — explicit instantiation.** `ParameterReader` (`src/core/parameter_reader.hpp`) is a Pimpl whose `read<T>` is `extern template` in the header and instantiated once in the linked transport's `parameter_reader.cpp` — no `#ifdef`.

An entry point (`keyboard_control.cpp`, `zenoh_control.cpp`) is therefore just a *composition root*: it constructs the I/O adapters and the gateway, registers and activates modes, and calls `run_control_runtime`. The choice of transport is made entirely in `CMakeLists.txt`.

## How to add an Operator I/O

An I/O is a pair of adapters plus a composition root.

1. **Write the adapter(s).** An intent source `struct` with `Intent update()` (put it under `src/io/intent/`), and/or a telemetry sink with `void publish(const Telemetry&)` (under `src/io/telemetry/`). They share the same `Intent`/`Telemetry` vocabulary as every other I/O — see `src/io/intent/keyboard.hpp` for the smallest source and `src/io/telemetry/console.hpp` for the smallest sink.
2. **Write a composition root** `src/<name>_control.cpp` modelled on `keyboard_control.cpp`: build the gateway and reader, register and activate modes, construct your adapters, call `run_control_runtime`.
3. **Add a build option + executable** in `CMakeLists.txt` alongside `TELEOP_WITH_KEYBOARD` / `TELEOP_WITH_ZENOH` — one `option(...)` and one `add_executable(...)` per transport branch.

Nothing in `core/`, `modes/` or `transport/` changes; the new I/O composes with either transport for free.

## How to add an Autoware transport

A transport is a gateway implementation plus a parameter reader.

1. **Implement the gateway** `src/transport/<name>/autoware_gateway.cpp`: define `AutowareGateway::Impl` and every method of the interface, plus the static `init_system` / `shutdown_system` / `ok_system` and `make_parameter_reader`.
2. **Implement the reader** `src/transport/<name>/parameter_reader.cpp`: define `ParameterReader::Impl` and the explicit instantiations of `read<T>` for the five supported types (`float`, `double`, `std::string`, `std::vector<std::string>`, `std::vector<double>`).
3. **Add a branch** in `CMakeLists.txt` for `TELEOP_AUTOWARE_TRANSPORT=<name>`: find its dependencies and append the two sources to `TELEOP_GATEWAY_SRC` for the keyboard and Zenoh executables.

The control loop, the modes and every I/O are reused unchanged. The `native_zenoh` transport is the worked example of doing this without any ROS at all — see [Autoware Transport](autoware-transport.md) and [Building](building.md).

## How to add a drive mode

Drive modes are a strategy pattern behind `DriveModeFactory`. The smallest worked example is `src/modes/stop_mode.hpp` (~40 lines) — read it first. To add one:

1. **Implement** `src/modes/<name>_mode.hpp`: a class inheriting `DriveMode` (`src/core/drive_mode.hpp`) with `static constexpr const char* kName = "<name>";`, a `struct Params` plus `static Params loadParams(const ParameterReader&)` reading its tuning via `reader.read<T>(...)`, a constructor taking `const Params&`, and `ControlCommand update(float dt, const Intent&, const VehicleState&)` (optionally `onEnter` / `onExit` / `getStatusString`).
2. **Register** it: one line `register_mode<YourDriveMode>(factory, reader);` in `register_all_modes` (`src/core/register_modes.hpp`).
3. **Enable** it: add its `kName` to the `modes:` list in your config (and a tuning block if it has params).

One mode file, one register line, one config entry — no enum, no factory edits, no loop changes.
