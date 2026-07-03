# Autoware Manual Control

A modular manual-driving node for [Autoware](https://github.com/autowarefoundation/autoware.universe): drop it into a ROS 2 workspace, drive an Autoware vehicle from the keyboard in minutes, and — when you are ready — drive it from your own UI over [Zenoh](https://zenoh.io/) instead, with no change to the control or mode logic.

🎥 **[Watch the demo video](https://www.youtube.com/watch?v=Pyi3uyONG8A)**

## Two independent axes

The node is organised around two axes that vary **independently**. Zenoh shows up on both — but they are different things and you choose them separately.

- **Operator I/O** — how intent reaches the node and how telemetry leaves it. Selected at build time by the entry-point flags `TELEOP_WITH_KEYBOARD` (the `keyboard_control` executable) and `TELEOP_WITH_ZENOH` (the `zenoh_control` executable).
  - **keyboard** — a local terminal reads `WASD` and friends from stdin (intent in).
  - **console** — the same terminal renders live vehicle telemetry (telemetry out). Keyboard and console are the two halves of the local terminal operator and are built together by `TELEOP_WITH_KEYBOARD`.
  - **zenoh** — intent and telemetry are exchanged as JSON over Zenoh keys, so any client (a browser UI, a script, another machine) can drive and observe. Built by `TELEOP_WITH_ZENOH`.
- **Autoware transport** — how the node talks to Autoware. Selected by the CMake variable `TELEOP_AUTOWARE_TRANSPORT`.
  - **`rclcpp`** (default) — native ROS 2: publishers, subscribers and AD-API service calls. Build it into any Autoware workspace as an ordinary ROS node.
  - **`native_zenoh`** — ROS-free: the node speaks Autoware's DDS/CDR wire format directly over Zenoh; its only runtime library is Zenoh's `libzenohc`, with Fast-CDR and libyaml linked in statically. No ROS, no message packages.

Because Operator I/O and Autoware transport meet only at a link-time seam (a generic control loop over an *intent source* and a *telemetry sink*, behind a transport-blind `AutowareGateway`), every I/O can pair with either transport.

### The matrix

The two I/O entry points each link against either Autoware transport — two executables × two transports = four buildable binaries, one per cell:

| Operator I/O ↓ \ Autoware transport → | `rclcpp` (native ROS) | `native_zenoh` (ROS-free) |
| :-- | :-- | :-- |
| **`keyboard_control`** (keyboard intent + console telemetry) | ✅ **keyboard demo** — the primary getting-started path | ✅ local terminal, no ROS |
| **`zenoh_control`** (zenoh JSON intent + telemetry) | ✅ **your UI over Zenoh, Autoware over ROS** | ✅ **cross-machine remote driving, fully ROS-free** |

Each row is one entry-point executable: `keyboard_control` bundles the keyboard intent and the console telemetry (the two halves of the local terminal operator, never selected apart), and `zenoh_control` carries both over Zenoh. Each half is transport-agnostic, so either executable builds against either transport. The three paths in bold are the ones to know:

- **`keyboard_control` × `rclcpp`** — the demo in [Quickstart](#quickstart). Build as a ROS node, drive from the terminal.
- **`zenoh_control` × `rclcpp`** — drive from your own client over Zenoh while the node still integrates with Autoware over ROS. The everyday integration path: see [Operator I/O](#operator-io).
- **`zenoh_control` × `native_zenoh`** — operator I/O over Zenoh *and* Autoware transport over Zenoh: no ROS anywhere on the operator side, suitable for cross-machine remote driving over the Internet. Covered in the [book](#going-further).

This README covers the ROS-node side of the matrix (the `rclcpp` transport, with keyboard or Zenoh I/O). The `native_zenoh` transport, cross-machine remote driving, architecture, building and testing live in the **[book](#going-further)**.

## Quickstart

The fastest way to drive: the published release container brings up Autoware and the teleop node together, and you attach with one script. The first run pulls a multi-GB `autoware:universe`-based image and starts the planning simulator, so budget several minutes for the one-time setup; the interactive drive itself takes about two.

### 1. Seed the map (once)

The Autoware container needs a map volume. Populate it once:

```bash
docker run --rm -v autoware_map:/map \
  ghcr.io/autowarefoundation/autoware-tools:scenario-simulator \
  cp -r /opt/autoware/share/kashiwanoha_map/map/. /map/
```

### 2. Bring up Autoware + teleop

```bash
docker compose -f docker-compose-release.yaml up -d
```

### 3. Attach and drive

```bash
./run_teleop.sh
```

You are now at the keyboard controller. Press `R` to seed the initial pose, `Z` to engage (leave `STOP`), `M` to select a moving drive mode (the initial mode is `stop`, which holds max braking), `X` for Drive, then `WASD` to drive.

### Build from source as a ROS node

If you already have a ROS 2 Humble + Autoware workspace, the node drops in like any other package:

```bash
mkdir -p autoware_manual_control_ws/src
cd autoware_manual_control_ws/src
git clone https://github.com/evshary/autoware_manual_control.git
cd ..
colcon build --packages-select autoware_manual_control
source install/setup.bash
ros2 run autoware_manual_control keyboard_control
```

`keyboard_control` is the keyboard-I/O / `rclcpp`-transport executable — the default build (`TELEOP_WITH_KEYBOARD=ON`, `TELEOP_AUTOWARE_TRANSPORT=rclcpp`), so no extra flags are needed.

### Controls

| Key | Action |
| :-- | :-- |
| `W` / `S` | Throttle / brake |
| `A` / `D` | Steer left / right |
| `X` / `C` / `V` | Shift to Drive / Reverse / Park (brakes to a stop, then shifts) |
| `Z` | Toggle the AD-API operation mode between `STOP` and the configured `operator_mode` (`REMOTE` by default). A single press with no separate engage key: the node auto-enables Autoware control once the drive mode is active (self-engage). Does not change the active drive mode (that is `M`) |
| `M` | Cycle the active drive mode, in the config `modes:` order (`stop` → `physics` → `cruise` → …) |
| `R` | Cycle the initial-pose presets and seed `/initialpose` |
| `Space` | Emergency stop: any drive mode → `stop` (max braking); press again resumes the previous drive mode |
| `Q` | Quit |

The status line shows the live operation mode, gear, real and target speed, and steer. A `Keys: [....]` line echoes `WASD` (UPPERCASE = held, lowercase = tapped, `.` = idle).

The shipped drive modes are **Physics** (realistic inertia and friction; steering auto-centers on release), **Cruise** (`W`/`S` snap the target speed by 1 km/h on tap or ramp it on hold; steering holds its angle), and **Stop** (the required initial and emergency mode; commands max braking). Modes are config-selected plugins.

## Development

Where the [Quickstart](#quickstart) pulls the prebuilt release image, [`run_containers.sh`](run_containers.sh) builds and runs the whole stack from source — Autoware, the teleop node (built from this checkout via [`docker-compose.yaml`](docker-compose.yaml)), and, on the Zenoh transport, the DDS↔Zenoh bridge. It takes `--transport ros|zenoh`, `--isolated`, `--no-autoware` (bring your own Autoware), and `--operator` (a minimal ROS-free operator host that drives a remote vehicle over Zenoh — see the book's [Cross-Machine Remote Driving](book/src/cross-machine.md)), and pairs with [`run_teleop.sh`](run_teleop.sh), which attaches to the running stack and compiles the matching Autoware backend (`rclcpp` or `native_zenoh`). See the book's [Running Locally](book/src/running-locally.md) chapter for the commands and the compose details.

## Operator I/O

Both operator-I/O executables run over the same `rclcpp` Autoware transport — you pick I/O by which executable you build, independently of the transport.

### keyboard + console (`keyboard_control`)

The local terminal operator. `WASD` and the action keys above produce intent; the console renders telemetry. This is the default build and the Quickstart path.

```bash
ros2 run autoware_manual_control keyboard_control --ros-args --params-file <your-params>.yaml
```

### zenoh I/O (`zenoh_control`)

The same control logic, but intent and telemetry travel over Zenoh as JSON instead of the terminal — so your own UI drives the vehicle while the node still talks to Autoware over ROS. Build the Zenoh-I/O executable:

```bash
colcon build --packages-select autoware_manual_control \
  --cmake-args -DTELEOP_WITH_KEYBOARD=OFF -DTELEOP_WITH_ZENOH=ON
ros2 run autoware_manual_control zenoh_control --ros-args --params-file <your-params>.yaml
```

`zenoh_control` has no local keyboard. It subscribes operator intent on `manual_control/<scope>/intent` and publishes vehicle telemetry on `manual_control/<scope>/telemetry`, both JSON, where `<scope>` defaults to `v1` (the `scope` parameter). The input vocabulary is the same `WASD`/`RZXCVM` as the keyboard — a client just sends the decoded values.

**Intent** (client → node), one JSON object per control tick:

| Field | Type | Meaning |
| :-- | :-- | :-- |
| `throttle` / `brake` / `steer` | number | drive axes (the keyboard's `W` / `S` / `A`·`D`) |
| `gear` | string | `PARK`, `DRIVE`, or `REVERSE` |
| `mode_cycle` / `toggle_auto` / `reset_pose` / `estop` | integer | monotonic counters — increment to trigger the `M` / `Z` / `R` / `Space` actions (drop- and duplicate-safe). `estop` toggles emergency braking once per increment, mirroring the keyboard `Space`; a client increments the value to fire, it does **not** hold a non-zero level. |

**Telemetry** (node → client), published every control tick:

| Field | Meaning |
| :-- | :-- |
| `operation_mode` / `mode` / `mode_status` | AD-API operation mode, active drive mode, mode status text |
| `velocity` / `steer_angle` / `gear` | live vehicle state |
| `target_velocity` / `target_acceleration` / `target_steer` | the command being sent |
| `shift_state` / `pending_gear` | gear-shift progress |
| `info` | human-readable status / last error |
| `watchdog_tripped` | bool — the deadman / arrival-timeout state; `true` when no fresh intent arrived within `arrival_timeout_ms` |
| `timestamp` | publish time (ms) |

If no fresh intent arrives within `arrival_timeout_ms` (default 500 ms), the node deadman-brakes until input resumes — a stale or disconnected client cannot leave the vehicle driving.

## Slotting into an Autoware launch

The node integrates with Autoware over the standard AD-API and vehicle interface. With the `rclcpp` transport it:

- **publishes** control to `/external/selected/control_cmd` (transient-local QoS), gear to `/external/selected/gear_cmd`, the initial pose to `/initialpose`, and an operator heartbeat to `/external/local/heartbeat` or `/external/remote/heartbeat`;
- **subscribes** vehicle state on `/vehicle/status/{velocity,steering,gear}_status`, operation mode on `/api/operation_mode/state`, and localization on `/api/localization/initialization_state`;
- **calls** three AD-API operation-mode services: `/api/operation_mode/enable_autoware_control`, `/api/operation_mode/change_to_stop`, and `/api/operation_mode/change_to_local` *or* `change_to_remote` (whichever `operator_mode` selects — a single build calls one, not both).

`/external/selected/control_cmd` is the input both the LOCAL and REMOTE operation-mode gates forward, so one publish path drives the vehicle in either mode. To add the node to your own launch, run the executable as a ROS node and hand it a params file.

The shipped [`config/teleop.yaml`](config) is a standard ROS 2 params file (`/ManualControl: ros__parameters:`), so `ros2 launch` and `--ros-args --params-file` consume it as-is — and the `native_zenoh` binary reads the *same* file via `--config`, unwrapping the envelope itself. One file, no transformation on any path:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="autoware_manual_control",
            executable="keyboard_control",  # or zenoh_control
            output="screen",
            parameters=["/path/to/teleop.yaml"],
        ),
    ])
```

## Configuration

Configuration lives under [`config/`](config). A fully-commented template ships as [`config/teleop.example.yaml`](config/teleop.example.yaml); copy it to a local (gitignored) `config/teleop.yaml` before tuning:

```bash
cp config/teleop.example.yaml config/teleop.yaml
```

The file is a standard ROS 2 params file; both transports read it as-is. The most-used keys (all under `ros__parameters:`):

| Key | Meaning |
| :-- | :-- |
| `operator_mode` | The AD-API operation mode this operator commands: `local` or `remote`. `STOP` is the implicit idle/safe operation mode; `Z` toggles between `STOP` and this mode — a single press with no separate engage key, after which the node auto-enables Autoware control once the drive mode is active (self-engage). |
| `modes` | The active drive modes, in `M`-cycle order. Must include `stop`; an empty list, a missing `stop`, or an unknown name aborts startup with an explanatory error. |
| `control_rate_hz`, `shift_stop_tolerance`, `shift_brake_accel` | Shared control-loop settings (loop rate; the stop-wait-shift gear-change behaviour). |
| `physics:` / `cruise:` / `stop:` | Per-mode tuning blocks (speeds, accel/brake limits, steering rates, ...). Each mode reads its own block. |
| `init_pose.presets` | Named poses (`[x, y, z, yaw]` in the map frame) cycled by the `R` key. |

`run_teleop.sh` feeds this file to the node unchanged: `--ros-args --params-file` for the `rclcpp` transport, `--config` for the `native_zenoh` binary.

The Zenoh-only keys (`scope`, `arrival_timeout_ms`, `zenoh_config`) and the full per-key reference are in the book's [Configuration](book/src/configuration.md) chapter.

## Going further

Everything beyond the ROS node lives in the **book** (`book/`):

- **[Introduction](book/src/introduction.md)** — the two axes, the full matrix, and the link-time seam that makes them independent.
- **[Autoware Transport](book/src/autoware-transport.md)** — `rclcpp` and the ROS-free `native_zenoh` transport in depth: the regenerated CDR codec and the wire contract.
- **[Cross-Machine Remote Driving](book/src/cross-machine.md)** — driving a vehicle from another machine over Zenoh with no ROS on the operator side (the shipped ROS-free local terminal, or Zenoh operator I/O as a swap-in), over the Internet via a cloud Zenoh router (NAT traversal, TLS); the `--isolated` local proof.
- **[Configuration](book/src/configuration.md)** — every parameter, grounded in the config files and the parameter reader.
- **[Architecture](book/src/architecture.md)** — the `io/` `transport/` `core/` layout and the seam; how to add an I/O or a transport.
- **[Building](book/src/building.md)** — the ROS build, the truly ROS-free `native_zenoh` build, and the IDL static vendoring.
- **[Running Locally](book/src/running-locally.md)** — `run_containers.sh`: the whole stack built and run from source, the `--transport`/`--isolated` options, and the Zenoh bridge compose service.
- **[Testing & CI](book/src/testing.md)** — the CDR golden gate, the golden-drift guard, and the no-ROS CI job.

Render it locally with [mdBook](https://rust-lang.github.io/mdBook/):

```bash
mdbook serve book   # or: mdbook build book
```

## Troubleshooting

**Global status error / missing map.** A "Global Status Error" in RViz, or TF errors about a missing `map` frame, usually means the `autoware_map` volume is empty. Run the map-seed command from [Quickstart](#quickstart).

## Maintainers

| Avatar | GitHub ID | Name |
| --- | --- | --- |
| <a href="https://github.com/evshary"><img src="https://github.com/evshary.png" width="48" alt="evshary" /></a> | [@evshary](https://github.com/evshary) | ChenYing Kuo |
| <a href="https://github.com/Shiritai"><img src="https://github.com/Shiritai.png" width="48" alt="Shiritai" /></a> | [@Shiritai](https://github.com/Shiritai) | Tzu-Ching Yang |
