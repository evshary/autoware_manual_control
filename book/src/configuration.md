# Configuration

All configuration lives under `config/`. The behaviour is driven by one file — a local, gitignored `config/teleop.yaml` you copy from the fully-commented `config/teleop.example.yaml`:

```bash
cp config/teleop.example.yaml config/teleop.yaml
```

The file is a **standard ROS 2 params file** (`/ManualControl: ros__parameters:`), and the same file feeds both transports unchanged; only the reading path differs (see [How parameters are read](#how-parameters-are-read)). Every key is optional and falls back to the default shown below.

## Shared keys

These apply to every build.

| Key | Type | Default | Meaning |
| :-- | :-- | :-- | :-- |
| `operator_mode` | string | `remote` | The AD-API operation mode this operator commands: `local` or `remote`. `STOP` is the implicit idle/safe operation mode; `Z` toggles between `STOP` and this mode — a single press with no separate engage key, after which the node auto-enables Autoware control once the drive mode is active (self-engage). Selects the heartbeat key (`/external/local/heartbeat` vs `/external/remote/heartbeat`) and which availability flag gates driving. |
| `modes` | list&lt;string&gt; | `["stop", "physics", "cruise"]` | The active drive modes in `M`-cycle order. **Must include `stop`** (the initial and emergency mode). An empty list, a missing `stop`, or an unknown name aborts startup with an explanatory error. |
| `control_rate_hz` | double | `60.0` | Control-loop rate. |
| `shift_stop_tolerance` | float | `0.05` | m/s — speed below which a pending gear shift proceeds (stop-wait-shift). |
| `shift_brake_accel` | float | `-10.0` | m/s² — override acceleration applied while braking to a stop for a shift. |
| `init_pose.presets.names` | list&lt;string&gt; | `["origin"]` | Names of the initial-pose presets cycled by the `R` key. |
| `init_pose.presets.<name>` | list&lt;double&gt; | `[0, 0, 0, 0]` | The `[x, y, z, yaw]` (map frame) for each preset name. Pressing `R` cycles the names and seeds `/initialpose` from the matching array. |

> The two `init_pose` rows above are the **code fallbacks**, used only when the key is absent. The shipped `config/teleop.example.yaml` overrides them with a single preset named `start` on a drivable lane of the bundled sample map (`start: [3765.09, 73743.88, 0.0, -0.5445]`), so a fresh copy localises the demo in one `R` press; replace it with poses valid for your own map.

## Per-mode tuning

Each drive mode reads its own block; a mode that is not in `modes` is still registered but inert. The keys are exactly the fields read in each mode's `loadParams` (`src/modes/*_mode.hpp`).

### `physics:` — `PhysicsDriveMode`

| Key | Default | Meaning |
| :-- | :-- | :-- |
| `physics.max_speed` | `27.78` | m/s (≈100 km/h) speed ceiling. |
| `physics.max_steer` | `0.6` | rad — steering limit. |
| `physics.steer_rate` | `0.8` | rad/s while a steer key is held. |
| `physics.steer_decay` | `1.0` | rad/s auto-center rate on release. |
| `physics.steer_deadzone` | `0.01` | rad. |
| `physics.accel_max` | `3.5` | m/s² at full throttle. |
| `physics.brake_max` | `5.0` | m/s² at full brake. |
| `physics.coast_decel` | `2.0` | m/s² with no input (engine braking). |
| `physics.max_vel_offset` | `3.0` | m/s — how far the velocity setpoint may lead the real speed (anti-windup). |

### `cruise:` — `CruiseDriveMode`

| Key | Default | Meaning |
| :-- | :-- | :-- |
| `cruise.max_speed` | `27.78` | m/s speed ceiling. |
| `cruise.steer_rate` | `0.3` | rad/s. |
| `cruise.steer_limit` | `0.6` | rad. |
| `cruise.vel_inc_hold_kph_s` | `5.0` | km/h per second while holding throttle. |
| `cruise.vel_dec_hold_kph_s` | `10.0` | km/h per second while holding brake. |
| `cruise.accel_p_gain` | `3.0` | proportional gain on the speed error. |
| `cruise.max_accel` | `5.0` | m/s² ceiling. |
| `cruise.min_accel` | `-10.0` | m/s² floor. |

### `stop:` — `StopDriveMode`

| Key | Default | Meaning |
| :-- | :-- | :-- |
| `stop.brake_accel` | `-10.0` | m/s² braking command in `STOP` / emergency. |

## Zenoh keys

These are read only by the Zenoh paths — the `zenoh_control` entry point (operator I/O) and the `native_zenoh` transport (`AutowareGateway`). They are inert in a keyboard + `rclcpp` build.

| Key | Type | Default | Meaning |
| :-- | :-- | :-- | :-- |
| `scope` | string | `v1` | Key prefix. Operator I/O uses `manual_control/<scope>/intent` and `manual_control/<scope>/telemetry`; the `native_zenoh` transport uses `<scope>/…` for the Autoware topics. Must match the DDS↔Zenoh bridge namespace (e.g. `-n /v1`). |
| `arrival_timeout_ms` | float | `500.0` | Two uses, both in the Zenoh paths: the `zenoh_control` operator-intent watchdog (no fresh intent within this window → deadman brake), and the `native_zenoh` AD-API service-call timeout. |
| `zenoh_config` | string | `""` | Path to a Zenoh JSON5 config; empty means Zenoh's built-in default. In a `zenoh × native_zenoh` build, both the operator and transport sessions read it, so they join the same Zenoh network. See [Cross-Machine Remote Driving](cross-machine.md). |

## How parameters are read

The same YAML is consumed two different ways, behind the one transport-blind `ParameterReader` interface (`src/core/parameter_reader.hpp`):

- **`rclcpp`** — the file is passed as-is with `--ros-args --params-file`; parameters arrive as ROS parameters. The `ManualControl` node declares parameters from the overrides; the reader coerces a YAML integer to the requested float/double rather than throwing (`src/transport/rclcpp/parameter_reader.cpp`).
- **`native_zenoh`** — the binary reads the same file from `--config <path>`. `src/transport/zenoh/parameter_reader.cpp` parses it with libyaml (statically linked), unwraps the `<node>: {ros__parameters: {...}}` envelope (plain nested YAML without one also works), and flattens the DOM to dotted names: a mapping recurses with its key appended (`physics: { max_speed: x }` → `physics.max_speed`), a sequence becomes a vector, a scalar becomes a one-element vector. libyaml resolves quotes, comments, flow style and UTF-8, so inline sequences (`["stop", "physics"]`), quoted values with trailing comments, and empty-string-as-default all behave as you would expect. A non-numeric value where a number is expected logs a warning and falls back to the default.

Both readers expose the same generic `read<T>(name, default)` for `float`, `double`, `std::string`, `std::vector<std::string>`, and `std::vector<double>`; each transport provides the one explicit instantiation, so neither the core nor the modes ever see `rclcpp` or Zenoh.
