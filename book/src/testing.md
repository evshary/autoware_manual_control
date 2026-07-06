# Testing & CI

The `native_zenoh` transport's codec *is* the `rosidl_typesupport_fastrtps_cpp` output for the message contract, so its wire bytes are correct **by construction** — and that correctness is *safety-relevant*, since those bytes steer a vehicle. The test suite pins that codec, and CI captures real-rmw wire bytes (via `rmw_cyclonedds_cpp`) to catch upstream Autoware message drift. Everything except that one drift guard runs with **no ROS**.

## The two test suites

The tests are registered only for the `native_zenoh` transport (`CMakeLists.txt`). Build that transport, then run them:

```bash
# colcon (ROS workspace)
colcon build --packages-select autoware_manual_control \
  --cmake-args -DTELEOP_AUTOWARE_TRANSPORT=native_zenoh -DTELEOP_WITH_ZENOH=ON
colcon test --packages-select autoware_manual_control \
  --ctest-args -R 'test_cdr|test_param_reader'
colcon test-result --verbose

# or plain ctest (ROS-free build)
ctest --test-dir build --output-on-failure
```

(A bare `colcon test` additionally runs the package's `ament_lint` suite; the `-R` filter scopes the run to the two functional gtests.)

### `test_cdr` — the CDR golden gate

31 tests asserting the native codec's output equals golden reference bytes captured from a real rmw (`rmw_cyclonedds_cpp`). The golden bytes are committed into the test sources, so the gate builds and runs with no Autoware/rmw runtime. It covers three fronts:

- **`Production`** — the actual control/telemetry messages with the literal wire values that matter (the safety-critical `Control`, `GearCommand`, `OperationModeState`, `PoseWithCovarianceStamped`, the `ChangeOperationMode` service, …).
- **`Shapes`** — the vendored `test/test_msgs_idl` corpus, which exercises *every field shape* rosidl supports (all integer widths, floats, bool, byte/char, bounded/unbounded/empty strings, fixed and nested arrays, bounded and unbounded sequences, single- and multi-level nesting, constants, defaults, empty). This certifies the codec on every shape, not only the shapes today's production messages happen to use.
- **`Foxglove`** — an independent CDR decoder oracle, so the bytes are validated both by re-encoding (`encode(value) == golden`) and by an external decoder (`decode(golden) == value`).

### `test_param_reader` — the native parameter reader

12 tests pinning the `native_zenoh` reader's contract: dotted nested keys, inline string and float sequences, type-mismatch fallback, empty-string-as-default, present-but-empty sequences, quoting and trailing-comment semantics, independent sibling sections, the `ros__parameters` envelope unwrap, and the missing-file all-defaults case.

## CI

`.github/workflows/test.yml` runs two jobs.

### `build-rosfree` — proving the ROS-free build

This job is the executable proof that the `native_zenoh` build needs no ROS. It runs on a plain `ubuntu-latest`, installs only `build-essential cmake git ca-certificates curl` (the last two just to fetch the Zenoh signing key), `nlohmann-json3-dev libyaml-dev libgtest-dev` and the Zenoh apt packages — **no ROS at all** — and then explicitly asserts the environment is ROS-free:

```bash
test ! -d /opt/ros
test -z "${AMENT_PREFIX_PATH}"
```

It builds with plain CMake (`-DTELEOP_AUTOWARE_TRANSPORT=native_zenoh -DTELEOP_WITH_ZENOH=ON -DTELEOP_WITH_KEYBOARD=OFF`) and runs the golden gate (`ctest --test-dir build`). If this job ever needs ROS to pass, the ROS-free claim has regressed.

### `golden-drift` — the real-rmw byte compare

ROS is needed in exactly one place: confirming the *committed goldens still match a live rmw*. This job runs in a `ros:humble` container, builds the `native_zenoh` transport via colcon, runs the two gtests, and then runs the drift guard:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 test/capture_golden.py --check
```

`test/capture_golden.py` is a separate refresh/drift tool, **not** part of `colcon test`. It re-serialises every production and test-corpus message through `rclpy` under `rmw_cyclonedds_cpp` — i.e. the live DDS-CDR wire — and `--check` diffs that against the committed goldens. A drifted Autoware message therefore turns CI red and prompts a re-vendor (`tools/vendor_idl.sh`, see [Building](building.md#regenerating-the-codec)). With no flag, the same script prints the freshly captured bytes for pasting into `test_cdr.cpp`'s `GOLDEN(...)` literals.

Together the two jobs close the loop: `build-rosfree` proves the shipped binary carries the codec with no ROS, and `golden-drift` proves that codec still equals the real wire.

## Release image

A separate workflow, `.github/workflows/docker-publish.yml`, builds and pushes the release container image (`ghcr.io/<owner>/autoware_manual_control`) from `Dockerfile.release` on pushes to `main` and `feat/*` and on `v*.*.*` tags (pull requests to `main` build it too, without pushing). It ignores documentation-only changes (`**.md`, `docs/**`).
