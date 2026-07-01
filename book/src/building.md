# Building

There are two fundamentally different builds, picked by `TELEOP_AUTOWARE_TRANSPORT`: a ROS build (`rclcpp`) that needs an Autoware workspace, and a **truly ROS-free** build (`native_zenoh`) that needs only CMake, a C++ compiler and Fast-CDR. The I/O flags `TELEOP_WITH_KEYBOARD` (default `ON`) and `TELEOP_WITH_ZENOH` (default `OFF`) select the executable(s) and are orthogonal to the transport.

## The ROS build (`rclcpp`)

This is the default and integrates as a normal ROS 2 package in an Autoware (Humble) workspace.

```bash
# keyboard + console, rclcpp transport — the default, no flags needed
colcon build --packages-select autoware_manual_control
ros2 run autoware_manual_control keyboard_control
```

For Zenoh operator I/O over the ROS transport, switch the entry-point flags. This adds two dependencies — `nlohmann_json` (JSON for the operator I/O) and the Zenoh C/C++ SDK (`zenohc` / `zenohcxx`):

```bash
colcon build --packages-select autoware_manual_control \
  --cmake-args -DTELEOP_WITH_KEYBOARD=OFF -DTELEOP_WITH_ZENOH=ON
ros2 run autoware_manual_control zenoh_control --ros-args --params-file config/teleop.yaml
```

Getting the Zenoh SDK:

- **apt** — add the Eclipse Zenoh repository and `sudo apt install libzenohc-dev libzenohcpp-dev`. Use a Zenoh 1.x release matching the peer you connect to, or the endpoints will not discover each other.
- **from an `rmw_zenoh` workspace** — the Zenoh vendor is already bundled; point CMake at it with `-DZENOH_VENDOR_PREFIX=/path/to/.../zenoh_cpp_vendor` (or export the env var of the same name), and add its `lib` to `LD_LIBRARY_PATH`.

## The ROS-free build (`native_zenoh`)

With `TELEOP_AUTOWARE_TRANSPORT=native_zenoh`, `ament_cmake` is optional and skipped when absent (`CMakeLists.txt`): the package builds as a plain CMake project. The dependencies are only the Zenoh SDK, `nlohmann_json`, a system `libyaml` (linked **static**, so no `libyaml.so` joins the runtime), and Fast-CDR — which CMake fetches itself (`FetchContent`, pinned `v1.1.1`, built static + PIC and folded into the binary). No ROS, no message packages, no rosidl tooling.

```bash
cmake -S . -B build \
  -DTELEOP_AUTOWARE_TRANSPORT=native_zenoh \
  -DTELEOP_WITH_ZENOH=ON -DTELEOP_WITH_KEYBOARD=OFF \
  -DZENOH_VENDOR_PREFIX=/path/to/zenoh   # omit if zenohc/zenohcxx are installed system-wide
cmake --build build -j
./build/zenoh_control --config config/teleop.yaml
```

The resulting binary links only `libzenohc`, the static Fast-CDR and the static libyaml, plus the C/C++ runtime — and its wire bytes are correct by construction: it runs the same `rosidl_typesupport_fastrtps_cpp` serialiser `rmw_fastrtps` uses (see [Autoware Transport](autoware-transport.md)). You can also build the local-terminal operator with no ROS by flipping the I/O flags (`-DTELEOP_WITH_KEYBOARD=ON -DTELEOP_WITH_ZENOH=OFF`); CMake then builds `keyboard_control` against the same ROS-free transport.

## IDL static vendoring

The native transport's codec is not generated at build time — it is **committed** under `src/transport/zenoh/generated/`, marked `linguist-generated` in `.gitattributes`. That directory holds the `rosidl_generator_cpp` message headers, the `rosidl_typesupport_fastrtps_cpp` serialisers, and the rosidl runtime headers those sources include. Because it is checked in, the normal build needs no rosidl tooling and no ROS.

The vendored sources only change when the upstream Autoware messages change. Regenerating them is the *one* step that needs a ROS 2 + Autoware environment.

### Regenerating the codec

```bash
source /opt/ros/humble/setup.bash   # or /opt/autoware/setup.bash
tools/vendor_idl.sh                 # rewrites src/transport/zenoh/generated/, then commit it
```

`tools/vendor_idl.sh` runs in four steps:

1. Builds the rosidl generators from the tiny `tools/regen` ament package (`tools/regen/CMakeLists.txt` lists the wire contract as `TELEOP_MSG_IDL`; `stage_idl.py` stages the installed `.idl` under this project's namespace so the generated types are namespaced correctly). `tools/COLCON_IGNORE` keeps this helper package out of the normal workspace build.
2. Copies the generated `rosidl_generator_cpp` and `rosidl_typesupport_fastrtps_cpp` output into `src/transport/zenoh/generated/`.
3. Computes the include closure of that generated code with `g++ -M` and vendors the external rosidl runtime headers it pulls in (`rosidl_runtime_c`, `rosidl_runtime_cpp`, `rosidl_typesupport_*`, `rmw`, `rcutils`).
4. Writes the files and reports the counts — it does not `git add` them, so commit the result yourself.

After regenerating, the CI golden-drift guard ([Testing & CI](testing.md)) confirms the new bytes still match a live rmw. CI flags when a regen is *due* by failing that same guard against the old committed bytes.
