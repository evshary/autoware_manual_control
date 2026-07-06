# Running Locally

`run_containers.sh` brings up the whole stack **built from this checkout** on one machine: Autoware, the teleop node compiled from source, and — on the Zenoh transport — the DDS↔Zenoh bridge. It is the development counterpart to the prebuilt release image: where the Quickstart's `docker-compose-release.yaml` pulls a published `autoware:universe`-based teleop image, this stack uses `docker-compose.yaml`, which builds the teleop service locally (`build: .`) from the working tree. That tree is also bind-mounted into the container, so `run_teleop.sh` recompiles against your edits on each attach.

## Bringing the stack up

```bash
./run_containers.sh up [--transport ros|zenoh] [--isolated] [--no-autoware]
```

`up` starts the `autoware`, `visualizer` and `teleop` services — and, with `--transport zenoh`, the `zenoh_bridge`. The options:

- `--transport ros|zenoh` (default `ros`) — selects the **Autoware transport** the teleop builds against: `ros` for the `rclcpp` build, `zenoh` for the ROS-free `native_zenoh` build (see [Building](building.md)). `zenoh` additionally enables the compose `zenoh` profile, which pulls in the `zenoh_bridge` service (below).
- `--isolated` (default: host network) — places the `teleop` container on its own bridge network so it reaches Autoware only over the bridge's explicit TCP endpoint, instead of sharing the host's DDS network. This is the single-host stand-in for cross-machine operation; the [`--isolated` local proof](cross-machine.md#--isolated-the-local-proof) covers what it proves and why.
- `--no-autoware` — leaves the `autoware` service out of this stack, for when Autoware already runs elsewhere (another machine, or a separately launched instance).

`up` records the chosen transport and isolation in a per-project state file (`tmp/state.<project>.env`), so `run_teleop.sh` builds and configures to match. Keying the state by compose project lets an operator and a vehicle stack coexist on one host without clobbering each other.

## The Zenoh bridge service

Autoware speaks DDS, so the `native_zenoh` transport needs a DDS↔Zenoh bridge between it and Autoware. That bridge ships as a compose service guarded by the `zenoh` profile:

```yaml
zenoh_bridge:
  profiles: ["zenoh"]
  image: eclipse/zenoh-bridge-ros2dds:1.9.0
  command: ["-n", "/v1", "-c", "/config/zenoh-bridge.json5", "-l", "tcp/0.0.0.0:7447"]
```

Because the profile gates it, the plain `ros` transport never starts it; `--transport zenoh` activates the profile and brings it up alongside the rest. `down` activates the profile too, so the bridge is torn down regardless of which transport started the stack. The bridge's `/v1` namespace and its allow-list are detailed in [Cross-Machine Remote Driving](cross-machine.md).

## Attaching and driving

```bash
./run_teleop.sh
```

`run_teleop.sh` finds the compose project that has `teleop` running (preferring this local stack over a release stack when both are up), reads that project's `tmp/state.<project>.env`, and compiles the matching backend inside the container — `native_zenoh` for the Zenoh transport, `rclcpp` otherwise — before launching the local keyboard operator. Edit the source and re-run `run_teleop.sh` to recompile against the mounted checkout.

## Inspecting and tearing down

```bash
./run_containers.sh ps      # service status
./run_containers.sh logs    # aggregated logs
./run_containers.sh down    # stop and remove the stack (bridge included)
```

`ps`, `logs` and `config` pass straight through to `docker compose`. `down` stops and removes every service — the `zenoh` profile is forced on so the `zenoh_bridge` goes even after a `ros` run — and deletes the generated `tmp/` override and state files.
