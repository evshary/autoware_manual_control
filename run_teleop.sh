#!/bin/bash
set -euo pipefail

YELLOW='\033[1;33m'; GREEN='\033[0;32m'; RED='\033[0;31m'; NC='\033[0m'

# Find the running teleop and its compose project. Operator wins over dev/release so
# a remote operator sharing a host with a vehicle stack still lands in its own
# container; dev wins over release so a source-mounted iteration is preferred.
has_teleop() { docker compose "$@" ps --services --filter status=running 2>/dev/null | grep -q '^teleop$'; }
if has_teleop -f docker-compose-operator.yaml; then
    ROLE="operator"; COMPOSE=(-f docker-compose-operator.yaml); PROJECT="autoware_manual_control_operator"
elif has_teleop; then
    ROLE="dev"; COMPOSE=(-f docker-compose.yaml); PROJECT="autoware_manual_control_ws"
elif has_teleop -f docker-compose-release.yaml; then
    ROLE="release"; COMPOSE=(-f docker-compose-release.yaml); PROJECT="autoware_manual_control_ws"
else
    echo -e "${RED}[Error]${NC} Teleop service is not running."; exit 1
fi

# State from run_containers.sh (keyed by project), with live env vars overriding it.
ZENOH_CONNECT_ENV="${ZENOH_CONNECT:-}"; ZENOH_LISTEN_ENV="${ZENOH_LISTEN:-}"; ZENOH_MODE_ENV="${ZENOH_MODE:-}"
TRANSPORT="ros"; ISOLATED="false"; ZENOH_CONNECT=""; ZENOH_LISTEN=""; ZENOH_MODE=""
STATE="tmp/state.${PROJECT}.env"
if [ -f "$STATE" ]; then source "$STATE"; fi
if [ -n "$ZENOH_CONNECT_ENV" ]; then ZENOH_CONNECT="$ZENOH_CONNECT_ENV"; fi
if [ -n "$ZENOH_LISTEN_ENV" ]; then ZENOH_LISTEN="$ZENOH_LISTEN_ENV"; fi
if [ -n "$ZENOH_MODE_ENV" ]; then ZENOH_MODE="$ZENOH_MODE_ENV"; fi
ZENOH_MODE="${ZENOH_MODE:-peer}"

# Emit a Zenoh client config. Scouting is off -- the teleop reaches the bridge by an
# explicit endpoint (dial ZENOH_CONNECT, or listen as a server on ZENOH_LISTEN), not
# by discovery, so it needs no shared broadcast domain.
gen_zenoh_client() {  # mode connect listen
    printf '{\n  mode: "%s",\n  scouting: { multicast: { enabled: false }, gossip: { enabled: false } },\n' "$1"
    if [ -n "$2" ]; then printf '  connect: { endpoints: ["%s"] },\n' "$2"; fi
    if [ -n "$3" ]; then printf '  listen: { endpoints: ["%s"] },\n' "$3"; fi
    printf '}\n'
}

if [ "$ROLE" = "operator" ]; then
    # Prebuilt ROS-free keyboard_control. Reach the vehicle by dialing out
    # (ZENOH_CONNECT) or by listening as a server it dials into (ZENOH_LISTEN);
    # default to the same-host bridge. Generate the client config, copy it in, run.
    if [ -z "$ZENOH_CONNECT" ] && [ -z "$ZENOH_LISTEN" ]; then ZENOH_CONNECT="tcp/host.docker.internal:7447"; fi
    echo -e "${YELLOW}[Teleop]${NC} Operator (${ZENOH_MODE}${ZENOH_LISTEN:+ listen ${ZENOH_LISTEN}}${ZENOH_CONNECT:+ -> ${ZENOH_CONNECT}})"
    gen_zenoh_client "$ZENOH_MODE" "$ZENOH_CONNECT" "$ZENOH_LISTEN" > tmp/operator_zenoh_client.json5
    docker compose "${COMPOSE[@]}" cp tmp/operator_zenoh_client.json5 teleop:/tmp/zenoh_client.json5
    docker compose "${COMPOSE[@]}" exec -it teleop bash -c '
        cd /opt/teleop &&
        sed "s|zenoh_config: \"\"|zenoh_config: \"/tmp/zenoh_client.json5\"|g" config/teleop.example.yaml > /tmp/teleop_config.yaml &&
        echo -e "\n\033[1;32mStarting Keyboard Control...\033[0m" &&
        exec ./keyboard_control --config /tmp/teleop_config.yaml'
    exit $?
fi

if [ "$ROLE" = "dev" ]; then
    CMAKE_ARGS="-DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release"
    if [ "$TRANSPORT" = "zenoh" ]; then
        CMAKE_ARGS="$CMAKE_ARGS -DTELEOP_AUTOWARE_TRANSPORT=native_zenoh -DTELEOP_WITH_ZENOH=ON"
        # Same client-config generator as the operator, copied in for the rebuilt
        # binary. The default endpoint follows the teleop's network: localhost on the
        # host network, host.docker.internal when --isolated on a bridge network.
        if [ -z "$ZENOH_CONNECT" ] && [ -z "$ZENOH_LISTEN" ]; then
            if [ "$ISOLATED" = "true" ]; then ZENOH_CONNECT="tcp/host.docker.internal:7447"; else ZENOH_CONNECT="tcp/localhost:7447"; fi
        fi
        gen_zenoh_client "$ZENOH_MODE" "$ZENOH_CONNECT" "$ZENOH_LISTEN" > tmp/dev_zenoh_client.json5
        docker compose "${COMPOSE[@]}" cp tmp/dev_zenoh_client.json5 teleop:/tmp/zenoh_client.json5
    else
        CMAKE_ARGS="$CMAKE_ARGS -DTELEOP_AUTOWARE_TRANSPORT=rclcpp"
    fi
    echo -e "${YELLOW}[Teleop]${NC} Dev (transport=${TRANSPORT}, isolated=${ISOLATED}) -- rebuilding..."
    docker compose "${COMPOSE[@]}" exec -it teleop bash -c "
        source /opt/autoware/setup.bash &&
        cd /autoware_manual_control_ws &&
        colcon build --cmake-args $CMAKE_ARGS &&
        source install/setup.bash &&
        cfg=src/autoware_manual_control/config/teleop.example.yaml &&
        if [ -f src/autoware_manual_control/config/teleop.yaml ]; then cfg=src/autoware_manual_control/config/teleop.yaml; fi &&
        if [ \"$ISOLATED\" = true ]; then export ROS_LOCALHOST_ONLY=1; fi &&
        echo -e '\n\033[1;32mStarting Keyboard Control...\033[0m' &&
        if [ \"$TRANSPORT\" = zenoh ]; then
            sed 's|zenoh_config: \"\"|zenoh_config: \"/tmp/zenoh_client.json5\"|g' \"\$cfg\" > /tmp/teleop_config.yaml &&
            exec install/autoware_manual_control/lib/autoware_manual_control/keyboard_control --config /tmp/teleop_config.yaml
        else
            { printf '/ManualControl:\n  ros__parameters:\n'; sed 's/^/    /' \"\$cfg\"; } > /tmp/teleop_config.yaml &&
            exec ros2 run autoware_manual_control keyboard_control --ros-args --params-file /tmp/teleop_config.yaml
        fi"
    exit $?
fi

# release: the packaged rclcpp binary reads its config from the installed share.
echo -e "${YELLOW}[Teleop]${NC} Release"
docker compose "${COMPOSE[@]}" exec -it teleop bash -c '
    source install/setup.bash &&
    cfg="$(ros2 pkg prefix --share autoware_manual_control)/config/teleop.example.yaml" &&
    if [ -f config/teleop.yaml ]; then cfg=config/teleop.yaml; fi &&
    { printf "/ManualControl:\n  ros__parameters:\n"; sed "s/^/    /" "$cfg"; } > /tmp/teleop_config.yaml &&
    echo -e "\n\033[1;32mStarting Keyboard Control...\033[0m" &&
    exec ros2 run autoware_manual_control keyboard_control --ros-args --params-file /tmp/teleop_config.yaml'
