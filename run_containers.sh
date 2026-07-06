#!/bin/bash
set -euo pipefail

# compose expands ${HOSTNAME} in extra_hosts.
export HOSTNAME="${HOSTNAME:-$(hostname)}"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'

show_help() {
    cat <<EOF
Usage: $(basename "$0") [up|down|ps|logs|config|dry-run] [options]

Commands:
  up        Start the container stack
  down      Stop and remove the stack
  ps        List container status
  logs      Show container logs
  config    Validate and view the merged compose config
  dry-run   Render the merged compose config without starting anything

Options for 'up':
  --transport <ros|zenoh>  Autoware transport: 'ros' (default) or 'zenoh'
  --isolated               Put the teleop on its own bridge network (default: host)
  --no-autoware            Exclude Autoware (run it separately)
  --operator               Operator host: run only the minimal ROS-free teleop that
                           reaches a remote vehicle over Zenoh. Implies --transport zenoh.
  -h, --help               Show this help

Environment (either side can be the server or the dialer):
  ZENOH_CONNECT  Dial OUT to this endpoint (tcp/host:7447) -- the far side's listen
                 address, or a cloud router. Unset on the vehicle = the bridge listens.
  ZENOH_LISTEN   Be a fixed-IP server on this endpoint (tcp/0.0.0.0:7447) the far side
                 dials into. The vehicle bridge listens by default; set this on
                 --operator to make the operator the server the vehicle dials.
  ZENOH_MODE     Dial mode: 'peer' (default, direct to the far end) or 'client' (the
                 far end is a router -- a router relays only between clients).
EOF
}

CMD=""
TRANSPORT="ros"
ISOLATED="false"
NO_AUTOWARE="false"
OPERATOR="false"
ZENOH_CONNECT="${ZENOH_CONNECT:-}"
ZENOH_LISTEN="${ZENOH_LISTEN:-}"
ZENOH_MODE="${ZENOH_MODE:-}"
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help) show_help; exit 0 ;;
        up|down|ps|logs|config|dry-run) CMD="$1"; shift ;;
        --transport) TRANSPORT="$2"; shift 2 ;;
        --isolated) ISOLATED="true"; shift ;;
        --no-autoware) NO_AUTOWARE="true"; shift ;;
        --operator) OPERATOR="true"; shift ;;
        *) ARGS+=("$1"); shift ;;
    esac
done
[ -n "$CMD" ] || CMD="up"
mkdir -p tmp

# The vehicle bridge command, built from its role: -l to be a fixed-IP server
# (ZENOH_LISTEN), -e to dial out (ZENOH_CONNECT), and the ZENOH_MODE positional when
# dialing (see --help for why a router needs 'client').
bridge_command() {
    local cmd='"-n", "/v1", "-c", "/config/zenoh-bridge.json5"'
    if [ -n "$ZENOH_LISTEN" ]; then cmd="$cmd, \"-l\", \"$ZENOH_LISTEN\""; fi
    if [ -n "$ZENOH_CONNECT" ]; then cmd="$cmd, \"-e\", \"$ZENOH_CONNECT\""; fi
    if [ -n "$ZENOH_CONNECT" ] && [ -n "$ZENOH_MODE" ]; then cmd="$cmd, \"$ZENOH_MODE\""; fi
    printf '[%s]' "$cmd"
}

is_render_cmd() { case "$CMD" in up|config|dry-run) return 0 ;; *) return 1 ;; esac; }

# Resolve the role into its compose project, files, override, and services. Each
# role has its own project name (from the compose `name:`) so an operator and a
# vehicle can coexist on one host without clobbering each other's state.
COMPOSE=(); SERVICES=()
if [ "$OPERATOR" = "true" ]; then
    TRANSPORT="zenoh"
    PROJECT="autoware_manual_control_operator"
    OVERRIDE="tmp/${PROJECT}.override.yaml"
    COMPOSE=("-f" "docker-compose-operator.yaml")
    if is_render_cmd; then
        rm -f "$OVERRIDE"
        if [ -n "$ZENOH_LISTEN" ]; then
            # Publish the listen port so a remote vehicle can dial this operator.
            port="${ZENOH_LISTEN##*:}"
            printf 'services:\n  teleop:\n    ports: ["%s:%s"]\n' "$port" "$port" > "$OVERRIDE"
        fi
    fi
    if [ -f "$OVERRIDE" ]; then COMPOSE+=("-f" "$OVERRIDE"); fi
    SERVICES=("teleop")
else
    PROJECT="autoware_manual_control_ws"
    OVERRIDE="tmp/${PROJECT}.override.yaml"
    COMPOSE=("-f" "docker-compose.yaml")
    # `down` activates the zenoh profile too, so the bridge is removed whatever the
    # transport `up` was given.
    if [ "$TRANSPORT" = "zenoh" ] || [ "$CMD" = "down" ]; then COMPOSE+=("--profile" "zenoh"); fi
    if is_render_cmd; then
        bridge_role="false"
        if [ "$TRANSPORT" = "zenoh" ] && { [ -n "$ZENOH_CONNECT" ] || [ -n "$ZENOH_LISTEN" ]; }; then
            bridge_role="true"
        fi
        if [ "$ISOLATED" = "true" ] || [ "$bridge_role" = "true" ]; then
            {
                echo "services:"
                if [ "$ISOLATED" = "true" ]; then
                    printf '  teleop:\n    network_mode: bridge\n    extra_hosts:\n      - "host.docker.internal:host-gateway"\n'
                fi
                if [ "$bridge_role" = "true" ]; then
                    printf '  zenoh_bridge:\n    command: %s\n' "$(bridge_command)"
                fi
            } > "$OVERRIDE"
        else
            rm -f "$OVERRIDE"
        fi
    fi
    if [ -f "$OVERRIDE" ]; then COMPOSE+=("-f" "$OVERRIDE"); fi
    SERVICES=("visualizer" "teleop")
    if [ "$TRANSPORT" = "zenoh" ]; then SERVICES+=("zenoh_bridge"); fi
    if [ "$NO_AUTOWARE" = "false" ]; then SERVICES=("autoware" "${SERVICES[@]}"); fi
fi

STATE="tmp/state.${PROJECT}.env"

case "$CMD" in
    up)
        echo -e "${YELLOW}[TELEOP]${NC} Starting ${GREEN}${PROJECT}${NC} (transport=${TRANSPORT}, isolated=${ISOLATED}, no-autoware=${NO_AUTOWARE})..."
        # State for run_teleop.sh, keyed by project so roles never overwrite each other.
        printf 'TRANSPORT="%s"\nISOLATED="%s"\nZENOH_CONNECT="%s"\nZENOH_LISTEN="%s"\nZENOH_MODE="%s"\n' \
            "$TRANSPORT" "$ISOLATED" "$ZENOH_CONNECT" "$ZENOH_LISTEN" "$ZENOH_MODE" > "$STATE"
        docker compose "${COMPOSE[@]}" up -d ${ARGS[@]+"${ARGS[@]}"} "${SERVICES[@]}"
        ;;
    down)
        echo -e "${RED}[TELEOP]${NC} Stopping ${PROJECT}..."
        docker compose "${COMPOSE[@]}" down ${ARGS[@]+"${ARGS[@]}"}
        rm -f "$OVERRIDE" "$STATE"
        echo -e "${GREEN}[TELEOP]${NC} Cleaned up."
        ;;
    dry-run)
        echo -e "${YELLOW}[TELEOP]${NC} Dry run (${PROJECT}, transport=${TRANSPORT}, isolated=${ISOLATED})"
        docker compose "${COMPOSE[@]}" config ${ARGS[@]+"${ARGS[@]}"} "${SERVICES[@]}"
        ;;
    *)
        docker compose "${COMPOSE[@]}" "$CMD" ${ARGS[@]+"${ARGS[@]}"}
        ;;
esac
