#!/bin/bash

# Color definitions
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

run_compose() {
    local context_name="$1"
    shift
    local target_services="$1"
    shift

    # Read the command passed by the user. If not provided, default to "up".
    # Using parameter expansion to separate command and arguments.
    local cmd="${1:-up}"
    shift
    local args="$@"
    
    echo -e "${YELLOW}[${context_name}]${NC} Target Services: ${GREEN}${target_services}${NC}"

    # Logic handling
    case "$cmd" in
        "up")
            # If user inputs only 'up' or nothing (default), we might want to add -d, 
            # but here we respect user arguments.
            echo -e "${YELLOW}[${context_name}]${NC} Starting services..."
            docker compose up $args $target_services
            ;;
            
        "down")
            # Special handling: 'docker compose down' removes the entire network, affecting the other side.
            # Here we use 'stop' + 'rm -v' to remove specific services, simulating 'down' effect.
            echo -e "${RED}[${context_name}]${NC} Stopping and removing services (with volumes)..."
            docker compose stop $target_services
            docker compose rm -f -v $target_services
            ;;
            
        "dry-run")
            echo -e "${YELLOW}[${context_name}]${NC} [Dry Run] Would start services: ${GREEN}${target_services}${NC}"
            echo -e "${YELLOW}[${context_name}]${NC} Validating compose configuration..."
            docker compose config
            ;;

        *)
            # Pass through other commands (e.g., start, stop, restart, logs, pull, ps)
            echo -e "${YELLOW}[${context_name}]${NC} Executing: docker compose $cmd $args ..."
            docker compose $cmd${args:+ $args} $target_services
            ;;
    esac
}

# Define Edge services
TELEOP_SERVICES="autoware scenario_simulator visualizer teleop"

# Argument parsing
CMD=""
ARGS=()

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        --no-sim)
            export SCENARIO_SIMULATION="false"
            # Remove scenario_simulator from TELEOP_SERVICES
            TELEOP_SERVICES="${TELEOP_SERVICES/scenario_simulator/}"
            # Clean up extra spaces if any
            TELEOP_SERVICES=$(echo "$TELEOP_SERVICES" | xargs)
            shift
            ;;
        up|down|ps|logs|config|dry-run)
            CMD="$1"
            shift
            ;;
        *)
            ARGS+=("$1")
            shift
            ;;
    esac
done

# Export default if not set
export SCENARIO_SIMULATION="${SCENARIO_SIMULATION:-true}"

# Default command is 'up' if not specified
if [ -z "$CMD" ]; then
    CMD="up"
fi

# Run Compose
run_compose "TELEOP" "$TELEOP_SERVICES" "$CMD" "${ARGS[@]}"
