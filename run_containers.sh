#!/bin/bash

# Export HOSTNAME so compose can expand ${HOSTNAME} in extra_hosts.
export HOSTNAME="${HOSTNAME:-$(hostname)}"

# Color definitions
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

show_help() {
    echo "Usage: $0 [up|down|ps|logs|config|dry-run] [options]"
    echo ""
    echo "Commands:"
    echo "  up             Start the container stack"
    echo "  down           Stop and clean up the container stack"
    echo "  ps             List container status"
    echo "  logs           Show container logs"
    echo "  config         Validate and view compose config"
    echo "  dry-run        Dry run validation"
    echo ""
    echo "Options for 'up':"
    echo "  --transport <ros|zenoh>   Specify transport mode: 'ros' (default) or 'zenoh'"
    echo "  --isolated                Run teleop in an isolated bridge network (default: host network)"
    echo "  --no-autoware             Exclude autoware service from this stack (run it separately)"
    echo "  -h, --help                Show this help message"
}

# Parse options
CMD=""
TRANSPORT="ros"
ISOLATED="false"
NO_AUTOWARE="false"
ARGS=()

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        up|down|ps|logs|config|dry-run)
            CMD="$1"
            shift
            ;;
        --transport)
            TRANSPORT="$2"
            shift 2
            ;;
        --isolated)
            ISOLATED="true"
            shift
            ;;
        --no-autoware)
            NO_AUTOWARE="true"
            shift
            ;;
        *)
            ARGS+=("$1")
            shift
            ;;
    esac
done

if [ -z "$CMD" ]; then
    CMD="up"
fi

# Ensure tmp directory exists
mkdir -p tmp

COMPOSE_FILES=("-f" "docker-compose.yaml")

# The zenoh transport pulls in the zenoh_bridge service (compose profile).
# `down` always activates it so the bridge is torn down regardless of the
# transport `up` was given.
if [ "$TRANSPORT" = "zenoh" ] || [ "$CMD" = "down" ]; then
    COMPOSE_FILES+=("--profile" "zenoh")
fi

# If isolated, prepare override file
OVERRIDE_FILE="tmp/docker-compose-override.yaml"
if [ "$ISOLATED" = "true" ] || [ -f "$OVERRIDE_FILE" ]; then
    # Generate the override when isolation is requested for any command that
    # renders/starts the stack (so config/dry-run reflect the bridge network the
    # same way up does), or keep an existing one for ps/logs.
    if [ "$ISOLATED" = "true" ] && { [ "$CMD" = "up" ] || [ "$CMD" = "config" ] || [ "$CMD" = "dry-run" ]; }; then
        echo -e "${YELLOW}[TELEOP]${NC} Generating isolated bridge override file..."
        cat << 'EOF' > "$OVERRIDE_FILE"
version: '3.8'
services:
  teleop:
    network_mode: bridge
    extra_hosts:
      - "host.docker.internal:host-gateway"
EOF
    fi
    
    if [ -f "$OVERRIDE_FILE" ]; then
        COMPOSE_FILES+=("-f" "$OVERRIDE_FILE")
    fi
fi

# Define services to manage
SERVICES="visualizer teleop"
if [ "$TRANSPORT" = "zenoh" ]; then
    SERVICES="$SERVICES zenoh_bridge"
fi
if [ "$NO_AUTOWARE" = "false" ]; then
    SERVICES="autoware $SERVICES"
fi

# Handle execution
case "$CMD" in
    "up")
        echo -e "${YELLOW}[TELEOP]${NC} Starting services in ${GREEN}${TRANSPORT}${NC} mode (Isolated: ${GREEN}${ISOLATED}${NC}, Separate Autoware: ${GREEN}${NO_AUTOWARE}${NC})..."
        
        # Save state for run_teleop.sh
        echo -e "TRANSPORT=\"$TRANSPORT\"\nISOLATED=\"$ISOLATED\"" > tmp/test_state.env
        
        # Start containers (zenoh_bridge is part of the stack via the zenoh profile)
        docker compose "${COMPOSE_FILES[@]}" up -d "${ARGS[@]}" $SERVICES
        ;;
        
    "down")
        echo -e "${RED}[TELEOP]${NC} Stopping and removing services..."
        
        # Stop containers (the zenoh profile is active for down, so the
        # zenoh_bridge service is removed too).
        docker compose "${COMPOSE_FILES[@]}" down "${ARGS[@]}"

        # Clean up temporary configurations
        rm -f "$OVERRIDE_FILE"
        rm -f tmp/test_state.env
        echo -e "${GREEN}[TELEOP]${NC} Stack cleaned up successfully."
        ;;
        
    "dry-run")
        # Render and validate the merged compose config without starting anything.
        echo -e "${YELLOW}[TELEOP]${NC} Dry run (Transport: ${GREEN}${TRANSPORT}${NC}, Isolated: ${GREEN}${ISOLATED}${NC}, Separate Autoware: ${GREEN}${NO_AUTOWARE}${NC})"
        docker compose "${COMPOSE_FILES[@]}" config "${ARGS[@]}" $SERVICES
        ;;

    *)
        # Passthrough config, ps, logs, etc.
        docker compose "${COMPOSE_FILES[@]}" "$CMD" "${ARGS[@]}"
        ;;
esac

