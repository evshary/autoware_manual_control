#!/bin/bash

# Pick the compose project that has teleop running. Dev wins when both are
# up so a developer iterating source always lands in the mounted container
# (the colcon-refresh branch below depends on package.xml being present).
COMPOSE_OPTS=""
if docker compose ps --services --filter "status=running" 2>/dev/null | grep -q "^teleop$"; then
    :
elif docker compose -f docker-compose-release.yaml ps --services --filter "status=running" 2>/dev/null | grep -q "^teleop$"; then
    COMPOSE_OPTS="-f docker-compose-release.yaml"
else
    echo -e "\033[0;31m[Error]\033[0m Teleop service is not running."
    exit 1
fi

# Read test environment state
TRANSPORT="ros"
ISOLATED="false"
if [ -f tmp/test_state.env ]; then
    source tmp/test_state.env
fi

# Determine colcon cmake arguments
CMAKE_ARGS="-DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release"
if [ "$TRANSPORT" = "zenoh" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DTELEOP_AUTOWARE_TRANSPORT=native_zenoh -DTELEOP_WITH_ZENOH=ON"
else
    CMAKE_ARGS="$CMAKE_ARGS -DTELEOP_AUTOWARE_TRANSPORT=rclcpp"
fi

echo -e "\033[1;33m[Teleop]\033[0m Terminal Mode (Transport: ${TRANSPORT}, Isolated: ${ISOLATED})"
echo "Connecting to container..."

# Execute interactive bash with environment sourced
docker compose $COMPOSE_OPTS exec -it teleop bash -c "
    source /opt/autoware/setup.bash && \
    cd /autoware_manual_control_ws && \

    PARAMS_ARG='' && \
    if [ -f src/autoware_manual_control/package.xml ]; then \
        echo -e '\033[1;33m[Info]\033[0m Dev Mode: Checking for updates and compiling for $TRANSPORT...' && \
        colcon build --cmake-args $CMAKE_ARGS && \
        source install/setup.bash && \
        TELEOP_CFG='src/autoware_manual_control/config/teleop.example.yaml' && \
        if [ -f src/autoware_manual_control/config/teleop.yaml ]; then \
            TELEOP_CFG='src/autoware_manual_control/config/teleop.yaml'; \
        else \
            echo -e '\033[1;33m[Info]\033[0m using config/teleop.example.yaml (no user config).'; \
        fi && \
        if [ \"$TRANSPORT\" = \"zenoh\" ]; then \
            ZENOH_CLIENT_SRC='src/autoware_manual_control/config/zenoh-client.example.json5' && \
            if [ -f src/autoware_manual_control/config/zenoh-client.json5 ]; then \
                ZENOH_CLIENT_SRC='src/autoware_manual_control/config/zenoh-client.json5'; \
            fi && \
            cp \$ZENOH_CLIENT_SRC /tmp/zenoh_client.json5 && \
            if [ \"$ISOLATED\" != \"true\" ]; then \
                sed -i '/scouting:/d' /tmp/zenoh_client.json5; \
            fi && \
            cp \$TELEOP_CFG /tmp/teleop_config_zenoh.yaml && \
            sed -i 's|zenoh_config: \"\"|zenoh_config: \"/tmp/zenoh_client.json5\"|g' /tmp/teleop_config_zenoh.yaml && \
            PARAMS_ARG='--config /tmp/teleop_config_zenoh.yaml'; \
        else \
            { printf '/ManualControl:\n  ros__parameters:\n'; sed 's/^/    /' \$TELEOP_CFG; } > /tmp/teleop_config_ros.yaml && \
            PARAMS_ARG='--ros-args --params-file /tmp/teleop_config_ros.yaml'; \
        fi; \
    else \
        TELEOP_CFG='config/teleop.example.yaml' && \
        if [ -f config/teleop.yaml ]; then \
            echo -e '\033[1;33m[Info]\033[0m Release Mode: Custom config found.' && \
            TELEOP_CFG='config/teleop.yaml'; \
        else \
            echo -e '\033[1;33m[Info]\033[0m Release Mode: using config/teleop.example.yaml (no user config).'; \
        fi && \
        { printf '/ManualControl:\n  ros__parameters:\n'; sed 's/^/    /' \$TELEOP_CFG; } > /tmp/teleop_config_ros.yaml && \
        PARAMS_ARG='--ros-args --params-file /tmp/teleop_config_ros.yaml'; \
    fi && \

    if [ \"$ISOLATED\" = \"true\" ]; then \
        export ROS_LOCALHOST_ONLY=1; \
    fi && \

    echo -e '\n\033[1;32mStarting Keyboard Control...\033[0m' && \
    if [ \"$TRANSPORT\" = \"zenoh\" ]; then \
        exec install/autoware_manual_control/lib/autoware_manual_control/keyboard_control \$PARAMS_ARG; \
    else \
        exec ros2 run autoware_manual_control keyboard_control \$PARAMS_ARG; \
    fi"
