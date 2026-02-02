#!/bin/bash

# Check if teleop service is running
if ! docker compose ps --services --filter "status=running" | grep -q "teleop"; then
    echo -e "\033[0;31m[Error]\033[0m Teleop service is not running."
    exit 1
fi

echo -e "\033[1;33m[Teleop]\033[0m Terminal Mode"
echo "Connecting to container..."

# Execute interactive bash with environment sourced
docker compose exec -it teleop bash -c "
    source /opt/autoware/setup.bash && \
    cd /autoware_manual_control_ws && \
    
    # logic to handle dev vs release
    PARAMS_ARG='' && \
    if [ -f src/autoware_manual_control/package.xml ]; then \
        echo -e '\033[1;33m[Info]\033[0m Dev Mode: Checking for updates...' && \
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
        PARAMS_ARG='--ros-args --params-file src/autoware_manual_control/teleop_config.yaml'; \
    elif [ -f teleop_config.yaml ]; then \
        echo -e '\033[1;33m[Info]\033[0m Release Mode: Custom config found.' && \
        PARAMS_ARG='--ros-args --params-file teleop_config.yaml'; \
    else \
        echo -e '\033[1;33m[Info]\033[0m Release Mode: Using default parameters.' ; \
    fi && \

    source install/setup.bash && \
    echo -e '\n\033[1;32mStarting Keyboard Control...\033[0m' && \
    ros2 run autoware_manual_control keyboard_control \$PARAMS_ARG"
