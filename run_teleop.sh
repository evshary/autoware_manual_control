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
    echo -e '\033[1;33m[Info]\033[0m Checking for updates/building...' && \
    cd /autoware_manual_control_ws && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source install/setup.bash && \
    echo -e '\n\033[1;32mStarting Keyboard Control (Physics Mode)...\033[0m' && \
    ros2 run autoware_manual_control keyboard_control --ros-args --params-file /autoware_manual_control_ws/src/autoware_manual_control/teleop_config.yaml"
