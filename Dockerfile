FROM ghcr.io/autowarefoundation/autoware:universe AS autoware_source

FROM ros:humble

# Copy Autoware installation (contains message definitions)
COPY --from=autoware_source /opt/autoware /opt/autoware

# Install build tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace
WORKDIR /autoware_manual_control_ws/src
COPY . /autoware_manual_control_ws/src/autoware_manual_control

# Install dependencies
WORKDIR /autoware_manual_control_ws

# Build
# Source Autoware setup to find dependencies (messages)
# We expect /opt/autoware/setup.bash to correctly set CMAKE_PREFIX_PATH for the copied libs
RUN bash -c "source /opt/autoware/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Install Python dependencies for Latency Test Suite
RUN apt-get update && apt-get install -y python3-pip && rm -rf /var/lib/apt/lists/*
RUN pip3 install eclipse-zenoh rich textual pandas fastapi uvicorn websockets

# Setup bashrc
RUN echo "source /autoware_manual_control_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /autoware_manual_control_ws/install/setup.bash" >> /etc/bash.bashrc

CMD ["sleep", "infinity"]
