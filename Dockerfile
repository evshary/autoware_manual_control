FROM ghcr.io/autowarefoundation/autoware:universe AS autoware_source

FROM ros:humble

# Autoware message definitions (copied from the universe image).
COPY --from=autoware_source /opt/autoware /opt/autoware

# Build toolchain + deps. geographic_msgs / unique-identifier-msgs back
# autoware_adapi_v1_msgs; nlohmann-json + zenoh-cpp-vendor build remote_control.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        python3-pip \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-geographic-msgs \
        ros-humble-unique-identifier-msgs \
        ros-humble-zenoh-cpp-vendor \
        nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# Python deps for the latency test suite / tooling.
RUN pip3 install --no-cache-dir \
        eclipse-zenoh rich textual pandas fastapi uvicorn websockets

# Workspace.
WORKDIR /autoware_manual_control_ws
COPY . /autoware_manual_control_ws/src/autoware_manual_control

# ZENOH_VENDOR_PREFIX lets CMake locate the zenoh-cpp vendor package.
ENV ZENOH_VENDOR_PREFIX=/opt/ros/humble/opt/zenoh_cpp_vendor
RUN bash -c "source /opt/autoware/setup.bash && \
             source /opt/ros/humble/setup.bash && \
             colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "source /autoware_manual_control_ws/install/setup.bash" \
        | tee -a /root/.bashrc /etc/bash.bashrc > /dev/null

CMD ["sleep", "infinity"]
