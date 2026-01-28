# Autoware Manual Controller

A robust, modular keyboard teleoperation node designed for Autoware.universe.
This project provides high-precision vehicle control, supporting physics-based inertial driving experiences and stable cruise control functionalities.

## 🛠️ Build & Run

A highlight of this project is the standardization of the **Containerized Verification Workflow**. Through well-encapsulated scripts, developers can verify logic and test without installing a complex Autoware / ROS 2 environment on the Host.

### Build (Normal ROS2)

#### Prerequisites
*   OS: Ubuntu 22.04 / 24.04
*   ROS 2: Humble

```bash
mkdir -p autoware_manual_control_ws/src
cd autoware_manual_control_ws/src
git clone https://github.com/evshary/autoware_manual_control.git
cd ..
colcon build
```

### Build and Testing (Docker) - Recommended

We provide a simplified Docker setup that communicates with Autoware via the Host Network using DDS.

#### Prerequisites
*   Docker & Docker Compose

#### 1. Start Containers

```bash
git clone https://github.com/evshary/autoware_manual_control.git
cd autoware_manual_control

# Start containers
./run_containers.sh up --build -d
```

#### 2. Enter Control Node
We provide a convenient script `run_teleop.sh` that automatically performs the following:
1. Connects to the running container.
2. Sources environment variables.
3. Automatically builds the latest code.
4. Starts the `manual_control` node.

```bash
./run_teleop.sh
```

#### 3. Stop Services
```bash
./run_containers.sh down
```
