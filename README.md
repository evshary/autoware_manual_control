# Autoware Manual Controller

Keyboard controller for Autoware.

# Build

## Native Host

* Source ROS and Autoware.universe workspace first.

* Build the code

```shell
mkdir -p autoware_manual_control_ws/src
cd autoware_manual_control_ws/src
git clone https://github.com/evshary/autoware_manual_control.git
cd ..
colcon build
```

## docker with latest Autoware

* Get the code

```shell
mkdir -p $HOME/autoware_manual_control_ws/src
cd $HOME/autoware_manual_control_ws/src
git clone https://github.com/evshary/autoware_manual_control.git
cd ..
```

* Run the latest docker and build

```shell
# Run docker
rocker --network host --privileged --x11 --user --volume $HOME/autoware_manual_control_ws --volume $HOME/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:galactic-latest-prebuilt-amd64 bash
# Build
cd $HOME/autoware_manual_control_ws
colcon build
```

# Run

```shell
source install/local_setup.bash
ros2 run autoware_manual_control keyboard_control
```

# Usage

1. Toggle to external mode
2. Set Gear Type to Drive
3. Adjust speed and steering angle
4. Enjoy driving :-)

```
------------------------------------
| Different Mode:                  |
|   z: Toggle auto & external mode |
|   x: Gear Type => Drive          |
|   c: Gear Type => Reverse        |
|   v: Gear Type => Park           |
|   s: View current mode           |
| Speed:                           |
|   u: Increase speed              |
|   i: Set speed to 0              |
|   o: Decrease speed              |
| Steering Angle                   |
|   j: Left turn                   |
|   k: Set angle to 0              |
|   l: Right turn                  |
------------------------------------
```
