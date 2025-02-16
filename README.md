# ROS2 Packages for Raspberry Pi 5

This repository contains ROS2 packages designed for the Raspberry Pi 5.

## Installation

### Install **ROS2**

```bash
sudo apt install ros-jazzy-desktop-full
```


### Install **Rosbridge server**

```bash
sudo apt install ros-jazzy-rosbridge-server
```



### Install **colcon**

```bash
sudo apt install colcon
```

### Install ROS Packages

Install dependencies using `rosdep`:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Build Packages

This project includes the following packages:

*   **ultrasonic:** Publishes distance (cm) on the `/ultrasonic_range` topic.
*   **obstacle_avoidance:** Publishes linear and angular velocity commands on the `/cmd_vel` topic.
*   **motor_driver:** Subscribes to the `/cmd_vel` topic and sends commands to the wheel motor driver.

To build these packages, execute the following commands:

```bash
colcon build --packages-select ultrasonic obstacle_avoidance motor_driver
. ./install/setup.bash
```

## Run Packages

After building, you can run the packages individually in separate terminals:

```bash
ros2 run ultrasonic ultrasonic_publisher
ros2 run obstacle_avoidance laser_obstacle_avoidance
ros2 run motor_driver cmd_vel_subscriber
```
