# Robot Controller Package

## Overview

The `robot_controller` package contains multiple ROS 2 nodes that interact with each other in a robot control system. It includes a **Status Publisher**, **Command Subscriber**, and **Velocity Publisher**. This package demonstrates basic ROS 2 node communication.

## Prerequisites

- ROS 2 (Foxy, Galactic, or newer)
- Python 3.x
- colcon build tool

## Setup Instructions

1. **Create a ROS 2 workspace** (if you haven't already):

    ```bash
    mkdir -p ~/robot_project_ws/src
    cd ~/robot_project_ws/src
    ```

2. **Clone the `robot_controller` package** into your workspace:

    ```bash
    git clone <your-repository-url> robot_controller
    ```

3. **Install dependencies**:

    Make sure your ROS 2 dependencies are installed:

    ```bash
    sudo apt update
    sudo apt install python3-colcon-common-extensions
    ```

4. **Build the workspace**:

    Navigate to your workspace directory and build the package:

    ```bash
    cd ~/robot_project_ws
    colcon build --symlink-install
    ```

5. **Source the workspace**:

    After the build completes, source your workspace to set up your environment:

    ```bash
    source ~/robot_project_ws/install/setup.bash
    ```

## Nodes in the Package

### 1. **Status Publisher Node**

- **Description**: Publishes the robot's status (e.g., "operational", "idle") to the `/robot/status` topic every 2 seconds.
- **Topic**: `/robot/status` (Type: `std_msgs/String`)
- **Command to run**:
    ```bash
    ros2 run robot_controller status_publisher
    ```

### 2. **Command Subscriber Node**

- **Description**: Subscribes to the `/robot/command` topic and processes commands such as `"start"`, `"stop"`, and `"reset"`.
- **Topic**: `/robot/command` (Type: `std_msgs/String`)
- **Command to run**:
    ```bash
    ros2 run robot_controller command_subscriber
    ```

### 3. **Velocity Publisher Node**

- **Description**: Publishes velocity commands to the `/cmd_vel` topic to control the robot's motion. The message type is `geometry_msgs/Twist`.
- **Topic**: `/cmd_vel` (Type: `geometry_msgs/Twist`)
- **Command to run**:
    ```bash
    ros2 run robot_controller velocity_publisher
    ```

## Testing the Nodes

1. In one terminal, run the **Command Subscriber**:

    ```bash
    ros2 run robot_controller command_subscriber
    ```

2. In another terminal, publish a command to the `/robot/command` topic:

    ```bash
    ros2 topic pub /robot/command std_msgs/String "data: 'start'"
    ```

   You should see the **Command Subscriber** print:

    ```
    [INFO] [command_subscriber]: Received command: "start"
    ```

3. Run other nodes as needed (e.g., **Status Publisher** or **Velocity Publisher**).

## License

This project is licensed under the MIT License.
