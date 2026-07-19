# ROS 2 Wrapper of Unitree SDK for Go1 Quadruped Robot

A ROS 2 wrapper for the **Unitree Go1 Quadruped Robot** interfacing with the high-level Sport Mode API of the `unitree_legged_sdk`. 

This package is written in C++17, replacing old Boost dependencies with standard library primitives. It combines UDP communication, sensor broadcasting, parameter tuning, and safety controls into a single ROS 2 Node.

---

## 🛠️ Key Features

* **Single-Node Architecture (`LeggedControllerNode`)**: Unifies command listeners, telemetry publishers, and high-frequency hardware UDP synchronization loops to reduce CPU overhead and DDS participant count.
* **Startup Telemetry Guard**: Prevents publishing default zero-filled state arrays upon launching. Publishers remain inactive until the first valid, CRC-verified UDP packet from the robot is received.
* **Command Timeout Watchdog**: Stops the robot's motion if the control link (e.g., `cmd_vel`) is delayed for more than a specified timeout threshold (default: `0.5` seconds).
* **Body Height Safeguards**: Restricts absolute body height commands to a physically verified envelope of `[0.15m, 0.40m]` to prevent vertical collapses due to standard teleop tools or navigation stacks defaulting `linear.z` to `0.0`.
* **Graceful Stand-Down Destructor**: Safely halts robot velocities and commands standing mode (`mode = 0`) upon node exit (`Ctrl+C` or a crash) before destroying the communication socket.
* **QoS Configuration**: Publishes high-frequency telemetry (IMU, Odometry, Motor States) using standard **SensorDataQoS (Best Effort)** to prevent network queueing over wireless connections.
* **IMU & Odometry Covariances**: Populates diagonal covariance matrices on IMU and Odometry topics, enabling integration with EKF estimators (such as `robot_localization`).
* **Dynamic Parameter Tuning**: Allows adjustment of variables—such as the watchdog timeout, TF broadcasting, and foot contact force thresholds—at runtime.
* **Standard Joint State Publishing**: Consolidates the 12 joint states into a single standard `sensor_msgs/msg/JointState` topic, matching official Unitree Go1 URDF configurations.
* **Foot Contact Trackers**: Compares raw foot airbag pressure values against force thresholds to publish discrete boolean contact indicators.
* **Embedded Diagnostics**: Monitors brushless motor temperatures and battery State of Charge (SOC), logging warning messages when values exceed standard operating limits.

---

## 📊 Coordinate & Joint Conventions

### Joint Mapping (Standard URDF Representation)
Joints are published on `/go1/joint_states` and mapped in the following standard order matching official Go1 URDF configurations:

| Index | Joint Name | Leg Location | Description |
| :--- | :--- | :--- | :--- |
| **0** | `FR_hip_joint` | Front Right | Hip Abduction/Adduction |
| **1** | `FR_thigh_joint`| Front Right | Thigh Hip Flexion/Extension |
| **2** | `FR_calf_joint` | Front Right | Calf Knee Flexion/Extension |
| **3** | `FL_hip_joint` | Front Left | Hip Abduction/Adduction |
| **4** | `FL_thigh_joint`| Front Left | Thigh Hip Flexion/Extension |
| **5** | `FL_calf_joint` | Front Left | Calf Knee Flexion/Extension |
| **6** | `RR_hip_joint` | Rear Right | Hip Abduction/Adduction |
| **7** | `RR_thigh_joint`| Rear Right | Thigh Hip Flexion/Extension |
| **8** | `RR_calf_joint` | Rear Right | Calf Knee Flexion/Extension |
| **9** | `RL_hip_joint` | Rear Left | Hip Abduction/Adduction |
| **10**| `RL_thigh_joint`| Rear Left | Thigh Hip Flexion/Extension |
| **11**| `RL_calf_joint` | Rear Left | Calf Knee Flexion/Extension |

---

## 🔌 ROS 2 Interfaces

All topics are published under the standard namespace `/go1/` when launched using the provided launch file.

### Subscribed Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `cmd_vel` | `geometry_msgs/msg/Twist` | Planar velocity commands (`linear.x` for forward, `linear.y` for sideways, `angular.z` for yaw speed). `linear.z` acts as a secondary body height channel. |
| `cmd_body_height` | `std_msgs/msg/Float32` | Dedicated channel to command absolute standing body height (physically bounded between `0.15m` and `0.40m`). |
| `cmd_mode` | `unitree_ros2_cpp/msg/HighCmd` | Standard SDK state machine modes (e.g., `0` for Idle Stand, `1` for Force Stand, `2` for Walk, `7` for Damping, etc.). |
| `cmd_pos` | `unitree_ros2_cpp/msg/HighCmd` | Target coordinates in inertial standing position mode. |
| `cmd_foot_raise_height` | `unitree_ros2_cpp/msg/HighCmd` | Adjusts walking foot step-clearance height. |
| `cmd_euler` | `unitree_ros2_cpp/msg/HighCmd` | Adjusts body attitude (Roll, Pitch, Yaw) when standing. |

### Published Topics

| Topic | Message Type | QoS | Frequency | Description |
| :--- | :--- | :--- | :--- | :--- |
| `odom` | `nav_msgs/msg/Odometry` | Best Effort | 500 Hz | Robot odometry (pose & twist) containing pose and twist covariances. |
| `joint_states` | `sensor_msgs/msg/JointState` | Best Effort | 500 Hz | Consolidated joint positions, velocities, and torques. Compatible with `robot_state_publisher`. |
| `legged_data/sensors/imu` | `sensor_msgs/msg/Imu` | Best Effort | 500 Hz | Linear acceleration, angular velocity, and orientation quaternion containing covariance matrices. |
| `legged_data/sensors/foot_contact/fr` | `std_msgs/msg/Bool` | Reliable | 10 Hz | Ground contact state of the Front Right foot. |
| `legged_data/sensors/foot_contact/fl` | `std_msgs/msg/Bool` | Reliable | 10 Hz | Ground contact state of the Front Left foot. |
| `legged_data/sensors/foot_contact/rr` | `std_msgs/msg/Bool` | Reliable | 10 Hz | Ground contact state of the Rear Right foot. |
| `legged_data/sensors/foot_contact/rl` | `std_msgs/msg/Bool` | Reliable | 10 Hz | Ground contact state of the Rear Left foot. |
| `legged_data/sensors/bms` | `unitree_ros2_cpp/msg/BmsState` | Reliable | 1 Hz | Battery diagnostics (voltages, SoC, current, cycles). |
| `legged_data/sensors/foot_force` | `unitree_ros2_cpp/msg/HighState` | Reliable | 10 Hz | Raw foot force airbag sensor outputs. |
| `legged_data/sensors/system_temperature` | `sensor_msgs/msg/Temperature` | Reliable | 1 Hz | Temperature of the IMU sensor. |
| `legged_data/status/mode` | `unitree_ros2_cpp/msg/HighState` | Reliable | 10 Hz | Active state machine mode of the robot. |
| `legged_data/status/gait_type` | `unitree_ros2_cpp/msg/HighState` | Reliable | 10 Hz | Active gait type (e.g., Trot, Stair, Obstacle). |
| `legged_data/status/foot_raise_height`| `unitree_ros2_cpp/msg/HighState` | Reliable | 10 Hz | Foot clearance raise height. |
| `legged_data/status/about_go1` | `unitree_ros2_cpp/msg/HighState` | Reliable | 1 Hz | SDK firmware version and network bandwidth. |
| `legged_data/actuators/motor_0`..`11` | `unitree_ros2_cpp/msg/MotorState` | Best Effort | 500 Hz | Individual raw feedback for each joint (mode, position, speed, torque, temp). |

---

## ⚙️ Parameters & Configuration

### Startup Parameters (Launch Arguments)
These parameters can be configured inside `launch/go1.launch.py` or overridden when executing the launch command:

| Name | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `robot_ip` | `string` | `192.168.123.161` | Onboard Sport Mode computer IP. |
| `local_port` | `int` | `8090` | Local port of the control PC. |
| `remote_port`| `int` | `8082` | Target port on the Go1 robot. |
| `cmd_watchdog_timeout`| `double`| `0.5` | Stop motion if no `cmd_vel` is received for this many seconds. |
| `publish_tf` | `bool` | `true` | Toggle direct `odom` $\rightarrow$ `base_link` TF broadcasting. Disable when running an external EKF. |
| `odom_frame` | `string` | `odom` | Name of the parent odometry coordinate frame. |
| `base_frame` | `string` | `base_link` | Name of the child robot base coordinate frame. |
| `foot_contact_threshold`| `int` | `40` | Force sensor value threshold to register a boolean ground contact. |

### Dynamic Runtime Reconfiguration
The following parameters can be dynamically tuned during runtime using standard ROS 2 commands:

* **Watchdog Timeout**: Bounded between `0.05` and `10.0` seconds.
  ```bash
  ros2 param set /go1/legged_controller cmd_watchdog_timeout 0.3
  ```
* **Foot Contact Sensitivity**: Bounded between `5` and `200` force units.
  ```bash
  ros2 param set /go1/legged_controller foot_contact_threshold 50
  ```
* **TF Broadcast Toggle**: Activate or deactivate driver TF broadcasting dynamically.
  ```bash
  ros2 param set /go1/legged_controller publish_tf false
  ```

---

## 🛠️ Dependencies & Installation

### Prerequisite System Libraries
This package relies on standard C++17, udev, and ROS 2 desktop environments.

```bash
sudo apt update
sudo apt install libudev-dev
```

### Building the Package
Clone the package into your ROS 2 workspace (e.g., `colcon_ws`), ensure your ROS 2 environment is sourced, and compile:

```bash
cd ~/colcon_ws
colcon build --packages-select unitree_ros2_cpp
source install/setup.bash
```

---

## 🎮 How to Use

### 1. Launching the Driver (Direct TF Mode)
Connect your control PC to the robot (ethernet or WiFi) and launch:

```bash
ros2 launch unitree_ros2_cpp go1.launch.py
```
This launches the controller with direct TF broadcasting enabled (`odom` $\rightarrow$ `base_link`), allowing visualization in RViz2.

### 2. Launching the Driver (External Fusion / EKF Mode)
If you are running an external State Estimator (such as EKF from `robot_localization` or SLAM) that publishes its own `odom` transform, disable direct broadcasting:

```bash
ros2 launch unitree_ros2_cpp go1.launch.py publish_tf:=false
```

### 3. Command the Robot to Walk
Publish velocity commands on the `/go1/cmd_vel` topic:

```bash
# Move forward at 0.3 m/s
ros2 topic pub --once /go1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 4. Command Body Height Explicitly
Command the standing body height using the dedicated topic:

```bash
# Command the body to crouch to 0.20m
ros2 topic pub --once /go1/cmd_body_height std_msgs/msg/Float32 "{data: 0.20}"
```
