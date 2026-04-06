# Robocept Control — Differential Drive Base Controller

ROS 2 control layer for a differential-drive (all-wheel) robot. Bridges high-level velocity commands (`/cmd_vel`) to motor hardware and publishes odometry feedback (`/odom`).

## Status

**Motor driver: PENDING** — hardware not yet purchased. The `base_driver` node is a placeholder interface. Once you have your motor controller board, implement the serial/I2C/CAN communication in `robocept_base/base_driver.py`.

## Architecture

```
/cmd_vel (geometry_msgs/Twist)
    │
    ▼
┌──────────────┐
│  base_driver  │  ← YOUR HARDWARE CODE HERE
│  (this repo)  │
└──────┬───────┘
       │  serial / I2C / CAN / GPIO
       ▼
   Motor Controller Board
       │
       ▼
   Motors + Encoders
       │
       ▼
┌──────────────┐
│  base_driver  │  reads encoders, computes odometry
└──────┬───────┘
       │
       ▼
/odom (nav_msgs/Odometry)  +  TF: odom → base_link
```

## Packages

| Package | Type | Purpose |
|---|---|---|
| `robocept_base` | ament_python | Motor driver node + diff-drive kinematics |

## Interface Contract

### Subscriptions

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command (linear.x, angular.z) |

### Publications

| Topic | Type | Hz | Purpose |
|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | 20 | Wheel odometry |
| `/robocept/base/status` | `std_msgs/msg/String` | 1 | Driver status (connected/disconnected/error) |

### TF Broadcasts

| Transform | Type | Purpose |
|---|---|---|
| `odom` → `base_link` | Dynamic (20 Hz) | Robot pose from wheel odometry |

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `wheel_radius` | 0.0 | Wheel radius in meters (MUST configure) |
| `wheel_separation` | 0.0 | Distance between left and right wheels in meters (MUST configure) |
| `max_linear_vel` | 1.0 | Maximum linear velocity (m/s) |
| `max_angular_vel` | 2.0 | Maximum angular velocity (rad/s) |
| `encoder_ticks_per_rev` | 0 | Encoder resolution (0 = no encoders) |
| `control_rate_hz` | 20.0 | Control loop frequency |
| `cmd_vel_timeout_sec` | 0.5 | Stop motors if no cmd_vel received within this time |
| `device_port` | "" | Serial/I2C device path (hardware-specific) |

## Build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/robocept-control
colcon build --symlink-install
source install/setup.bash
```

## Usage

```bash
# Start the base driver
ros2 launch robocept_base base.launch.py

# Test with keyboard teleop (install teleop_twist_keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Configuration

Edit `src/robocept_base/config/base.yaml` with your robot's physical parameters and motor controller settings.

## Adding Your Motor Driver

1. Open `src/robocept_base/robocept_base/base_driver.py`
2. Implement the `HardwareInterface` class methods:
   - `connect()` — open serial/I2C/CAN connection
   - `send_velocities(left_vel, right_vel)` — command motor speeds
   - `read_encoders()` — return (left_ticks, right_ticks)
   - `disconnect()` — close connection
3. Update `config/base.yaml` with your robot's dimensions and device port
