# Robocept Control — Differential Drive Base Controller

ROS 2 control layer for a differential-drive (all-wheel) robot. Bridges high-level velocity commands (`/cmd_vel`) to motor hardware and publishes odometry feedback (`/odom`).

The diff-drive math now lives in a pure controller module so the same
control path can run against either real hardware or Gazebo Sim.

## Status

**Hardware transport: PENDING** — the hardware-specific I/O in
`base_driver` is still a placeholder. The controller math and Gazebo
adapter are in place; once you know the real motor controller, implement
the serial/I2C/CAN communication in `robocept_base/base_driver.py`.

## Architecture

```
/cmd_vel (geometry_msgs/Twist)
    │
    ▼
                ┌────────────────────────┐
                │ diff_drive_controller  │
                │ pure math + odometry   │
                └───────────┬────────────┘
                            │
               ┌────────────┴────────────┐
               ▼                         ▼
         base_driver               base_sim_adapter
       real robot path               Gazebo path
               │                         │
               │ serial / I2C / CAN      │ /robocept/base/drive_cmd
               ▼                         ▼
       Motor Controller Board      Gazebo diff-drive plugin
               │
               ▼
         Motors + Encoders
               │
               ▼
      /odom (nav_msgs/Odometry) + TF
```

## Packages

| Package | Type | Purpose |
|---|---|---|
| `robocept_base` | ament_python | Hardware driver, Gazebo adapter, and shared diff-drive controller |

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

# Run the same controller path against Gazebo Sim
ros2 launch robocept_base base_sim.launch.py

# Test with keyboard teleop (install teleop_twist_keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Configuration

Edit `src/robocept_base/config/base.yaml` with your robot's physical parameters and motor controller settings.

For simulation, `src/robocept_base/config/base_sim.yaml` matches the
Gazebo robot dimensions and publishes the adapter output to
`/robocept/base/drive_cmd`.

## Adding Your Motor Driver

1. Open `src/robocept_base/robocept_base/base_driver.py`
2. Implement the `HardwareInterface` class methods:
   - `connect()` — open serial/I2C/CAN connection
   - `send_velocities(left_vel, right_vel)` — command motor speeds
   - `read_encoders()` — return (left_ticks, right_ticks)
   - `disconnect()` — close connection
3. Update `config/base.yaml` with your robot's dimensions and device port
