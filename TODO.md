# Robocept Control — TODO

## Hardware (blocked: robot chassis not yet purchased)
- [ ] Choose motor controller board (e.g., Yahboom driver, L298N, ODrive, custom STM32)
- [ ] Determine communication protocol (serial UART, I2C, CAN, GPIO/PWM)
- [ ] Determine if wheel encoders are available and their resolution (ticks/rev)
- [ ] Measure wheel radius and wheel separation — set in `config/base.yaml`

## Implementation
- [ ] Implement `HardwareInterface.connect()` in `base_driver.py` for your motor controller
- [ ] Implement `HardwareInterface.send_velocities()` — translate rad/s to motor commands
- [ ] Implement `HardwareInterface.read_encoders()` — read encoder ticks for odometry
- [ ] Implement `HardwareInterface.disconnect()` — clean shutdown
- [ ] Set `device_port` in `config/base.yaml` (e.g., `/dev/ttyUSB0`, `/dev/ttyAMA0`)
- [ ] Create udev rule for stable motor controller device name (like RPLIDAR's `/dev/rplidar`)

## Testing
- [ ] Test in Gazebo simulation first (diff-drive plugin handles kinematics)
- [ ] Test with `teleop_twist_keyboard` on real hardware
- [ ] Verify `/odom` accuracy: drive 1m forward, check odometry reports ~1m
- [ ] Verify `cmd_vel_timeout_sec` safety: stop sending cmd_vel, motors should stop in 0.5s
- [ ] Test emergency stop behavior

## Tuning
- [ ] Tune `max_linear_vel` and `max_angular_vel` for your robot's capabilities
- [ ] Add PID control if open-loop velocity control is too imprecise
- [ ] Calibrate odometry drift over long runs
