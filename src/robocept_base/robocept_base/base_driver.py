"""
Differential-drive base driver node.

Subscribes to /cmd_vel, converts Twist to left/right wheel velocities
using diff-drive kinematics, sends commands to the hardware interface,
reads encoder feedback, and publishes /odom + TF.

HARDWARE INTERFACE: The HardwareInterface class at the bottom of this
file is a placeholder. Implement its methods for your specific motor
controller (serial, I2C, CAN, GPIO, etc.) once you have the hardware.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class HardwareInterface:
    """
    Placeholder hardware interface.

    Replace this with your actual motor controller communication.
    The base_driver node calls these methods — do not change the signatures.
    """

    def connect(self, port: str, baudrate: int) -> bool:
        """Open connection to motor controller. Return True on success."""
        # TODO: Implement for your hardware.
        # Example for serial:
        #   import serial
        #   self.ser = serial.Serial(port, baudrate, timeout=0.1)
        #   return self.ser.is_open
        return False

    def send_velocities(self, left_vel: float, right_vel: float) -> None:
        """
        Send wheel velocity commands to the motor controller.

        Args:
            left_vel: Left wheel velocity in rad/s.
            right_vel: Right wheel velocity in rad/s.
        """
        # TODO: Implement for your hardware.
        # Example: self.ser.write(f"L{left_vel:.3f},R{right_vel:.3f}\n".encode())
        pass

    def read_encoders(self) -> tuple[int, int]:
        """
        Read encoder tick counts from the motor controller.

        Returns:
            (left_ticks, right_ticks) — cumulative tick counts.
            Return (0, 0) if encoders are not available.
        """
        # TODO: Implement for your hardware.
        return (0, 0)

    def disconnect(self) -> None:
        """Close the hardware connection."""
        # TODO: Implement for your hardware.
        pass


class BaseDriver(Node):
    """Differential-drive base controller node."""

    def __init__(self):
        super().__init__('base_driver')

        # Declare parameters.
        self.declare_parameter('wheel_radius', 0.0)
        self.declare_parameter('wheel_separation', 0.0)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('encoder_ticks_per_rev', 0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('device_port', '')
        self.declare_parameter('device_baudrate', 115200)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        # Read parameters.
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        self.control_rate = self.get_parameter('control_rate_hz').value
        self.cmd_timeout = self.get_parameter('cmd_vel_timeout_sec').value
        self.device_port = self.get_parameter('device_port').value
        self.device_baudrate = self.get_parameter('device_baudrate').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.do_publish_tf = self.get_parameter('publish_tf').value

        # Validate critical parameters.
        if self.wheel_radius <= 0.0 or self.wheel_separation <= 0.0:
            self.get_logger().error(
                'wheel_radius and wheel_separation must be > 0. '
                'Set them in config/base.yaml for your robot.'
            )

        # Hardware interface.
        self.hw = HardwareInterface()
        self.hw_connected = False

        if self.device_port:
            self.hw_connected = self.hw.connect(
                self.device_port, self.device_baudrate
            )
            if self.hw_connected:
                self.get_logger().info(
                    f'Connected to motor controller on {self.device_port}'
                )
            else:
                self.get_logger().warn(
                    f'Failed to connect to {self.device_port}. '
                    'Running in dry-run mode (no motor commands sent).'
                )
        else:
            self.get_logger().warn(
                'No device_port configured. Running in dry-run mode.'
            )

        # Odometry state.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        # Latest cmd_vel.
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Publishers.
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.status_pub = self.create_publisher(
            String, '/robocept/base/status', 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber.
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )

        # Control loop timer.
        period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(period, self._control_loop)

        # Status timer (1 Hz).
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info('Base driver started.')

    def _cmd_vel_callback(self, msg: Twist):
        """Store latest velocity command."""
        self.target_linear = max(
            -self.max_linear, min(self.max_linear, msg.linear.x)
        )
        self.target_angular = max(
            -self.max_angular, min(self.max_angular, msg.angular.z)
        )
        self.last_cmd_time = self.get_clock().now()

    def _control_loop(self):
        """Main control loop: send commands, read encoders, publish odom."""
        now = self.get_clock().now()

        # Safety: stop if cmd_vel is stale.
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.target_linear = 0.0
            self.target_angular = 0.0

        # Diff-drive inverse kinematics: Twist → wheel velocities.
        if self.wheel_radius > 0.0 and self.wheel_separation > 0.0:
            left_vel = (
                self.target_linear - self.target_angular
                * self.wheel_separation / 2.0
            ) / self.wheel_radius
            right_vel = (
                self.target_linear + self.target_angular
                * self.wheel_separation / 2.0
            ) / self.wheel_radius
        else:
            left_vel = 0.0
            right_vel = 0.0

        # Send to hardware.
        if self.hw_connected:
            self.hw.send_velocities(left_vel, right_vel)

        # Read encoders and compute odometry.
        dt = 1.0 / self.control_rate
        if self.hw_connected and self.ticks_per_rev > 0:
            left_ticks, right_ticks = self.hw.read_encoders()
            d_left = (
                (left_ticks - self.prev_left_ticks)
                / self.ticks_per_rev * 2.0 * math.pi * self.wheel_radius
            )
            d_right = (
                (right_ticks - self.prev_right_ticks)
                / self.ticks_per_rev * 2.0 * math.pi * self.wheel_radius
            )
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
        else:
            # Open-loop: estimate from commanded velocities.
            d_left = left_vel * self.wheel_radius * dt
            d_right = right_vel * self.wheel_radius * dt

        # Forward kinematics.
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / max(self.wheel_separation, 0.001)

        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Compute linear/angular velocity for odom message.
        v_linear = d_center / dt if dt > 0 else 0.0
        v_angular = d_theta / dt if dt > 0 else 0.0

        # Publish odometry.
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular
        self.odom_pub.publish(odom)

        # Publish TF: odom → base_link.
        if self.do_publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = math.sin(self.theta / 2.0)
            t.transform.rotation.w = math.cos(self.theta / 2.0)
            self.tf_broadcaster.sendTransform(t)

    def _publish_status(self):
        """Publish driver status at 1 Hz."""
        msg = String()
        if not self.device_port:
            msg.data = 'no_device_configured'
        elif self.hw_connected:
            msg.data = 'connected'
        else:
            msg.data = 'disconnected'
        self.status_pub.publish(msg)

    def destroy_node(self):
        """Clean shutdown: stop motors, disconnect."""
        if self.hw_connected:
            self.hw.send_velocities(0.0, 0.0)
            self.hw.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
