"""Gazebo-facing base adapter that reuses the diff-drive controller core."""

from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from robocept_base.diff_drive_controller import (
    ControllerConfig,
    DiffDriveController,
)


class BaseSimAdapter(Node):
    """Apply the base controller path in front of Gazebo Sim."""

    def __init__(self):
        super().__init__('base_sim_adapter')

        self.declare_parameter('wheel_radius', 0.0)
        self.declare_parameter('wheel_separation', 0.0)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('command_in_topic', '/cmd_vel')
        self.declare_parameter(
            'sim_command_out_topic', '/robocept/base/drive_cmd'
        )
        self.declare_parameter(
            'sim_odom_in_topic', '/robocept/base/sim_odom'
        )
        self.declare_parameter('odom_out_topic', '/odom')
        self.declare_parameter(
            'status_topic', '/robocept/base/status'
        )

        config = ControllerConfig(
            wheel_radius=self.get_parameter('wheel_radius').value,
            wheel_separation=self.get_parameter('wheel_separation').value,
            max_linear_vel=self.get_parameter('max_linear_vel').value,
            max_angular_vel=self.get_parameter('max_angular_vel').value,
        )
        self.controller = DiffDriveController(config)
        self.control_rate = self.get_parameter('control_rate_hz').value
        self.cmd_timeout = self.get_parameter('cmd_vel_timeout_sec').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.do_publish_tf = self.get_parameter('publish_tf').value

        command_in_topic = self.get_parameter('command_in_topic').value
        sim_command_out_topic = self.get_parameter(
            'sim_command_out_topic'
        ).value
        sim_odom_in_topic = self.get_parameter('sim_odom_in_topic').value
        odom_out_topic = self.get_parameter('odom_out_topic').value
        status_topic = self.get_parameter('status_topic').value

        if not self.controller.is_configured():
            self.get_logger().error(
                'wheel_radius and wheel_separation must be > 0. '
                'Set them in config/base.yaml for simulation too.'
            )

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_sim_odom_time = None
        self.sim_connected = False

        self.drive_pub = self.create_publisher(Twist, sim_command_out_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_out_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Twist, command_in_topic, self._cmd_vel_callback, 10,
        )
        self.create_subscription(
            Odometry, sim_odom_in_topic, self._sim_odom_callback, 10,
        )

        period = 1.0 / self.control_rate
        self.create_timer(period, self._control_loop)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            'Base sim adapter started: '
            f'{command_in_topic} -> {sim_command_out_topic}, '
            f'{sim_odom_in_topic} -> {odom_out_topic}'
        )

    def _cmd_vel_callback(self, msg: Twist):
        self.target_linear, self.target_angular = self.controller.clamp_command(
            msg.linear.x,
            msg.angular.z,
        )
        self.last_cmd_time = self.get_clock().now()

    def _control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        linear = self.target_linear
        angular = self.target_angular

        if elapsed > self.cmd_timeout:
            linear = 0.0
            angular = 0.0

        wheel_velocities = self.controller.body_to_wheel_velocities(
            linear, angular
        )
        sim_linear, sim_angular = self.controller.wheel_to_body_command(
            wheel_velocities
        )

        drive_cmd = Twist()
        drive_cmd.linear.x = sim_linear
        drive_cmd.angular.z = sim_angular
        self.drive_pub.publish(drive_cmd)

    def _sim_odom_callback(self, msg: Odometry):
        self.last_sim_odom_time = self.get_clock().now()
        self.sim_connected = True

        odom = deepcopy(msg)
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        self.odom_pub.publish(odom)

        if self.do_publish_tf:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = odom.child_frame_id
            t.transform.translation.x = odom.pose.pose.position.x
            t.transform.translation.y = odom.pose.pose.position.y
            t.transform.translation.z = odom.pose.pose.position.z
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def _publish_status(self):
        if self.last_sim_odom_time is not None:
            age = (
                self.get_clock().now() - self.last_sim_odom_time
            ).nanoseconds / 1e9
            self.sim_connected = age <= max(1.0, self.cmd_timeout * 2.0)

        msg = String()
        msg.data = 'sim_connected' if self.sim_connected else 'sim_waiting'
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.drive_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseSimAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
