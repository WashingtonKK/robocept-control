"""Unit tests for differential-drive control math."""

import pytest

from robocept_base.diff_drive_controller import (
    ControllerConfig,
    DiffDriveController,
    Pose2D,
    WheelVelocities,
)


@pytest.fixture
def controller():
    return DiffDriveController(
        ControllerConfig(
            wheel_radius=0.05,
            wheel_separation=0.3,
            max_linear_vel=0.5,
            max_angular_vel=1.5,
            encoder_ticks_per_rev=1024,
        )
    )


class TestInverseKinematics:
    """Test Twist → wheel velocity conversion."""

    def test_straight_forward(self, controller):
        wheels = controller.body_to_wheel_velocities(0.5, 0.0)
        assert wheels.left == pytest.approx(wheels.right)
        assert wheels.left == pytest.approx(0.5 / 0.05)

    def test_pure_rotation(self, controller):
        wheels = controller.body_to_wheel_velocities(0.0, 1.0)
        assert wheels.left == pytest.approx(-wheels.right)

    def test_stop(self, controller):
        wheels = controller.body_to_wheel_velocities(0.0, 0.0)
        assert wheels.left == pytest.approx(0.0)
        assert wheels.right == pytest.approx(0.0)

    def test_command_clamp(self, controller):
        linear, angular = controller.clamp_command(5.0, -10.0)
        assert linear == pytest.approx(0.5)
        assert angular == pytest.approx(-1.5)


class TestForwardKinematics:
    """Test wheel displacements → robot motion."""

    def test_straight(self, controller):
        pose, motion = controller.integrate_motion(
            Pose2D(), 0.1, 0.1, 0.05
        )
        assert pose.x == pytest.approx(0.1)
        assert pose.y == pytest.approx(0.0)
        assert motion.d_center == pytest.approx(0.1)
        assert motion.d_theta == pytest.approx(0.0)

    def test_pure_rotation(self, controller):
        pose, motion = controller.integrate_motion(
            Pose2D(), -0.05, 0.05, 0.05
        )
        assert pose.x == pytest.approx(0.0)
        assert motion.d_center == pytest.approx(0.0)
        assert motion.d_theta == pytest.approx(0.1 / 0.3)

    def test_roundtrip(self, controller):
        """IK then FK should recover original motion."""
        linear, angular = 0.4, 0.3
        dt = 0.05  # 20 Hz

        wheel_velocities = controller.body_to_wheel_velocities(
            linear, angular
        )
        d_left, d_right = controller.wheel_travel_from_wheel_velocities(
            wheel_velocities,
            dt,
        )
        _pose, motion = controller.integrate_motion(
            Pose2D(),
            d_left,
            d_right,
            dt,
        )

        recovered_linear = motion.v_linear
        recovered_angular = motion.v_angular
        assert recovered_linear == pytest.approx(linear, abs=1e-9)
        assert recovered_angular == pytest.approx(angular, abs=1e-9)

    def test_wheel_to_body_roundtrip(self, controller):
        wheel_velocities = WheelVelocities(left=5.5, right=6.5)
        linear, angular = controller.wheel_to_body_command(wheel_velocities)
        recovered_wheels = controller.body_to_wheel_velocities(
            linear,
            angular,
        )
        assert recovered_wheels.left == pytest.approx(wheel_velocities.left)
        assert recovered_wheels.right == pytest.approx(wheel_velocities.right)

    def test_encoder_ticks_to_travel(self, controller):
        d_left, d_right = controller.wheel_travel_from_encoder_ticks(
            100,
            140,
            80,
            100,
        )
        assert d_left > 0.0
        assert d_right > d_left
