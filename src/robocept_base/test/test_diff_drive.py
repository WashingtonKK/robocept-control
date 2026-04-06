"""Unit tests for differential-drive kinematics."""

import math
import pytest


def inverse_kinematics(linear, angular, wheel_radius, wheel_separation):
    """Twist → (left_vel, right_vel) in rad/s."""
    left = (linear - angular * wheel_separation / 2.0) / wheel_radius
    right = (linear + angular * wheel_separation / 2.0) / wheel_radius
    return left, right


def forward_kinematics(d_left, d_right, wheel_separation):
    """Wheel displacements → (d_center, d_theta)."""
    d_center = (d_left + d_right) / 2.0
    d_theta = (d_right - d_left) / wheel_separation
    return d_center, d_theta


class TestInverseKinematics:
    """Test Twist → wheel velocity conversion."""

    def test_straight_forward(self):
        left, right = inverse_kinematics(1.0, 0.0, 0.05, 0.3)
        assert left == pytest.approx(right)
        assert left == pytest.approx(1.0 / 0.05)

    def test_pure_rotation(self):
        left, right = inverse_kinematics(0.0, 1.0, 0.05, 0.3)
        assert left == pytest.approx(-right)

    def test_stop(self):
        left, right = inverse_kinematics(0.0, 0.0, 0.05, 0.3)
        assert left == pytest.approx(0.0)
        assert right == pytest.approx(0.0)


class TestForwardKinematics:
    """Test wheel displacements → robot motion."""

    def test_straight(self):
        dc, dt = forward_kinematics(0.1, 0.1, 0.3)
        assert dc == pytest.approx(0.1)
        assert dt == pytest.approx(0.0)

    def test_pure_rotation(self):
        dc, dt = forward_kinematics(-0.05, 0.05, 0.3)
        assert dc == pytest.approx(0.0)
        assert dt == pytest.approx(0.1 / 0.3)

    def test_roundtrip(self):
        """IK then FK should recover original motion."""
        linear, angular = 0.5, 0.3
        R, L = 0.05, 0.3
        dt = 0.05  # 20 Hz

        left_vel, right_vel = inverse_kinematics(linear, angular, R, L)
        d_left = left_vel * R * dt
        d_right = right_vel * R * dt
        d_center, d_theta = forward_kinematics(d_left, d_right, L)

        recovered_linear = d_center / dt
        recovered_angular = d_theta / dt
        assert recovered_linear == pytest.approx(linear, abs=1e-9)
        assert recovered_angular == pytest.approx(angular, abs=1e-9)
