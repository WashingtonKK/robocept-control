"""Pure diff-drive controller and odometry math."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ControllerConfig:
    """Static controller configuration."""

    wheel_radius: float
    wheel_separation: float
    max_linear_vel: float
    max_angular_vel: float
    encoder_ticks_per_rev: int = 0


@dataclass(frozen=True)
class Pose2D:
    """Planar robot pose."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass(frozen=True)
class WheelVelocities:
    """Left and right wheel angular velocities in rad/s."""

    left: float
    right: float


@dataclass(frozen=True)
class MotionDelta:
    """Incremental motion derived from wheel travel."""

    d_left: float
    d_right: float
    d_center: float
    d_theta: float
    v_linear: float
    v_angular: float


class DiffDriveController:
    """Pure diff-drive math shared by hardware and simulation adapters."""

    def __init__(self, config: ControllerConfig):
        self.config = config

    def is_configured(self) -> bool:
        return (
            self.config.wheel_radius > 0.0
            and self.config.wheel_separation > 0.0
        )

    def clamp_command(
        self,
        linear: float,
        angular: float,
    ) -> tuple[float, float]:
        """Clamp a body command to configured limits."""
        clamped_linear = max(
            -self.config.max_linear_vel,
            min(self.config.max_linear_vel, linear),
        )
        clamped_angular = max(
            -self.config.max_angular_vel,
            min(self.config.max_angular_vel, angular),
        )
        return clamped_linear, clamped_angular

    def body_to_wheel_velocities(
        self,
        linear: float,
        angular: float,
    ) -> WheelVelocities:
        """Convert body twist to wheel angular velocities."""
        if not self.is_configured():
            return WheelVelocities(left=0.0, right=0.0)

        left = (
            linear - angular * self.config.wheel_separation / 2.0
        ) / self.config.wheel_radius
        right = (
            linear + angular * self.config.wheel_separation / 2.0
        ) / self.config.wheel_radius
        return WheelVelocities(left=left, right=right)

    def wheel_to_body_command(
        self,
        wheel_velocities: WheelVelocities,
    ) -> tuple[float, float]:
        """Convert wheel angular velocities back to a body twist."""
        if not self.is_configured():
            return 0.0, 0.0

        linear = (
            self.config.wheel_radius
            * (wheel_velocities.left + wheel_velocities.right)
            / 2.0
        )
        angular = (
            self.config.wheel_radius
            * (wheel_velocities.right - wheel_velocities.left)
            / self.config.wheel_separation
        )
        return self.clamp_command(linear, angular)

    def wheel_travel_from_encoder_ticks(
        self,
        left_ticks: int,
        right_ticks: int,
        prev_left_ticks: int,
        prev_right_ticks: int,
    ) -> tuple[float, float]:
        """Convert encoder tick deltas into wheel travel distances."""
        if not self.is_configured() or self.config.encoder_ticks_per_rev <= 0:
            return 0.0, 0.0

        meters_per_tick = (
            2.0 * math.pi * self.config.wheel_radius
            / self.config.encoder_ticks_per_rev
        )
        d_left = (left_ticks - prev_left_ticks) * meters_per_tick
        d_right = (right_ticks - prev_right_ticks) * meters_per_tick
        return d_left, d_right

    def wheel_travel_from_wheel_velocities(
        self,
        wheel_velocities: WheelVelocities,
        dt: float,
    ) -> tuple[float, float]:
        """Estimate wheel travel from commanded wheel velocities."""
        if not self.is_configured() or dt <= 0.0:
            return 0.0, 0.0

        d_left = wheel_velocities.left * self.config.wheel_radius * dt
        d_right = wheel_velocities.right * self.config.wheel_radius * dt
        return d_left, d_right

    def integrate_motion(
        self,
        pose: Pose2D,
        d_left: float,
        d_right: float,
        dt: float,
    ) -> tuple[Pose2D, MotionDelta]:
        """Advance pose and compute instantaneous body velocity."""
        separation = max(self.config.wheel_separation, 0.001)
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / separation

        new_pose = Pose2D(
            x=pose.x + d_center * math.cos(pose.theta + d_theta / 2.0),
            y=pose.y + d_center * math.sin(pose.theta + d_theta / 2.0),
            theta=pose.theta + d_theta,
        )
        motion = MotionDelta(
            d_left=d_left,
            d_right=d_right,
            d_center=d_center,
            d_theta=d_theta,
            v_linear=d_center / dt if dt > 0.0 else 0.0,
            v_angular=d_theta / dt if dt > 0.0 else 0.0,
        )
        return new_pose, motion
