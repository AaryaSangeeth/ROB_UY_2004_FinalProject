"""
Ackermann odometry.

Converts encoder tick deltas + steering angle into robot pose updates.

The model assumes a bicycle kinematic approximation of Ackermann steering:
the two rear wheels are treated as one virtual wheel at the center of the
rear axle, and the two front wheels as one virtual wheel at the center of
the front axle. This is standard for path planning on car-like robots and
is accurate at indoor speeds.

State:     (x, y, theta)                  meters, meters, radians
Inputs:    encoder ticks, steering angle (degrees offset from center)
"""

import math
from robot_driver import config


class AckermannOdometry:
    """Tracks robot pose by integrating encoder ticks and steering commands."""

    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.x     = x
        self.y     = y
        self.theta = theta
        self._last_ticks: int | None = None

        # Precompute the distance per encoder tick.
        circumference = 2.0 * math.pi * config.WHEEL_RADIUS_M
        self._meters_per_tick = circumference / config.ENCODER_TICKS_PER_REVOLUTION

    def update(self, ticks: int, steering_offset_deg: int) -> tuple[float, float, float]:
        """Integrate one step of motion.

        Args:
            ticks:               cumulative encoder count from the Arduino.
            steering_offset_deg: degrees from center; +right, -left.

        Returns:
            Current pose (x, y, theta).
        """
        # First call: just record the baseline and return current pose.
        if self._last_ticks is None:
            self._last_ticks = ticks
            return self.x, self.y, self.theta

        # Distance traveled since last update.
        delta_ticks    = ticks - self._last_ticks
        self._last_ticks = ticks
        distance_m     = delta_ticks * self._meters_per_tick

        # Convert steering offset (degrees, servo-relative) to radians.
        # Positive offset = right turn = negative steering angle in standard
        # math convention (counter-clockwise positive). Flip the sign if your
        # robot turns the opposite way than expected.
        steering_rad = -math.radians(steering_offset_deg)

        # Ackermann / bicycle model integration.
        if abs(steering_rad) < 1e-4:
            # Straight-line motion — avoid division by zero in the turn radius.
            self.x += distance_m * math.cos(self.theta)
            self.y += distance_m * math.sin(self.theta)
        else:
            # Turning motion — compute the instantaneous turn radius and arc.
            turn_radius = config.WHEELBASE_M / math.tan(steering_rad)
            delta_theta = distance_m / turn_radius

            # Integrate along the arc.
            self.x += turn_radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y -= turn_radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
            self.theta += delta_theta

        # Keep theta in [-π, π] for readability.
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        return self.x, self.y, self.theta

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """Reset pose (useful when starting a new experiment)."""
        self.x     = x
        self.y     = y
        self.theta = theta
        self._last_ticks = None

    @property
    def pose(self) -> tuple[float, float, float]:
        """Current pose as a tuple."""
        return self.x, self.y, self.theta