"""
High-level Robot interface.

Wraps the UDP client, enforces safety limits, and caches the last sensor reading.

Typical use:
    robot = Robot()
    robot.drive(speed=50, steering_angle=0)    # forward, wheels straight
    ticks = robot.get_encoder_ticks()
    robot.stop()
    robot.close()
"""

from robot_driver import config
from robot_driver.udp_client import UDPClient


class Robot:
    """High-level interface to the Ackermann mobile robot."""

    def __init__(self):
        self._client = UDPClient()
        self._last_sensor: dict | None = None

    # ─── Control ───────────────────────────────────────────────────

    def drive(self, speed: int, steering_angle: int = 0) -> None:
        """Send a drive command, clamped to safe limits."""
        safe_speed    = self._clamp(speed,
                                    config.MIN_SPEED,
                                    config.MAX_SPEED)
        safe_steering = self._clamp(steering_angle,
                                    config.MIN_STEERING_OFFSET,
                                    config.MAX_STEERING_OFFSET)
        self._client.send_command(safe_speed, safe_steering)

    def stop(self) -> None:
        """Bring the robot to a full stop with wheels centered."""
        self._client.send_command(0, 0)

    # ─── Sensing ───────────────────────────────────────────────────

    def update_sensors(self) -> dict | None:
        """Fetch the latest sensor packet. Returns None on timeout."""
        reading = self._client.read_sensor()
        if reading is not None:
            self._last_sensor = reading
        return reading

    def get_encoder_ticks(self) -> int | None:
        """Cumulative encoder count since Arduino boot, or None if no data yet."""
        if self._last_sensor is None:
            self.update_sensors()
        return self._last_sensor["encoder"] if self._last_sensor else None

    def get_steering_angle(self) -> int | None:
        """Last commanded steering offset, as echoed back by the Arduino."""
        if self._last_sensor is None:
            self.update_sensors()
        return self._last_sensor["steering"] if self._last_sensor else None

    # ─── Lifecycle ─────────────────────────────────────────────────

    def close(self) -> None:
        """Stop the robot and release the socket. Always call before exit."""
        self.stop()
        self._client.close()

    # ─── Helpers ───────────────────────────────────────────────────

    @staticmethod
    def _clamp(value: int, lo: int, hi: int) -> int:
        return max(lo, min(hi, value))