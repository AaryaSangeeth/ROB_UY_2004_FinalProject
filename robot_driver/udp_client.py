"""
Low-level UDP client for talking to the Arduino.

This module is deliberately thin — it converts Python values to bytes for the
Arduino and back. All protocol details (the specific comma-separated string
format) live here.

Protocol (matches robot_firmware.ino):
  Laptop → Arduino:  "speed,steering_angle\\n"        e.g.  "60,-10\\n"
  Arduino → Laptop:  "encoder,steering,n_rays[,angle,dist]...\\n"

No other file in the project should import the `socket` module directly.
"""

import socket
from robot_driver import config


class UDPClient:
    """Manages a single UDP socket for Arduino communication."""

    def __init__(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("", config.LAPTOP_PORT))
        self._sock.settimeout(config.SOCKET_TIMEOUT_SEC)

    def send_command(self, speed: int, steering_angle: int) -> None:
        """Send a motion command to the Arduino."""
        msg = f"{speed},{steering_angle}\n".encode("utf-8")
        self._sock.sendto(msg, (config.ARDUINO_IP, config.ARDUINO_PORT))

    def read_sensor(self) -> dict | None:
        """Read one sensor packet from the Arduino.

        Returns a dict with keys 'encoder', 'steering', 'n_rays', 'lidar_rays',
        or None on timeout (no packet arrived in time).
        """
        try:
            raw, _ = self._sock.recvfrom(2048)
        except socket.timeout:
            return None

        parts = raw.decode("utf-8").strip().split(",")
        try:
            encoder  = int(parts[0])
            steering = int(parts[1])
            n_rays   = int(parts[2])
        except (ValueError, IndexError):
            # Malformed packet — drop it rather than crash.
            return None

        # LiDAR rays come as (angle, distance) pairs after the header fields.
        # Empty until LiDAR is enabled in the Arduino sketch — that's fine.
        ray_data = parts[3:]
        lidar_rays = [
            (int(ray_data[i]), int(ray_data[i + 1]))
            for i in range(0, len(ray_data) - 1, 2)
        ]

        return {
            "encoder":    encoder,
            "steering":   steering,
            "n_rays":     n_rays,
            "lidar_rays": lidar_rays,
        }

    def close(self) -> None:
        """Close the underlying socket. Always call before program exit."""
        self._sock.close()