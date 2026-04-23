"""
Keyboard teleop for the Ackermann robot.

Run from the project root:
    python scripts/teleop_keyboard.py

Controls:
    W / S       : forward / reverse
    A / D       : steer left / steer right
    SPACE       : stop (zero speed, center steering)
    Q           : quit
    R           : reset pose display to origin

Prints live pose updates so you can see odometry working.
"""

import sys
import time
import termios
import tty
import select
from pathlib import Path

# Make project root importable
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robot_driver.robot      import Robot
from robot_driver.odometry   import AckermannOdometry
from robot_driver            import config


# ─── Control parameters ─────────────────────────────────────────────
SPEED_STEP            = 10   # how much W/S changes speed per press
STEERING_STEP         =  5   # how much A/D changes steering per press
UPDATE_RATE_HZ        = 20   # control loop frequency
TELEOP_PRINT_EVERY    = 10   # print pose every N loop iterations


# ─── Raw keyboard reading (Unix / macOS / Linux) ───────────────────
def get_key_nonblocking() -> str | None:
    """Return a single key press if available, else None. Non-blocking."""
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None


def main():
    print("Keyboard teleop")
    print("  W/S: speed   A/D: steer   SPACE: stop   R: reset   Q: quit")
    print("Connecting to robot...")

    robot = Robot()
    odom  = AckermannOdometry()

    # Put terminal in raw mode so we read keys without needing Enter.
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    current_speed    = 0
    current_steering = 0
    loop_count       = 0

    try:
        tty.setcbreak(fd)
        print("Connected. Drive with WASD. Press Q to quit.\n")

        while True:
            loop_start = time.time()

            # ─── Handle input ────────────────────────────────
            key = get_key_nonblocking()
            if key:
                k = key.lower()
                if   k == "w":  current_speed    += SPEED_STEP
                elif k == "s":  current_speed    -= SPEED_STEP
                elif k == "a":  current_steering -= STEERING_STEP
                elif k == "d":  current_steering += STEERING_STEP
                elif k == " ":  current_speed = 0; current_steering = 0
                elif k == "r":  odom.reset(); print("\n[pose reset]")
                elif k == "q":  break

                # Clamp to config limits.
                current_speed    = max(config.MIN_SPEED,
                                        min(config.MAX_SPEED, current_speed))
                current_steering = max(config.MIN_STEERING_OFFSET,
                                        min(config.MAX_STEERING_OFFSET, current_steering))

            # ─── Send command ─────────────────────────────────
            robot.drive(current_speed, current_steering)

            # ─── Update odometry from sensor feedback ────────
            reading = robot.update_sensors()
            if reading is not None:
                x, y, theta = odom.update(reading["encoder"], reading["steering"])

                # Periodic pose print (overwriting one line).
                if loop_count % TELEOP_PRINT_EVERY == 0:
                    print(
                        f"\rspeed={current_speed:+4d}  "
                        f"steer={current_steering:+4d}  |  "
                        f"x={x:+.2f}m  y={y:+.2f}m  θ={theta:+.2f}rad",
                        end="", flush=True,
                    )

            # ─── Rate limit ────────────────────────────────────
            elapsed = time.time() - loop_start
            sleep_time = max(0, 1.0 / UPDATE_RATE_HZ - elapsed)
            time.sleep(sleep_time)
            loop_count += 1

    finally:
        # Always restore terminal + stop the robot.
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\n\nShutting down...")
        robot.close()
        print("Done.")


if __name__ == "__main__":
    main()