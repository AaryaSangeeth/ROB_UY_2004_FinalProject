"""
Central configuration for the robot driver.

All network addresses, physical dimensions, and control limits live here.
Change a value in this file and the rest of the code picks it up automatically.
"""

# ─── Network configuration ──────────────────────────────────────────
# These must match the values in arduino/robot_firmware/robot_firmware.ino
LAPTOP_PORT  = 4010                 # UDP port the laptop LISTENS on
ARDUINO_IP   = "192.168.0.198"       # ← PUT ARDUINO'S IP HERE (from Serial Monitor)
ARDUINO_PORT = 4010                 # UDP port the Arduino LISTENS on

# ─── Control limits ─────────────────────────────────────────────────
# Safety bounds enforced before any command is sent to the Arduino.
# The Arduino multiplies speed by 2 internally, so MAX_SPEED=100 → PWM 200.
MAX_SPEED            = 100
MIN_SPEED            = -100
MAX_STEERING_OFFSET  = 30           # degrees from center; +right, -left
MIN_STEERING_OFFSET  = -30

# ─── Physical dimensions (Ackermann geometry) ──────────────────────
# Used in Layer 2 for odometry. Measure your actual robot.
WHEELBASE_M                   = 0.20    # front axle to rear axle, meters
WHEEL_RADIUS_M                = 0.035   # rear wheel radius, meters
ENCODER_TICKS_PER_REVOLUTION  = 40      # check motor datasheet or measure

# ─── Communication timing ──────────────────────────────────────────
SOCKET_TIMEOUT_SEC = 1.0            # wait before declaring a missed packet