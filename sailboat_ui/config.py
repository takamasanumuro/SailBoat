"""
Configuration settings for the Sailboat Control UI
"""

# Serial Communication Settings
DEFAULT_BAUDRATE = 9600
SERIAL_TIMEOUT = 1.0
CONNECTION_DELAY = 2.0  # Seconds to wait after connecting for Arduino reset

# UI Update Settings
UPDATE_INTERVAL_MS = 100  # How often to update the display (milliseconds)

# Control Limits
RUDDER_MIN = -100  # Minimum rudder percentage
RUDDER_MAX = 100   # Maximum rudder percentage
MOTOR_MIN = 0      # Minimum motor percentage
MOTOR_MAX = 100    # Maximum motor percentage
ANGLE_MIN = -45    # Minimum angle in degrees
ANGLE_MAX = 45     # Maximum angle in degrees

# PID Presets
PID_PRESETS = {
    "gentle": {"kp": 0.1, "ki": 0.0, "kd": 0.0},
    "default": {"kp": 0.2, "ki": 0.01, "kd": 0.05},
    "ultra_smooth": {"kp": 0.05, "ki": 0.0, "kd": 0.02},
}

# UI Colors
COLORS = {
    "connected": "green",
    "disconnected": "red",
    "emergency": "red",
    "normal": "black"
}

# Window Settings
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 700
WINDOW_TITLE = "Sailboat Control Interface"

# Debug Settings
MAX_DEBUG_LINES = 1000  # Maximum lines in debug window before clearing old ones
TIMESTAMP_FORMAT = "%H:%M:%S"