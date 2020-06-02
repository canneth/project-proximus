
from enum import Enum

# For linear_accel_capture_point, used in calculateNewFootLocation() in LegSwingController
linear_accel_capture_point_gain = 0.05
# For rp_capture_point, used in calculateNewFootLocation() in LegSwingController
roll_gain = 0.16
pitch_gain = 0.0
# For rp_rate_capture_point, used in calculateNewFootLocation() in LegSwingController
roll_rate_gain = 0.02
pitch_rate_gain = 0.0

class Mode(Enum):
    REST = 0
    TROT = 1

class Gait(Enum):
    TROT = 0

class FootTrajectory(Enum):
    SEMICIRCULAR = 0,
    TRIANGULAR = 1