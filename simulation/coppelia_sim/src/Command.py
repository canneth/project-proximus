
import numpy as np

from GlobalConstants import Mode
from GlobalConstants import Gait
from GlobalConstants import FootTrajectory

class Command:
    def __init__(
        self
    ):

        self.stance_polygon_length = 0.4
        self.stance_polygon_width = 0.18
        self.stance_height = 0.2
        
        self.body_roll = 0.0 # In radians
        self.body_pitch = 0.0 # In radians
        self.body_yaw = 0.0 # In radians

        self.body_velocity = np.zeros((3))
        self.gait_yaw_speed = 0.0 # In radians/s
        self.swing_height = 0.1

        self.mode = Mode.REST
