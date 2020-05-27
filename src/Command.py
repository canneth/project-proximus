
import numpy as np

from Robot import Mode

class Command:
    def __init__(
        self
    ):

        self.body_velocity = np.zeros((3))
        self.gait_yaw_speed = 0 # In radians/s
        self.body_roll = 0 # In radians
        self.body_pitch = 0 # In radians
        self.body_yaw = 0 # In radians
        self.stance_height = 0.225

        self.mode = Mode.REST
