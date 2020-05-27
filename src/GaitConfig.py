

import numpy as np

from enum import Enum
from Config import Config

class Gait(Enum):
    TROT = 0

class GaitConfig:
    def __init__(
        self,
        config,
        gait = Gait.TROT
    ):
        self.gait = gait
        self.config = config

        if (self.gait == Gait.TROT):
            ### TROT PARAMS ###
            self._gait_stance_duration = 0.1 # In seconds; Trot stance is when all 4 feet are in stance
            self._gait_swing_duration = 0.3 # In seconds; Trot swing is when 2 feet are in swing, 2 feet are in stance
            self.contact_schedule = (
                np.array(
                    [[1, 1, 1, 0],
                    [1, 0, 1, 1],
                    [1, 0, 1, 1],
                    [1, 1, 1, 0]]
                )
            ) # 0 = swing, 1 = stance
            self.gait_number_of_phases = self.contact_schedule.shape[1]
            self.gait_stance_duration_in_ticks = int(self._gait_stance_duration/self.config.dt)
            self.gait_swing_duration_in_ticks = int(self._gait_swing_duration/self.config.dt)
            self.gait_cycle_duration_in_ticks = 2*self.gait_stance_duration_in_ticks + 2*self.gait_swing_duration_in_ticks
            self.gait_phase_durations_in_ticks = np.array([self.gait_stance_duration_in_ticks, self.gait_swing_duration_in_ticks, self.gait_stance_duration_in_ticks, self.gait_swing_duration_in_ticks])
            self.swing_height = 0.1
            
            self.leg_swing_duration_in_ticks = self.gait_swing_duration_in_ticks
            self.leg_stance_duration_in_ticks = 2*self.gait_stance_duration_in_ticks + self.gait_swing_duration_in_ticks
