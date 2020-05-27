
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
            self.contact_schedule = (
                np.array(
                    [[1, 1, 1, 0]
                    [1, 0, 1, 1]
                    [1, 0, 1, 1]
                    [1, 1, 1, 0]]
                )
            ) # 0 = swing, 1 = stance
            self.number_of_phases = self.contact_schedule.shape[1]
            self.stance_duration = 0.15 # In seconds; Trot stance is when all 4 feet are in stance
            self.swing_duration = 0.10 # In seconds; Trot swing is when 2 feet are in swing, 2 feet are in stance
            self.phase_durations = np.array([self.stance_duration, self.swing_duration, self.stance_duration, self.swing_duration])
            self.cycle_duration = 2*self.stance_duration + 2*swing_duration
            self.stance_duration_in_ticks = int(self.stance_duration/self.config.dt)
            self.swing_duration_in_ticks = int(self.swing_duration/self.config.dt)
            self.cycle_duration_in_ticks = 2*self.stance_duration_in_ticks + 2*swing_duration_in_ticks
            self.phase_durations_in_ticks = np.array([self.stance_duration_in_ticks, self.swing_duration_in_ticks, self.stance_duration_in_ticks, self.swing_duration_in_ticks])
