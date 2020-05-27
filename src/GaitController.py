
from GaitConfig import GaitConfig
from GaitConfig import Gait

class GaitController:
    def __init__(
        self,
        config,
        gait = Gait.TROT
    ):
        self.gait = Gait.TROT
        self.config = config
        self.gait_config = GaitConfig(self.gait, self.config)

    def calculate_gait_phase_index(self, ticks):
        """
        DESCRIPTION:
        Calculates and returns the phase index of the gait that the robot is in given elapsed time in number of ticks.

        ARGUMENTS:
        + ticks: An integer; The elapsed time in ticks, where 1 tick is 1s/dt. This value is maintained in, and thus should be obtained from, the robot object.
        
        RETURNS:
        + gait_phase_index: An integer; The phase index of the current phase of the gait.
        """
        ticks_into_current_gait_cycle = ticks % self.gait_config.cycle_duration_in_ticks
        ticks_sum = 0
        for gait_phase_index in range(self.gait_config.number_of_phases):
            ticks_sum += self.gait_config.phase_durations_in_ticks[gait_phase_index]
            if ticks_into_current_gait_cycle < tick_sum:
                return gait_phase_index

    def calculate_ticks_into_current_phase(self, ticks):
        """
        DESCRIPTION:
        Calculates and returns the elapsed time, in ticks, from the beginning of the current phase of the gait.

        ARGUMENTS:
        + ticks: An integer; The elapsed time in ticks, where 1 tick is 1s/dt. This value is maintained in, and thus should be obtained from, the robot object.

        RETURNS:
        + ticks_into_current_phase: The elapsed time, in ticks, from the start of the current phase of the gait.
        """
        ticks_into_current_gait_cycle = ticks % self.gait_config.cycle_duration_in_ticks
        ticks_sum = 0
        for gait_phase_index in range(self.gait_config.number_of_phases):
            ticks_sum += self.gait_config.phase_durations_in_ticks[gait_phase_index]
            if ticks_into_current_gait_cycle < tick_sum:
                ticks_into_current_phase = ticks_into_current_gait_cycle - ticks_sum + self.gait_config.phase_durations_in_ticks[gait_phase_index]
                return ticks_into_current_phase

    def calculate_contact_pattern(self, ticks):
        """
        DESCRIPTION:
        Calculates and returns the contact pattern of the feet given the elapsed time in ticks.

        ARGUMENTS:
        + ticks: An integer; The elapsed time in ticks, where 1 tick is 1s/dt. This value is maintained in, and thus should be obtained from, the robot object.

        RETURNS:
        + contact_pattern: A (4,) array; The contact pattern of the legs at the time given in ticks. 0 = swing, 1 = stance, [FL, FR, BL, BR].
        """
        current_gait_phase_index = self.calculate_gait_phase_index(ticks)
        return self.gait_config.contact_schedule[:, current_gait_phase_index]

