
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
        self.gait_config = GaitConfig(config = self.config, gait = self.gait)

    def calculateGaitPhaseIndex(self, ticks):
        """
        DESCRIPTION:
        Calculates and returns the phase index of the gait that the robot is in given elapsed time in number of ticks.

        ARGUMENTS:
        + ticks: An integer; The elapsed time in ticks, where 1 tick is 1s/dt. This value is maintained in, and thus should be obtained from, the robot object.
        
        RETURNS:
        + gait_phase_index: An integer; The phase index of the current phase of the gait.
        """
        ticks_into_current_gait_cycle = ticks % self.gait_config.gait_cycle_duration_in_ticks
        tick_sum = 0
        for gait_phase_index in range(self.gait_config.gait_number_of_phases):
            tick_sum += self.gait_config.gait_phase_durations_in_ticks[gait_phase_index]
            if ticks_into_current_gait_cycle < tick_sum:
                return gait_phase_index

    def calculateTicksIntoCurrentLegPhase(self, ticks, leg_index):
        """
        DESCRIPTION:
        Calculates and returns the number of ticks into the current leg phase (NOT GAIT PHASE!!!) for the given leg
        specified by leg_index.

        ARGUMENTS:
        + ticks: An integer; The elapsed time in ticks, where 1 tick is 1s/dt. This value is maintained in, and thus should be obtained from, the robot object.
        + leg_index: An integer; The index of the leg to calculate for. {FL: 0, FR: 1, BL: 2, BR: 3}

        RETURNS:
        + ticks_into_current_leg_phase: An integer; the number of ticks into the leg's own phase (NOT GAIT PHASE!!!).
        """
        current_gait_phase_index = self.calculateGaitPhaseIndex(ticks)
        ticks_into_current_gait_phase = self.calculateTicksIntoCurrentGaitPhase(ticks)
        ticks_into_current_leg_phase = None
        if (leg_index == 0):
            # Front-left leg
            if (current_gait_phase_index == 0):
                # FL is just starting its leg stance phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase
            elif (current_gait_phase_index == 1):
                # FL is in the middle of its leg stance phase
                ticks_into_current_leg_phase = self.gait_config.gait_stance_duration_in_ticks + ticks_into_current_gait_phase
            elif (current_gait_phase_index == 2):
                # FL is approaching end of its leg stance phase
                ticks_into_current_leg_phase = self.gait_config.gait_stance_duration_in_ticks + self.gait_config.gait_swing_duration_in_ticks + ticks_into_current_gait_phase
            elif (current_gait_phase_index == 3):
                # FL is in its swing phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase
        if (leg_index == 1):
            # Front-right leg
            if (current_gait_phase_index == 0):
                # FR is approaching end of its leg stance phase
                ticks_into_current_leg_phase = self.gait_config.gait_stance_duration_in_ticks + self.gait_config.gait_swing_duration_in_ticks + ticks_into_current_gait_phase
            elif (current_gait_phase_index == 1):
                # FR is in its swing phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase
            elif (current_gait_phase_index == 2):
                # FR is just starting its leg stance phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase
            elif (current_gait_phase_index == 3):
                # FR is in the middle of its leg stance phase
                ticks_into_current_leg_phase = self.gait_config.gait_stance_duration_in_ticks + ticks_into_current_gait_phase
        if (leg_index == 2):
            # Back-left leg
            if (current_gait_phase_index == 0):
                # BL is approaching end of its leg stance phase
                ticks_into_current_leg_phase = self.gait_config.gait_stance_duration_in_ticks + self.gait_config.gait_swing_duration_in_ticks + ticks_into_current_gait_phase
            elif (current_gait_phase_index == 1):
                # BL is in its swing phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase
            elif (current_gait_phase_index == 2):
                # BL is just starting its leg stance phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase
            elif (current_gait_phase_index == 3):
                # BL is in the middle of its leg stance phase
                ticks_into_current_leg_phase = self.gait_config.gait_stance_duration_in_ticks + ticks_into_current_gait_phase
        if (leg_index == 3):
            # Back-right leg
            if (current_gait_phase_index == 0):
                # BR is just starting its leg stance phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase
            elif (current_gait_phase_index == 1):
                # BR is in the middle of its leg stance phase
                ticks_into_current_leg_phase = self.gait_config.gait_stance_duration_in_ticks + ticks_into_current_gait_phase
            elif (current_gait_phase_index == 2):
                # BR is approaching end of its leg stance phase
                ticks_into_current_leg_phase = self.gait_config.gait_stance_duration_in_ticks + self.gait_config.gait_swing_duration_in_ticks + ticks_into_current_gait_phase
            elif (current_gait_phase_index == 3):
                # BR is in its swing phase
                ticks_into_current_leg_phase = ticks_into_current_gait_phase
        
        return ticks_into_current_leg_phase
     
    def calculateTicksIntoCurrentGaitPhase(self, ticks):
        """
        DESCRIPTION:
        Calculates and returns the elapsed time, in ticks, from the beginning of the current phase of the gait (NOT LEG PHASE!!!).

        ARGUMENTS:
        + ticks: An integer; The elapsed time in ticks, where 1 tick is 1s/dt. This value is maintained in, and thus should be obtained from, the robot object.

        RETURNS:
        + ticks_into_current_phase: The elapsed time, in ticks, from the start of the current phase of the gait.
        """
        ticks_into_current_gait_cycle = ticks % self.gait_config.gait_cycle_duration_in_ticks
        tick_sum = 0
        for gait_phase_index in range(self.gait_config.gait_number_of_phases):
            tick_sum += self.gait_config.gait_phase_durations_in_ticks[gait_phase_index]
            if ticks_into_current_gait_cycle < tick_sum:
                ticks_into_current_phase = ticks_into_current_gait_cycle - tick_sum + self.gait_config.gait_phase_durations_in_ticks[gait_phase_index]
                return ticks_into_current_phase

    def calculateContactPattern(self, ticks):
        """
        DESCRIPTION:
        Calculates and returns the contact pattern of the feet given the elapsed time in ticks.

        ARGUMENTS:
        + ticks: An integer; The elapsed time in ticks, where 1 tick is 1s/dt. This value is maintained in, and thus should be obtained from, the robot object.

        RETURNS:
        + contact_pattern: A (4,) array; The contact pattern of the legs at the time given in ticks. 0 = swing, 1 = stance, [FL, FR, BL, BR].
        """
        current_gait_phase_index = self.calculateGaitPhaseIndex(ticks)
        return self.gait_config.contact_schedule[:, current_gait_phase_index]

