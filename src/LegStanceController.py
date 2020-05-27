
import numpy as np
from transforms3d.euler import euler2mat

class LegStanceController:
    def __init__(
        self,
        gait_config
    ):
        self.gait_config = gait_config

    def calculateFootLocationDelta(self, robot, command, leg_index):
        """
        DESCRIPTION:
        Calculates and returns the change in foot position (wrt body) after a single tick.

        ARGUMENTS:
        + robot: A robot object; the robot to control and command.
        + command: A Command object; it contains the input commands into the robot.
        + leg_index: An integer; The index of the leg to calculate for: (FL = 0, FR = 1, BL = 2, BR = 3).

        RETURNS:
        + foot_delta_p: A (3,) array; The change in position of the foot after a single tick.
        + foot_delta_R: A (3, 3) array; The rotation matrix to be applied to the foot position due to yaw in a single tick.
        """
        foot_velocity = (
            np.array(
                [
                    -command.body_velocity[0],
                    -command.body_velocity[1],
                    0
                ]
            )
        )
        foot_delta_p = foot_velocity * self.gait_config.config.dt
        foot_delta_R = euler2mat(0, 0, -command.gait_yaw_speed * self.gait_config.config.dt) # -ve since feet move in the opposite of desired body yaw
        return (foot_delta_p, foot_delta_R)

    def calculateNewFootLocation(self, robot, command, leg_index):
        """
        DESCRIPTION:
        Calculates and returns the new foot position (wrt body) after a single tick.

        ARGUMENTS:
        + robot: A robot object; the robot to control and command.
        + command: A Command object; it contains the input commands into the robot.
        + leg_index: An integer; The index of the leg to calculate for: (FL = 0, FR = 1, BL = 2, BR = 3).

        RETURNS:
        + new_foot_location: A (3,) array; The x, y, z coordinates of the new foot position (wrt body) after a single tick.
        """
        current_foot_location_assuming_no_body_rpy = robot.foot_locations_wrt_body_assuming_no_body_rpy[:, leg_index]
        foot_delta_p, foot_delta_R = self.calculateFootLocationDelta(robot, command, leg_index)
        new_foot_location = foot_delta_R @ current_foot_location_assuming_no_body_rpy + foot_delta_p
        new_foot_location[2] = -command.stance_height
        return new_foot_location