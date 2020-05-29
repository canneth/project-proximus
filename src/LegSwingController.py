

import numpy as np
from numpy import pi as PI
from transforms3d.euler import euler2mat

class LegSwingController:
    def __init__(
        self,
        gait_config
    ):
        self.gait_config = gait_config
        self.alpha = 0.5 # Ratio between touchdown distance from neutral foot position and total xy planar body movement
        self.beta = 0.5 # Ratio between touchdown distance from neutral foot position and total xy planar body movement

    def calculateRaibertTouchdownLocation(self, robot, command, leg_index):
        """
        DESCRIPTION:
        Calculates the touchdown location of the foot in swing using the Raibert heuristic.

        ARGUMENTS:
        + robot: A robot object; the robot to control and command.
        + command: A Command object; it contains the input commands into the robot.
        + leg_index: An integer; The index of the leg to calculate for: (FL = 0, FR = 1, BL = 2, BR = 3).

        RETURNS:
        + touchdown_location: A (3,) array; the x, y, z coordinates of the touchdown location wrt body.
        """
        touchdown_displacement_from_neutral_location = (
            np.dot(
                self.alpha
                * self.gait_config.leg_stance_duration_in_ticks
                * self.gait_config.config.dt,
                command.body_velocity
            )
        )
        touchdown_displacement_from_neutral_location[2] = 0 # z-coordinate not important for touchdown location
        projected_total_gait_yaw_during_stance_phase = (
            self.beta
            * command.gait_yaw_speed
            * self.gait_config.leg_stance_duration_in_ticks
            * self.gait_config.config.dt
        )
        projected_total_gait_yaw_rotation_matrix = euler2mat(0, 0, projected_total_gait_yaw_during_stance_phase)
        touchdown_location = (
            projected_total_gait_yaw_rotation_matrix @ robot.foot_locations_wrt_body_at_rest[:, leg_index]
            + touchdown_displacement_from_neutral_location
        )
        return touchdown_location

    def calculateCapturePoint(self, robot, command, leg_index):
        """
        DESCRIPTION:
        The capture point of each leg is the offset distance to which the foot must step in order to stop a fall.
        The capture point is linearly combined with the Raibert touchdown location to compute the touchdown location.
        Returns the capture point.

        ARUGMENTS:
        + robot: A robot object; the robot to control and command.
        + command: A Command object; it contains the input commands into the robot.
        + leg_index: An integer; The index of the leg to calculate for: (FL = 0, FR = 1, BL = 2, BR = 3).

        RETURNS:
        + capture_point: A (3,) array; the offsets from Raibert touchdown location to be used to calculate the new foot touchdown location.
        """
        desired_velocity = command.body_velocity
        current_velocity = robot.body_velocity
        nominal_gait_height = robot.stance_height
        gravitational_constant = 9.81
        capture_point = np.sqrt(nominal_gait_height/gravitational_constant)*(current_velocity - desired_velocity)
        return capture_point


    def calculateNewFootLocation(self, robot, command, leg_index, swing_proportion_completed):
        """
        DESCRIPTION:
        Calculates and returns the new foot position (wrt body) after a single tick.

        ARGUMENTS:
        + robot: A robot object; the robot to control and command.
        + command: A Command object; it contains the input commands into the robot.
        + leg_index: An integer; The index of the leg to calculate for: (FL = 0, FR = 1, BL = 2, BR = 3).
        + swing_proportion_completed: A float; the proportion of the swing phase completed by the foot.

        RETURNS:
        + new_foot_location: A (3,) array; The x, y, z coordinates of the new foot position (wrt body) after a single tick.
        """
        assert swing_proportion_completed >= 0 and swing_proportion_completed <= 1
        current_foot_location_assuming_no_body_rpy = robot.foot_locations_wrt_body_assuming_no_body_rpy[:, leg_index]
        capture_point = self.calculateCapturePoint(robot, command, leg_index)
        raibert_touchdown_location = self.calculateRaibertTouchdownLocation(robot, command, leg_index)
        touchdown_location = raibert_touchdown_location + capture_point
        time_to_touchdown = self.gait_config.config.dt * self.gait_config.leg_swing_duration_in_ticks * (1.0 - swing_proportion_completed)
        foot_delta_p = (touchdown_location - current_foot_location_assuming_no_body_rpy)/(time_to_touchdown / self.gait_config.config.dt)
        foot_delta_p[2] = 0
        z_from_ground = self.gait_config.swing_height*np.sin(swing_proportion_completed*PI)
        new_foot_location = current_foot_location_assuming_no_body_rpy + foot_delta_p
        new_foot_location[2] = -command.stance_height + z_from_ground
        return new_foot_location