

from GlobalConstants import FootTrajectory
from GlobalConstants import linear_accel_capture_point_gain
from GlobalConstants import roll_gain
from GlobalConstants import pitch_gain
from GlobalConstants import roll_rate_gain
from GlobalConstants import pitch_rate_gain

import numpy as np
from numpy import pi as PI
from transforms3d.euler import euler2mat, quat2euler

class LegSwingController:
    def __init__(
        self,
        gait_config,
        imu
    ):
        self.gait_config = gait_config
        self.imu = imu
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

    def calculateCapturePoint(self, robot, command):
        """
        DESCRIPTION:
        The capture point of each leg is the offset distance to which the foot must step in order to stop a fall.
        The capture point is linearly combined with the Raibert touchdown location to compute the touchdown location.
        Returns the capture point.

        ARUGMENTS:
        + robot: A robot object; the robot to control and command.
        + command: A Command object; it contains the input commands into the robot.

        RETURNS:
        + capture_point: A (3,) array; the offsets from Raibert touchdown location to be used to calculate the new foot touchdown location.
        """
        desired_velocity = command.body_velocity
        current_velocity = robot.body_velocity
        nominal_gait_height = robot.stance_height
        gravitational_constant = 9.81
        capture_point = np.sqrt(nominal_gait_height/gravitational_constant)*(current_velocity - desired_velocity).reshape(3)
        # capture_point[0] = np.clip(capture_point[0], -0.2, 0.2)
        # capture_point[1] = np.clip(capture_point[1], -0.2, 0.2)
        capture_point[2] = 0 # z-coordinate irrelevant
        return capture_point

    def calculateLinearAccelCapturePoint(self, gain = 0.0):
        """
        DESCRIPTION:
        Calculates a "capture point" based on the linear acceleration of the body of the robot.

        ARGUMENTS:
        + gain: A float; the P-gain on the linear acceleration to produce the capture point.

        RETURNS:
        + linear_accel_capture_point: A (3,) array; the offsets from the touchdown location dependent on linear acceleration of robot body.
        """
        x_accel = self.imu.accel_vals[0]
        y_accel = self.imu.accel_vals[1]
        z_accel = self.imu.accel_vals[2]
        linear_accel_capture_point = gain*np.array([x_accel, y_accel, z_accel])
        linear_accel_capture_point[0] = 0.0 # This capture point is mostly for roll-stability, so x-coordinate is irrelevant
        linear_accel_capture_point[2] = 0.0 # z-coordinate irrelevant
        return linear_accel_capture_point

    def calculateRollPitchCapturePoint(self, command, roll_gain = 0.0, pitch_gain = 0.0):
        """
        DESCRIPTION:
        Calculates a "capture point" based on the roll and pitch angle errors of the body of the robot.

        ARGUMENTS:
        + command: A Command object; the commanded body roll and pitch angles are drawn from this.
        + roll_gain: A float; the P-gain on the roll angle.
        + pitch_gain: A float; the P-gain on the pitch angle.

        RETURNS:
        + rp_capture_point: A (3,) array; the offsets from the touchdown location dependent on
        the roll and pitch angle rates of the robot body.
        """
        heading_quaternion = self.imu.quaternion_vals
        heading_quaternion = np.roll(heading_quaternion, 1) # IMU gives [x, y, z, w]; This rolls it into [w, x, y, z]
        rpy_angles = quat2euler(heading_quaternion)
        roll_angle = rpy_angles[0]
        pitch_angle = rpy_angles[1]
        roll_error = roll_angle - command.body_roll
        pitch_error = pitch_angle - command.body_pitch
        # When roll error is +ve, it means that the robot is rolling towards the right
        # The correction saggital-axis displacement of the feet should thus be towards the body's right (-ve y-direction)
        # When pitch error is +ve, it means the robot is pitching forwards
        # The correction frontal-axis displacement of the feet should thus be towards the body's front (+ve x-direction)
        rp_capture_point = np.array([pitch_gain*pitch_error, -roll_gain*roll_error,  0]).reshape(3) # z-coordinate is irrelevant
        return rp_capture_point

    def calculateRollPitchRateCapturePoint(self, roll_rate_gain = 0.0, pitch_rate_gain = 0.0):
        """
        DESCRIPTION:
        Calculates a "capture point" based on the roll and pitch rates of the body of the robot.

        ARGUMENTS:
        + roll_rate_gain: A float; the P-gain on the roll angle rate.
        + pitch_rate_gain: A float; the P-gain on the pitch angle rate.

        RETURNS:
        + rp_rate_capture_point: A (3,) array; the offsets from the touchdown location dependent on
        the roll and pitch angle rates of the robot body.
        """
        roll_rate = self.imu.gyro_vals[0]
        pitch_rate = self.imu.gyro_vals[1]
        # When roll rate is +ve, it means that the robot is rolling towards the right
        # The correction saggital-axis displacement of the feet should thus be towards the body's right (-ve y-direction)
        # When pitch rate is +ve, it means the robot is pitching forwards
        # The correction frontal-axis displacement of the feet should thus be towards the body's front (+ve x-direction)
        rp_rate_capture_point = np.array([pitch_rate_gain*pitch_rate, -roll_rate_gain*roll_rate, 0.0]).reshape(3)
        return rp_rate_capture_point

    def calculateNewFootLocation(
        self,
        robot,
        command,
        leg_index,
        swing_proportion_completed,
        trajectory_shape = FootTrajectory.TRIANGULAR,
        use_capture_point = False,
        use_accel_capture_point = False,
        use_rp_capture_point = False,
        use_rp_rate_capture_point = False
    ):
        """
        DESCRIPTION:
        Calculates and returns the new foot position (wrt body) after a single tick.

        ARGUMENTS:
        + robot: A robot object; the robot to control and command.
        + command: A Command object; it contains the input commands into the robot.
        + leg_index: An integer; The index of the leg to calculate for: (FL = 0, FR = 1, BL = 2, BR = 3).
        + swing_proportion_completed: A float; the proportion of the swing phase completed by the foot.
        + trajectory_shape: An integer; 0 = semi-circular, 1 = trianglular
        + use_capture_point: A boolean; True if you want to use capture point, False otherwise.

        RETURNS:
        + new_foot_location: A (3,) array; The x, y, z coordinates of the new foot position (wrt body) after a single tick.
        """
        assert swing_proportion_completed >= 0 and swing_proportion_completed <= 1
        current_foot_location_assuming_no_body_rpy = robot.foot_locations_wrt_body_assuming_no_body_rpy[:, leg_index]
        raibert_touchdown_location = self.calculateRaibertTouchdownLocation(robot, command, leg_index)
        
        touchdown_location = raibert_touchdown_location

        if (use_capture_point == True):
            capture_point = self.calculateCapturePoint(robot, command)
            touchdown_location = touchdown_location + capture_point

        if (use_accel_capture_point == True):
            linear_accel_capture_point = self.calculateLinearAccelCapturePoint(gain = linear_accel_capture_point_gain)
            if ((linear_accel_capture_point[1] > 0) and (leg_index == 0 or leg_index == 2)):
                # Robot is moving with an undesired velocity in the +ve y-direction (the robot's left)
                # Move only the legs on the left of the robot to counteract this
                touchdown_location = touchdown_location + linear_accel_capture_point
            elif ((linear_accel_capture_point[1] < 0) and (leg_index == 1 or leg_index == 3)):
                # Robot is moving with an undesired velocity in the -ve y-direction (the robot's right)
                # Move only the legs on the right of the robot to counteract this
                touchdown_location = touchdown_location + linear_accel_capture_point

        if (use_rp_capture_point == True):
            rp_capture_point = self.calculateRollPitchCapturePoint(command = command, roll_gain = roll_gain, pitch_gain = pitch_gain)
            touchdown_location[0] = touchdown_location[0] + rp_capture_point[0] # Apply x-offset to all legs
            # Apply y-offset conditionally to the legs as follows
            if ((rp_capture_point[1] > 0) and (leg_index == 0 or leg_index == 2)):
                # Robot is moving with an undesired velocity in the +ve y-direction (the robot's left)
                # Along the y-direction, move only the legs on the left of the robot to counteract this
                touchdown_location[1] = touchdown_location[1] + rp_capture_point[1]
            elif ((rp_capture_point[1] < 0) and (leg_index == 1 or leg_index == 3)):
                # Robot is moving with an undesired velocity in the -ve y-direction (the robot's right)
                # Along the y-direction, move only the legs on the right of the robot to counteract this
                touchdown_location[1] = touchdown_location[1] + rp_capture_point[1]

        if (use_rp_rate_capture_point == True):
            rp_rate_capture_point = self.calculateRollPitchRateCapturePoint(roll_rate_gain = roll_rate_gain, pitch_rate_gain = pitch_rate_gain)
            touchdown_location[0] = touchdown_location[0] + rp_rate_capture_point[0] # Apply x-offset to all legs
            # Apply y-offset conditionally to the legs as follows
            if ((rp_rate_capture_point[1] > 0) and (leg_index == 0 or leg_index == 2)):
                # Robot is moving with an undesired velocity in the +ve y-direction (the robot's left)
                # Along the y-direction, move only the legs on the left of the robot to counteract this
                touchdown_location[1] = touchdown_location[1] + rp_rate_capture_point[1]
            elif ((rp_rate_capture_point[1] < 0) and (leg_index == 1 or leg_index == 3)):
                # Robot is moving with an undesired velocity in the -ve y-direction (the robot's right)
                # Along the y-direction, move only the legs on the right of the robot to counteract this
                touchdown_location[1] = touchdown_location[1] + rp_rate_capture_point[1]
            

        time_to_touchdown = self.gait_config.config.dt * self.gait_config.leg_swing_duration_in_ticks * (1.0 - swing_proportion_completed)
        foot_delta_p = (touchdown_location - current_foot_location_assuming_no_body_rpy)/(time_to_touchdown / self.gait_config.config.dt)
        foot_delta_p[2] = 0
        z_from_ground = None
        if (trajectory_shape == FootTrajectory.TRIANGULAR):
            z_from_ground = self.gait_config.swing_height*np.sin(swing_proportion_completed*PI)
        elif (trajectory_shape == FootTrajectory.SEMICIRCULAR):
            if (swing_proportion_completed <= 0.5):
                z_from_ground = self.gait_config.swing_height*swing_proportion_completed
            else:
                z_from_ground = self.gait_config.swing_height*(1 - swing_proportion_completed)
        new_foot_location = current_foot_location_assuming_no_body_rpy + foot_delta_p
        new_foot_location[2] = -command.stance_height + z_from_ground
        return new_foot_location

