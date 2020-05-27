
from GaitConfig import Gait
from GaitController import GaitController
from Robot import Mode

import numpy as np
from transforms3d.euler import euler2mat

class MasterController:
    def __init__(
        self,
        config
    ):

        self.config = config
        self.trot_controller = GaitController(config = config, gait = Gait.TROT)

    def step_gait(self, robot, command, gait_controller):
        """
        DESCRIPTION:
        Calculates and returns the foot positions for the next tick
        NOTE: robot attributes are altered in this function!

        ARGUMENTS:
        + robot: A robot object; the robot to control and command.
        + command: A Command object; it contains the input commands into the robot.
        + gait_controller: A GaitController object; the gait controller used to propagate the gait.
        """
        # Find phases of each leg (swing or stance)
        contact_pattern = gait_controller.calculate_contact_pattern(robot.ticks)
        for leg_index in range(4):
            leg_phase = contact_pattern[leg_index] # 0 = swing, 1 = stance
            if (leg_phase == 0):
                # Leg is in swing phase
                # Calculate position of leg's foot at the next tick
            elif (leg_phase == 1):
                # Leg is in stance phase
                # Calculate position of leg's foot at the next tick
        # Collect new leg foot positions into matrix
        # Transform leg foot position matrix according to commanded body rpy
        # Move feet to newly calculated foot positions
        # Update robot attributes
        # TODO


    def step_once(self, robot, command = None):
        """
        DESCRIPTION:
        Takes in a Robot object and a Command object and steps the state of the robot one tick forward based on inputs from command.
        NOTE: robot attributes are altered in this function!

        ARGUMENTS:
        + robot: A Robot object; the robot to control and command.
        + command: A Command object; it contains the input commands into the robot.
        """
        if (command == None):
            # Move to default stance
            robot.foot_locations_wrt_body = np.concatenate( \
                ( \
                    np.array([[robot.stance_polygon_length/2, robot.stance_polygon_width/2, -robot.stance_height]]).T, \
                    np.array([[robot.stance_polygon_length/2, -robot.stance_polygon_width/2, -robot.stance_height]]).T, \
                    np.array([[-robot.stance_polygon_length/2, robot.stance_polygon_width/2, -robot.stance_height]]).T, \
                    np.array([[-robot.stance_polygon_length/2, -robot.stance_polygon_width/2, -robot.stance_height]]).T, \
                ),
                axis = 1
            )
            # Move feet to calculated positions
            robot.front_left_leg.moveFoot(robot.foot_locations_wrt_body[:, 0])
            robot.front_right_leg.moveFoot(robot.foot_locations_wrt_body[:, 1])
            robot.back_left_leg.moveFoot(robot.foot_locations_wrt_body[:, 2])
            robot.back_right_leg.moveFoot(robot.foot_locations_wrt_body[:, 3])
            return
        else:
            pass

        if command.mode == Mode.REST:
            robot.mode = Mode.REST
        elif command.mode == Mode.TROT:
            robot.mode = Mode.TROT

        if robot.mode == Mode.REST:
            # Calculate neutral stance positions for each leg
            robot.foot_locations_wrt_body = np.concatenate( \
                ( \
                    np.array([[robot.stance_polygon_length/2, robot.stance_polygon_width/2, -command.stance_height]]).T, \
                    np.array([[robot.stance_polygon_length/2, -robot.stance_polygon_width/2, -command.stance_height]]).T, \
                    np.array([[-robot.stance_polygon_length/2, robot.stance_polygon_width/2, -command.stance_height]]).T, \
                    np.array([[-robot.stance_polygon_length/2, -robot.stance_polygon_width/2, -command.stance_height]]).T, \
                ),
                axis = 1
            )
            # Desired body orientation matrix
            target_body_rpy_matrix = (
                euler2mat(
                    command.body_roll,
                    command.body_pitch,
                    command.body_yaw,
                )
            )
            # Apply body rpy
            robot.foot_locations_wrt_body = (
                target_body_rpy_matrix.T
                @ robot.foot_locations_wrt_body
            )
            # Move feet to calculated positions
            robot.front_left_leg.moveFoot(robot.foot_locations_wrt_body[:, 0])
            robot.front_right_leg.moveFoot(robot.foot_locations_wrt_body[:, 1])
            robot.back_left_leg.moveFoot(robot.foot_locations_wrt_body[:, 2])
            robot.back_right_leg.moveFoot(robot.foot_locations_wrt_body[:, 3])
            # Foot positions and joint angles in Legs are automatically updated when moveFoot is called.
            # Update robot attributes
            robot.stance_height = command.stance_height
            robot.body_roll = command.body_roll
            robot.body_pitch = command.body_pitch
            robot.body_yaw = command.body_yaw
        elif robot.mode == Mode.TROT:
            self.step_gait(robot = robot, command = command, gait_controller = self.trot_controller)
            pass

        robot.ticks += 1