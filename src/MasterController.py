
from GaitConfig import Gait
from GaitController import GaitController
from LegStanceController import LegStanceController
from LegSwingController import LegSwingController
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

    def stepGait(self, robot, command, gait_controller):
        """
        DESCRIPTION:
        Calculates and returns the foot positions for the next tick
        NOTE: robot attributes are altered in this function!

        ARGUMENTS:
        + robot: A robot object; the robot to control and command.
        + command: A Command object; it contains the input commands into the robot.
        + gait_controller: A GaitController object; the gait controller used to propagate the gait.
        """
        # Create leg controllers depending on gait_controller
        leg_stance_controller = LegStanceController(gait_config = gait_controller.gait_config)
        leg_swing_controller = LegSwingController(gait_config = gait_controller.gait_config)
        # Find phases of each leg (swing or stance)
        contact_pattern = gait_controller.calculate_contact_pattern(robot.ticks)
        new_foot_locations_wrt_body = np.zeros((3, 4))
        for leg_index in range(4):
            leg_phase = contact_pattern[leg_index] # 0 = swing, 1 = stance
            if (leg_phase == 0):
                # Leg is in swing phase
                # Calculate position of leg's foot at the next tick
                swing_proportion_completed = (
                    gait_controller.calculate_ticks_into_current_phase(robot.ticks)
                    / gait_controller.gait_config.swing_duration_in_ticks
                )
                new_foot_locations_wrt_body[:, leg_index] = (
                    leg_swing_controller.calculateNewFootLocation(robot, command, leg_index, swing_proportion_completed)
                )
            elif (leg_phase == 1):
                # Leg is in stance phase
                # Calculate position of leg's foot at the next tick
                new_foot_locations_wrt_body[:, leg_index] = leg_stance_controller.calculateNewFootLocation(robot, command, leg_index)
        # Track foot trajectory without body rpy
        robot.updateFootLocationsAssumingNoBodyRPY(new_foot_locations_wrt_body)
        # Desired body orientation matrix
        body_rpy_matrix = (
            euler2mat(
                command.body_roll,
                command.body_pitch,
                command.body_yaw
            )
        )
        # Apply body rpy
        new_foot_locations_wrt_body = (
            body_rpy_matrix.T
            @ new_foot_locations_wrt_body
        )
        # Move feet to calculated positions
        robot.moveAllFeet(new_foot_locations_wrt_body) # Foot positions (true), joint angles in Legs and Robot are internally updated.
        # Update robot attributes
        robot.stance_height = command.stance_height
        robot.body_roll = command.body_roll
        robot.body_pitch = command.body_pitch
        robot.body_yaw = command.body_yaw
        print("FL FOOT:: Phase: {} | xyz: {}".format(contact_pattern[0], robot.foot_locations_wrt_body[:, 0]))
        
    def stepOnce(self, robot, command = None):
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
            robot.moveAllFeet(robot.foot_locations_wrt_body_at_rest) # Foot positions, joint angles in Legs and Robot are internally updated.
            return
        else:
            pass

        if command.mode == Mode.REST:
            robot.mode = Mode.REST
        elif command.mode == Mode.TROT:
            robot.mode = Mode.TROT

        if robot.mode == Mode.REST:
            # Calculate neutral stance positions for each leg
            new_foot_locations_wrt_body = np.concatenate( \
                ( \
                    np.array([[robot.stance_polygon_length/2, robot.stance_polygon_width/2, -command.stance_height]]).T, \
                    np.array([[robot.stance_polygon_length/2, -robot.stance_polygon_width/2, -command.stance_height]]).T, \
                    np.array([[-robot.stance_polygon_length/2, robot.stance_polygon_width/2, -command.stance_height]]).T, \
                    np.array([[-robot.stance_polygon_length/2, -robot.stance_polygon_width/2, -command.stance_height]]).T, \
                ),
                axis = 1
            )
            # Desired body orientation matrix
            body_rpy_matrix = (
                euler2mat(
                    command.body_roll,
                    command.body_pitch,
                    command.body_yaw,
                )
            )
            # Apply body rpy
            new_foot_locations_wrt_body = (
                body_rpy_matrix.T
                @ new_foot_locations_wrt_body
            )
            # Move feet to calculated positions
            robot.moveAllFeet(new_foot_locations_wrt_body) # Foot positions, joint angles in Legs and Robot are internally updated.
            # Track foot trajectory without body rpy
            robot.updateFootLocationsAssumingNoBodyRPY(new_foot_locations_wrt_body)
            # Update robot resting foot locations
            robot.foot_locations_wrt_body_at_rest = new_foot_locations_wrt_body
            # Update robot attributes
            robot.stance_height = command.stance_height
            robot.body_roll = command.body_roll
            robot.body_pitch = command.body_pitch
            robot.body_yaw = command.body_yaw
        elif robot.mode == Mode.TROT:
            self.stepGait(robot = robot, command = command, gait_controller = self.trot_controller)

        robot.ticks += 1