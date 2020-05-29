
from GaitConfig import Gait
from GaitController import GaitController
from LegStanceController import LegStanceController
from LegSwingController import LegSwingController
from Robot import Mode

import numpy as np
from transforms3d.euler import euler2mat

from scipy.special import erf


class MasterController:
    def __init__(
        self,
        config
    ):

        self.config = config
        self.trot_controller = GaitController(config = config, gait = Gait.TROT)

    def calculateFootLocationsForNextGaitStep(self, robot, command, gait_controller):
        """
        DESCRIPTION:
        Calculates and returns the foot positions for the next tick
        NOTE: the robot object is not changed in this function; it is only passed into this function for reading purposes.

        ARGUMENTS:
        + robot: A robot object; the robot to control and command.
        + command: A Command object; it contains the input commands into the robot.
        + gait_controller: A GaitController object; the gait controller used to propagate the gait.

        RETURNS:
        + new_foot_locations_wrt_body: A (3,) array; the x, y, z coordinates wrt body of the foot's calculated next location.
        + contact_pattern: A (4,) array; the contact pattern of the robot at the time step that the foot location is calculated for.
        + foot_phase_proportions_completed: A (4,) array; the proportion of each foot's respective phase that has been completed, collected into an array. [FL, FR, BL, BR].
        """
        # Update swing_height in gait_config from command
        gait_controller.gait_config.swing_height = command.swing_height
        # Create leg controllers depending on gait_controller
        leg_stance_controller = LegStanceController(gait_config = gait_controller.gait_config)
        leg_swing_controller = LegSwingController(gait_config = gait_controller.gait_config)
        # Find phases of each leg (swing or stance)
        new_foot_locations_wrt_body = np.zeros((3, 4))
        contact_pattern = gait_controller.calculateContactPattern(robot.ticks)
        foot_phase_proportions_completed = np.array([0, 0, 0, 0])
        for leg_index in range(4):
            leg_phase = contact_pattern[leg_index] # 0 = swing, 1 = stance
            if (leg_phase == 0):
                # Leg is in swing phase
                leg_swing_proportion_completed = (
                    gait_controller.calculateTicksIntoCurrentLegPhase(robot.ticks)
                    / gait_controller.gait_config.leg_swing_duration_in_ticks
                )
                new_foot_locations_wrt_body[:, leg_index] = (
                    leg_swing_controller.calculateNewFootLocation(robot, command, leg_index, leg_swing_proportion_completed)
                )
                foot_phase_proportions_completed[i] = leg_swing_proportion_completed
            elif (leg_phase == 1):
                # Leg is in stance phase
                leg_stance_proportion_completed = (
                    gait_controller.calculateTicksIntoCurrentLegPhase(robot.ticks)
                    / gait_controller.gait_config.leg_stance_duration_in_ticks
                )
                new_foot_locations_wrt_body[:, leg_index] = leg_stance_controller.calculateNewFootLocation(robot, command, leg_index)
                foot_phase_proportions_completed[i] = leg_swing_proportion_completed

        raibert_td = leg_swing_controller.calculateRaibertTouchdownLocation(robot = robot, command = command, leg_index = 0)
        capture_point = leg_swing_controller.calculateCapturePoint(robot = robot, command = command, leg_index = 0)

        np.set_printoptions(precision = 3, suppress = True)
        print("FL FOOT:: Phase: {} | xyz: {} | Raibert TD: {} | Capture point: {} | Commanded TD: {}".format(
                contact_pattern[0],
                new_foot_locations_wrt_body[:, 0],
                raibert_td,
                capture_point,
                raibert_td + capture_point
            )
        )

        return new_foot_locations_wrt_body, contact_pattern, foot_phase_proportions_completed

    def calculateVPSPFootWeight(
        self,
        leg_phase_proportion_completed,
        leg_phase,
        sigma_0_stance = 0.2,
        sigma_1_stance = 0.2,
        sigma_0_swing = 0.2,
        sigma_1_swing = 0.2
    ):
        """
        DESCRIPTION:
        Calculates and returns the weighting factor based on leg_phase_proportion_completed and leg_phase (whether leg is in swing or stance).
        (NOT gait swing or stance, but LEG swing or stance!!!)
        
        ARGUMENTS:
        + leg_proportion_completed: A float; the proportion of the leg's current phase that it has traversed. 0 to 1.
        + leg_phase: An integer; either 0 or 1; 0 = swing, 1 = stance. LEG phase, NOT GAIT PHASE.
        + sigma_0_stance: A float; a constant that modifies the profile of the error functions used.
        + sigma_1_stance: A float; a constant that modifies the profile of the error functions used.
        + sigma_0_swing: A float; a constant that modifies the profile of the error functions used.
        + sigma_1_swing: A float; a constant that modifies the profile of the error functions used.
        """
        assert (leg_phase_proportion_completed >= 0 and leg_phase_proportion_completed <= 1) and (leg_phase == 0 or leg_phase == 1)
        K_stance = (
            0.5
            * (
                erf((leg_phase_proportion_completed/(sigma_0_stance*np.sqrt(2))))
                + erf(((1.0 - leg_phase_proportion_completed)/(sigma_1_stance*np.sqrt(2))))
            )
        )
        K_swing = (
            0.5
            * (
                2
                + erf((-1.0*leg_phase_proportion_completed/(sigma_0_swing*np.sqrt(2))))
                + erf(((leg_phase_proportion_completed - 1.0)/(sigma_1_swing*np.sqrt(2))))
            )
        )
        if (leg_phase == 1):
            # Leg is in stance
            return K_stance
        elif (leg_phase == 0):
            # Leg is in swing
            return K_swing

    def calculateVPSPBodyLocation(self, new_foot_locations_wrt_body, contact_pattern, foot_phase_proportions_completed):
        """
        DESCRIPTION:
        VPSP: Virtual Predictive Support Polygon
        Calculates and returns the body location in the next time step, wrt current body location, based on the VPSP
        derived from the anticipated foot locations in that time step, as fed into this function as new_foot_locations_wrt_body.

        ARGUMENTS:
        + new_foot_locations_wrt_body: A (3, 4) array; the foot locations in the time step to calculate the VPSP for. Each column is for one foot. [FL, FR, BL, BR].
        + contact_pattern: A (4,) array; the contact pattern of the feet. [FL, FR, BL, BR]; 0 = swing, 1 = stance.

        RETURNS:
        + p_b_vpsp: A (3,) array; the recommended body location, wrt current body location, to move the body towards as recommended by the VPSP.
        """
        # Calculate weights for each foot
        foot_weights = np.zeros((4))
        for i in range(4):
            foot_weights[i] = calculateVPSPFootWeight(
                leg_phase_proportion_completed = foot_phase_proportions_completed.reshape((4))[i],
                leg_phase = contact_pattern.reshape((4))[i],
            )
            # TODO: You last left off here! Calculate virtual points, etc...


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
            # Track foot trajectory without body rpy
            robot.updateFootLocationsAssumingNoBodyRPY(new_foot_locations_wrt_body)
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
            # Update robot resting foot locations
            robot.foot_locations_wrt_body_at_rest = new_foot_locations_wrt_body
            # Update robot attributes
            robot.stance_height = command.stance_height
            robot.body_roll = command.body_roll
            robot.body_pitch = command.body_pitch
            robot.body_yaw = command.body_yaw

        elif robot.mode == Mode.TROT:
            new_foot_locations_wrt_body, contact_pattern, foot_phase_proportions_completed = (
                self.calculateFootLocationsForNextGaitStep(robot = robot, command = command, gait_controller = self.trot_controller)
            )
            vpsp_body_location = calculateVPSPBodyLocation(new_foot_locations_wrt_body, contact_pattern, foot_phase_proportions_completed)
            # TODO: Use vpsp_body_location to change foot position targets
            # Update robot contact pattern
            robot.contact_pattern = contact_pattern
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
            # Update foot velocities in robot
            robot.foot_velocities_wrt_body = (1.0/self.config.dt)*(new_foot_locations_wrt_body - robot.foot_locations_wrt_body_true)
            # Move feet to calculated positions
            robot.moveAllFeet(new_foot_locations_wrt_body) # Foot positions (true), joint angles in Legs and Robot are internally updated.
            # Update robot attributes
            robot.stance_height = command.stance_height
            robot.body_roll = command.body_roll
            robot.body_pitch = command.body_pitch
            robot.body_yaw = command.body_yaw
        
        

        robot.ticks += 1