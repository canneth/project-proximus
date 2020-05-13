
import sim

import numpy as np
from math import radians, degrees
from math import pi as PI

import time

import matplotlib.pyplot as plt

class Leg:
    def __init__(
        self,
        client_id,
        body_frame,
        coxa_joint,
        coxa,
        femur_joint,
        femur,
        tibia_joint,
        tibia,
        foot,
        foot_origin = [0, 0, 0]
    ):

        self.client_id = client_id
        self.body_frame = body_frame
        self.coxa_joint = coxa_joint
        self.coxa = coxa
        self.femur_joint = femur_joint
        self.femur = femur
        self.tibia_joint = tibia_joint
        self.tibia = tibia
        self.foot = foot
        self.foot_origin = foot_origin

        # Getting constants for ik calculation
        _, self.d = sim.simxGetObjectPosition(self.client_id, self.coxa_joint, self.body_frame, sim.simx_opmode_blocking)
        self.d_x = self.d[0]
        self.d_y = self.d[1]
        _, self.l_1 = sim.simxGetObjectPosition(self.client_id, self.coxa_joint, self.femur_joint, sim.simx_opmode_blocking)
        self.l_1_y = -self.l_1[2]
        self.l_1_x = -self.l_1[1]
        _, self.l_2 = sim.simxGetObjectPosition(self.client_id, self.femur_joint, self.tibia_joint, sim.simx_opmode_blocking)
        self.l_2 = abs(self.l_2[0])
        _, self.l_3 = sim.simxGetObjectPosition(self.client_id, self.tibia_joint, self.foot, sim.simx_opmode_blocking)
        self.l_3 = abs(self.l_3[2])

    def moveCoxaToAngle(self, angle):
        """
        Arguments:
        + angle: Angle to move the coxa to, in radians.
        Description:
        + Sends a command to move the corresponding coxa joint to the specified angle.
        """
        sim.simxSetJointTargetPosition(self.client_id, self.coxa_joint, angle, sim.simx_opmode_streaming)
    def moveFemurToAngle(self, angle):
        """
        Arguments:
        + angle: Angle to move the femur to, in radians.
        Description:
        + Sends a command to move the corresponding femur joint to the specified angle.
        """
        sim.simxSetJointTargetPosition(self.client_id, self.femur_joint, angle, sim.simx_opmode_streaming)
    def moveTibiaToAngle(self, angle):
        """
        Arguments:
        + angle: Angle to move the tibia to, in radians.
        Description:
        + Sends a command to move the corresponding tibia joint to the specified angle.
        """
        sim.simxSetJointTargetPosition(self.client_id, self.tibia_joint, angle, sim.simx_opmode_streaming)

    def ikFoot(self, dest):
        # NOTE: For some reason, there seems to be significant in-precision in the IK results (up to about 7 degrees in joint angle errors), but the general position is thereabouts.
        """
        DESCRIPTION:
        Calculates the joint angles required to bring the foot to the dest vector, defined with respect to the body_frame.

        ARGUMENTS:
        + dest: A size 3 iterable; the destination coordinates [x, y, z] of the foot.
        RETURNS:
        + theta_1, theta_2, theta_3: The joint angles, in radians, that bring the foot to the target specified by dest.
        """
        d_x = self.d_x
        d_y = self.d_y
        l_1_y = self.l_1_y
        l_1_x = self.l_1_x
        l_2 = self.l_2
        l_3 = self.l_3

        p_x = dest[0]
        p_y = dest[1]
        p_z = dest[2]
        p_yz = np.sqrt(p_y**2 + p_z**2)

        print("p_x: {:.5f} | p_y: {:.5f} | p_z: {:.5f}".format(p_x, p_y, p_z))
        print("d_x: {:.5f} | d_y: {:.5f} | l_1_x: {:.5f} | l_1_y: {:.5f} | l_2: {:.5f} | l_3: {:.5f}".format(self.d_x, self.d_y, self.l_1_x, self.l_1_y, self.l_2, self.l_3))

        # Part 1: Finding theta_1
        l_a = ((p_y - d_y)**2 + p_z**2)**0.5
        # rho = np.arctan(abs(p_z)/abs(p_y - d_y)) # Causes float division by 0 when p_y = d_y
        rho = np.arcsin(abs(p_z)/l_a)
        beta = np.arccos(abs(l_1_y)/l_a)
        theta_1 = beta - rho
        # Part 2: Finding theta_2 and theta_3
        l_b = np.sqrt(l_a**2 - l_1_y**2)
        l_eff = np.sqrt(l_b**2 + (p_x - d_x - l_1_x)**2)
        theta_3 = np.arccos((l_eff**2 - l_2**2 - l_3**2)/(-2*l_2*l_3)) - PI/2
        gamma = np.arccos((l_3**2 - l_eff**2 - l_2**2)/(-2*l_eff*l_2))
        theta_2 = -(np.arctan2(p_x - d_x - l_1_x, l_b) - gamma + PI/2)

        print("theta_1: {:.5f} rad {:.2f} deg | theta_2: {:.5f} rad {:.2f} deg | theta_3: {:.5f} rad {:.2f} deg".format(theta_1, degrees(theta_1), theta_2, degrees(theta_2), theta_3, degrees(theta_3)))

        return theta_1, theta_2, theta_3

    def moveFoot(self, dest):
        """
        DESCRIPTION:
        Runs ikFoot on given dest and sends a command to the simulator to move the foot to the calculated joint angles.

        ARGUMENTS:
        + dest: A size 3 iterable; the destination coordinates [x, y, z] of the foot.
        RETURNS:
        -nothing-
        """
        theta_1, theta_2, theta_3 = self.ikFoot(dest)
        _ = sim.simxSetJointTargetPosition(self.client_id, self.coxa_joint, theta_1, sim.simx_opmode_streaming)
        _ = sim.simxSetJointTargetPosition(self.client_id, self.femur_joint, theta_2, sim.simx_opmode_streaming)
        _ = sim.simxSetJointTargetPosition(self.client_id, self.tibia_joint, theta_3, sim.simx_opmode_streaming)

    def moveToPhase(self, phase, stride_length = 0.12, swing_height = 0.08, swing_to_stance_ratio = 2):
        """
        DESCRIPTION:
        Moves the leg to the position in its trajectory as dictated by the phase.
        ARGUMENTS:
        + phase: The phase specifying the position in the leg's trajectory to send the foot to.
        """
        if phase >= 2*PI:
            phase = phase - 2*PI

        foot_pos = [0, 0, 0]

        # Transition points:
        phi_1 = PI/(swing_to_stance_ratio + 1)
        phi_2 = ((2*swing_to_stance_ratio + 1)/(swing_to_stance_ratio + 1))*PI

        if (phase >= 0 and phase < phi_1):
            # Stance phase (from-origin half)
            mapped_phase = ((PI/2)/phi_1)*phase
            foot_x = self.foot_origin[0] - stride_length*np.sin(mapped_phase)
            foot_y = self.foot_origin[1]
            foot_z = self.foot_origin[2]
            foot_pos = [foot_x, foot_y, foot_z]
        elif (phase >= phi_1 and phase < phi_2):
            # Swing phase
            mapped_phase = (PI/(phi_2 - phi_1))*(phase - phi_1) + PI/2
            foot_x = self.foot_origin[0] - stride_length*np.sin(mapped_phase)
            foot_y = self.foot_origin[1]
            foot_z = self.foot_origin[2] - swing_height*np.cos(mapped_phase)
            foot_pos = [foot_x, foot_y, foot_z]
        elif (phase >= phi_2 and phase < 2*PI):
            # Stance phase (towards-origin half)
            mapped_phase = ((PI/2)/2*PI - phi_2)*(phase - phi_2) + (3/2)*PI
            foot_x = self.foot_origin[0] - stride_length*np.sin(mapped_phase)
            foot_y = self.foot_origin[1]
            foot_z = self.foot_origin[2]
            foot_pos = [foot_x, foot_y, foot_z]

        # if ((phase >= 0 and phase <= phi_1) or (phase >= phi_2 and phase <= 2*PI)):
        #     # Stance phase
        #     foot_x = self.foot_origin[0] - stride_length*np.sin(phase)
        #     foot_y = self.foot_origin[1]
        #     foot_z = self.foot_origin[2]
        #     foot_pos = [foot_x, foot_y, foot_z]
        # else:
        #     # Swing phase
        #     foot_x = self.foot_origin[0] - stride_length*np.sin(phase)
        #     foot_y = self.foot_origin[1]
        #     foot_z = self.foot_origin[2] - swing_height*np.cos(phase)
        #     foot_pos = [foot_x, foot_y, foot_z]
        
        self.moveFoot(foot_pos)

