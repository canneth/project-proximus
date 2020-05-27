
import sim

import numpy as np
from numpy.linalg import norm
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
        foot
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

        self.theta_1 = 0
        self.theta_2 = 0
        self.theta_3 = 0

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

        self.foot_location_wrt_body = np.zeros((3))

    def moveCoxaToAngle(self, angle):
        """
        DESCRIPTION:
        Updates coxa angle attribute theta_1 and sends a command to move the
        corresponding joint to the specified angle.

        ARGUMENTS:
        + angle: Angle to move the coxa to, in radians.
        """
        self.theta_1 = angle
        sim.simxSetJointTargetPosition(self.client_id, self.coxa_joint, self.theta_1, sim.simx_opmode_streaming)
    def moveFemurToAngle(self, angle):
        """
        DESCRIPTION:
        Updates femur angle attribute theta_2 and sends a command to move the
        corresponding joint to the specified angle.

        ARGUMENTS:
        + angle: Angle to move the femur to, in radians.
        """
        self.theta_2 = angle
        sim.simxSetJointTargetPosition(self.client_id, self.femur_joint, self.theta_2, sim.simx_opmode_streaming)
    def moveTibiaToAngle(self, angle):
        """
        DESCRIPTION:
        Updates tibia angle attribute theta_3 and sends a command to move the
        corresponding joint to the specified angle.

        ARGUMENTS:
        + angle: Angle to move the tibia to, in radians.
        """
        self.theta_3 = angle
        sim.simxSetJointTargetPosition(self.client_id, self.tibia_joint, self.theta_3, sim.simx_opmode_streaming)

    def ikFoot(self, dest):
        # NOTE: For some reason, there seems to be significant in-precision in the IK results (up to about 7 degrees in joint angle errors), but the general position is thereabouts.
        """
        DESCRIPTION:
        Calculates the joint angles required to bring the foot to the dest vector, defined with respect to the body_frame.
        Updates own joint angle attributes.

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

        # print("p_x: {:.5f} | p_y: {:.5f} | p_z: {:.5f}".format(p_x, p_y, p_z))
        # print("d_x: {:.5f} | d_y: {:.5f} | l_1_x: {:.5f} | l_1_y: {:.5f} | l_2: {:.5f} | l_3: {:.5f}".format(self.d_x, self.d_y, self.l_1_x, self.l_1_y, self.l_2, self.l_3))

        # Part 1: Finding theta_1
        l_a = ((p_y - d_y)**2 + p_z**2)**0.5
        # rho = np.arctan(abs(p_z)/abs(p_y - d_y)) # Causes float division by 0 when p_y = d_y
        rho = np.arcsin(abs(p_z)/l_a)
        beta = np.arccos(abs(l_1_y)/l_a)
        self.theta_1 = beta - rho
        # Part 2: Finding theta_2 and theta_3
        l_b = np.sqrt(l_a**2 - l_1_y**2)
        l_eff = np.sqrt(l_b**2 + (p_x - d_x - l_1_x)**2)
        self.theta_3 = np.arccos((l_eff**2 - l_2**2 - l_3**2)/(-2*l_2*l_3)) - PI/2
        gamma = np.arccos((l_3**2 - l_eff**2 - l_2**2)/(-2*l_eff*l_2))
        self.theta_2 = -(np.arctan2(p_x - d_x - l_1_x, l_b) - gamma + PI/2)

        # print("theta_1: {:.5f} rad {:.2f} deg | theta_2: {:.5f} rad {:.2f} deg | theta_3: {:.5f} rad {:.2f} deg".format(theta_1, degrees(theta_1), theta_2, degrees(theta_2), theta_3, degrees(theta_3)))

        return self.theta_1, self.theta_2, self.theta_3

    def moveFoot(self, dest):
        """
        DESCRIPTION:
        Runs ikFoot on given dest and sends a command to the simulator to move the foot to the calculated joint angles.
        Updates own joint angle and foot location attributes.

        ARGUMENTS:
        + dest: A size 3 iterable; the destination coordinates [x, y, z] of the foot.

        RETURNS:
        -nothing-
        """
        self.ikFoot(dest)
        self.foot_location_wrt_body[0] = dest[0]
        self.foot_location_wrt_body[1] = dest[1]
        self.foot_location_wrt_body[2] = dest[2]
        _ = sim.simxSetJointTargetPosition(self.client_id, self.coxa_joint, self.theta_1, sim.simx_opmode_streaming)
        _ = sim.simxSetJointTargetPosition(self.client_id, self.femur_joint, self.theta_2, sim.simx_opmode_streaming)
        _ = sim.simxSetJointTargetPosition(self.client_id, self.tibia_joint, self.theta_3, sim.simx_opmode_streaming)