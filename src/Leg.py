
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
        self.l_1_y = abs(self.l_1[2])
        self.l_1_x = abs(self.l_1[1])
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
        rho = np.arctan(abs(p_z)/abs(p_y - d_y))
        beta = np.arccos(l_1_y/l_a)
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

    def gaitPhase(self, phase, stride_length = 0.12, swing_height = 0.08, swing_to_stance_speed_ratio = 2):
        """
        DESCRIPTION:
        Moves the leg to the position in its trajectory as dictated by the phase.
        ARGUMENTS:
        + phase: The phase specifying the position in the leg's trajectory to send the foot to.
        """
        foot_pos = [0, 0, 0]

        if ((phase <= PI/2 and phase >= 0) or (phase >= (3/2)*PI and phase <= 2*PI)):
            # Stance phase
            foot_x = self.foot_origin[0] - stride_length*np.sin(phase)
            foot_y = self.foot_origin[1]
            foot_z = self.foot_origin[2]
            foot_pos = [foot_x, foot_y, foot_z]
            # Slower during stance phase
            # TODO: Slow down phase
        else:
            # Swing phase
            foot_x = self.foot_origin[0] - stride_length*np.sin(phase)
            foot_y = self.foot_origin[1]
            foot_z = self.foot_origin[2] - swing_height*np.cos(phase)
            foot_pos = [foot_x, foot_y, foot_z]
            # Faster during swing phase
            # TODO: Speed up phase
        
        self.moveFoot(foot_pos)



if __name__ == "__main__":
    
    ### BEGIN SIM CONNECTION ###
    connection_successful = False
    print("=== Programme START ===")
    sim.simxFinish(-1) # just in case, close all opened connections
    client_id = sim.simxStart("127.0.0.1",19999,True,True,5000,5) # Connect to CoppeliaSim
    # Connection ID of 19997 connects to the simulator without having the simulation running
    if client_id!=-1:
        print ("Connected to remote API server")
        connection_successful = True
    else:
        print ("Failed connecting to remote API server")
        connection_successful = False

    if connection_successful:
        ### GET OBJECT HANDLES ###
        _, body_frame = sim.simxGetObjectHandle(client_id, "body_frame", sim.simx_opmode_blocking)
        # Front left leg
        _, front_left_coxa_joint = sim.simxGetObjectHandle(client_id, "shoulder_front_left", sim.simx_opmode_blocking)
        _, front_left_coxa = sim.simxGetObjectHandle(client_id, "shoulder_front_left_respondable", sim.simx_opmode_blocking)
        _, front_left_femur_joint = sim.simxGetObjectHandle(client_id, "femur_front_left", sim.simx_opmode_blocking)
        _, front_left_femur = sim.simxGetObjectHandle(client_id, "femur_front_left_respondable", sim.simx_opmode_blocking)
        _, front_left_tibia_joint = sim.simxGetObjectHandle(client_id, "tibia_front_left", sim.simx_opmode_blocking)
        _, front_left_tibia = sim.simxGetObjectHandle(client_id, "tibia_front_left_respondable", sim.simx_opmode_blocking)
        _, front_left_foot = sim.simxGetObjectHandle(client_id, "foot_front_left", sim.simx_opmode_blocking)

        ### SETUP ###
        leg = Leg(
            client_id,
            body_frame,
            front_left_coxa_joint,
            front_left_coxa,
            front_left_femur_joint,
            front_left_femur,
            front_left_tibia_joint,
            front_left_tibia,
            front_left_foot,
            foot_origin = [0.2, 0.1, -0.225]
        )

        print("Setup done, entering while loop...")
    
        # leg.moveFoot([0.19302406907081604, 0.11094817519187927, -0.22628307342529297]) # Corresponds to joint angles (in degrees) [0, -45, 0]
        leg.moveFoot([0.14550861716270447, 0.046028509736061096, -0.1908787190914154]) # Corresponds to joint angles (in degrees) [-20, -20, -20]
        time.sleep(0.5)
        _, foot_pos = sim.simxGetObjectPosition(client_id, leg.foot, leg.body_frame, sim.simx_opmode_blocking)
        print("foot_pos: {0}".format(foot_pos))

        # TODO: For some reason there seems to be a massive loss of precision in the IK (a consistent error of up to 7 degress in the IK results)

        ### CLOSE CONNECTION TO SIM ###
        sim.simxGetPingTime(client_id)
        sim.simxFinish(client_id)
    print("=== Programme end ===")