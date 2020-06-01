
import pybullet

import numpy as np
from numpy.linalg import norm
from math import radians, degrees
from math import pi as PI

import time

import matplotlib.pyplot as plt

class Leg:
    def __init__(
        self,
        robot_sim_id,
        coxa_joint_sim_id,
        femur_joint_sim_id,
        tibia_joint_sim_id,
        d_x,
        d_y,
        d_j2_j1_bx,
        d_j2_j1_by,
        l_2,
        l_3,
        joint_servo_directions = [1, 1, 1]
    ):
        """
        + d_x: Displacement of coxa_joint from body_frame along body_frame x-axis.
        + d_y: Displacement of coxa_joint from body_frame along body_frame y-axis.
        + d_j2_j1_bx: Displacement of femur_joint from coxa_joint along body_frame x-axis at home configuration.
        + d_j2_j1_by: Displacement of femur_joint from coxa_joint along body_frame y-axis at home configuration.
        + l_2: Length of femur.
        + l_3: Length of tibia.
        """
        self.robot_sim_id = robot_sim_id
        self.coxa_joint_sim_id = coxa_joint_sim_id
        self.femur_joint_sim_id = femur_joint_sim_id
        self.tibia_joint_sim_id = tibia_joint_sim_id
        self.d_x = d_x
        self.d_y = d_y
        self.d_j2_j1_bx = d_j2_j1_bx
        self.d_j2_j1_by = d_j2_j1_by
        self.l_2 = l_2
        self.l_3 = l_3
        self.joint_servo_directions = joint_servo_directions

        self.joint_angles = np.zeros((3))

        self.foot_location_wrt_body = np.zeros((3))

        ## SERVO PARAMS ##
        self.servo_max_speed = np.radians(300) # 300 degrees/s
        self.servo_max_torque = 10 # in Nm

        ## MISC ##
        self.sim_joint_angle_correction = np.radians(-12)

        ## Pybullet settings ##
        pybullet.changeDynamics(
            bodyUniqueId = self.robot_sim_id,
            linkIndex = self.tibia_joint_sim_id, # sim_id of link is the same as that of joint; this ensures little to no foot slippage
            lateralFriction = 2.0
        )

    def moveCoxaToAngle(self, angle):
        """
        DESCRIPTION:
        Updates coxa angle attribute theta_1 and sends a command to move the
        corresponding joint to the specified angle.

        ARGUMENTS:
        + angle: Angle to move the coxa to, in radians.
        """
        self.joint_angles[0] = angle
        pybullet.setJointMotorControlArray(
            bodyUniqueId = self.robot_sim_id,
            jointIndices = [self.coxa_joint_sim_id],
            controlMode = pybullet.POSITION_CONTROL,
            targetPositions = [self.joint_servo_directions[0]*self.joint_angles[0] + self.sim_joint_angle_correction],
            targetVelocities = [self.servo_max_speed],
            forces = [self.servo_max_torque]
        )
    def moveFemurToAngle(self, angle):
        """
        DESCRIPTION:
        Updates femur angle attribute theta_2 and sends a command to move the
        corresponding joint to the specified angle.

        ARGUMENTS:
        + angle: Angle to move the femur to, in radians.
        """
        self.joint_angles[1] = angle
        pybullet.setJointMotorControlArray(
            bodyUniqueId = self.robot_sim_id,
            jointIndices = [self.femur_joint_sim_id],
            controlMode = pybullet.POSITION_CONTROL,
            targetPositions = [self.joint_servo_directions[1]*self.joint_angles[1] + self.sim_joint_angle_correction],
            targetVelocities = [self.servo_max_speed],
            forces = [self.servo_max_torque]
        )
    def moveTibiaToAngle(self, angle):
        """
        DESCRIPTION:
        Updates tibia angle attribute theta_3 and sends a command to move the
        corresponding joint to the specified angle.

        ARGUMENTS:
        + angle: Angle to move the tibia to, in radians.
        """
        self.joint_angles[2] = angle
        pybullet.setJointMotorControlArray(
            bodyUniqueId = self.robot_sim_id,
            jointIndices = [self.tibia_joint_sim_id],
            controlMode = pybullet.POSITION_CONTROL,
            targetPositions = [self.joint_servo_directions[2]*self.joint_angles[2] + self.sim_joint_angle_correction],
            targetVelocities = [self.servo_max_speed],
            forces = [self.servo_max_torque]
        )

    def moveLegToAngles(self, joint_angles):
        """
        DESCRIPTION:
        Moves all joints in the legs to those specified in joint_angles.
        
        ARGUMENTS:
        + joint_angles: A (3,) array; the joint angles to move to, [j1, j2, j3].
        """
        self.moveCoxaToAngle(joint_angles[0])
        self.moveFemurToAngle(joint_angles[1])
        self.moveTibiaToAngle(joint_angles[2])

    def ikFoot(self, dest):
        # NOTE: 
        # For some reason, there seems to be significant preicse inaccuracy in the IK results
        # (up to about 7 degrees in joint angle errors), but the general position is thereabouts.
        # I realise that the origins of the femur and tibia in the URDF and math model used here
        # do not agree. I suppose that's where the inaccuracy stems from.
        # The URDF generator for SW doesn't work as it should...
        """
        DESCRIPTION:
        Calculates the joint angles required to bring the foot to the dest vector, defined with respect to the body_frame.
        Updates own joint angle attributes.

        ARGUMENTS:
        + dest: A size 3 iterable; the destination coordinates [x, y, z] of the foot.
        
        RETURNS:
        + self.joint_angles: The joint angles, in radians, that bring the foot to the target specified by dest. [j1, j2, j3]
        """
        d_x = self.d_x
        d_y = self.d_y
        d_j2_j1_by = self.d_j2_j1_by
        d_j2_j1_bx = self.d_j2_j1_bx
        l_2 = self.l_2
        l_3 = self.l_3

        p_x = dest[0]
        p_y = dest[1]
        p_z = dest[2]
        p_yz = np.sqrt(p_y**2 + p_z**2)

        # Part 1: Finding theta_1
        l_a = ((p_y - d_y)**2 + p_z**2)**0.5
        # rho = np.arctan(abs(p_z)/abs(p_y - d_y)) # Causes float division by 0 when p_y = d_y
        rho = np.arcsin(abs(p_z)/l_a)
        beta = np.arccos(abs(d_j2_j1_by)/l_a)
        self.joint_angles[0] = beta - rho
        # Part 2: Finding theta_2 and theta_3
        l_b = np.sqrt(l_a**2 - d_j2_j1_by**2)
        l_eff = np.sqrt(l_b**2 + (p_x - d_x - d_j2_j1_bx)**2)
        self.joint_angles[2] = (np.arccos((l_eff**2 - l_2**2 - l_3**2)/(-2*l_2*l_3)) - PI/2)
        gamma = np.arccos((l_3**2 - l_eff**2 - l_2**2)/(-2*l_eff*l_2))
        self.joint_angles[1] = (np.arctan2(p_x - d_x - d_j2_j1_bx, l_b) - gamma + PI/2)

        return self.joint_angles

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
        self.foot_location_wrt_body = np.array(dest).reshape(3)
        self.moveLegToAngles(self.joint_angles)