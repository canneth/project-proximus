
import pybullet

from pathlib import Path

import numpy as np
from numpy.linalg import norm
from math import pi as PI

from GlobalConstants import Mode
from GlobalConstants import Gait
from GlobalConstants import FootTrajectory

from Leg import Leg
from IMU import IMU

class Robot:
    def __init__(
        self,
        stance_polygon_length = 0.4,
        stance_polygon_width = 0.2,
        stance_height = 0.225
    ):
        
        self.robot_urdf_path = Path("..") / "sim_model" / "A001_full_assem" / "urdf" / "A001_full_assem.urdf"
        self.sim_id = pybullet.loadURDF(
            fileName = str(self.robot_urdf_path),
            basePosition = [0, 0, 0.3],
            useFixedBase = 0
        )
        self.num_of_joints = pybullet.getNumJoints(self.sim_id)
        self.sim_leg_index_dict = (
            {pybullet.getJointInfo(bodyUniqueId = self.sim_id, jointIndex = i)[1].decode("ascii") : i for i in range(self.num_of_joints)}
        )

        # Legs
        self.front_left_leg = Leg(
            robot_sim_id = self.sim_id,
            coxa_joint_sim_id = self.sim_leg_index_dict["front_left_j1"],
            femur_joint_sim_id = self.sim_leg_index_dict["front_left_j2"],
            tibia_joint_sim_id = self.sim_leg_index_dict["front_left_j3"],
            d_x = 0.155,
            d_y = 0.04,
            d_j2_j1_bx = 0.038,
            d_j2_j1_by = 0.071,
            l_2 = 0.160,
            l_3 = 0.160,
            joint_servo_directions = [1, -1, 1]
        )
        self.front_right_leg = Leg(
            robot_sim_id = self.sim_id,
            coxa_joint_sim_id = self.sim_leg_index_dict["front_right_j1"],
            femur_joint_sim_id = self.sim_leg_index_dict["front_right_j2"],
            tibia_joint_sim_id = self.sim_leg_index_dict["front_right_j3"],
            d_x = 0.155,
            d_y = -0.04,
            d_j2_j1_bx = 0.038,
            d_j2_j1_by = -0.071,
            l_2 = 0.160,
            l_3 = 0.160,
            joint_servo_directions = [-1, 1, -1]
        )
        self.back_left_leg = Leg(
            robot_sim_id = self.sim_id,
            coxa_joint_sim_id = self.sim_leg_index_dict["back_left_j1"],
            femur_joint_sim_id = self.sim_leg_index_dict["back_left_j2"],
            tibia_joint_sim_id = self.sim_leg_index_dict["back_left_j3"],
            d_x = -0.155,
            d_y = 0.04,
            d_j2_j1_bx = -0.038,
            d_j2_j1_by = 0.071,
            l_2 = 0.160,
            l_3 = 0.160,
            joint_servo_directions = [-1, 1, -1]
        )
        self.back_right_leg = Leg(
            robot_sim_id = self.sim_id,
            coxa_joint_sim_id = self.sim_leg_index_dict["back_right_j1"],
            femur_joint_sim_id = self.sim_leg_index_dict["back_right_j2"],
            tibia_joint_sim_id = self.sim_leg_index_dict["back_right_j3"],
            d_x = -0.155,
            d_y = -0.04,
            d_j2_j1_bx = -0.038,
            d_j2_j1_by = -0.071,
            l_2 = 0.160,
            l_3 = 0.160,
            joint_servo_directions = [1, -1, 1]
        )

        # Robot state
        self.stance_polygon_length = stance_polygon_length
        self.stance_polygon_width = stance_polygon_width
        self.stance_height = stance_height
        self.body_roll = 0.0 # In radians
        self.body_pitch = 0.0 # In radians
        self.body_yaw = 0.0 # In radians
        self.body_velocity = np.zeros((3))

        self.foot_locations_wrt_body_true = np.zeros((3, 4)) # 3x4, Each column represents a foot
        self.foot_locations_wrt_body_assuming_no_body_rpy = np.zeros((3, 4))
        self.foot_locations_wrt_body_at_rest = np.zeros((3, 4))

        self.foot_velocities_wrt_body = np.zeros((3, 4))

        self.joint_angles = np.zeros((3, 4))
        self.contact_pattern = np.ones((4)) # By default, all legs in stance

        self.p_b_vpsp = np.zeros((3))
        self.virtual_points = np.zeros((4, 2, 3))
        self.vpsp_vertices = np.zeros((3, 4))

    def __repr__(self):
        print("An instance of the custom Robot class.")

    def updateFootLocationsAssumingNoBodyRPY(self, foot_locations_wrt_body):
        """
        DESCRIPTION:
        Since the trajectory of the foot without taking into account body rpy is used as a reference to compute the
        true foot positions during the swing phase, self.foot_locations_wrt_body_assuming_no_body_rpy needs to be maintained
        and updated separately from the true foot locations, at the same time.
        This function updates self.foot_locations_wrt_body_assuming_no_body_rpy needs with the specified foot locations wrt body.

        ARGUMENTS:
        + foot_locations_wrt_body: A (3, 4) array; the foot locations sans body rpy.
        """
        self.foot_locations_wrt_body_assuming_no_body_rpy = foot_locations_wrt_body

    def moveAllFeet(self, foot_locations_wrt_body):
        """
        DESCRIPTION:
        Moves all feet to their respective locations as specified in foot_locations_wrt_body, where
        each column represents the x, y, z coordinates of each foot.

        ARUGMENTS:
        + foot_locations_wrt_body: A (3, 4) array; Each column represents the x, y, z coordinates of the foot, [FL, FR, BL, BR].
        """
        self.front_left_leg.moveFoot(foot_locations_wrt_body[:, 0])
        self.front_right_leg.moveFoot(foot_locations_wrt_body[:, 1])
        self.back_left_leg.moveFoot(foot_locations_wrt_body[:, 2])
        self.back_right_leg.moveFoot(foot_locations_wrt_body[:, 3])
        # Update robot attributes
        self.foot_locations_wrt_body_true = foot_locations_wrt_body

    
