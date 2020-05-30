
import sim

import numpy as np
from numpy.linalg import norm
from math import pi as PI

from enum import Enum

from Leg import Leg
from IMU import IMU

from simple_pid import PID

class Mode(Enum):
    REST = 0
    TROT = 1

class Robot:
    def __init__(
        self,
        client_id,
        stance_polygon_length = 0.4,
        stance_polygon_width = 0.2,
        stance_height = 0.225,
        swing_height = 0.08
    ):
        
        self.client_id = client_id

        ### GET OBJECT HANDLES ###
        _, self.body_frame = sim.simxGetObjectHandle(client_id, "body_frame", sim.simx_opmode_blocking)
        _, self.body = sim.simxGetObjectHandle(client_id, "base_link_respondable", sim.simx_opmode_blocking)
        # Front left leg
        _, front_left_coxa_joint = sim.simxGetObjectHandle(client_id, "shoulder_front_left", sim.simx_opmode_blocking)
        _, front_left_coxa = sim.simxGetObjectHandle(client_id, "shoulder_front_left_respondable", sim.simx_opmode_blocking)
        _, front_left_femur_joint = sim.simxGetObjectHandle(client_id, "femur_front_left", sim.simx_opmode_blocking)
        _, front_left_femur = sim.simxGetObjectHandle(client_id, "femur_front_left_respondable", sim.simx_opmode_blocking)
        _, front_left_tibia_joint = sim.simxGetObjectHandle(client_id, "tibia_front_left", sim.simx_opmode_blocking)
        _, front_left_tibia = sim.simxGetObjectHandle(client_id, "tibia_front_left_respondable", sim.simx_opmode_blocking)
        _, front_left_foot = sim.simxGetObjectHandle(client_id, "foot_front_left", sim.simx_opmode_blocking)
        # Front right leg
        _, front_right_coxa_joint = sim.simxGetObjectHandle(client_id, "shoulder_front_right", sim.simx_opmode_blocking)
        _, front_right_coxa = sim.simxGetObjectHandle(client_id, "shoulder_front_right_respondable", sim.simx_opmode_blocking)
        _, front_right_femur_joint = sim.simxGetObjectHandle(client_id, "femur_front_right", sim.simx_opmode_blocking)
        _, front_right_femur = sim.simxGetObjectHandle(client_id, "femur_front_right_respondable", sim.simx_opmode_blocking)
        _, front_right_tibia_joint = sim.simxGetObjectHandle(client_id, "tibia_front_right", sim.simx_opmode_blocking)
        _, front_right_tibia = sim.simxGetObjectHandle(client_id, "tibia_front_right_respondable", sim.simx_opmode_blocking)
        _, front_right_foot = sim.simxGetObjectHandle(client_id, "foot_front_right", sim.simx_opmode_blocking)
        # Back left leg
        _, back_left_coxa_joint = sim.simxGetObjectHandle(client_id, "shoulder_rear_left", sim.simx_opmode_blocking)
        _, back_left_coxa = sim.simxGetObjectHandle(client_id, "shoulder_rear_left_respondable", sim.simx_opmode_blocking)
        _, back_left_femur_joint = sim.simxGetObjectHandle(client_id, "femur_rear_left", sim.simx_opmode_blocking)
        _, back_left_femur = sim.simxGetObjectHandle(client_id, "femur_rear_left_respondable", sim.simx_opmode_blocking)
        _, back_left_tibia_joint = sim.simxGetObjectHandle(client_id, "tibia_rear_left", sim.simx_opmode_blocking)
        _, back_left_tibia = sim.simxGetObjectHandle(client_id, "tibia_rear_left_respondable", sim.simx_opmode_blocking)
        _, back_left_foot = sim.simxGetObjectHandle(client_id, "foot_rear_left", sim.simx_opmode_blocking)
        # Back right leg
        _, back_right_coxa_joint = sim.simxGetObjectHandle(client_id, "shoulder_rear_right", sim.simx_opmode_blocking)
        _, back_right_coxa = sim.simxGetObjectHandle(client_id, "shoulder_rear_right_respondable", sim.simx_opmode_blocking)
        _, back_right_femur_joint = sim.simxGetObjectHandle(client_id, "femur_rear_right", sim.simx_opmode_blocking)
        _, back_right_femur = sim.simxGetObjectHandle(client_id, "femur_rear_right_respondable", sim.simx_opmode_blocking)
        _, back_right_tibia_joint = sim.simxGetObjectHandle(client_id, "tibia_rear_right", sim.simx_opmode_blocking)
        _, back_right_tibia = sim.simxGetObjectHandle(client_id, "tibia_rear_right_respondable", sim.simx_opmode_blocking)
        _, back_right_foot = sim.simxGetObjectHandle(client_id, "foot_rear_right", sim.simx_opmode_blocking)

        # Legs
        self.front_left_leg = Leg(
            client_id,
            self.body_frame,
            front_left_coxa_joint,
            front_left_coxa,
            front_left_femur_joint,
            front_left_femur,
            front_left_tibia_joint,
            front_left_tibia,
            front_left_foot
        )
        self.front_right_leg = Leg(
            client_id,
            self.body_frame,
            front_right_coxa_joint,
            front_right_coxa,
            front_right_femur_joint,
            front_right_femur,
            front_right_tibia_joint,
            front_right_tibia,
            front_right_foot
        )
        self.back_left_leg = Leg(
            client_id,
            self.body_frame,
            back_left_coxa_joint,
            back_left_coxa,
            back_left_femur_joint,
            back_left_femur,
            back_left_tibia_joint,
            back_left_tibia,
            back_left_foot
        )
        self.back_right_leg = Leg(
            client_id,
            self.body_frame,
            back_right_coxa_joint,
            back_right_coxa,
            back_right_femur_joint,
            back_right_femur,
            back_right_tibia_joint,
            back_right_tibia,
            back_right_foot
        )

        # Robot state
        self.body_velocity = np.zeros((4))
        self.body_roll = 0 # In radians
        self.body_pitch = 0 # In radians
        self.body_yaw = 0 # In radians
        self.stance_polygon_length = stance_polygon_length
        self.stance_polygon_width = stance_polygon_width
        self.stance_height = stance_height
        self.foot_locations_wrt_body_true = np.zeros((3, 4)) # 3x4, Each column represents a foot
        self.foot_locations_wrt_body_assuming_no_body_rpy = np.zeros((3, 4))
        self.foot_velocities_wrt_body = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))
        self.contact_pattern = np.ones((4)) # By default, all legs in stance
        self.ticks = 0
        self.mode = Mode.REST

        self.p_b_vpsp = np.zeros((3))
        self.virtual_points = np.zeros((4, 2, 3))
        self.vpsp_vertices = np.zeros((3, 4))

        self.foot_locations_wrt_body_at_rest = np.concatenate( \
            ( \
                np.array([[self.stance_polygon_length/2, self.stance_polygon_width/2, -self.stance_height]]).T, \
                np.array([[self.stance_polygon_length/2, -self.stance_polygon_width/2, -self.stance_height]]).T, \
                np.array([[-self.stance_polygon_length/2, self.stance_polygon_width/2, -self.stance_height]]).T, \
                np.array([[-self.stance_polygon_length/2, -self.stance_polygon_width/2, -self.stance_height]]).T, \
            ),
            axis = 1
        )

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

    
