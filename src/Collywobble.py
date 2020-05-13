
import sim

import numpy as np
from numpy.linalg import norm
from math import pi as PI

from Leg import Leg
from Accelerometer import Accelerometer
from Gyrosensor import Gyrosensor

from simple_pid import PID

class Collywobble:
    def __init__(
        self,
        client_id,
        stance_polygon_length = 0.4,
        stance_polygon_width = 0.2,
        stance_height = 0.225,
        stride_length = 0.120,
        swing_height = 0.08,
        swing_to_stance_ratio = 2,
    ):
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

        # Sensors
        self.accelerometer = Accelerometer(client_id)
        self.gyro = Gyrosensor(client_id)

        self.client_id = client_id

        self.stance_polygon_length = stance_polygon_length
        self.stance_polygon_width = stance_polygon_width
        self.stance_height = stance_height

        self.front_left_leg.foot_origin = [self.stance_polygon_length/2, self.stance_polygon_width/2, -self.stance_height]
        self.front_right_leg.foot_origin = [self.stance_polygon_length/2, -self.stance_polygon_width/2, -self.stance_height]
        self.back_left_leg.foot_origin = [-self.stance_polygon_length/2, self.stance_polygon_width/2, -self.stance_height]
        self.back_right_leg.foot_origin = [-self.stance_polygon_length/2, -self.stance_polygon_width/2, -self.stance_height]

    def __repr__(self):
        print("An instance of the custom Collywobble class.")

    def moveToPhaseInTrotTranslate(self, phase, direction_vector = [1, 0], stride_length = 0, swing_height = 0.08, swing_to_stance_ratio = 0.2):
        """
        DESCIRPTION:
        Moves all legs to their respective phases to effect a trot gait, where each foot's position is augmented by stability controllers.

        ARGUMENTS:
        + phase: The phase in the gait cycle.
        + direction_vector: The direction in which to translate. The direction vector is normalised within this function.
        """
        direction_vec = direction_vector/norm(direction_vector)
        # Raw gait foot positions
        front_left_foot_neutral_pos = self.front_left_leg.getFootPositionAtPhase(phase = phase, direction_vector = direction_vec, stride_length = stride_length, swing_height = swing_height, swing_to_stance_ratio = swing_to_stance_ratio)
        front_right_foot_neutral_pos = self.front_right_leg.getFootPositionAtPhase(phase = phase + PI, direction_vector = direction_vec, stride_length = stride_length, swing_height = swing_height, swing_to_stance_ratio = swing_to_stance_ratio)
        back_left_foot_neutral_pos = self.back_left_leg.getFootPositionAtPhase(phase = phase + PI, direction_vector = direction_vec, stride_length = stride_length, swing_height = swing_height, swing_to_stance_ratio = swing_to_stance_ratio)
        back_right_foot_neutral_pos = self.back_right_leg.getFootPositionAtPhase(phase = phase, direction_vector = direction_vec, stride_length = stride_length, swing_height = swing_height, swing_to_stance_ratio = swing_to_stance_ratio)
        
        # Apply controller offsets
        front_left_foot_pos, front_right_foot_pos, back_left_foot_pos, back_right_foot_pos = \
            self.rpyController(
                front_left_foot_neutral_pos,
                front_right_foot_neutral_pos,
                back_left_foot_neutral_pos,
                back_right_foot_neutral_pos
            )

        self.front_left_leg.moveFoot(front_left_foot_pos)
        self.front_right_leg.moveFoot(front_right_foot_pos)
        self.back_left_leg.moveFoot(back_left_foot_pos)
        self.back_right_leg.moveFoot(back_right_foot_pos)

    def rpyController(
        self,
        front_left_foot_pos_before,
        front_right_foot_pos_before,
        back_left_foot_pos_before,
        back_right_foot_pos_before
    ):
        """
        DESCRIPTION:
        Applies an offset on the feet target positions based on linear acceleration disturbances on the robot.
        The resulting foot_pos's are returned.
        
        ARGUMENTS:
        + front_left_foot_pos_before: A size 3 list [x, y, z]; the foot coordinates, to be augmented.
        + front_right_foot_pos_before: A size 3 list [x, y, z]; the foot coordinates, to be augmented.
        + back_left_foot_pos_before: A size 3 list [x, y, z]; the foot coordinates, to be augmented.
        + back_right_foot_pos_before: A size 3 list [x, y, z]; the foot coordinates, to be augmented.
        RETURNS:
        + front_left_foot_pos: A size 3 list [x, y, z]; the augmented foot_pos coordinates.
        + front_right_foot_pos: A size 3 list [x, y, z]; the augmented foot_pos coordinates.
        + back_left_foot_pos: A size 3 list [x, y, z]; the augmented foot_pos coordinates.
        + back_right_foot_pos: A size 3 list [x, y, z]; the augmented foot_pos coordinates.
        """
        # Accelerometer data is opposite of acceleration of body: -ve data means body is accelerating in the +ve direction.
        # Hence, when accel reads +ve, it means body is accelerating in the -ve direction.

        accel_x = self.accelerometer.getX()
        accel_y = self.accelerometer.getY()

        roll_pid = PID(0.004, 0.003, 0, setpoint = 0)
        pitch_pid = PID(0.0001, 0.0001, 0, setpoint = 0)
        roll_control_value = roll_pid(accel_y)
        pitch_control_value = pitch_pid(accel_x)

        front_left_foot_pos = front_left_foot_pos_before.copy()
        front_right_foot_pos = front_right_foot_pos_before.copy()
        back_left_foot_pos = back_left_foot_pos_before.copy()
        back_right_foot_pos = back_right_foot_pos_before.copy()

        ### ROLL CONTROL ###
        # When accel_y is +ve, the robot is rolling towards the left; -ve, rolling towards the right.
        # Hence, when accel_y is +ve, left legs should extend, right legs flex.
        # When accel_y is +ve, the error wrt setpoint is -ve, so roll_control_value will be -ve.
        # When accel_y is -ve, the error wrt setpoint is +ve, so roll_control_value will be +ve.
        # Hence, robot is rolling towards the left, roll_control_value will be -ve.
        # Note that extension of legs is in the -ve z-direction.
        front_left_foot_pos[2] = front_left_foot_pos[2] - (-roll_control_value)
        front_right_foot_pos[2] = front_right_foot_pos[2] + (-roll_control_value)
        back_left_foot_pos[2] = back_left_foot_pos[2] - (-roll_control_value)
        back_right_foot_pos[2] = back_right_foot_pos[2] + (-roll_control_value)

        # ### STANCE-WIDENING APPROACH ###
        # # Widen stance by a factor scaling with error magnitude
        # # Left feet move in the +ve y-direction; Right feet move in the -ve y-direction.
        # stance_width_pid = PID(0.02, 0.004, 0, setpoint = 0)
        # stance_width_augmenting_factor = stance_width_pid(accel_y)
        # front_left_foot_pos[1] = front_left_foot_pos[1] + abs(stance_width_augmenting_factor)
        # front_right_foot_pos[1] = front_right_foot_pos[1] - abs(stance_width_augmenting_factor)
        # back_left_foot_pos[1] = back_left_foot_pos[1] + abs(stance_width_augmenting_factor)
        # back_right_foot_pos[1] = back_right_foot_pos[1] - abs(stance_width_augmenting_factor)

        ### PITCH CONTROL ###
        # When accel_x is +ve, the robot is pitching forwards; -ve, pitching backwards.
        # Hence, when accel_x is +ve, front legs should extend, back legs flex.
        # When accel_x is +ve, the error wrt setpoint is -ve, so pitch_control_value will be -ve.
        # When accel_x is -ve, the error wrt setpoint is +ve, so pitch_control_value will be +ve.
        # Hence, robot is pitching forwards, pitch_control_value will be -ve.
        # Note that extension of legs is in the -ve z-direction.
        front_left_foot_pos[2] = front_left_foot_pos[2] - (-pitch_control_value)
        front_right_foot_pos[2] = front_right_foot_pos[2] - (-pitch_control_value)
        back_left_foot_pos[2] = back_left_foot_pos[2] + (-pitch_control_value)
        back_right_foot_pos[2] = back_right_foot_pos[2] + (-pitch_control_value)
        
        # print("accel_x: {} | x_control_value: {} | accel_y: {} | y_control_value: {}".format(accel_x, x_control_value, accel_y, y_control_value))

        return front_left_foot_pos, front_right_foot_pos, back_left_foot_pos, back_right_foot_pos

    def moveFeetToOrigins(self):
        """
        DESCRIPTION:
        Moves all feet to their trajectory origins.
        """
        self.front_left_leg.moveFootToOrigin()
        self.front_right_leg.moveFootToOrigin()
        self.back_left_leg.moveFootToOrigin()
        self.back_right_leg.moveFootToOrigin()

    def simStep():
        """
        A single step of the simulation.
        To be run in a loop.
        """
        pass