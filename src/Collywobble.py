
import sim

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

        # PID controllers
        self.x_accel_pid = PID(0.005, 0.05, 0, setpoint = 0)
        self.y_accel_pid = PID(0.005, 0.05, 0, setpoint = 0)
        self.x_accel_pid.output_limits = (-0.1, 0.1)
        self.y_accel_pid.output_limits = (-0.1, 0.1)
        self.x_gyro_pid = PID(0.005, 0.05, 0, setpoint = 0) # TODO: Tune this! (P too strong)
        self.y_gyro_pid = PID(0.005, 0.05, 0, setpoint = 0) # TODO: Tune this! (P too strong)
        self.x_gyro_pid.output_limits = (-0.1, 0.1)
        self.y_gyro_pid.output_limits = (-0.1, 0.1)

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

    def moveToPhaseInTrotGait(self, phase, stride_length = 0, swing_height = 0.08, swing_to_stance_ratio = 0.2):
        """
        DESCRIPTION:
        Move all legs to their respective phases to effect a trot gait, coordinated by the specified gait phase.

        ARGUMENTS:
        + phase: The phase angle dictating the phase of the gait.
        """
        print("### Moving front_left_leg ###")
        self.front_left_leg.moveToPhase(phase = phase, stride_length = stride_length, swing_height = swing_height, swing_to_stance_ratio = swing_to_stance_ratio)
        print("### Moving front_right_leg ###")
        self.front_right_leg.moveToPhase(phase = phase + PI, stride_length = stride_length, swing_height = swing_height, swing_to_stance_ratio = swing_to_stance_ratio)
        print("### Moving back_left_leg ###")
        self.back_left_leg.moveToPhase(phase = phase + PI, stride_length = stride_length, swing_height = swing_height, swing_to_stance_ratio = swing_to_stance_ratio)
        print("### Moving back_right_leg ###")
        self.back_right_leg.moveToPhase(phase = phase, stride_length = stride_length, swing_height = swing_height, swing_to_stance_ratio = swing_to_stance_ratio)

    def moveToPhaseInTrotGaitStabilised(self, phase, stride_length = 0, swing_height = 0.08, swing_to_stance_ratio = 0.2):
        """
        DESCIRPTION:
        Moves all legs to their respective phases to effect a trot gait, where each foot's position is augmented by stability controllers.

        ARGUMENTS:
        + phase: The phase in the gait cycle.
        """
        front_left_foot_neutral_pos = self.front_left_leg.getFootPositionAtPhase(phase = phase, stride_length = stride_length, swing_height = swing_height, swing_to_stance_ratio = swing_to_stance_ratio)
        front_left_foot_pos = self.translationStabilityController(front_left_foot_neutral_pos) # Apply translation stability controller
        front_left_foot_pos = self.rotationStabilityController(front_left_foot_pos) # Apply rotation stability controller

        front_right_foot_neutral_pos = self.front_right_leg.getFootPositionAtPhase(phase = phase + PI, stride_length = stride_length, swing_height = swing_height, swing_to_stance_ratio = swing_to_stance_ratio)
        front_right_foot_pos = self.translationStabilityController(front_right_foot_neutral_pos) # Apply translation stability controller
        front_right_foot_pos = self.rotationStabilityController(front_right_foot_pos) # Apply rotation stability controller

        back_left_foot_neutral_pos = self.back_left_leg.getFootPositionAtPhase(phase = phase + PI, stride_length = stride_length, swing_height = swing_height, swing_to_stance_ratio = swing_to_stance_ratio)
        back_left_foot_pos = self.translationStabilityController(back_left_foot_neutral_pos) # Apply translation stability controller
        back_left_foot_pos = self.rotationStabilityController(back_left_foot_pos) # Apply rotation stability controller

        back_right_foot_neutral_pos = self.back_right_leg.getFootPositionAtPhase(phase = phase, stride_length = stride_length, swing_height = swing_height, swing_to_stance_ratio = swing_to_stance_ratio)
        back_right_foot_pos = self.translationStabilityController(back_right_foot_neutral_pos) # Apply translation stability controller
        back_right_foot_pos = self.rotationStabilityController(back_right_foot_pos) # Apply rotation stability controller

        self.front_left_leg.moveFoot(front_left_foot_pos)
        self.front_right_leg.moveFoot(front_right_foot_pos)
        self.back_left_leg.moveFoot(back_left_foot_pos)
        self.back_right_leg.moveFoot(back_right_foot_pos)

    # Create a controller that shifts the coordinates of each foot in real-time...
    # 2 sensors will contribute to these augmentations
    # Accelerometer: As acceleration increases in one direction, foot coordinates will move in the opposite direction
    # Gyroscope: As rate of rotation increases in one direction, foot coordinates will move to oppose that motion

    # TODO: Grab accelerometer data and learn how to clean it

    def translationStabilityController(self, neutral_foot_pos):
        """
        DESCRIPTION:
        Takes in the neutral_foot_pos that the foot is supposed to be in undisturbed, and applies an offset based on
        linear acceleration disturbances on the robot. The resulting foot_pos is returned.
        
        ARGUMENTS:
        + neutral_foot_pos: The undisturbed position of the foot.
        RETURNS:
        + foot_pos: A size 3 list [x, y, z]; the augmented foot_pos coordinates to counter-act the linear acceleration disturbances.
        """
        foot_pos = neutral_foot_pos.copy()
        # Control in x-direction
        accel_x = self.accelerometer.getX()
        x_control_value = self.x_accel_pid(accel_x)
        foot_pos[0] = neutral_foot_pos[0] + x_control_value
        # Control in y-direction
        accel_y = self.accelerometer.getY()
        y_control_value = self.y_accel_pid(accel_y)
        foot_pos[1] = neutral_foot_pos[1] + y_control_value
        
        # print("accel_x: {} | x_control_value: {} | accel_y: {} | y_control_value: {}".format(accel_x, x_control_value, accel_y, y_control_value))

        return foot_pos

    def rotationStabilityController(self, neutral_foot_pos):
        """
        DESCRIPTION:
        Takes in the neutral_foot_pos that the foot is supposed to be in undisturbed, and applies an offset based on
        rotational disturbances on the robot. The resulting foot_pos is returned.
        
        ARGUMENTS:
        + neutral_foot_pos: The undisturbed position of the foot.
        RETURNS:
        + foot_pos: A size 3 list [x, y, z]; the augmented foot_pos coordinates to counter-act the rotational disturbances.
        """
        foot_pos = neutral_foot_pos.copy()
        # Control in x-direction
        gyro_y_accel = self.gyro.getYAccel()
        x_control_value = self.x_gyro_pid(gyro_y_accel) # When robot pitches forwards, we want the feet to move further forwards (in the +x direction)
        foot_pos[0] = neutral_foot_pos[0] + x_control_value
        # Control in y-direction
        gyro_x_accel = self.gyro.getXAccel()
        y_control_value = self.y_gyro_pid(gyro_x_accel) # When robot rolls to the right, we want the feet to move further right-wards (in the -y direction)
        foot_pos[1] = neutral_foot_pos[1] - y_control_value
        
        print("gyro_y_accel: {} | x_control_value: {} | gyro_x_accel: {} | y_control_value: {}".format(gyro_y_accel, x_control_value, gyro_x_accel, y_control_value))

        return foot_pos

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