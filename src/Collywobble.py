
import sim

from math import pi as PI

class Collywobble:
    def __init__(
        self,
        client_id,
        world_frame,
        body_frame,
        body,
        front_left_leg,
        front_right_leg,
        back_left_leg,
        back_right_leg,
        stance_polygon_length = 0.4,
        stance_polygon_width = 0.2,
        stance_height = 0.225,
        stride_length = 0.120,
        swing_height = 0.08,
        swing_to_stance_ratio = 2,
    ):

        self.client_id = client_id
        self.world_frame = world_frame
        self.body_frame = body_frame
        self.body = body
        self.front_left_leg = front_left_leg
        self.front_right_leg = front_right_leg
        self.back_left_leg = back_left_leg
        self.back_right_leg = back_right_leg

        self.stance_polygon_length = stance_polygon_length
        self.stance_polygon_width = stance_polygon_width
        self.stance_height = stance_height

        self.stride_length = stride_length
        self.swing_height = swing_height
        self.swing_to_stance_ratio = swing_to_stance_ratio

        self.front_left_leg.foot_origin = [self.stance_polygon_length/2, self.stance_polygon_width/2, -self.stance_height]
        self.front_right_leg.foot_origin = [self.stance_polygon_length/2, -self.stance_polygon_width/2, -self.stance_height]
        self.back_left_leg.foot_origin = [-self.stance_polygon_length/2, self.stance_polygon_width/2, -self.stance_height]
        self.back_right_leg.foot_origin = [-self.stance_polygon_length/2, -self.stance_polygon_width/2, -self.stance_height]

    def __repr__(self):
        print("An instance of the custom Collywobble class.")

    def moveToPhaseInTrotGait(self, phase):
        """
        DESCRIPTION:
        Move all legs to their respective phases to effect a trot gait, coordinated by the specified gait phase.

        ARGUMENTS:
        + phase: The phase angle dictating the phase of the gait.
        """
        print("### Moving front_left_leg ###")
        self.front_left_leg.moveToPhase(phase = phase, stride_length = self.stride_length, swing_height = self.swing_height, swing_to_stance_ratio = self.swing_to_stance_ratio)
        print("### Moving front_right_leg ###")
        self.front_right_leg.moveToPhase(phase = phase + PI, stride_length = self.stride_length, swing_height = self.swing_height, swing_to_stance_ratio = self.swing_to_stance_ratio)
        print("### Moving back_left_leg ###")
        self.back_left_leg.moveToPhase(phase = phase + PI, stride_length = self.stride_length, swing_height = self.swing_height, swing_to_stance_ratio = self.swing_to_stance_ratio)
        print("### Moving back_right_leg ###")
        self.back_right_leg.moveToPhase(phase = phase, stride_length = self.stride_length, swing_height = self.swing_height, swing_to_stance_ratio = self.swing_to_stance_ratio)

    # Create a controller that shifts the coordinates of each foot in real-time...
    # 2 sensors will contribute to these augmentations
    # Accelerometer: As acceleration increases in one direction, foot coordinates will move in the opposite direction
    # Gyroscope: As rate of rotation increases in one direction, foot coordinates will move to oppose that motion

    # TODO: Grab accelerometer data and learn how to clean it

    def simStep():
        """
        A single step of the simulation.
        To be run in a loop.
        """
        pass