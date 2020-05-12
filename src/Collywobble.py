
import sim

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
        back_right_leg
    ):

        self.client_id = client_id
        self.world_frame = world_frame
        self.body_frame = body_frame
        self.body = body
        self.front_left_leg = front_left_leg
        self.front_right_leg = front_right_leg
        self.back_left_leg = back_left_leg
        self.back_right_leg = back_right_leg

        self.stance_polygon_length = 0.4
        self.stance_polygon_width = 0.2
        self.stance_height = 0.225

        self.stride_length = 0.120
        self.swing_height = 0.08
        self.swing_to_stance_speed_ratio = 2

        self.front_left_leg.foot_origin = [self.stance_polygon_length/2, self.stance_polygon_width/2, -self.stance_height]
        self.front_right_leg.foot_origin = [self.stance_polygon_length/2, -self.stance_polygon_width/2, -self.stance_height]
        self.back_left_leg.foot_origin = [-self.stance_polygon_length/2, self.stance_polygon_width/2, -self.stance_height]
        self.back_right_leg.foot_origin = [-self.stance_polygon_length/2, -self.stance_polygon_width/2, -self.stance_height]

    def __repr__(self):
        print("An instance of the custom Collywobble class.")

    def simStep():
        """
        A single step of the simulation.
        To be run in a loop.
        """
        pass