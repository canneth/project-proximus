
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

    def __repr__(self):
        print("An instance of the custom Collywobble class.")

    def simStep():
        """
        A single step of the simulation.
        To be run in a loop.
        """
        pass
