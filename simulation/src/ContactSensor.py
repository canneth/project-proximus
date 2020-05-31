
import sim

class ContactSensor:
    def __init__(
        self,
        client_id,
        collision_handle_name
    ):
        self.client_id = client_id
        self.collision_handle_name = collision_handle_name

        _, self.collision_handle = sim.simxGetCollisionHandle(self.client_id, collision_handle_name, sim.simx_opmode_blocking)

    def getState(self):
        _, collision_state = sim.simxReadCollision(self.client_id, self.collision_handle, sim.simx_opmode_streaming)
        return collision_state
        