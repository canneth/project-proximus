
import sim

class Accelerometer:
    def __init__(
        self,
        client_id
    ):
        self.client_id = client_id

        self.x = None
        self.y = None
        self.z = None

    def getX(self):
        _, accelerometer_x = sim.simxGetFloatSignal(self.client_id, "accelerometer_x", sim.simx_opmode_streaming)
        self.x = accelerometer_x
        return self.x
    def getY(self):
        _, accelerometer_y = sim.simxGetFloatSignal(self.client_id, "accelerometer_y", sim.simx_opmode_streaming)
        self.y = accelerometer_y
        return self.y
    def getZ(self):
        _, accelerometer_z = sim.simxGetFloatSignal(self.client_id, "accelerometer_z", sim.simx_opmode_streaming)
        self.z = accelerometer_z
        return self.z

