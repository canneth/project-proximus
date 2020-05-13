
import sim

class Gyrosensor:
    def __init__(
        self,
        client_id
    ):
        self.client_id = client_id

        self.x = None
        self.y = None
        self.z = None
        self.x_accel = None
        self.y_accel = None
        self.z_accel = None

    def getX(self):
        _, gyro_x = sim.simxGetFloatSignal(self.client_id, "gyro_x", sim.simx_opmode_streaming)
        self.x = gyro_x
        return self.x
    def getY(self):
        _, gyro_y = sim.simxGetFloatSignal(self.client_id, "gyro_y", sim.simx_opmode_streaming)
        self.y = gyro_y
        return self.y
    def getZ(self):
        _, gyro_z = sim.simxGetFloatSignal(self.client_id, "gyro_z", sim.simx_opmode_streaming)
        self.z = gyro_z
        return self.z

    def getXAccel(self):
        _, gyro_x_accel = sim.simxGetFloatSignal(self.client_id, "gyro_x_accel", sim.simx_opmode_streaming)
        self.x_accel = gyro_x_accel
        return self.x_accel
    def getYAccel(self):
        _, gyro_y_accel = sim.simxGetFloatSignal(self.client_id, "gyro_y_accel", sim.simx_opmode_streaming)
        self.y_accel = gyro_y_accel
        return self.y_accel
    def getZAccel(self):
        _, gyro_z_accel = sim.simxGetFloatSignal(self.client_id, "gyro_z_accel", sim.simx_opmode_streaming)
        self.z_accel = gyro_z_accel
        return self.z_accel




