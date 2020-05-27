
class IMU:
    def __init__(
        self,
        client_id
    ):
        self.client_id = client_id
        self._accel = [0, 0, 0]
        self._gyro = [0, 0, 0]
        self._quaternion = [0, 0, 0, 0]

    """
    Float signals from sim:
    + imu_accel_x
    + imu_accel_y
    + imu_accel_z
    + imu_gyro_x
    + imu_gyro_y
    + imu_gyro_z
    + imu_quaternion_x
    + imu_quaternion_y
    + imu_quaternion_z
    + imu_quaternion_w
    """
    @property
    def accel_vals(self):
        """
        DESCRIPTION:
        Updates relevant attributes and returns them.

        RETURNS:
        + self._accel: A (3,) list of accelerometer readings.
        """
        _, self._accel[0] = sim.simxGetFloatSignal(self.client_id, "imu_accel_x", sim.simx_opmode_streaming)
        _, self._accel[1] = sim.simxGetFloatSignal(self.client_id, "imu_accel_y", sim.simx_opmode_streaming)
        _, self._accel[2] = sim.simxGetFloatSignal(self.client_id, "imu_accel_z", sim.simx_opmode_streaming)
        return self._accel

    @property
    def gyro_vals(self):
        """
        DESCRIPTION:
        Updates relevant attributes and returns them.

        RETURNS:
        + self._gyro: A (3,) list of gyro readings.
        """
        _, self._gyro[0] = sim.simxGetFloatSignal(self.client_id, "imu_gyro_x", sim.simx_opmode_streaming)
        _, self._gyro[1] = sim.simxGetFloatSignal(self.client_id, "imu_gyro_y", sim.simx_opmode_streaming)
        _, self._gyro[2] = sim.simxGetFloatSignal(self.client_id, "imu_gyro_z", sim.simx_opmode_streaming)
        return self._gyro

    @property
    def quaternion_vals(self):
        """
        DESCRIPTION:
        Updates relevant attributes and returns them.

        RETURNS:
        + self._quaternion: A (4,) list representing the quaternion representation of the heading wrt world_frame.
        """
        
        _, self._quaternion[0] = sim.simxGetFloatSignal(self.client_id, "imu_quaternion_x", sim.simx_opmode_streaming)
        _, self._quaternion[1] = sim.simxGetFloatSignal(self.client_id, "imu_quaternion_y", sim.simx_opmode_streaming)
        _, self._quaternion[2] = sim.simxGetFloatSignal(self.client_id, "imu_quaternion_z", sim.simx_opmode_streaming)
        _, self._quaternion[3] = sim.simxGetFloatSignal(self.client_id, "imu_quaternion_w", sim.simx_opmode_streaming)
        return self._quaternion
