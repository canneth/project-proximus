
import numpy as np
import pybullet

class IMU:
    def __init__(
        self,
        robot_sim_id,
        config
    ):
        self.robot_sim_id = robot_sim_id
        self.config = config
        
        self._vel = np.zeros((3))
        self._accel = np.zeros((3))
        self._gyro = np.zeros((3))
        self._quaternion = np.zeros((4))

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
        + self._accel: A (3,) array of accelerometer readings.
        """
        last_vel = self._vel
        current_vel = pybullet.getBaseVelocity(self.robot_sim_id)[0]
        self._accel = (1.0/self.config.dt)*(current_vel - last_vel)
        return self._accel

    @property
    def gyro_vals(self):
        """
        DESCRIPTION:
        Updates relevant attributes and returns them.

        RETURNS:
        + self._gyro: A (3,) array of gyro readings.
        """
        self.gyro = pybullet.getBaseVelocity(self.robot_sim_id)[1]
        return self._gyro

    @property
    def quaternion_vals(self):
        """
        DESCRIPTION:
        Updates relevant attributes and returns them.

        RETURNS:
        + self._quaternion: A (4,) array representing the quaternion representation of the heading wrt world_frame. [x, y, z, w].
        """
        self._quaternion = pybullet.getBasePositionAndOrientation(bodyUniqueId = self.robot_sim_id)[1]
        return self._quaternion
