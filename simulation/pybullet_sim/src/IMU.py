
import numpy as np
import pybullet

from transforms3d.quaternions import quat2mat

class IMU:
    def __init__(
        self,
        robot_sim_id,
        config,
        noise_std = {
            "accel": 0.0,
            "gyro": 0.0,
            "quaternion": 0.0
        }
    ):
        self.robot_sim_id = robot_sim_id
        self.config = config
        self.noise_std = noise_std
        
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
        + self._accel: A (3,) array of accelerometer readings. Note that this is proper acceleration, absent of gravity bias.
        """
        last_vel = self._vel
        v_0_b = np.array(pybullet.getBaseVelocity(self.robot_sim_id)[0]).reshape(3, 1)
        R_0_b = quat2mat(np.array(pybullet.getBasePositionAndOrientation(bodyUniqueId = self.robot_sim_id)[1]).reshape(4))
        v_b_b = R_0_b.T @ v_0_b
        current_vel = v_b_b.reshape(3)
        self._accel = (1.0/self.config.dt)*(current_vel - last_vel)
        self._vel = current_vel
        self._accel[0] = self._accel[0] + np.random.normal(loc = 0.0, scale = self.noise_std["accel"])
        self._accel[1] = self._accel[1] + np.random.normal(loc = 0.0, scale = self.noise_std["accel"])
        self._accel[2] = self._accel[2] + np.random.normal(loc = 0.0, scale = self.noise_std["accel"])
        return self._accel

    @property
    def gyro_vals(self):
        """
        DESCRIPTION:
        Updates relevant attributes and returns them.

        RETURNS:
        + self._gyro: A (3,) array of gyro readings.
        """
        w_0_b = np.array(pybullet.getBaseVelocity(self.robot_sim_id)[1]).reshape(3, 1)
        R_0_b = quat2mat(np.array(pybullet.getBasePositionAndOrientation(bodyUniqueId = self.robot_sim_id)[1]).reshape(4))
        w_b_b = R_0_b.T @ w_0_b
        self._gyro = w_b_b.reshape(3)
        self._gyro[0] = self._gyro[0] + np.random.normal(loc = 0.0, scale = self.noise_std["gyro"])
        self._gyro[1] = self._gyro[1] + np.random.normal(loc = 0.0, scale = self.noise_std["gyro"])
        self._gyro[2] = self._gyro[2] + np.random.normal(loc = 0.0, scale = self.noise_std["gyro"])
        return self._gyro

    @property
    def quaternion_vals(self):
        """
        DESCRIPTION:
        Updates relevant attributes and returns them.

        RETURNS:
        + self._quaternion: A (4,) array representing the quaternion representation of the heading wrt world_frame. [x, y, z, w].
        """
        self._quaternion = np.array(pybullet.getBasePositionAndOrientation(bodyUniqueId = self.robot_sim_id)[1]).reshape(4)
        self._quaternion[0] = self._quaternion[0] + np.random.normal(loc = 0.0, scale = self.noise_std["quaternion"])
        self._quaternion[1] = self._quaternion[1] + np.random.normal(loc = 0.0, scale = self.noise_std["quaternion"])
        self._quaternion[2] = self._quaternion[2] + np.random.normal(loc = 0.0, scale = self.noise_std["quaternion"])
        self._quaternion[3] = self._quaternion[3] + np.random.normal(loc = 0.0, scale = self.noise_std["quaternion"])
        return self._quaternion
