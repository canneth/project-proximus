
import sim

from GlobalConstants import FootTrajectory

from Leg import Leg
from Robot import Robot
from Robot import Mode
from Command import Command
from Config import Config
from MasterController import MasterController
from DataLogger import DataLogger
from IMU import IMU

from collections import defaultdict

import numpy as np
from math import pi as PI
from transforms3d.euler import euler2mat
from transforms3d.quaternions import quat2mat

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from pathlib import Path

import time

def ik_check(client_id, robot):
    
    _, foot_pos = sim.simxGetObjectPosition(client_id, robot.back_left_foot, robot.body_frame, sim.simx_opmode_blocking)
    print("Target position: {}".format(foot_pos))

    robot.front_left_leg.moveFoot([0.19302406907081604, 0.11094817519187927, -0.22628307342529297]) # Corresponds to joint angles (in degrees) [0, -45, 0]
    robot.front_right_leg.moveFoot([0.19302406907081604, -0.11094817519187927, -0.22628307342529297]) # Corresponds to joint angles (in degrees) [0, -45, 0]
    robot.back_left_leg.moveFoot([-0.1929716169834137, 0.11089962720870972, -0.22629320621490479]) # Corresponds to joint angles (in degrees) [0, -45, 0]
    robot.back_right_leg.moveFoot([-0.19296878576278687, -0.11091902107000351, -0.22629079222679138]) # Corresponds to joint angles (in degrees) [0, -45, 0]

    time.sleep(0.5)
    
    _, foot_pos = sim.simxGetObjectPosition(client_id, back_left_foot, body_frame, sim.simx_opmode_blocking)
    print("Current position: {}".format(foot_pos))

def getContactPatternIndex(contact_pattern, gait_schedule):
    """
    DESCRIPTION:
    Finds and returns the contact_pattern_index corresponding to the contact_pattern in gait_schedule.
    contact_pattern_index is {0, 1, 2, 3}, the column index of the gait_schedule that gives the column equal to contact_pattern.
    For contact_patterns that occur more than once in gait_schedule, the lowest index is returned.
    This is used to keep track of the Q matrix used in the Kalman Filter, since it changes with leg contacts.
    NOTE: This is solely used for logging convenience, and not in the Kalman Filter.

    ARGUMENTS:
    + contact_pattern: A (1, 4) or (4, 1) array; The contact_pattern for the legs.
    + gait_schedule: A (4, n) array; The contact pattern schedule, where each column represents a contact pattern in the gait cycle.
    """
    pattern = contact_pattern.reshape(1, 4)
    for i in range(gait_schedule.shape[1]):
        if (np.prod(gait_schedule[:, i].reshape(1, 4) == pattern)) == 1:
            return i

def generateControlInputMatrix(imu, command):
    """
    DESCRIPTION:
    Generates and returns the control input matrix using sensor values from the imu and the velocity inputs from command.

    ARGUMENTS:
    + imu: An IMU object
    + command: A Command object

    RETURNS:
    + B: A (18, 1) array; it is the control input matrix.
    """
    # Read quaternion from IMU
    q = np.roll(np.array(imu.quaternion_vals), 1) # Roll is necessary cuz quar2mat expects q = [w, x, y, z], but imu outputs q = [x, y, z, w]
    # Convert quaternion into rotation matrix
    R_0_b = quat2mat(q)
    # Read acceleration from IMU
    a_b_b = np.array(imu.accel_vals).reshape(3, 1)
    # Apply rotation matrix to acceleration
    a_0_b = R_0_b @ a_b_b
    # Get velocity command from command
    v_b_command = np.array(command.body_velocity).reshape(3, 1)
    # Apply rotation matrix to velocity command
    v_0_command = R_0_b @ v_b_command
    # Build B
    B = np.block(
        [
            [0.5*(dt**2)*a_0_b], # p_b
            [dt*a_0_b], # v_b
            [np.zeros((3, 1))], # p_1
            [np.zeros((3, 1))], # p_2
            [np.zeros((3, 1))], # p_3
            [np.zeros((3, 1))] # p_4
        ]
    )
    return B

def generateStateCovarianceMatrix(p_b_std, v_b_std, p_1_std, p_2_std, p_3_std, p_4_std, P_prev = None, contact_pattern = None):
    """
    DESCRIPTION:
    Generates a new state covariance matrix given the previous P_prev and the current contact_pattern_index.
    Returns this generated matrix as P_new which is a modified P.
    If P_prev = None and/or contact_pattern = None, then it creates a P_init to be fed as initial input into the Kalman Filter.

    ARGUMENTS:
    + p_b_std: A float; the scalar standard deviation representing the uncertainty in p_b.
    + v_b_std: A float; the scalar standard deviation representing the uncertainty in v_b.
    + p_1_std: A float; the scalar standard deviation representing the uncertainty in p_1.
    + p_2_std: A float; the scalar standard deviation representing the uncertainty in p_2.
    + p_3_std: A float; the scalar standard deviation representing the uncertainty in p_3.
    + p_4_std: A float; the scalar standard deviation representing the uncertainty in p_4.
    + P_prev: A (18, 18) array; the previous covariance matrix.
    + contact_pattern: A (4) array; the contact pattern of the feet, where 0 = swing, 1 = stance.

    RETURNS:
    + P_new: A (18, 18) array; the new state covariance matrix with modified elements corresponding to the current contact pattern.
    """
    I = np.identity(3)
    O = np.zeros((3, 3))
    p_i_stds = [p_1_std, p_2_std, p_3_std, p_4_std]
    if ((P_prev is None) or (contact_pattern is None)):
        # Create P_init
        P_new = np.block(
            [
                [(p_b_std**2)*I, O, O, O, O, O], # p_b
                [O, (v_b_std**2)*I, O, O, O, O], # v_b
                [O, O, (p_1_std**2)*I, O, O, O], # p_1
                [O, O, O, (p_2_std**2)*I, O, O], # p_2
                [O, O, O, O, (p_3_std**2)*I, O], # p_3
                [O, O, O, O, O, (p_4_std**2)*I] # p_4
            ]
        )
        return P_new
    else:
        P_new = np.copy(P_prev)
        # Modify elements based on contact_pattern_index
        inf = 9999
        # Iterate in blocks of size (3, 3)
        n = 3
        for i in range(contact_pattern.reshape(4).shape[0]):
            block_index = 2 + i # Offset to begin counting from block corresponding to p_1 state
            if (contact_pattern[i] == 0):
                # In swing, set uncertainty to a super high value
                P_new[block_index*n : (block_index + 1)*n, block_index*n : (block_index + 1)*n] = inf*np.identity(n)
            else:
                # In stance, set uncertainty back to init values
                P_new[block_index*n : (block_index + 1)*n, block_index*n : (block_index + 1)*n] = (p_i_stds[i]**2)*np.identity(n)
        return P_new

def generateProcessCovarianceMatrix(v_b_std, p_1_std, p_2_std, p_3_std, p_4_std, dt, contact_pattern = [1, 1, 1, 1]):
    """
    DESCRIPTION:
    Generates a new process covariance matrix given the previous Q_prev and the current contact_pattern_index.
    Returns this generated matrix as Q.

    ARGUMENTS:
    + v_b_std: A float; the scalar standard deviation representing the uncertainty in v_b.
    + p_1_std: A float; the scalar standard deviation representing the uncertainty in p_1.
    + p_2_std: A float; the scalar standard deviation representing the uncertainty in p_2.
    + p_3_std: A float; the scalar standard deviation representing the uncertainty in p_3.
    + p_4_std: A float; the scalar standard deviation representing the uncertainty in p_4.
    + dt: A float; the time step of the discrete model.
    + contact_pattern: A (4) array; the contact pattern of the feet, where 0 = swing, 1 = stance.

    RETURNS:
    + Q: A (18, 18) array; the process covariance matrix with elements corresponding to the current contact pattern.
    """
    I = np.identity(3)
    O = np.zeros((3, 3))
    Q_intermediate = Q_discrete_white_noise(2, dt = dt)
    Q_body = np.block( # Q_body is (6, 6)
        [
            [Q_intermediate[0, 0]*I, Q_intermediate[0, 1]*I],
            [Q_intermediate[1, 0]*I, Q_intermediate[1, 1]*I],
        ]
    )
    Q_body = (v_b_std**2)*Q_body
    p_i_stds = np.array([p_1_std, p_2_std, p_3_std, p_4_std]).reshape(4)
    mask_from_contact_pattern = np.array(contact_pattern).reshape(4)
    inf = 9999
    p_i_stds[mask_from_contact_pattern == 0] = inf # For feet in swing, send uncertainty into the sky
    Q_feet = np.block( # Q_feet is (12, 12)
        [
            [p_i_stds[0]*I*dt, O, O, O],
            [O, p_i_stds[1]*I*dt, O, O],
            [O, O, p_i_stds[2]*I*dt, O],
            [O, O, O, p_i_stds[3]*I*dt]
        ]
    )
    Q = np.block(
        [
            [Q_body, np.zeros((6, 12))],
            [np.zeros((12, 6)), Q_feet]
        ]
    )
    return Q

def generateMeasurementMatrix(robot, imu):
    """
    DESCRIPTION:
    Generates and returns the measurement matrix z derived from the states tracked within the robot object and IMU sensor.

    ARGUMENTS:
    + robot: A Robot object; the robot from which to extract the necessary states.
    + imu: An IMU object; the orientation of the robot used for reference frame transforms is extracted from this.

    RETURNS:
    + z: A (25,) array; the measurement matrix (really a vector).
    """
    z_p_1_in_b_frame = robot.foot_locations_wrt_body_true[:, 0].reshape(3, 1)
    z_p_2_in_b_frame = robot.foot_locations_wrt_body_true[:, 1].reshape(3, 1)
    z_p_3_in_b_frame = robot.foot_locations_wrt_body_true[:, 2].reshape(3, 1)
    z_p_4_in_b_frame = robot.foot_locations_wrt_body_true[:, 3].reshape(3, 1)
    z_v_1_in_b_frame = -1.0*robot.foot_velocities_wrt_body[:, 0].reshape(3, 1) # This is an approximation of body velocity, which is -ve of foot velocity
    z_v_2_in_b_frame = -1.0*robot.foot_velocities_wrt_body[:, 1].reshape(3, 1) # This is an approximation of body velocity, which is -ve of foot velocity
    z_v_3_in_b_frame = -1.0*robot.foot_velocities_wrt_body[:, 2].reshape(3, 1) # This is an approximation of body velocity, which is -ve of foot velocity
    z_v_4_in_b_frame = -1.0*robot.foot_velocities_wrt_body[:, 3].reshape(3, 1) # This is an approximation of body velocity, which is -ve of foot velocity
    z_b_height = robot.stance_height
    # Read quaternion from IMU
    q = np.roll(np.array(imu.quaternion_vals), 1) # Roll is needed since quat2mat requires q = [w, x, y, z], while imu outputs q = [x, y, z, w]
    # Convert quaternion into rotation matrix
    R_0_b = quat2mat(q)
    # Change reference frame to world frame
    z_p_1 = R_0_b @ z_p_1_in_b_frame
    z_p_2 = R_0_b @ z_p_2_in_b_frame
    z_p_3 = R_0_b @ z_p_3_in_b_frame
    z_p_4 = R_0_b @ z_p_4_in_b_frame
    z_v_1 = R_0_b @ z_v_1_in_b_frame
    z_v_2 = R_0_b @ z_v_2_in_b_frame
    z_v_3 = R_0_b @ z_v_3_in_b_frame
    z_v_4 = R_0_b @ z_v_4_in_b_frame
    # Build z
    z = np.block(
        [
            [z_p_1],
            [z_p_2],
            [z_p_3],
            [z_p_4],
            [z_v_1],
            [z_v_2],
            [z_v_3],
            [z_v_4],
            [z_b_height]
        ]
    )
    return z

def generateMeasurementCovarianceMatrix(z_p_1_std, z_p_2_std, z_p_3_std, z_p_4_std, z_v_1_std, z_v_2_std, z_v_3_std, z_v_4_std, z_b_height_std, contact_pattern = [1, 1, 1, 1]):
    """
    DESCRIPTION:
    Generates and returns a measurement covariance matrix given the respective standard deviations.
    Elements corresponding to legs in swing are set to high values, and thus the necessary changes are
    coordinated by the contact_pattern.

    ARGUMENTS:
    + z_p_1_std: A float; the standard deviation for the measurement z_p_1.
    + z_p_2_std: A float; the standard deviation for the measurement z_p_2.
    + z_p_3_std: A float; the standard deviation for the measurement z_p_3.
    + z_p_4_std: A float; the standard deviation for the measurement z_p_4.
    + z_v_1_std: A float; the standard deviation for the measurement z_v_1.
    + z_v_2_std: A float; the standard deviation for the measurement z_v_2.
    + z_v_3_std: A float; the standard deviation for the measurement z_v_3.
    + z_v_4_std: A float; the standard deviation for the measurement z_v_4.
    + z_b_height_std: A float; the standard deviation for the measurement z_b_height.
    + contact_pattern: A (4) array; the contact pattern of the feet, where 0 = swing, 1 = stance.

    RETURNS:
    + R: A (25, 25) array; the measurement covariance matrix.
    """
    I = np.identity(3) # For convenience
    O = np.zeros((3, 3)) # For convenience
    z_p_i_stds = np.array([z_p_1_std, z_p_2_std, z_p_3_std, z_p_4_std]).reshape(4)
    z_v_i_stds = np.array([z_v_1_std, z_v_2_std, z_v_3_std, z_v_4_std]).reshape(4)
    mask_from_contact_pattern = np.array(contact_pattern).reshape(4)
    inf = 9999
    z_p_i_stds[mask_from_contact_pattern == 0] = inf # For feet in swing, send uncertainty into the sky
    z_v_i_stds[mask_from_contact_pattern == 0] = inf # For feet in swing, send uncertainty into the sky
    R = np.block(
        [
            [(z_p_i_stds[0]**2)*I, O, O, O,     O, O, O, O], # z_p_1
            [O, (z_p_i_stds[1]**2)*I, O, O,     O, O, O, O], # z_p_2
            [O, O, (z_p_i_stds[2]**2)*I, O,     O, O, O, O], # z_p_3
            [O, O, O, (z_p_i_stds[3]**2)*I,     O, O, O, O], # z_p_4
            [O, O, O, O,     (z_v_i_stds[0]**2)*I, O, O, O], # z_v_1
            [O, O, O, O,     O, (z_v_i_stds[1]**2)*I, O, O], # z_v_2
            [O, O, O, O,     O, O, (z_v_i_stds[2]**2)*I, O], # z_v_3
            [O, O, O, O,     O, O, O, (z_v_i_stds[3]**2)*I],  # z_v_4
        ]
    )
    R = np.concatenate((R, np.zeros((24, 1))), axis = 1)
    R = np.concatenate(
        (
            R,
            np.concatenate((np.zeros((24)), [z_b_height_std**2]), axis = 0).reshape(1, 25)  # z_b_height
        ),
        axis = 0
    )
    return R

if __name__ == "__main__":
    
    ### BEGIN SIM CONNECTION ###
    connection_successful = False
    print("=== Programme START ===")
    sim.simxFinish(-1) # just in case, close all opened connections
    client_id = sim.simxStart("127.0.0.1", 19999, True, True, 5000, 5) # Connect to CoppeliaSim
    # Connection ID of 19997 connects to the simulator without having the simulation running
    if client_id!=-1:
        print ("Connected to remote API server")
        connection_successful = True
    else:
        print ("Failed connecting to remote API server")
        connection_successful = False

    if connection_successful:
        ### GET OBJECT HANDLES ###
        _, world_frame = sim.simxGetObjectHandle(client_id, "world_frame", sim.simx_opmode_blocking)
        # Create configuration object which stores all configuration parameters used by almost everything
        config = Config()
        # Master Controller
        master_controller = MasterController(
            config = config,
            trajectory_shape = FootTrajectory.TRIANGULAR,
            use_capture_point = False,
            use_vpsp = True
        )
        # Robot
        robot = Robot(
            client_id = client_id,
            stance_polygon_length = 0.4,
            stance_polygon_width = 0.15,
            stance_height = 0.225,
            swing_height = 0.08
        )

        # Sensors
        imu = IMU(client_id)

        ### Kalman Filter Setup ###
        dt = config.dt

        I = np.identity(3) # For convenience
        O = np.zeros((3, 3)) # For convenience

        # F should be (18, 18)
        F = np.block(
            [
                [I, dt*I, O, O, O, O], # p_b
                [O, I, O, O, O, O], # v_b
                [O, O, I, O, O, O], # p_1
                [O, O, O, I, O, O], # p_2
                [O, O, O, O, I, O], # p_3
                [O, O, O, O, O, I] # p_4
            ]
        )
        # H should be (25, 18)
        H = np.block(
            [
                [-1.0*I, O, I, O, O, O], # z_1 --- z_p_1
                [-1.0*I, O, O, I, O, O], # z_2 --- z_p_2
                [-1.0*I, O, O, O, I, O], # z_3 --- z_p_3
                [-1.0*I, O, O, O, O, I], # z_4 --- z_p_4
                [O, I, O, O, O, O], # z_5 --- z_v_1
                [O, I, O, O, O, O], # z_6 --- z_v_2
                [O, I, O, O, O, O], # z_7 --- z_v_3
                [O, I, O, O, O, O], # z_8 --- z_v_4
                [0, 0, 1, np.zeros((15))], # z_9 --- z_b_height
            ]
        )
        # x should be (18, 1)
        x_init = np.block(
            [
                [np.array([0, 0, robot.stance_height]).reshape(3, 1)], # p_b
                [np.zeros((3, 1))], # v_b
                [np.concatenate((robot.foot_locations_wrt_body_at_rest[:2, 0].reshape(2), [0]), axis = 0).reshape(3, 1)], # p_1
                [np.concatenate((robot.foot_locations_wrt_body_at_rest[:2, 1].reshape(2), [0]), axis = 0).reshape(3, 1)], # p_2
                [np.concatenate((robot.foot_locations_wrt_body_at_rest[:2, 2].reshape(2), [0]), axis = 0).reshape(3, 1)], # p_3
                [np.concatenate((robot.foot_locations_wrt_body_at_rest[:2, 3].reshape(2), [0]), axis = 0).reshape(3, 1)], # p_4
            ]
        ).reshape(18)
        # P should be (18, 18)
        P_init = generateStateCovarianceMatrix(
            p_b_std = 0.001, # Very certain about initial body position
            v_b_std = 0.001, # Very certain about initial body velocity
            p_1_std = 0.001, # Very certain about initial foot position
            p_2_std = 0.001, # Very certain about initial foot position
            p_3_std = 0.001, # Very certain about initial foot position
            p_4_std = 0.001 # Very certain about initial foot position
        )

        # Create Kalman Filter
        kalman_filter = KalmanFilter(dim_x = 18, dim_z = 25)
        # Q, R and B are created and changed every iteration of the filter, and so are created within the main loop below
        kalman_filter.F = F
        kalman_filter.H = H
        kalman_filter.x = x_init
        kalman_filter.P = P_init

        # Data-logging stuff
        data_field_list_kf = (
            [
                "t",
                "contact_pattern_index",
                "p_x_true", "p_y_true", "p_z_true",
                "v_x_true", "v_y_true", "v_z_true",
                "p_1x_true", "p_1y_true", "p_1z_true",
                "p_2x_true", "p_2y_true", "p_2z_true",
                "p_3x_true", "p_3y_true", "p_3z_true",
                "p_4x_true", "p_4y_true", "p_4z_true",
                "p_x_est", "p_y_est", "p_z_est",
                "v_x_est", "v_y_est", "v_z_est",
                "p_1x_est", "p_1y_est", "p_1z_est",
                "p_2x_est", "p_2y_est", "p_2z_est",
                "p_3x_est", "p_3y_est", "p_3z_est",
                "p_4x_est", "p_4y_est", "p_4z_est",
                "y_p_1x", "y_p_1y", "y_p_1z",
                "y_p_2x", "y_p_2y", "y_p_2z",
                "y_p_3x", "y_p_3y", "y_p_3z",
                "y_p_4x", "y_p_4y", "y_p_4z",
                "y_v_1x", "y_v_1y", "y_v_1z",
                "y_v_2x", "y_v_2y", "y_v_2z",
                "y_v_3x", "y_v_3y", "y_v_3z",
                "y_v_4x", "y_v_4y", "y_v_4z",
                "P_p_bx_std", "P_p_by_std", "P_p_bz_std",
                "P_v_bx_std", "P_v_by_std", "P_v_bz_std",
                "P_p_1x_std", "P_p_1y_std", "P_p_1z_std",
                "P_p_2x_std", "P_p_2y_std", "P_p_2z_std",
                "P_p_3x_std", "P_p_3y_std", "P_p_3z_std",
                "P_p_4x_std", "P_p_4y_std", "P_p_4z_std",
                "p_vpsp_x", "p_vpsp_y", "p_vpsp_z"
            ]
        )
        data_logger_kf = DataLogger(
            data_fields = data_field_list_kf,
            save_file_path = Path(".") / "data" / "kf_data.csv"
        )
        data_field_list_vpsp = (
            [
                "t",
                "p_b_vpsp_x", "p_b_vpsp_y",
                "FL_cw_vp_x", "FL_cw_vp_y",
                "FR_cw_vp_x", "FR_cw_vp_y",
                "BL_cw_vp_x", "BL_cw_vp_y",
                "BR_cw_vp_x", "BR_cw_vp_y",
                "FL_ccw_vp_x", "FL_ccw_vp_y",
                "FR_ccw_vp_x", "FR_ccw_vp_y",
                "BL_ccw_vp_x", "BL_ccw_vp_y",
                "BR_ccw_vp_x", "BR_ccw_vp_y",
                "FL_vt_x", "FL_vt_y",
                "FR_vt_x", "FR_vt_y",
                "BL_vt_x", "BL_vt_y",
                "BR_vt_x", "BR_vt_y"

            ]
        )
        data_logger_vpsp = DataLogger(
            data_fields = data_field_list_vpsp,
            save_file_path = Path(".") / "data" / "vpsp_data.csv"
        )

        command = Command()

        last_time = time.time()
        start_time = time.time()

        initialisation_time = 1.0
        end_time = 10.0

        ### LOOP ###
        while True:
            current_time = time.time()
            elapsed_time = current_time - last_time
            if (elapsed_time < config.dt):
                continue
            last_time = time.time()

            time_since_start = current_time - start_time
            if (time_since_start < initialisation_time):
                # Initialisation time for KF and simulation to stabilise
                command.stance_height = 0.2
                command.mode = Mode.REST
                command.body_velocity = [0, 0, 0]
            elif (time_since_start < end_time):
                command.stance_height = 0.2
                command.mode = Mode.TROT
                command.body_velocity = [0.2, 0, 0]
                command.swing_height = 0.1
            else:
                # End sim
                break


            master_controller.stepOnce(robot, command)
            # Generate Q according to contact_pattern
            # Q should be (18, 18)
            Q = generateProcessCovarianceMatrix(
                v_b_std = 0.5, # Process noise is high for velocity because model does not account for disturbances due to foot movement
                p_1_std = 0.1, # Feet slip quite a bit, so the process noise is high for the foot locations
                p_2_std = 0.1, # Feet slip quite a bit, so the process noise is high for the foot locations
                p_3_std = 0.1, # Feet slip quite a bit, so the process noise is high for the foot locations
                p_4_std = 0.1, # Feet slip quite a bit, so the process noise is high for the foot locations
                dt = dt,
                contact_pattern = robot.contact_pattern
            )
            # Generate R according to contact_pattern
            # R should be (24, 24)
            R = generateMeasurementCovarianceMatrix(
                z_p_1_std = 0.005, # Uncertainty in measured relative foot positions is low because servos move to commanded position very accurately
                z_p_2_std = 0.005, # Uncertainty in measured relative foot positions is low because servos move to commanded position very accurately
                z_p_3_std = 0.005, # Uncertainty in measured relative foot positions is low because servos move to commanded position very accurately
                z_p_4_std = 0.005, # Uncertainty in measured relative foot positions is low because servos move to commanded position very accurately
                z_v_1_std = 0.1, # v_i represents the body velocity as extrapolated from foot velocity, and so it's pretty inaccurate due to slippage
                z_v_2_std = 0.1, # v_i represents the body velocity as extrapolated from foot velocity, and so it's pretty inaccurate due to slippage
                z_v_3_std = 0.1, # v_i represents the body velocity as extrapolated from foot velocity, and so it's pretty inaccurate due to slippage
                z_v_4_std = 0.1, # v_i represents the body velocity as extrapolated from foot velocity, and so it's pretty inaccurate due to slippage
                z_b_height_std = 0.1,
                contact_pattern = robot.contact_pattern
            )
            # Generate control input matrix
            # B should be (18, 1)
            B = generateControlInputMatrix(imu = imu, command = command)
            # Register new Q, R and B into Kalman filter
            kalman_filter.Q = Q
            kalman_filter.R = R
            kalman_filter.B = B
            # Get "measurements" z
            z = generateMeasurementMatrix(robot = robot, imu = imu)
            # Run Kalman filter
            kalman_filter.predict(u = np.array([1]), B = B)
            kalman_filter.update(z = z)
            # Update robot's velocity attribute with KF estimate
            velocity_correction_coeff = 1 # 0.5 seems closer to true speed but it still doesn't render body velocity well enough to calculate a good capture point
            robot.body_velocity = np.array(
                [
                    float(kalman_filter.x.reshape(18)[3]),
                    float(kalman_filter.x.reshape(18)[4]),
                    float(kalman_filter.x.reshape(18)[5])
                ]
            )*velocity_correction_coeff

            ### Collecting and formatting data for collection ###
            # Getting true values
            t = time.time() - start_time
            _, p_0b = sim.simxGetObjectPosition(client_id, robot.body_frame, world_frame, sim.simx_opmode_streaming)
            _, v_0b, _ = sim.simxGetObjectVelocity(client_id, robot.body_frame, sim.simx_opmode_streaming)
            _, p_1_rel = sim.simxGetObjectPosition(client_id, robot.front_left_leg.foot, robot.body_frame, sim.simx_opmode_streaming)
            _, p_2_rel = sim.simxGetObjectPosition(client_id, robot.front_right_leg.foot, robot.body_frame, sim.simx_opmode_streaming)
            _, p_3_rel = sim.simxGetObjectPosition(client_id, robot.back_left_leg.foot, robot.body_frame, sim.simx_opmode_streaming)
            _, p_4_rel = sim.simxGetObjectPosition(client_id, robot.back_right_leg.foot, robot.body_frame, sim.simx_opmode_streaming)
            _, p_1_abs = sim.simxGetObjectPosition(client_id, robot.front_left_leg.foot, world_frame, sim.simx_opmode_streaming)
            _, p_2_abs = sim.simxGetObjectPosition(client_id, robot.front_right_leg.foot, world_frame, sim.simx_opmode_streaming)
            _, p_3_abs = sim.simxGetObjectPosition(client_id, robot.back_left_leg.foot, world_frame, sim.simx_opmode_streaming)
            _, p_4_abs = sim.simxGetObjectPosition(client_id, robot.back_right_leg.foot, world_frame, sim.simx_opmode_streaming)
            contact_pattern_index = getContactPatternIndex(
                contact_pattern = robot.contact_pattern,
                gait_schedule = master_controller.trot_controller.gait_config.contact_schedule
            )
            # Getting estimates from Kalman filter
            new_x = kalman_filter.x.reshape(18)
            new_P = kalman_filter.P
            new_y = kalman_filter.y.reshape(25)
            p_x_est = float(new_x.reshape(18)[0])
            p_y_est = float(new_x.reshape(18)[1])
            p_z_est = float(new_x.reshape(18)[2])
            v_x_est = float(new_x.reshape(18)[3])
            v_y_est = float(new_x.reshape(18)[4])
            v_z_est = float(new_x.reshape(18)[5])
            p_1x_est = float(new_x.reshape(18)[6])
            p_1y_est = float(new_x.reshape(18)[7])
            p_1z_est = float(new_x.reshape(18)[8])
            p_2x_est = float(new_x.reshape(18)[9])
            p_2y_est = float(new_x.reshape(18)[10])
            p_2z_est = float(new_x.reshape(18)[11])
            p_3x_est = float(new_x.reshape(18)[12])
            p_3y_est = float(new_x.reshape(18)[13])
            p_3z_est = float(new_x.reshape(18)[14])
            p_4x_est = float(new_x.reshape(18)[15])
            p_4y_est = float(new_x.reshape(18)[16])
            p_4z_est = float(new_x.reshape(18)[17])
            y_p_1x = float(new_y.reshape(25)[0])
            y_p_1y = float(new_y.reshape(25)[1])
            y_p_1z = float(new_y.reshape(25)[2])
            y_p_2x = float(new_y.reshape(25)[3])
            y_p_2y = float(new_y.reshape(25)[4])
            y_p_2z = float(new_y.reshape(25)[5])
            y_p_3x = float(new_y.reshape(25)[6])
            y_p_3y = float(new_y.reshape(25)[7])
            y_p_3z = float(new_y.reshape(25)[8])
            y_p_4x = float(new_y.reshape(25)[9])
            y_p_4y = float(new_y.reshape(25)[10])
            y_p_4z = float(new_y.reshape(25)[11])
            y_v_1x = float(new_y.reshape(25)[12])
            y_v_1y = float(new_y.reshape(25)[13])
            y_v_1z = float(new_y.reshape(25)[14])
            y_v_2x = float(new_y.reshape(25)[15])
            y_v_2y = float(new_y.reshape(25)[16])
            y_v_2z = float(new_y.reshape(25)[17])
            y_v_3x = float(new_y.reshape(25)[18])
            y_v_3y = float(new_y.reshape(25)[19])
            y_v_3z = float(new_y.reshape(25)[20])
            y_v_4x = float(new_y.reshape(25)[21])
            y_v_4y = float(new_y.reshape(25)[22])
            y_v_4z = float(new_y.reshape(25)[23])
            P_p_bx_std = float(np.sqrt(new_P[0, 0]))
            P_p_by_std = float(np.sqrt(new_P[1, 2]))
            P_p_bz_std = float(np.sqrt(new_P[2, 2]))
            P_v_bx_std = float(np.sqrt(new_P[3, 3]))
            P_v_by_std = float(np.sqrt(new_P[4, 4]))
            P_v_bz_std = float(np.sqrt(new_P[5, 5]))
            P_p_1x_std = float(np.sqrt(new_P[6, 6]))
            P_p_1y_std = float(np.sqrt(new_P[7, 7]))
            P_p_1z_std = float(np.sqrt(new_P[8, 8]))
            P_p_2x_std = float(np.sqrt(new_P[9, 9]))
            P_p_2y_std = float(np.sqrt(new_P[10, 10]))
            P_p_2z_std = float(np.sqrt(new_P[11, 11]))
            P_p_3x_std = float(np.sqrt(new_P[12, 12]))
            P_p_3y_std = float(np.sqrt(new_P[13, 13]))
            P_p_3z_std = float(np.sqrt(new_P[14, 14]))
            P_p_4x_std = float(np.sqrt(new_P[15, 15]))
            P_p_4y_std = float(np.sqrt(new_P[16, 16]))
            P_p_4z_std = float(np.sqrt(new_P[17, 17]))

            ### Building data_dicts ###
            # For Kalman filter visualisation #
            data_logger_kf.data_dict = {key: 0 for key in data_field_list_kf}
            data_logger_kf.data_dict["t"] = t
            data_logger_kf.data_dict["contact_pattern_index"] = contact_pattern_index
            data_logger_kf.data_dict["p_x_true"] = p_0b[0]
            data_logger_kf.data_dict["p_y_true"] = p_0b[1]
            data_logger_kf.data_dict["p_z_true"] = p_0b[2]
            data_logger_kf.data_dict["v_x_true"] = v_0b[0]
            data_logger_kf.data_dict["v_y_true"] = v_0b[1]
            data_logger_kf.data_dict["v_z_true"] = v_0b[2]
            data_logger_kf.data_dict["p_1x_true"] = p_1_abs[0]
            data_logger_kf.data_dict["p_1y_true"] = p_1_abs[1]
            data_logger_kf.data_dict["p_1z_true"] = p_1_abs[2]
            data_logger_kf.data_dict["p_2x_true"] = p_2_abs[0]
            data_logger_kf.data_dict["p_2y_true"] = p_2_abs[1]
            data_logger_kf.data_dict["p_2z_true"] = p_2_abs[2]
            data_logger_kf.data_dict["p_3x_true"] = p_3_abs[0]
            data_logger_kf.data_dict["p_3y_true"] = p_3_abs[1]
            data_logger_kf.data_dict["p_3z_true"] = p_3_abs[2]
            data_logger_kf.data_dict["p_4x_true"] = p_4_abs[0]
            data_logger_kf.data_dict["p_4y_true"] = p_4_abs[1]
            data_logger_kf.data_dict["p_4z_true"] = p_4_abs[2]
            data_logger_kf.data_dict["p_x_est"] = p_x_est
            data_logger_kf.data_dict["p_y_est"] = p_y_est
            data_logger_kf.data_dict["p_z_est"] = p_z_est
            data_logger_kf.data_dict["v_x_est"] = v_x_est
            data_logger_kf.data_dict["v_y_est"] = v_y_est
            data_logger_kf.data_dict["v_z_est"] = v_z_est
            data_logger_kf.data_dict["p_1x_est"] = p_1x_est
            data_logger_kf.data_dict["p_1y_est"] = p_1y_est
            data_logger_kf.data_dict["p_1z_est"] = p_1z_est
            data_logger_kf.data_dict["p_2x_est"] = p_2x_est
            data_logger_kf.data_dict["p_2y_est"] = p_2y_est
            data_logger_kf.data_dict["p_2z_est"] = p_2z_est
            data_logger_kf.data_dict["p_3x_est"] = p_3x_est
            data_logger_kf.data_dict["p_3y_est"] = p_3y_est
            data_logger_kf.data_dict["p_3z_est"] = p_3z_est
            data_logger_kf.data_dict["p_4x_est"] = p_4x_est
            data_logger_kf.data_dict["p_4y_est"] = p_4y_est
            data_logger_kf.data_dict["p_4z_est"] = p_4z_est
            data_logger_kf.data_dict["y_p_1x"] = y_p_1x
            data_logger_kf.data_dict["y_p_1y"] = y_p_1y
            data_logger_kf.data_dict["y_p_1z"] = y_p_1z
            data_logger_kf.data_dict["y_p_2x"] = y_p_2x
            data_logger_kf.data_dict["y_p_2y"] = y_p_2y
            data_logger_kf.data_dict["y_p_2z"] = y_p_2z
            data_logger_kf.data_dict["y_p_3x"] = y_p_3x
            data_logger_kf.data_dict["y_p_3y"] = y_p_3y
            data_logger_kf.data_dict["y_p_3z"] = y_p_3z
            data_logger_kf.data_dict["y_p_4x"] = y_p_4x
            data_logger_kf.data_dict["y_p_4y"] = y_p_4y
            data_logger_kf.data_dict["y_p_4z"] = y_p_4z
            data_logger_kf.data_dict["y_v_1x"] = y_v_1x
            data_logger_kf.data_dict["y_v_1y"] = y_v_1y
            data_logger_kf.data_dict["y_v_1z"] = y_v_1z
            data_logger_kf.data_dict["y_v_2x"] = y_v_2x
            data_logger_kf.data_dict["y_v_2y"] = y_v_2y
            data_logger_kf.data_dict["y_v_2z"] = y_v_2z
            data_logger_kf.data_dict["y_v_3x"] = y_v_3x
            data_logger_kf.data_dict["y_v_3y"] = y_v_3y
            data_logger_kf.data_dict["y_v_3z"] = y_v_3z
            data_logger_kf.data_dict["y_v_4x"] = y_v_4x
            data_logger_kf.data_dict["y_v_4y"] = y_v_4y
            data_logger_kf.data_dict["y_v_4z"] = y_v_4z
            data_logger_kf.data_dict["P_p_bx_std"] = P_p_bx_std
            data_logger_kf.data_dict["P_p_by_std"] = P_p_by_std
            data_logger_kf.data_dict["P_p_bz_std"] = P_p_bz_std
            data_logger_kf.data_dict["P_v_bx_std"] = P_v_bx_std
            data_logger_kf.data_dict["P_v_by_std"] = P_v_by_std
            data_logger_kf.data_dict["P_v_bz_std"] = P_v_bz_std
            data_logger_kf.data_dict["P_p_1x_std"] = P_p_1x_std
            data_logger_kf.data_dict["P_p_1y_std"] = P_p_1y_std
            data_logger_kf.data_dict["P_p_1z_std"] = P_p_1z_std
            data_logger_kf.data_dict["P_p_2x_std"] = P_p_2x_std
            data_logger_kf.data_dict["P_p_2y_std"] = P_p_2y_std
            data_logger_kf.data_dict["P_p_2z_std"] = P_p_2z_std
            data_logger_kf.data_dict["P_p_3x_std"] = P_p_3x_std
            data_logger_kf.data_dict["P_p_3y_std"] = P_p_3y_std
            data_logger_kf.data_dict["P_p_3z_std"] = P_p_3z_std
            data_logger_kf.data_dict["P_p_4x_std"] = P_p_4x_std
            data_logger_kf.data_dict["P_p_4y_std"] = P_p_4y_std
            data_logger_kf.data_dict["P_p_4z_std"] = P_p_4z_std
            data_logger_kf.data_dict["p_vpsp_x"] = robot.p_b_vpsp[0]
            data_logger_kf.data_dict["p_vpsp_y"] = robot.p_b_vpsp[1]
            data_logger_kf.data_dict["p_vpsp_z"] = robot.p_b_vpsp[2]
            
            # For VPSP visualising #
            data_logger_vpsp.data_dict = {key: 0 for key in data_field_list_vpsp}
            data_logger_vpsp.data_dict["t"] = t
            data_logger_vpsp.data_dict["p_b_vpsp_x"] = robot.p_b_vpsp.reshape(3)[0]
            data_logger_vpsp.data_dict["p_b_vpsp_y"] = robot.p_b_vpsp.reshape(3)[1]
            data_logger_vpsp.data_dict["FL_cw_vp_x"] = robot.virtual_points[0, 0, 0]
            data_logger_vpsp.data_dict["FL_cw_vp_y"] = robot.virtual_points[0, 0, 1]
            data_logger_vpsp.data_dict["FR_cw_vp_x"] = robot.virtual_points[1, 0, 0]
            data_logger_vpsp.data_dict["FR_cw_vp_y"] = robot.virtual_points[1, 0, 1]
            data_logger_vpsp.data_dict["BL_cw_vp_x"] = robot.virtual_points[2, 0, 0]
            data_logger_vpsp.data_dict["BL_cw_vp_y"] = robot.virtual_points[2, 0, 1]
            data_logger_vpsp.data_dict["BR_cw_vp_x"] = robot.virtual_points[3, 0, 0]
            data_logger_vpsp.data_dict["BR_cw_vp_y"] = robot.virtual_points[3, 0, 1]
            data_logger_vpsp.data_dict["FL_ccw_vp_x"] = robot.virtual_points[0, 1, 0]
            data_logger_vpsp.data_dict["FL_ccw_vp_y"] = robot.virtual_points[0, 1, 1]
            data_logger_vpsp.data_dict["FR_ccw_vp_x"] = robot.virtual_points[1, 1, 0]
            data_logger_vpsp.data_dict["FR_ccw_vp_y"] = robot.virtual_points[1, 1, 1]
            data_logger_vpsp.data_dict["BL_ccw_vp_x"] = robot.virtual_points[2, 1, 0]
            data_logger_vpsp.data_dict["BL_ccw_vp_y"] = robot.virtual_points[2, 1, 1]
            data_logger_vpsp.data_dict["BR_ccw_vp_x"] = robot.virtual_points[3, 1, 0]
            data_logger_vpsp.data_dict["BR_ccw_vp_y"] = robot.virtual_points[3, 1, 1]
            data_logger_vpsp.data_dict["FL_vt_x"] = robot.vpsp_vertices[0, 0]
            data_logger_vpsp.data_dict["FL_vt_y"] = robot.vpsp_vertices[1, 0]
            data_logger_vpsp.data_dict["FR_vt_x"] = robot.vpsp_vertices[0, 1]
            data_logger_vpsp.data_dict["FR_vt_y"] = robot.vpsp_vertices[1, 1]
            data_logger_vpsp.data_dict["BL_vt_x"] = robot.vpsp_vertices[0, 2]
            data_logger_vpsp.data_dict["BL_vt_y"] = robot.vpsp_vertices[1, 2]
            data_logger_vpsp.data_dict["BR_vt_x"] = robot.vpsp_vertices[0, 3]
            data_logger_vpsp.data_dict["BR_vt_y"] = robot.vpsp_vertices[1, 3]

            # Write data #
            data_logger_kf.writeData()
            data_logger_vpsp.writeData()

        ### CLOSE CONNECTION TO SIM ###
        sim.simxGetPingTime(client_id)
        sim.simxFinish(client_id)
    print("=== Programme end ===")