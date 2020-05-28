
import sim

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
    Generates the control input matrix using sensor values from the imu and the velocity inputs from command.

    ARGUMENTS:
    + imu: An IMU object
    + command: A Command object

    RETURNS:
    + B: A (18, 1) array; it is the control input matrix.
    """
    # Read quaternion from IMU
    q = np.array(imu.quaternion_vals)
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
            [0.5*dt*a_0_b + v_0_command], # v_b
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
    Returns a P_new which is a modified P.
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
    if ((P_prev is None) or (contact_pattern_index is None)):
        # Create P_init
        P_new = P_init
        return P_new
    else:
        P_new = np.copy(P_prev)
        # Modify elements based on contact_pattern_index
        inf = 10**100
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

def generateProcessCovarianceMatrix(v_b_std, p_1_std, p_2_std, p_3_std, p_4_std, dt, contact_pattern = None):
    """
    DESCRIPTION:
    Generates a new process covariance matrix given the previous Q_prev and the current contact_pattern_index.
    Returns a P_new which is a modified P.
    If contact_pattern = None, then it assumes a contact pattern of [1, 1, 1, 1] (all legs in stance).

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
    Q_body = np.block(
        [
            (v_b_std**2)*[Q_intermediate[0, 0]*I, Q_intermediate[0, 1]*I],
            (v_b_std**2)*[Q_intermediate[1, 0]*I, Q_intermediate[1, 1]*I],
        ]
    )
    p_i_stds = np.array([p_1_std, p_2_std, p_3_std, p_4_std]).reshape(4)
    mask_from_contact_pattern = contact_pattern.reshape(4)
    inf = 10**100
    p_i_stds[mask_from_contact_pattern == 0] = inf # For feet in swing, send uncertainty into the sky
    Q_feet = np.block(
        [
            p_i_stds[0]*[I*dt, O, O, O],
            p_i_stds[1]*[O, I*dt, O, O],
            p_i_stds[2]*[O, O, I*dt, O],
            p_i_stds[3]*[O, O, O, I*dt]
        ]
    )
    Q = np.block(
        [
            [Q_body, np.zeros((4, 4))],
            [np.zeros((4, 4)), Q_feet]
        ]
    )
    return Q

if __name__ == "__main__":
    
    ### BEGIN SIM CONNECTION ###
    connection_successful = False
    print("=== Programme START ===")
    sim.simxFinish(-1) # just in case, close all opened connections
    client_id = sim.simxStart("127.0.0.1",19999,True,True,5000,5) # Connect to CoppeliaSim
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
        
        # Sensors
        imu = IMU(client_id)

        # Kalman Filter variable preparation
        dt = config.dt
        x_init = np.block(
            [
                [np.zeros((3, 1))], # p_b
                [np.zeros((3, 1))], # v_b
                [robot.foot_locations_wrt_body_at_rest[:, 0].reshape(3, 1)], # p_1
                [robot.foot_locations_wrt_body_at_rest[:, 1].reshape(3, 1)], # p_2
                [robot.foot_locations_wrt_body_at_rest[:, 2].reshape(3, 1)], # p_3
                [robot.foot_locations_wrt_body_at_rest[:, 3].reshape(3, 1)], # p_4
            ]
        )
        
        I = np.identity(3) # For convenience
        O = np.zeros((3, 3)) # For convenience

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

        B = generateControlInputMatrix(imu, command)
        
        P_init = generateStateCovarianceMatrix(
            p_b_std = 0, # Very certain about initial body position
            v_b_std = 0, # Very certain about initial body velocity
            p_1_std = 0, # Very certain about initial foot position
            p_2_std = 0, # Very certain about initial foot position
            p_3_std = 0, # Very certain about initial foot position
            p_4_std = 0 # Very certain about initial foot position
        )

        # Kalman Filter
        kalman_filter = KalmanFilter(dim_x = 18, dim_z = 1)

        # Data-logging stuff
        data_field_list = (
            [
                "t",
                "contact_pattern_index",
                "v_x_true", "v_y_true", "v_z_true",
                "p_1x_true", "p_1y_true", "p_1z_true",
                "p_2x_true", "p_2y_true", "p_2z_true",
                "p_3x_true", "p_3y_true", "p_3z_true",
                "p_4x_true", "p_4y_true", "p_4z_true",
                "v_x_est", "v_y_est", "v_z_est",
                "p_1x_est", "p_1y_est", "p_1z_est",
                "p_2x_est", "p_2y_est", "p_2z_est",
                "p_3x_est", "p_3y_est", "p_3z_est",
                "p_4x_est", "p_4y_est", "p_4z_est"
            ]
        )
        data_logger = DataLogger(
            data_fields = data_field_list
        )

        # Create configuration object which stores all configuration parameters used by almost everything
        config = Config()

        # Master Controller
        master_controller = MasterController(config)

        # Robot
        robot = Robot(
            client_id,
            stance_polygon_length = 0.4,
            stance_polygon_width = 0.18,
            stance_height = 0.225,
            swing_height = 0.08
        )

        command = Command()
        command.stance_height = 0.2
        master_controller.stepOnce(robot, command)
        time.sleep(1) # Wait for simulation to settle

        last_time = time.time()
        start_time = time.time()

        ### LOOP ###
        while True:
            current_time = time.time()
            elapsed_time = current_time - last_time
            if (elapsed_time < config.dt):
                continue

            last_time = time.time()
            command.mode = Mode.TROT
            command.body_velocity = [0.4, 0, 0]
            master_controller.stepOnce(robot, command)

            # Collecting and formatting data for collection
            t = time.time() - start_time
            _, v_0b, _ = sim.simxGetObjectVelocity(client_id, robot.body_frame, sim.simx_opmode_streaming)
            # _, euler_angles = sim.simxGetObjectOrientation(client_id, robot.body_frame, world_frame, sim.simx_opmode_streaming)
            _, p_1_rel = sim.simxGetObjectPosition(client_id, robot.front_left_leg.foot, robot.body_frame, sim.simx_opmode_streaming)
            _, p_2_rel = sim.simxGetObjectPosition(client_id, robot.front_right_leg.foot, robot.body_frame, sim.simx_opmode_streaming)
            _, p_3_rel = sim.simxGetObjectPosition(client_id, robot.back_left_leg.foot, robot.body_frame, sim.simx_opmode_streaming)
            _, p_4_rel = sim.simxGetObjectPosition(client_id, robot.back_right_leg.foot, robot.body_frame, sim.simx_opmode_streaming)
            contact_pattern_index = getContactPatternIndex(
                contact_pattern = robot.contact_pattern,
                gait_schedule = master_controller.trot_controller.gait_config.contact_schedule
            )
            # R_0b = euler2mat(euler_angles[0], euler_angles[1], euler_angles[2])
            # v_0b = R_0b @ v_bb

            data_dict = {key: 0 for key in data_field_list}
            data_dict["t"] = t
            data_dict["contact_pattern_index"] = contact_pattern_index
            data_dict["v_x_true"] = v_0b[0]
            data_dict["v_y_true"] = v_0b[1]
            data_dict["v_z_true"] = v_0b[2]
            data_dict["p_1x_true"] = p_1_rel[0]
            data_dict["p_1y_true"] = p_1_rel[1]
            data_dict["p_1z_true"] = p_1_rel[2]
            data_dict["p_2x_true"] = p_2_rel[0]
            data_dict["p_2y_true"] = p_2_rel[1]
            data_dict["p_2z_true"] = p_2_rel[2]
            data_dict["p_3x_true"] = p_3_rel[0]
            data_dict["p_3y_true"] = p_3_rel[1]
            data_dict["p_3z_true"] = p_3_rel[2]
            data_dict["p_4x_true"] = p_4_rel[0]
            data_dict["p_4y_true"] = p_4_rel[1]
            data_dict["p_4z_true"] = p_4_rel[2]
            # TODO: Implement Kalman Filter and log the estimated states!
            data_dict["v_x_est"] = 0
            data_dict["v_y_est"] = 0
            data_dict["v_z_est"] = 0
            data_dict["p_1x_est"] = 0
            data_dict["p_1y_est"] = 0
            data_dict["p_1z_est"] = 0
            data_dict["p_2x_est"] = 0
            data_dict["p_2y_est"] = 0
            data_dict["p_2z_est"] = 0
            data_dict["p_3x_est"] = 0
            data_dict["p_3y_est"] = 0
            data_dict["p_3z_est"] = 0
            data_dict["p_4x_est"] = 0
            data_dict["p_4y_est"] = 0
            data_dict["p_4z_est"] = 0

            data_logger.writeData(
                data_dict
            )

        ### CLOSE CONNECTION TO SIM ###
        sim.simxGetPingTime(client_id)
        sim.simxFinish(client_id)
    print("=== Programme end ===")