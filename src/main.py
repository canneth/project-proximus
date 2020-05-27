
import sim

from Leg import Leg
from Robot import Robot
from Robot import Mode
from Command import Command
from Config import Config
from MasterController import MasterController
from DataLogger import DataLogger

from collections import defaultdict

import numpy as np
from math import pi as PI
from transforms3d.euler import euler2mat

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

        # Data-logging stuff
        data_field_list = (
            [
                "t",
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
            command.body_velocity = [0.2, 0, 0]
            master_controller.stepOnce(robot, command)

            # Collecting and formatting data for collection
            t = time.time() - start_time
            _, v_bb, _ = sim.simxGetObjectVelocity(client_id, robot.body_frame, sim.simx_opmode_streaming)
            _, euler_angles = sim.simxGetObjectOrientation(client_id, robot.body_frame, world_frame, sim.simx_opmode_streaming)
            _, p_1_rel = sim.simxGetObjectPosition(client_id, robot.front_left_leg.foot, robot.body_frame, sim.simx_opmode_streaming)
            _, p_2_rel = sim.simxGetObjectPosition(client_id, robot.front_right_leg.foot, robot.body_frame, sim.simx_opmode_streaming)
            _, p_3_rel = sim.simxGetObjectPosition(client_id, robot.back_left_leg.foot, robot.body_frame, sim.simx_opmode_streaming)
            _, p_4_rel = sim.simxGetObjectPosition(client_id, robot.back_right_leg.foot, robot.body_frame, sim.simx_opmode_streaming)
            R_0b = euler2mat(euler_angles[0], euler_angles[1], euler_angles[2])
            v_0b = R_0b @ v_bb

            data_dict = {key: 0 for key in data_field_list}
            data_dict["t"] = t
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