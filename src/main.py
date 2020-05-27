
import sim

from Leg import Leg
from Robot import Robot
from Robot import Mode
from Command import Command
from Config import Config
from MasterController import MasterController

import numpy as np
from math import pi as PI

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


        command = Command()
        command.stance_height = 0.2
        master_controller.stepOnce(robot, command)
        time.sleep(1) # Wait for simulation to settle

        last_time = time.time()

        ### LOOP ###
        while True:

            current_time = time.time()
            elapsed_time = current_time - last_time
            if (elapsed_time < config.dt):
                continue

            last_time = time.time()
            command.mode = Mode.TROT
            command.body_roll = np.deg2rad(10)
            command.body_velocity = [0.2, 0, 0]
            master_controller.stepOnce(robot, command)

        ### CLOSE CONNECTION TO SIM ###
        sim.simxGetPingTime(client_id)
        sim.simxFinish(client_id)
    print("=== Programme end ===")