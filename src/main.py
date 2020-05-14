
import sim

from Leg import Leg
from Collywobble import Collywobble

import numpy as np
from math import pi as PI

import time

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

        # Robot
        robot = Collywobble(
            client_id,
            stance_polygon_width = 0.18
        )

        print("Setup done, entering while loop...")
        
        ### IK CHECK ###
        # _, foot_pos = sim.simxGetObjectPosition(client_id, back_left_foot, body_frame, sim.simx_opmode_blocking)
        # print("Target position: {}".format(foot_pos))

        # robot.front_left_leg.moveFoot([0.19302406907081604, 0.11094817519187927, -0.22628307342529297]) # Corresponds to joint angles (in degrees) [0, -45, 0]
        # robot.front_right_leg.moveFoot([0.19302406907081604, -0.11094817519187927, -0.22628307342529297]) # Corresponds to joint angles (in degrees) [0, -45, 0]
        # robot.back_left_leg.moveFoot([-0.1929716169834137, 0.11089962720870972, -0.22629320621490479]) # Corresponds to joint angles (in degrees) [0, -45, 0]
        # robot.back_right_leg.moveFoot([-0.19296878576278687, -0.11091902107000351, -0.22629079222679138]) # Corresponds to joint angles (in degrees) [0, -45, 0]

        # time.sleep(0.5)
        
        # _, foot_pos = sim.simxGetObjectPosition(client_id, back_left_foot, body_frame, sim.simx_opmode_blocking)
        # print("Current position: {}".format(foot_pos))

        ### MOVE ROBOT TO FEET ORIGINS FIRST ###
        robot.moveFeetToOrigins()
        time.sleep(2) # Wait for simulation to settle

        phase = 0
        base_phase_step = 0.005

        direction_phase = 0
        direction_phase_step = 0.00008

        ### LOOP ###
        while True:
            circle_direction = [np.cos(direction_phase), np.sin(direction_phase)]
            if phase >= 2*PI:
                phase = phase - 2*PI
            if direction_phase >= 2*PI:
                direction_phase = direction_phase - 2*PI
            robot.moveToPhaseInTrotTranslate(phase, direction_vector = [0, 1], stride_length = 0.1, swing_height = 0.16, swing_to_stance_ratio = 0.4)
            phase = phase + base_phase_step
            direction_phase = direction_phase + direction_phase_step

        ### CLOSE CONNECTION TO SIM ###
        sim.simxGetPingTime(client_id)
        sim.simxFinish(client_id)
    print("=== Programme end ===")