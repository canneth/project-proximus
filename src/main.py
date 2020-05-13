
import sim

from Leg import Leg
from Collywobble import Collywobble

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
        _, body_frame = sim.simxGetObjectHandle(client_id, "body_frame", sim.simx_opmode_blocking)
        _, body = sim.simxGetObjectHandle(client_id, "base_link_respondable", sim.simx_opmode_blocking)
        # Front left leg
        _, front_left_coxa_joint = sim.simxGetObjectHandle(client_id, "shoulder_front_left", sim.simx_opmode_blocking)
        _, front_left_coxa = sim.simxGetObjectHandle(client_id, "shoulder_front_left_respondable", sim.simx_opmode_blocking)
        _, front_left_femur_joint = sim.simxGetObjectHandle(client_id, "femur_front_left", sim.simx_opmode_blocking)
        _, front_left_femur = sim.simxGetObjectHandle(client_id, "femur_front_left_respondable", sim.simx_opmode_blocking)
        _, front_left_tibia_joint = sim.simxGetObjectHandle(client_id, "tibia_front_left", sim.simx_opmode_blocking)
        _, front_left_tibia = sim.simxGetObjectHandle(client_id, "tibia_front_left_respondable", sim.simx_opmode_blocking)
        _, front_left_foot = sim.simxGetObjectHandle(client_id, "foot_front_left", sim.simx_opmode_blocking)
        # Front right leg
        _, front_right_coxa_joint = sim.simxGetObjectHandle(client_id, "shoulder_front_right", sim.simx_opmode_blocking)
        _, front_right_coxa = sim.simxGetObjectHandle(client_id, "shoulder_front_right_respondable", sim.simx_opmode_blocking)
        _, front_right_femur_joint = sim.simxGetObjectHandle(client_id, "femur_front_right", sim.simx_opmode_blocking)
        _, front_right_femur = sim.simxGetObjectHandle(client_id, "femur_front_right_respondable", sim.simx_opmode_blocking)
        _, front_right_tibia_joint = sim.simxGetObjectHandle(client_id, "tibia_front_right", sim.simx_opmode_blocking)
        _, front_right_tibia = sim.simxGetObjectHandle(client_id, "tibia_front_right_respondable", sim.simx_opmode_blocking)
        _, front_right_foot = sim.simxGetObjectHandle(client_id, "foot_front_right", sim.simx_opmode_blocking)
        # Back left leg
        _, back_left_coxa_joint = sim.simxGetObjectHandle(client_id, "shoulder_rear_left", sim.simx_opmode_blocking)
        _, back_left_coxa = sim.simxGetObjectHandle(client_id, "shoulder_rear_left_respondable", sim.simx_opmode_blocking)
        _, back_left_femur_joint = sim.simxGetObjectHandle(client_id, "femur_rear_left", sim.simx_opmode_blocking)
        _, back_left_femur = sim.simxGetObjectHandle(client_id, "femur_rear_left_respondable", sim.simx_opmode_blocking)
        _, back_left_tibia_joint = sim.simxGetObjectHandle(client_id, "tibia_rear_left", sim.simx_opmode_blocking)
        _, back_left_tibia = sim.simxGetObjectHandle(client_id, "tibia_rear_left_respondable", sim.simx_opmode_blocking)
        _, back_left_foot = sim.simxGetObjectHandle(client_id, "foot_rear_left", sim.simx_opmode_blocking)
        # Back right leg
        _, back_right_coxa_joint = sim.simxGetObjectHandle(client_id, "shoulder_rear_right", sim.simx_opmode_blocking)
        _, back_right_coxa = sim.simxGetObjectHandle(client_id, "shoulder_rear_right_respondable", sim.simx_opmode_blocking)
        _, back_right_femur_joint = sim.simxGetObjectHandle(client_id, "femur_rear_right", sim.simx_opmode_blocking)
        _, back_right_femur = sim.simxGetObjectHandle(client_id, "femur_rear_right_respondable", sim.simx_opmode_blocking)
        _, back_right_tibia_joint = sim.simxGetObjectHandle(client_id, "tibia_rear_right", sim.simx_opmode_blocking)
        _, back_right_tibia = sim.simxGetObjectHandle(client_id, "tibia_rear_right_respondable", sim.simx_opmode_blocking)
        _, back_right_foot = sim.simxGetObjectHandle(client_id, "foot_rear_right", sim.simx_opmode_blocking)

        ### SETUP ###
        # Legs
        front_left_leg = Leg(
            client_id,
            body_frame,
            front_left_coxa_joint,
            front_left_coxa,
            front_left_femur_joint,
            front_left_femur,
            front_left_tibia_joint,
            front_left_tibia,
            front_left_foot
        )
        front_right_leg = Leg(
            client_id,
            body_frame,
            front_right_coxa_joint,
            front_right_coxa,
            front_right_femur_joint,
            front_right_femur,
            front_right_tibia_joint,
            front_right_tibia,
            front_right_foot
        )
        back_left_leg = Leg(
            client_id,
            body_frame,
            back_left_coxa_joint,
            back_left_coxa,
            back_left_femur_joint,
            back_left_femur,
            back_left_tibia_joint,
            back_left_tibia,
            back_left_foot
        )
        back_right_leg = Leg(
            client_id,
            body_frame,
            back_right_coxa_joint,
            back_right_coxa,
            back_right_femur_joint,
            back_right_femur,
            back_right_tibia_joint,
            back_right_tibia,
            back_right_foot
        )
        # Robot
        robot = Collywobble(
            client_id,
            world_frame,
            body_frame,
            body,
            front_left_leg,
            front_right_leg,
            back_left_leg,
            back_right_leg,
            stance_polygon_width = 0.1,
            swing_to_stance_ratio = 0.5
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

        phase = 0
        base_phase_step = 0.03

        robot.stride_length = 0

        ### LOOP ###
        while True:
            if phase >= 2*PI:
                phase = phase - 2*PI
            robot.moveToPhaseInTrotGait(phase)
            phase = phase + base_phase_step
            pass


        ### CLOSE CONNECTION TO SIM ###
        sim.simxGetPingTime(client_id)
        sim.simxFinish(client_id)
    print("=== Programme end ===")