
import sim

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
        return_code, world_frame = sim.simxGetObjectHandle(client_id, "world_frame", sim.simx_opmode_blocking)
        return_code, body_frame = sim.simxGetObjectHandle(client_id, "body_frame", sim.simx_opmode_blocking)
        return_code, body = sim.simxGetObjectHandle(client_id, "base_link_respondable", sim.simx_opmode_blocking)
        # Front left leg
        return_code, front_left_coxa_joint = sim.simxGetObjectHandle(client_id, "shoulder_front_left", sim.simx_opmode_blocking)
        return_code, front_left_coxa = sim.simxGetObjectHandle(client_id, "shoulder_front_left_respondable", sim.simx_opmode_blocking)
        return_code, front_left_femur_joint = sim.simxGetObjectHandle(client_id, "femur_front_left", sim.simx_opmode_blocking)
        return_code, front_left_femur = sim.simxGetObjectHandle(client_id, "femur_front_left_respondable", sim.simx_opmode_blocking)
        return_code, front_left_tibia_joint = sim.simxGetObjectHandle(client_id, "tibia_front_left", sim.simx_opmode_blocking)
        return_code, front_left_tibia = sim.simxGetObjectHandle(client_id, "tibia_front_left_respondable", sim.simx_opmode_blocking)
        return_code, front_left_foot = sim.simxGetObjectHandle(client_id, "foot_front_left", sim.simx_opmode_blocking)
        # Front right leg
        return_code, front_right_coxa_joint = sim.simxGetObjectHandle(client_id, "shoulder_front_right", sim.simx_opmode_blocking)
        return_code, front_right_coxa = sim.simxGetObjectHandle(client_id, "shoulder_front_right_respondable", sim.simx_opmode_blocking)
        return_code, front_right_femur_joint = sim.simxGetObjectHandle(client_id, "femur_front_right", sim.simx_opmode_blocking)
        return_code, front_right_femur = sim.simxGetObjectHandle(client_id, "femur_front_right_respondable", sim.simx_opmode_blocking)
        return_code, front_right_tibia_joint = sim.simxGetObjectHandle(client_id, "tibia_front_right", sim.simx_opmode_blocking)
        return_code, front_right_tibia = sim.simxGetObjectHandle(client_id, "tibia_front_right_respondable", sim.simx_opmode_blocking)
        return_code, front_right_foot = sim.simxGetObjectHandle(client_id, "foot_front_right", sim.simx_opmode_blocking)
        # Back left leg
        return_code, back_left_coxa_joint = sim.simxGetObjectHandle(client_id, "shoulder_back_left", sim.simx_opmode_blocking)
        return_code, back_left_coxa = sim.simxGetObjectHandle(client_id, "shoulder_back_left_respondable", sim.simx_opmode_blocking)
        return_code, back_left_femur_joint = sim.simxGetObjectHandle(client_id, "femur_back_left", sim.simx_opmode_blocking)
        return_code, back_left_femur = sim.simxGetObjectHandle(client_id, "femur_back_left_respondable", sim.simx_opmode_blocking)
        return_code, back_left_tibia_joint = sim.simxGetObjectHandle(client_id, "tibia_back_left", sim.simx_opmode_blocking)
        return_code, back_left_tibia = sim.simxGetObjectHandle(client_id, "tibia_back_left_respondable", sim.simx_opmode_blocking)
        return_code, back_left_foot = sim.simxGetObjectHandle(client_id, "foot_back_left", sim.simx_opmode_blocking)
        # Back right leg
        return_code, back_right_coxa_joint = sim.simxGetObjectHandle(client_id, "shoulder_back_right", sim.simx_opmode_blocking)
        return_code, back_right_coxa = sim.simxGetObjectHandle(client_id, "shoulder_back_right_respondable", sim.simx_opmode_blocking)
        return_code, back_right_femur_joint = sim.simxGetObjectHandle(client_id, "femur_back_right", sim.simx_opmode_blocking)
        return_code, back_right_femur = sim.simxGetObjectHandle(client_id, "femur_back_right_respondable", sim.simx_opmode_blocking)
        return_code, back_right_tibia_joint = sim.simxGetObjectHandle(client_id, "tibia_back_right", sim.simx_opmode_blocking)
        return_code, back_right_tibia = sim.simxGetObjectHandle(client_id, "tibia_back_right_respondable", sim.simx_opmode_blocking)
        return_code, back_right_foot = sim.simxGetObjectHandle(client_id, "back_right_left", sim.simx_opmode_blocking)

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
        )

        print("Setup done, entering while loop...")

        ### LOOP ###
        while True:
            robot.simStep()
            pass


        ### CLOSE CONNECTION TO SIM ###
        sim.simxGetPingTime(client_id)
        sim.simxFinish(client_id)
    print("=== Programme end ===")