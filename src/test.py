
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

        _, front_left_collision_handle = sim.simxGetCollisionHandle(client_id, "front_left_collision_handle", sim.simx_opmode_blocking)
        _, front_right_collision_handle = sim.simxGetCollisionHandle(client_id, "front_right_collision_handle", sim.simx_opmode_blocking)
        _, back_left_collision_handle = sim.simxGetCollisionHandle(client_id, "back_left_collision_handle", sim.simx_opmode_blocking)
        _, back_right_collision_handle = sim.simxGetCollisionHandle(client_id, "back_right_collision_handle", sim.simx_opmode_blocking)

        while True:
            _, front_left_collision_state = sim.simxReadCollision(client_id, front_left_collision_handle, sim.simx_opmode_streaming)
            _, front_right_collision_state = sim.simxReadCollision(client_id, front_right_collision_handle, sim.simx_opmode_streaming)
            _, back_left_collision_state = sim.simxReadCollision(client_id, back_left_collision_handle, sim.simx_opmode_streaming)
            _, back_right_collision_state = sim.simxReadCollision(client_id, back_right_collision_handle, sim.simx_opmode_streaming)
            print(front_left_collision_state, front_right_collision_state, back_left_collision_state, back_right_collision_state)

        ### CLOSE CONNECTION TO SIM ###
        sim.simxGetPingTime(client_id)
        sim.simxFinish(client_id)
    print("=== Programme end ===")