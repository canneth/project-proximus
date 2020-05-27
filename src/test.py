
import sim
import numpy as np


if __name__ == "__main__":
    arr = np.zeros((2, 3))
    print(arr[:, 1])


    
    # ### BEGIN SIM CONNECTION ###
    # connection_successful = False
    # print("=== Programme START ===")
    # sim.simxFinish(-1) # just in case, close all opened connections
    # client_id = sim.simxStart("127.0.0.1",19999,True,True,5000,5) # Connect to CoppeliaSim
    # # Connection ID of 19997 connects to the simulator without having the simulation running
    # if client_id!=-1:
    #     print ("Connected to remote API server")
    #     connection_successful = True
    # else:
    #     print ("Failed connecting to remote API server")
    #     connection_successful = False

    # if connection_successful:

    #     while True:
    #         pass

    #     ### CLOSE CONNECTION TO SIM ###
    #     sim.simxGetPingTime(client_id)
    #     sim.simxFinish(client_id)
    # print("=== Programme end ===")