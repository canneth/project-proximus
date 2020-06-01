
import pybullet
import pybullet_data

import numpy as np
from numpy import pi as PI

import time

from pathlib import Path


if __name__ == "__main__":
    physics_client = pybullet.connect(pybullet.GUI)
    pybullet.setPhysicsEngineParameter(enableFileCaching = 0)
    
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = pybullet.loadURDF("plane.urdf")
    robot_urdf_path = Path("..") / "sim_model" / "A001_full_assem" / "urdf" / "A001_full_assem.urdf"
    robot_id = pybullet.loadURDF(
        fileName = str(robot_urdf_path),
        basePosition = [0, 0, 0.3],
        useFixedBase = 1
    )
    num_of_joints = pybullet.getNumJoints(robot_id)

    joint_positions_before = [joint_state[0] for joint_state in pybullet.getJointStates(robot_id, range(num_of_joints))]

    pybullet.setJointMotorControlArray(
        bodyUniqueId = robot_id,
        jointIndices = range(num_of_joints),
        controlMode = pybullet.POSITION_CONTROL,
        targetPositions = [np.radians(20)]*num_of_joints,
        targetVelocities = [np.radians(300)]*num_of_joints,
        forces = [2.9]*num_of_joints
    )

    ## SIMULATE ## 
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setRealTimeSimulation(0)
    time_step = 1/240 # in seconds
    sim_duration = 3 # in seconds
    for i in range(int(5/time_step)):
        pybullet.stepSimulation()
        time.sleep(time_step)
    
    joint_positions_after = [joint_state[0] for joint_state in pybullet.getJointStates(robot_id, range(num_of_joints))]
    print()
    print("number of joints: {}".format(num_of_joints))
    print("Joint positions before: {}".format(joint_positions_before))
    print("Joint positions after: {}".format(joint_positions_after))

    input()