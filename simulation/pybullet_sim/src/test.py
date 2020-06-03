
import pybullet

import numpy as np

from Robot import Robot

if __name__ == "__main__":
    physics_client = pybullet.connect(pybullet.GUI)
    pybullet.setPhysicsEngineParameter(enableFileCaching = 0)
    robot = Robot()
    joint_angles = robot.back_right_leg.ikFoot([-0.2, -0.1, -0.2])
    print(joint_angles)
    joint_angles = robot.back_right_leg.ikFoot([-0.15, 0, -0.1])
    print(joint_angles)

    """
    ##### IK TEST CASES #####

    ### FRONT-LEFT LEG:
    Foot position       |       Joint angles
    [0.2, 0.1, -0.2]    |       [-0.05549023  0.69678768 -0.24848678]
    [0.15, 0, -0.1]     |       [-0.33927255 -0.19747505 -0.98955413]
    ### FRONT-RIGHT LEG:
    Foot position       |       Joint angles
    [0.2, -0.1, -0.2]   |       [-0.05549023  0.69678768 -0.24848678]
    [0.15, 0, -0.1]     |       [-0.33927255 -0.19747505 -0.98955413]
    ### BACK-LEFT LEG:
    Foot position       |       Joint angles
    [-0.2, 0.1, -0.2]   |       [-0.05549023  0.62552186 -0.24848678]
    [0.15, 0, -0.1]     |       [-0.33927255  0.77871725 -0.98955413]
    ### BACK-RIGHT LEG:
    Foot position       |       Joint angles
    [-0.2, -0.1, -0.2]   |       [-0.05549023  0.62552186 -0.24848678]
    [0.15, 0, -0.1]     |       [-0.33927255  0.77871725 -0.98955413]

    """

    