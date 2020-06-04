
import numpy as np
from transforms3d.euler import euler2mat

if __name__ == "__main__":
    print(euler2mat(np.radians(10), np.radians(20), np.radians(5)))

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

    