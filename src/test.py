
import sim
import numpy as np
from random import randint

from GaitController import GaitController

if __name__ == "__main__":
    arr = np.array([3, 2, 5 ,1]).reshape(4, 1)
    arr_blocked = np.block(
        [arr, arr ,arr]
    )
    arr_blocked[:, 1] = np.array([0, 0, 0, 0])
    print(arr_blocked)