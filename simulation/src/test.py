
import sim
import numpy as np
from random import randint

from GaitController import GaitController

if __name__ == "__main__":
    arr = np.array([3, 2, 5 ,1])
    for i in range(4):
        arr[i] = 0
    print(arr)