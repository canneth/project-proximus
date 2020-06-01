
from enum import Enum


class Mode(Enum):
    REST = 0
    TROT = 1

class Gait(Enum):
    TROT = 0

class FootTrajectory(Enum):
    SEMICIRCULAR = 0,
    TRIANGULAR = 1
