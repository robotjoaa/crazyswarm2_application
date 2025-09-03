from enum import Enum
import numpy as np


DIRECTION_TO_MOVEMENT = {
    0: np.array([0, 1, 0]),
    1: np.array([0,-1, 0]),
    2: np.array([ 1, 0, 0]),
    3: np.array([-1, 0, 0]),
    4: np.array([0, 0, 1]),
    5: np.array([0, 0,-1]),
}


class Direction(Enum):
    @property
    def dx_dy_dz(self) -> np.ndarray:
        return DIRECTION_TO_MOVEMENT[self.value]

    NORTH = 0
    SOUTH = 1
    EAST = 2
    WEST = 3
    UP = 4
    DOWN = 5
