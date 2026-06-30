
import numpy as np

# 1 = Wall, 0 = Empty Path
# This is an 11x11 grid layout for the freshers' sandbox/testing stage.
TRAINING_MAZE = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
]

TRAINING_START = (1.5, 1.5)  # Center of top-left path cell
TRAINING_GOAL  = (9.5, 1.5)  # Target coordinates



class MazeEnvironment:
    def __init__(self, layout_matrix, start_pos, goal_pos):
        self.matrix = np.array(layout_matrix, dtype=np.uint8)
        self.height = self.matrix.shape[0]
        self.width = self.matrix.shape[1]
        self.start_pos = start_pos
        self.goal_pos = goal_pos

    def is_wall(self, x: float, y: float) -> bool:
        
        grid_x = int(math.floor(x))
        grid_y = int(math.floor(y))
        

        if grid_x < 0 or grid_x >= self.width or grid_y < 0 or grid_y >= self.height:
            return True
            
        return self.matrix[grid_y, grid_x] == 1


import math
active_env = MazeEnvironment(TRAINING_MAZE, TRAINING_START, TRAINING_GOAL)