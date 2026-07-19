import numpy as np
import random

MAZE_SIZE = 33
NUM_CELLS = 16

START_POS  = (1.5, 1.5)
GOAL_CELLS = [(7, 7), (7, 8), (8, 7), (8, 8)]
GOAL_CENTER = (8.0, 8.0)

def _cell_to_grid(row: int, col: int):
    """Cell (row, col) → center index in the 33x33 grid."""
    return (2 * row + 1, 2 * col + 1)


def _remove_wall(grid: np.ndarray, cell_a: tuple, cell_b: tuple):
    """Knock out the wall segment between two adjacent cells."""
    r1, c1 = cell_a
    r2, c2 = cell_b
    gr1, gc1 = _cell_to_grid(r1, c1)
    gr2, gc2 = _cell_to_grid(r2, c2)
    grid[(gr1 + gr2) // 2, (gc1 + gc2) // 2] = 0


def generate_maze(seed: int = 67) -> np.ndarray:
    
    rng = random.Random(seed)

    # Start fully walled, open every cell interior
    grid = np.ones((MAZE_SIZE, MAZE_SIZE), dtype=np.int8)
    for r in range(NUM_CELLS):
        for c in range(NUM_CELLS):
            gr, gc = _cell_to_grid(r, c)
            grid[gr, gc] = 0

    # Recursive backtracker DFS
    visited = np.zeros((NUM_CELLS, NUM_CELLS), dtype=bool)
    stack = [(0, 0)]
    visited[0, 0] = True

    while stack:
        r, c = stack[-1]
        # Collect unvisited orthogonal neighbours
        neighbours = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < NUM_CELLS and 0 <= nc < NUM_CELLS and not visited[nr, nc]:
                neighbours.append((nr, nc))

        if neighbours:
            nr, nc = rng.choice(neighbours)
            _remove_wall(grid, (r, c), (nr, nc))
            visited[nr, nc] = True
            stack.append((nr, nc))
        else:
            stack.pop()

    # Force the central 2x2 goal pocket fully open (no internal walls)
    _remove_wall(grid, (7, 7), (7, 8))
    _remove_wall(grid, (8, 7), (8, 8))
    _remove_wall(grid, (7, 7), (8, 7))
    _remove_wall(grid, (7, 8), (8, 8))

    return grid

# ==========================================
# MAP SETTINGS
# ==========================================
# True  = Uses the same map every time (good for testing/tuning).
# False = Generates a completely new random map on every run.
USE_FIXED_MAP = True

if USE_FIXED_MAP:
    print("[Maze Generator] Loading fixed layout (Seed: 67)...")
    MAZE_GRID: np.ndarray = generate_maze(seed=67)
else:
    print("[Maze Generator] Generating random layout...")
    MAZE_GRID: np.ndarray = generate_maze(seed=None)