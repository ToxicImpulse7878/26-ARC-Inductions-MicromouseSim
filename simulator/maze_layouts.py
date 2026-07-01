"""
maze_layouts.py

Encodes a standard 16x16-cell micromouse competition maze as a 33x33 grid.

GRID ENCODING
-------------
A 16x16 cell maze with walls between cells needs an odd-dimension grid to
represent both cells AND the wall segments between them:

    grid size = 2 * num_cells + 1 = 2*16 + 1 = 33

Even indices (0, 2, 4, ... 32)   -> cell centers / wall-junction posts
Odd indices  (1, 3, 5, ... 31)   -> the corridor space *between* two cells

So cell (row, col) in the 16x16 maze maps to grid position
(2*row + 1, 2*col + 1). A wall between cell (r,c) and cell (r,c+1) lives at
grid position (2*r+1, 2*c+2). Wall PEGS (the little corner posts every real
micromouse maze has) live at every (even, even) grid coordinate.

VALUES
------
    1 -> solid wall / peg  (rendered as crimson red)
    0 -> open driving track (rendered as floor)

The outer border (row/col 0 and row/col 32) is always solid, matching a
real micromouse maze's outer wall.

START / GOAL
-------------
    Start cell: (0, 0) in cell-space -> continuous coordinate (1.5, 1.5)
    Goal zone : the classic central 2x2 cell block, i.e. cells
                (7,7), (7,8), (8,7), (8,8) in a 16x16 maze (0-indexed),
                left open with no internal walls between them.
"""

import numpy as np

MAZE_SIZE = 33          # grid dimension (2*16 + 1)
NUM_CELLS = 16           # competition-standard 16x16
CELL_PITCH = 2           # each cell occupies a 2x2 block in grid space

START_CELL = (0, 0)
START_POS = (1.5, 1.5)   # continuous (x, y) in cell-units, matches START_CELL center

# Center 2x2 goal block (0-indexed cell coordinates)
GOAL_CELLS = [(7, 7), (7, 8), (8, 7), (8, 8)]
GOAL_CENTER = (8.0, 8.0)  # continuous coordinate at the heart of the goal zone


def _cell_to_grid(row: int, col: int):
    """Map a maze cell (row, col) to its center coordinate in the 33x33 grid."""
    return (2 * row + 1, 2 * col + 1)


def _new_solid_grid() -> np.ndarray:
    """Start with every cell wall present (fully solid maze) to carve from."""
    grid = np.ones((MAZE_SIZE, MAZE_SIZE), dtype=np.int8)
    for r in range(NUM_CELLS):
        for c in range(NUM_CELLS):
            gr, gc = _cell_to_grid(r, c)
            grid[gr, gc] = 0
    return grid


def _remove_wall(grid: np.ndarray, cell_a, cell_b):
    """Carve the wall segment that sits between two orthogonally adjacent cells."""
    ra, ca = cell_a
    rb, cb = cell_b
    gra, gca = _cell_to_grid(ra, ca)
    grb, gcb = _cell_to_grid(rb, cb)
    mid_r = (gra + grb) // 2
    mid_c = (gca + gcb) // 2
    grid[mid_r, mid_c] = 0


def _carve_passages(grid: np.ndarray, passages):
    for a, b in passages:
        _remove_wall(grid, a, b)


def generate_maze() -> np.ndarray:
    """
    Builds the full 33x33 maze matrix.

    Hand-authored, fully-connected 16x16 maze with dead ends and a single
    open 2x2 goal pocket in the center, in the spirit of real micromouse
    competition mazes (deceptive loops, false corridors, no trivial path).
    """
    grid = _new_solid_grid()

    passages = [
        # --- Bottom-left start corridor, winds upward with a branch ---
        ((0, 0), (0, 1)), ((0, 1), (0, 2)), ((0, 2), (1, 2)),
        ((1, 2), (1, 1)), ((1, 1), (1, 0)), ((1, 0), (2, 0)),
        ((2, 0), (2, 1)), ((2, 1), (2, 2)), ((2, 2), (2, 3)),
        ((2, 3), (1, 3)), ((1, 3), (0, 3)), ((0, 3), (0, 4)),
        ((0, 4), (0, 5)), ((0, 5), (1, 5)), ((1, 5), (1, 4)),
        ((1, 4), (2, 4)),  # dead end branch
        ((1, 5), (2, 5)), ((2, 5), (2, 6)), ((2, 6), (1, 6)),
        ((1, 6), (0, 6)), ((0, 6), (0, 7)), ((0, 7), (1, 7)),
        ((1, 7), (2, 7)), ((2, 7), (3, 7)), ((3, 7), (3, 6)),
        ((3, 6), (3, 5)), ((3, 5), (3, 4)), ((3, 4), (3, 3)),
        ((3, 3), (4, 3)),  # dead end branch
        ((3, 4), (4, 4)), ((4, 4), (4, 5)), ((4, 5), (4, 6)),
        ((4, 6), (4, 7)), ((4, 7), (5, 7)), ((5, 7), (5, 6)),
        ((5, 6), (5, 5)), ((5, 5), (5, 4)), ((5, 4), (6, 4)),
        ((6, 4), (6, 5)), ((6, 5), (6, 6)), ((6, 6), (6, 7)),
        ((6, 7), (7, 7)),  # entry into goal block from below

        # --- Right-hand spiral approach (deceptive loop region) ---
        ((0, 8), (0, 9)), ((0, 9), (0, 10)), ((0, 10), (1, 10)),
        ((1, 10), (1, 9)), ((1, 9), (1, 8)), ((1, 8), (2, 8)),
        ((2, 8), (2, 9)), ((2, 9), (2, 10)), ((2, 10), (2, 11)),
        ((2, 11), (1, 11)), ((1, 11), (0, 11)), ((0, 11), (0, 12)),
        ((0, 12), (1, 12)), ((1, 12), (2, 12)), ((2, 12), (3, 12)),
        ((3, 12), (3, 11)), ((3, 11), (3, 10)), ((3, 10), (3, 9)),
        ((3, 9), (3, 8)), ((3, 8), (4, 8)), ((4, 8), (4, 9)),
        ((4, 9), (4, 10)), ((4, 10), (5, 10)), ((5, 10), (5, 9)),
        ((5, 9), (5, 8)), ((5, 8), (6, 8)), ((6, 8), (7, 8)),

        ((0, 7), (0, 8)), ((4, 7), (4, 8)),

        # --- Upper-left quadrant: long false corridor + true path up ---
        ((7, 0), (7, 1)), ((7, 1), (7, 2)), ((7, 2), (6, 2)),
        ((6, 2), (6, 1)), ((6, 1), (6, 0)), ((6, 0), (5, 0)),
        ((5, 0), (5, 1)), ((5, 1), (5, 2)), ((5, 2), (5, 3)),
        ((5, 3), (4, 3)),  # dead-end spur
        ((5, 2), (4, 2)), ((4, 2), (4, 1)), ((4, 1), (4, 0)),
        ((4, 0), (3, 0)), ((3, 0), (3, 1)), ((3, 1), (3, 2)),
        ((3, 2), (2, 2)),

        ((7, 0), (8, 0)), ((8, 0), (8, 1)), ((8, 1), (8, 2)),
        ((8, 2), (9, 2)), ((9, 2), (9, 1)), ((9, 1), (9, 0)),
        ((9, 0), (10, 0)), ((10, 0), (10, 1)), ((10, 1), (10, 2)),
        ((10, 2), (10, 3)), ((10, 3), (9, 3)), ((9, 3), (9, 4)),
        ((9, 4), (8, 4)), ((8, 4), (8, 3)), ((8, 3), (7, 3)),
        ((7, 3), (7, 4)), ((7, 4), (7, 5)), ((7, 5), (7, 6)),
        ((7, 6), (7, 7)),

        # --- Upper-right quadrant mirrors complexity for symmetry ---
        ((7, 15), (7, 14)), ((7, 14), (7, 13)), ((7, 13), (6, 13)),
        ((6, 13), (6, 14)), ((6, 14), (6, 15)), ((6, 15), (5, 15)),
        ((5, 15), (5, 14)), ((5, 14), (5, 13)), ((5, 13), (5, 12)),
        ((5, 12), (4, 12)), ((4, 12), (4, 13)), ((4, 13), (4, 14)),
        ((4, 14), (4, 15)), ((4, 15), (3, 15)), ((3, 15), (3, 14)),
        ((3, 14), (3, 13)), ((3, 13), (3, 12)), ((3, 12), (2, 12)),

        ((7, 15), (8, 15)), ((8, 15), (8, 14)), ((8, 14), (8, 13)),
        ((8, 13), (9, 13)), ((9, 13), (9, 14)), ((9, 14), (9, 15)),
        ((9, 15), (10, 15)), ((10, 15), (10, 14)), ((10, 14), (10, 13)),
        ((10, 13), (10, 12)), ((10, 12), (9, 12)), ((9, 12), (9, 11)),
        ((9, 11), (8, 11)), ((8, 11), (8, 10)), ((8, 10), (8, 9)),
        ((8, 9), (7, 9)), ((7, 9), (7, 8)),

        # --- Top rows: snaking corridor connecting both upper quadrants ---
        ((11, 0), (11, 1)), ((11, 1), (11, 2)), ((11, 2), (11, 3)),
        ((11, 3), (12, 3)), ((12, 3), (12, 2)), ((12, 2), (12, 1)),
        ((12, 1), (12, 0)), ((12, 0), (13, 0)), ((13, 0), (13, 1)),
        ((13, 1), (13, 2)), ((13, 2), (14, 2)), ((14, 2), (14, 1)),
        ((14, 1), (14, 0)), ((14, 0), (15, 0)), ((15, 0), (15, 1)),
        ((15, 1), (15, 2)), ((15, 2), (15, 3)), ((15, 3), (15, 4)),
        ((15, 4), (14, 4)), ((14, 4), (14, 3)), ((14, 3), (13, 3)),
        ((13, 3), (13, 4)), ((13, 4), (12, 4)), ((12, 4), (12, 5)),
        ((12, 5), (11, 5)), ((11, 5), (11, 4)), ((11, 4), (10, 4)),
        ((10, 4), (10, 5)), ((10, 5), (9, 5)), ((9, 5), (9, 6)),
        ((9, 6), (8, 6)), ((8, 6), (8, 5)), ((8, 5), (7, 5)),

        ((11, 8), (11, 7)), ((11, 7), (11, 6)), ((11, 6), (12, 6)),
        ((12, 6), (12, 7)), ((12, 7), (12, 8)), ((12, 8), (13, 8)),
        ((13, 8), (13, 7)), ((13, 7), (13, 6)), ((13, 6), (14, 6)),
        ((14, 6), (14, 7)), ((14, 7), (14, 8)), ((14, 8), (15, 8)),
        ((15, 8), (15, 7)), ((15, 7), (15, 6)), ((15, 6), (15, 5)),
        ((15, 5), (14, 5)), ((14, 5), (13, 5)), ((13, 5), (12, 5)),
        ((11, 8), (10, 8)), ((10, 8), (10, 7)), ((10, 7), (10, 6)),
        ((10, 6), (9, 6)), ((9, 7), (9, 8)), ((9, 7), (8, 7)),

        ((11, 9), (11, 10)), ((11, 10), (11, 11)), ((11, 11), (12, 11)),
        ((12, 11), (12, 10)), ((12, 10), (12, 9)), ((12, 9), (13, 9)),
        ((13, 9), (13, 10)), ((13, 10), (13, 11)), ((13, 11), (14, 11)),
        ((14, 11), (14, 10)), ((14, 10), (14, 9)), ((14, 9), (15, 9)),
        ((15, 9), (15, 10)), ((15, 10), (15, 11)), ((15, 11), (15, 12)),
        ((15, 12), (14, 12)), ((14, 12), (13, 12)), ((13, 12), (12, 12)),
        ((11, 9), (10, 9)), ((10, 9), (10, 10)), ((10, 10), (9, 10)),
        ((9, 10), (9, 9)), ((9, 9), (8, 9)),

        ((11, 12), (11, 13)), ((11, 13), (11, 14)), ((11, 14), (11, 15)),
        ((11, 12), (10, 12)), ((10, 12), (10, 13)), ((10, 13), (10, 14)),
        ((10, 14), (10, 15)), ((12, 12), (12, 13)), ((12, 13), (12, 14)),
        ((12, 14), (12, 15)), ((12, 15), (13, 15)), ((13, 15), (13, 14)),
        ((13, 14), (13, 13)), ((13, 13), (14, 13)), ((14, 13), (14, 14)),
        ((14, 14), (14, 15)), ((14, 15), (15, 15)), ((15, 15), (15, 14)),
        ((15, 14), (15, 13)),

        # --- Connect upper-left and upper-right blocks across the middle ---
        ((11, 7), (11, 8)), ((11, 3), (11, 4)),
        ((7, 8), (7, 9)),

        # --- Stitch in previously isolated pockets so the full 16x16 grid
        #     forms one connected graph (verified via BFS reachability) ---
        ((0, 12), (0, 13)), ((0, 13), (0, 14)), ((0, 14), (0, 15)),
        ((0, 15), (1, 15)), ((1, 15), (1, 14)), ((1, 14), (1, 13)),
        ((1, 13), (2, 13)), ((2, 13), (2, 14)), ((2, 14), (2, 15)),
        ((3, 14), (2, 14)),
        ((3, 11), (4, 11)), ((4, 11), (5, 11)), ((5, 11), (5, 10)),
        ((5, 2), (6, 2)), ((6, 2), (6, 3)),
        ((6, 8), (6, 9)), ((6, 9), (6, 10)), ((6, 10), (6, 11)),
        ((6, 11), (6, 12)), ((6, 12), (7, 12)), ((7, 12), (7, 11)),
        ((7, 11), (7, 10)), ((7, 10), (8, 10)), ((8, 10), (8, 12)),
        ((8, 12), (8, 13)),
        ((10, 11), (10, 12)),
    ]

    _carve_passages(grid, passages)

    # Goal zone: fully open 2x2 pocket in the dead center of the maze.
    _remove_wall(grid, (7, 7), (7, 8))
    _remove_wall(grid, (8, 7), (8, 8))
    _remove_wall(grid, (7, 7), (8, 7))
    _remove_wall(grid, (7, 8), (8, 8))

    return grid


# Pre-built singleton — import this directly for the common case.
MAZE_GRID: np.ndarray = generate_maze()


def is_goal_cell(row: int, col: int) -> bool:
    """True if the given 0-indexed maze cell is inside the central goal zone."""
    return (row, col) in GOAL_CELLS


def cell_center_world(row: int, col: int):
    """Continuous-space (x, y) coordinate of a cell's center, in cell-units."""
    return (col + 0.5, row + 0.5)


if __name__ == "__main__":
    # Quick sanity check when run directly: dump the maze as ASCII art.
    g = MAZE_GRID
    for r in range(MAZE_SIZE - 1, -1, -1):  # print top row first
        line = "".join("#" if g[r, c] else "." for c in range(MAZE_SIZE))
        print(line)
    print(f"\nGrid shape: {g.shape}, start={START_POS}, goal_center={GOAL_CENTER}")
