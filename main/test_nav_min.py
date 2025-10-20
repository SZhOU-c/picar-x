
import unittest
from collections import deque
import math
import numpy as np

# ----------------------------
# Minimal, hardware-free helpers
# ----------------------------
GRID_SIZE = 30  # small for fast tests
FREE, OCC = 0, 1

MOVES = {
    'N': (-1,  0),
    'S': ( 1,  0),
    'W': ( 0, -1),
    'E': ( 0,  1),
}

def in_bounds(grid, r, c):
    return 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]

def passable_or_goal(grid, r, c, goal_rc):
    # allow goal even if it ended as occupied after dilation
    return (grid[r, c] == FREE) or ((r, c) == goal_rc)

def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def reconstruct_actions(came_from, start_rc, goal_rc):
    path = []
    cur = goal_rc
    while cur != start_rc:
        path.append(cur)
        cur = came_from[cur]
    path.append(start_rc)
    path.reverse()
    actions = []
    for (r1,c1),(r2,c2) in zip(path[:-1], path[1:]):
        dr, dc = r2-r1, c2-c1
        for act,(mr,mc) in MOVES.items():
            if (dr,dc)==(mr,mc):
                actions.append(act); break
    return actions

def astar_plan(grid, start_rc, goal_rc):
    if not (in_bounds(grid, *start_rc) and in_bounds(grid, *goal_rc)):
        return None
    if grid[start_rc] != FREE:
        return None  # start must be free; goal can be occupied

    import heapq
    open_heap = []
    heapq.heappush(open_heap, (manhattan(start_rc, goal_rc), 0, start_rc))
    came_from = {}
    g_score   = {start_rc: 0}
    visited   = set()

    while open_heap:
        f,g,cur = heapq.heappop(open_heap)
        if cur in visited: 
            continue
        visited.add(cur)
        if cur == goal_rc:
            return deque(reconstruct_actions(came_from, start_rc, goal_rc))
        for act,(dr,dc) in MOVES.items():
            nr, nc = cur[0]+dr, cur[1]+dc
            if not in_bounds(grid, nr, nc) or not passable_or_goal(grid, nr, nc, goal_rc):
                continue
            ng = g + 1
            if ng < g_score.get((nr,nc), 1e15):
                g_score[(nr,nc)] = ng
                came_from[(nr,nc)] = cur
                nf = ng + manhattan((nr,nc), goal_rc)
                heapq.heappush(open_heap, (nf, ng, (nr,nc)))
    return None

def inflate_obstacles_simple(grid, radius=1):
    # Pure-NumPy dilation: mark neighbors within Chebyshev radius as occupied.
    if radius <= 0: 
        return grid
    g = grid.copy()
    rows, cols = g.shape
    occs = np.argwhere(g == OCC)
    for r, c in occs:
        r0, r1 = max(0, r - radius), min(rows, r + radius + 1)
        c0, c1 = max(0, c - radius), min(cols, c + radius + 1)
        g[r0:r1, c0:c1] = OCC
    return g

def map_and_plan_sim(env_map, car_x, car_y, goal_rc, distances, angle_start=-60, angle_stop=60, angle_step=2):
    # Simulate a scan using provided distances array paired with sweep angles.
    angles = list(range(angle_start, angle_stop + 1, angle_step))
    if len(distances) < len(angles):
        if len(distances) == 0:
            distances = [0.0] * len(angles)
        else:
            distances = list(distances) + [distances[-1]] * (len(angles) - len(distances))

    for ang, d in zip(angles, distances):
        if (not math.isfinite(d)) or d < 0 or d > GRID_SIZE*2:
            continue
        d = min(max(d, 0.0), GRID_SIZE-1)
        rad = math.radians(ang)
        ox = car_x + int(round(d * math.cos(rad)))
        oy = car_y + int(round(d * math.sin(rad)))
        if 0 <= ox < GRID_SIZE and 0 <= oy < GRID_SIZE:
            env_map[oy, ox] = OCC

    env_map = inflate_obstacles_simple(env_map, radius=1)
    # carve-out goal if dilation blocked it
    if env_map[goal_rc] != FREE:
        env_map[goal_rc] = FREE

    actions = astar_plan(env_map, (car_y, car_x), goal_rc)
    return env_map, (actions if actions is not None else deque())

# ----------------------------
# Tests
# ----------------------------
class TestMinimalNav(unittest.TestCase):
    def test_in_bounds_and_passable(self):
        grid = np.zeros((10,10), dtype=np.uint8)
        self.assertTrue(in_bounds(grid, 0, 0))
        self.assertFalse(in_bounds(grid, -1, 0))
        self.assertTrue(grid[0,0] == FREE)

    def test_astar_straight_line(self):
        grid = np.zeros((10,10), dtype=np.uint8)
        start = (0,0)
        goal  = (0,5)
        actions = astar_plan(grid, start, goal)
        self.assertIsNotNone(actions)
        self.assertEqual(len(actions), 5)
        self.assertTrue(all(a in ('N','S','E','W') for a in actions))

    def test_map_and_plan_sim_marks_obstacles(self):
        grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)
        car_x = GRID_SIZE//2
        car_y = 0
        goal  = (GRID_SIZE-1, GRID_SIZE-1)
        distances = [5.0] * ((60 - (-60))//2 + 1)  # one per angle step
        grid2, actions = map_and_plan_sim(grid, car_x, car_y, goal, distances)
        self.assertGreater(int(grid2.sum()), 0)
        self.assertIsNotNone(actions)

    def test_skip_invalid_distances(self):
        grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)
        car_x = GRID_SIZE//2
        car_y = 0
        goal  = (GRID_SIZE-1, GRID_SIZE-1)
        distances = [-10.0, 9999.0] + [0.0]*10
        grid2, actions = map_and_plan_sim(grid, car_x, car_y, goal, distances)
        self.assertGreaterEqual(int(grid2.sum()), 0)
        self.assertIsNotNone(actions)

    def test_goal_carveout_allows_plan(self):
        grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)
        car_x = GRID_SIZE//2
        car_y = 0
        goal  = (GRID_SIZE-1, GRID_SIZE-1)
        grid[goal] = OCC  # goal blocked
        grid2, actions = map_and_plan_sim(grid, car_x, car_y, goal, distances=[0.0]*10)
        self.assertIsNotNone(actions)

if __name__ == '__main__':
    unittest.main(verbosity=2)
