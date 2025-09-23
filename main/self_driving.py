from picarx import Picarx
import time
import random
import numpy as np
import math
from vilib import Vilib
import readchar
import cv2 
import heapq
from collections import deque
#from scipy.ndimage import binary_dilation


POWER = 50
SAFE_DISTANCE = 40     # >40 cm = safe, go forward
BACKUP_DISTANCE = 15   # <20 cm = backup immediately

# Create an empty 2D map: 100x100 grid initialized with zeros
GRID_SIZE = 100
PIXEL_SIZE = 1
# Example car position at the bottom center (x=0, y=0 in world coords -> mapped to (50,0))

NORTH = 0
EAST = 30
WEST = -30
SOUTH = 0

px = Picarx()



def map_and_plan(env_map, car_x, car_y, Target_address):
    print("map and plan (" + car_x + car_y + Target_address + ")")
    px.set_cam_tilt_angle(-10)
    for i in range(-60, 61, 2):
        px.set_cam_pan_angle(i)
        
        distance = round(px.ultrasonic.read(), 2)
        if distance < 0 or distance > GRID_SIZE*2: 
            continue
        print(distance) 

        angle_rad = i * math.pi / 180
        obstacle_x = car_x + int(round(distance * math.cos(angle_rad)))
        obstacle_y = car_y + int(round(distance * math.sin(angle_rad)))
        if 0 <= obstacle_x < GRID_SIZE and 0 <= obstacle_y < GRID_SIZE:
            env_map[obstacle_y, obstacle_x] = 1

    print("scanned map:")
    print(env_map)
    # wrap the obstacles to fill the measuring gap.
    # After marking obstacles, you can expand them to account for obstacle width
    env_map = cv2.dilate(env_map, np.ones((3,3), np.uint8), iterations=1)
    goal_is_occ = (env_map[Target_address] != 0)
    if goal_is_occ:
        print("goal is occupied")
        env_map[Target_address] = 0

    print("dilated map:")
    print(env_map)

    actions = astar_plan(env_map, (car_y, car_x ), Target_address)
    print(actions)

    return env_map, actions

def move(car_x, car_y, actions):
    if not actions:
        return car_x, car_y
    match actions.popleft():
        case "N":
            px.set_dir_servo_angle(NORTH)

            px.forward(POWER)
            time.sleep(0.1)
            px.forward(0)

            car_y = car_y + 1

            print("moving N")
        case "E":
            px.set_dir_servo_angle(EAST)

            px.forward(POWER)
            time.sleep(0.1)
            px.forward(0)

            car_x = car_x + 1

            print("moving E")
        case "W":
            px.set_dir_servo_angle(WEST)

            px.forward(POWER)
            time.sleep(0.1)
            px.forward(0)

            car_x = car_x - 1
            print("moving W")
        case "S":
            px.set_dir_servo_angle(SOUTH)

            px.backward(POWER)
            time.sleep(0.1)
            px.backward(0)

            car_y = car_y - 1
            print("moving S")

    return car_x, car_y 


def main():
    Vilib.camera_start(vflip=False,hflip=False)
    Vilib.display(local=True,web=True)

    env_map = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)
    car_x = GRID_SIZE // 2
    car_y = 0
    Target_address = (GRID_SIZE-1, GRID_SIZE-1)
    status = "map"
    steps = 0
    actions = deque()

    try:
        while True:
            print("current status:" + status)
            if status == "move" and steps < 5:

                car_x, car_y = move(car_x, car_y, actions)
                steps = steps + 1

            elif status == "move" and steps >= 5:

                car_x, car_y = move(car_x, car_y, actions)
                status = "map"

            elif status == "map":

                env_map, actions = map_and_plan(env_map, car_x, car_y, Target_address)

                if actions is None or len(actions) == 0:
                    # No path found – you might stop, rotate, or expand search area
                    status = "stop"
                    actions = deque()
                else:
                    status = "move"
                    steps = 0

            elif status == "finished":

                return 
            
            elif status == "stop":
                time.sleep(0.5)  
    finally:
        px.forward(0)

MOVES = {
    'N': (-1,  0),
    'S': ( 1,  0),
    'W': ( 0, -1),
    'E': ( 0,  1),
}

def in_bounds(grid, r, c):
    return 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]

def passable(grid, r, c):
    return grid[r, c] == 0

def manhattan(a, b):
    # a, b are (row, col)
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def reconstruct_actions(came_from, start_rc, goal_rc):
    # Rebuild path as a list of (row,col), then convert to 'N','E','S','W'
    path = []
    cur = goal_rc
    while cur != start_rc:
        path.append(cur)
        cur = came_from[cur]
    path.append(start_rc)
    path.reverse()

    # Convert successive cell diffs to actions
    actions = []
    for (r1, c1), (r2, c2) in zip(path[:-1], path[1:]):
        dr, dc = r2 - r1, c2 - c1
        for act, (mr, mc) in MOVES.items():
            if (dr, dc) == (mr, mc):
                actions.append(act)
                break
    return actions

def astar_plan(grid, start_rc, goal_rc):
    """
    grid: 2D numpy array (0 free, 1 obstacle)
    start_rc: (row, col)
    goal_rc: (row, col)
    return: deque(['N','N','E',...]) or None if no path
    """
    print("astar planning")
    if not in_bounds(grid, *start_rc) or not in_bounds(grid, *goal_rc):
        return None
    if not passable(grid, *start_rc) or not passable(grid, *goal_rc):
        return None

    # Min-heap of (f_score, g_score, (row,col))
    open_heap = []
    heapq.heappush(open_heap, (0 + manhattan(start_rc, goal_rc), 0, start_rc))

    came_from = {}                  # (row,col) -> parent (row,col)
    g_score = {start_rc: 0}
    visited = set()

    while open_heap:
        f, g, cur = heapq.heappop(open_heap)
        if cur in visited:
            continue
        visited.add(cur)

        if cur == goal_rc:
            actions = reconstruct_actions(came_from, start_rc, goal_rc)
            return deque(actions)   # ready for actions.popleft() in your loop

        # Explore neighbors
        for act, (dr, dc) in MOVES.items():
            nr, nc = cur[0] + dr, cur[1] + dc
            if not in_bounds(grid, nr, nc) or not passable(grid, nr, nc):
                continue
            ng = g + 1
            if ng < g_score.get((nr, nc), 1e15):
                g_score[(nr, nc)] = ng
                came_from[(nr, nc)] = cur
                nf = ng + manhattan((nr, nc), goal_rc)
                heapq.heappush(open_heap, (nf, ng, (nr, nc)))

    return None  # no path

if __name__ == "__main__":
    main()