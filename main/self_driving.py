from picarx import Picarx
import time
import random
import numpy as np
import math
from vilib import Vilib
import cv2 
import heapq
from collections import deque
#from scipy.ndimage import binary_dilation


POWER = 50
SAFE_DISTANCE = 40     # >40 cm = safe, go forward
BACKUP_DISTANCE = 15   # <20 cm = backup immediately

# Create an empty 2D map: 100x100 grid initialized with zeros
GRID_SIZE = 200

# Example car position at the bottom center (x=0, y=0 in world coords -> mapped to (50,0))

CELL_CM = 0.5          # 0.5 cm per grid cell
TURN_DEG = 7.0
FWD_CM   = 8.5       # forward straight step
FARC_CM  = 6.50        # forward after 7° turn
BACK_CM  = 7.00        # backward straight step
HEADING_BINS = 32      # quantize theta for CLOSED set (≈11.25°/bin)

STEER_STRAIGHT = 0
DT_S = 0.3
STEER_LEFT = -30
STEER_RIGHT = 30
TURN_RAD = math.radians(TURN_DEG)

# Goal region in GRID coordinates (row 0 = bottom)
GOAL_ROW_MIN, GOAL_ROW_MAX = 180, 199
GOAL_COL_MIN, GOAL_COL_MAX = 180, 199

# Cost shaping (optional)
TURN_PENALTY = 0.1     # small cost to prefer fewer turns
BACK_PENALTY = 5.0     # big cost so reverse is last resort

# Collision sampling resolution along a step
SAMPLE_STEP_CM = CELL_CM  # sample every cell length along motion
# Goal region in GRID coords (row 0 at bottom)


px = Picarx()



def map_and_plan(env_map, car_x, car_y, th0):
    print(f"map and plan (car_x={car_x}, car_y={car_y})")
    px.set_cam_tilt_angle(-10)
    for i in range(-60, 61, 2):
        px.set_cam_pan_angle(i)
        
        distance = round(px.ultrasonic.read(), 2)
        if distance < 0 or distance > GRID_SIZE: 
            continue

        angle_rad = i * math.pi / 180
        beam = th0 + angle_rad  # add car heading
        # World hit point (in cm), relative to car’s pose
        hit_x_cm = car_x + distance * math.cos(beam)
        hit_y_cm = car_y + distance * math.sin(beam)

        # Convert to grid indices (row,col), cell = 0.5 cm
        obstacle_x = int(hit_x_cm / 0.5)
        obstacle_y = int(hit_y_cm / 0.5)
        if 0 <= obstacle_x < GRID_SIZE and 0 <= obstacle_y < GRID_SIZE:
            env_map[obstacle_y, obstacle_x] = 1

    print("scanned map:")
    print(env_map)
    # wrap the obstacles to fill the measuring gap.
    # After marking obstacles, you can expand them to account for obstacle width
    env_map = cv2.dilate(env_map, np.ones((3,3), np.uint8), iterations=1)


    print("dilated map:")
    print(env_map)
    
    actions = deque(A_star(env_map, car_x, car_y, th0))
    print("actions", actions)

    return env_map, actions



def move(x_cm, y_cm, theta, actions):
    if not actions:
        return x_cm, y_cm, theta

    label = actions.popleft()

    if label == 'f':
        # Straight forward
        px.set_dir_servo_angle(STEER_STRAIGHT)
        px.forward(POWER)
        time.sleep(DT_S)
        px.forward(0)
        time.sleep(DT_S)

        # Pose update: forward vector with θ=0 → +Y ⇒ (sinθ, cosθ)
        x_cm = x_cm + FWD_CM * math.sin(theta)
        y_cm = y_cm + FWD_CM * math.cos(theta)
        # theta unchanged

    elif label == 'fl':
        # Turn heading +7°, then move forward
        px.set_dir_servo_angle(STEER_LEFT)
        px.forward(POWER)
        time.sleep(DT_S)
        px.forward(0)
        # sleep to make sure consistent distance
        time.sleep(DT_S)

        theta = wrap_angle(theta + TURN_RAD)
        x_cm = x_cm + FARC_CM * math.sin(theta)
        y_cm = y_cm + FARC_CM * math.cos(theta)

    elif label == 'fr':
        # Turn heading -7°, then move forward
        px.set_dir_servo_angle(STEER_RIGHT)
        px.forward(POWER)
        time.sleep(DT_S)
        px.forward(0)
        time.sleep(DT_S)

        theta = wrap_angle(theta - TURN_RAD)
        x_cm = x_cm + FARC_CM * math.sin(theta)
        y_cm = y_cm + FARC_CM * math.cos(theta)

    elif label == 'b':
        # Straight backward
        px.set_dir_servo_angle(STEER_STRAIGHT)
        px.backward(POWER)
        time.sleep(DT_S)
        px.backward(0)
        time.sleep(DT_S)

        x_cm = x_cm - BACK_CM * math.sin(theta)
        y_cm = y_cm - BACK_CM * math.cos(theta)
        # theta unchanged

    else:
        # Unknown action: ignore
        pass

    return x_cm, y_cm, theta


def main():
    #Vilib.camera_start(vflip=False,hflip=False)
    #Vilib.display(local=True,web=True)

    env_map = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)
     # 200 * 0.5 / 2 = 50.0 cm
    car_x = (GRID_SIZE * CELL_CM) / 2.0
    car_y = 0
    status = "map"
    steps = 0
    actions = deque()
    th0    = 0.0  # facing +Y
    print("init sucessfully")

    try:
        while True:
            print("current status:", status, "steps = ", steps)
            if status == "move" and steps < 5:

                car_x, car_y, th0 = move(car_x, car_y, th0, actions)
                steps += 1

            elif status == "move" and steps >= 5:

                car_x, car_y, th0 = move(car_x, car_y, th0, actions)
                status = "map"

            elif status == "map":

                env_map, actions = map_and_plan(env_map, car_x, car_y, th0)

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

def in_bounds(grid, r, c):
    return 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]

def passable(grid, r, c):
    return grid[r, c] == 0

def world_to_grid(x_cm, y_cm):
    """World: x right, y up  ->  Grid index: (row bottom-up, col right)."""
    col = int(math.floor(x_cm / CELL_CM))
    row = int(math.floor(y_cm / CELL_CM))
    return row, col

def grid_to_world_center(row, col):
    """Center of a grid cell in world cm."""
    x_cm = (col + 0.5) * CELL_CM
    y_cm = (row + 0.5) * CELL_CM
    return x_cm, y_cm

def in_goal_region_rc(row, col):
    return (GOAL_ROW_MIN <= row <= GOAL_ROW_MAX) and (GOAL_COL_MIN <= col <= GOAL_COL_MAX)

def dist_to_goal_rect_cm(x_cm, y_cm):
    """Euclidean distance from (x,y) to the goal rectangle (in cm)."""
    x_min, x_max = GOAL_COL_MIN*CELL_CM, (GOAL_COL_MAX+1)*CELL_CM
    y_min, y_max = GOAL_ROW_MIN*CELL_CM, (GOAL_ROW_MAX+1)*CELL_CM
    dx = 0.0 if (x_min <= x_cm <= x_max) else (x_min - x_cm if x_cm < x_min else x_cm - x_max)
    dy = 0.0 if (y_min <= y_cm <= y_max) else (y_min - y_cm if y_cm < y_min else y_cm - y_max)
    return math.hypot(dx, dy)

def wrap_angle(theta):
    """Wrap to [-pi, pi)."""
    return (theta + math.pi) % (2*math.pi) - math.pi

def heading_bin(theta):
    """Quantize heading for CLOSED set; θ=0 is +Y."""
    t = wrap_angle(theta)
    step = 2*math.pi / HEADING_BINS
    return int(round(t / step)) % HEADING_BINS

# -------------------------
# Motion primitives (θ=0 → +Y)
# -------------------------


def step_forward(x, y, th):
    """f: move straight FWD_CM along current heading (θ=0 is +Y)."""
    s = FWD_CM
    x2 = x + s * math.sin(th)
    y2 = y + s * math.cos(th)
    th2 = th
    label = 'f'
    edge_cost = s  # path length cost
    return x2, y2, th2, edge_cost, label

def step_forward_left(x, y, th):
    """fl: turn +7° (left), then move straight FARC_CM along new heading."""
    th2 = wrap_angle(th + TURN_RAD)
    s   = FARC_CM
    x2  = x + s * math.sin(th2)
    y2  = y + s * math.cos(th2)
    label = 'fl'
    edge_cost = s + TURN_PENALTY
    return x2, y2, th2, edge_cost, label

def step_forward_right(x, y, th):
    """fr: turn -7° (right), then move straight FARC_CM along new heading."""
    th2 = wrap_angle(th - TURN_RAD)
    s   = FARC_CM
    x2  = x + s * math.sin(th2)
    y2  = y + s * math.cos(th2)
    label = 'fr'
    edge_cost = s + TURN_PENALTY
    return x2, y2, th2, edge_cost, label

def step_backward(x, y, th):
    """b: move BACK_CM backward (opposite heading)."""
    s = BACK_CM
    x2 = x - s * math.sin(th)
    y2 = y - s * math.cos(th)
    th2 = th
    label = 'b'
    edge_cost = s + BACK_PENALTY
    return x2, y2, th2, edge_cost, label

def successors(x, y, th):
    """Generate all 4 successors."""
    return (
        step_forward(x,y,th),
        step_forward_left(x,y,th),
        step_forward_right(x,y,th),
        step_backward(x,y,th),
    )

# -------------------------
# Collision checking
# -------------------------
def collision_free_segment(grid, x0,y0, x1,y1, samples_cm=SAMPLE_STEP_CM):
    """Sample along the straight chord between (x0,y0) and (x1,y1).
       With small steps + inflated map, this is accurate and fast."""
    total = math.hypot(x1-x0, y1-y0)
    n = max(1, int(math.ceil(total / samples_cm)))
    for i in range(n+1):
        t = i / n
        xs = x0 + t*(x1-x0)
        ys = y0 + t*(y1-y0)
        r, c = world_to_grid(xs, ys)
        if not in_bounds(grid, r, c) or not passable(grid, r, c):
            return False
    return True

# -------------------------
# A* (Hybrid-lite with 4 primitives)
# -------------------------
def A_star(env_map, car_x_cm, car_y_cm, theta):
    """
    A* over (x_cm, y_cm, theta) with 4 feasible moves:
      f, fr, fl, b
    env_map: 2D np.uint8 (0 free, 1 obstacle), cell = 0.5 cm, row 0 bottom
    car_x_cm, car_y_cm: start (cm), origin bottom-left
    theta: start heading (rad), θ=0 → +Y
    Returns: path_labels (list[str]) like ['f','fr','f',...], or [] if no path.
    """
    # Validate start
    r0, c0 = world_to_grid(car_x_cm, car_y_cm)
    if not in_bounds(env_map, r0, c0) or not passable(env_map, r0, c0):
        return []

    start = (car_x_cm, car_y_cm, wrap_angle(theta))

    # OPEN: (f, g, x, y, th)
    open_heap = []
    g0 = 0.0
    h0 = dist_to_goal_rect_cm(car_x_cm, car_y_cm)
    heapq.heappush(open_heap, (g0 + h0, g0, car_x_cm, car_y_cm, theta))

    # CLOSED & parent tracking
    came_from = {}   # key -> ((px,py,pth), label)
    g_cost    = {}   # key -> g
    key0 = (r0, c0, heading_bin(theta))
    g_cost[key0] = 0.0
    closed = set()

    while open_heap:
        f, g, x, y, th = heapq.heappop(open_heap)
        rr, cc = world_to_grid(x, y)
        key = (rr, cc, heading_bin(th))
        if key in closed:
            continue
        closed.add(key)

        # Goal test: in rectangle
        if in_goal_region_rc(rr, cc):
            # reconstruct labels from (x,y,th) backwards
            labels = deque()
            cur = key
            while cur in came_from:
                (px,py,pth), lab = came_from[cur]
                labels.appendleft(lab)
                pr, pc = world_to_grid(px, py)
                cur = (pr, pc, heading_bin(pth))
            return list(labels)

        # Expand 4 successors
        for (xn,yn,thn, edge_cost, label) in successors(x,y,th):
            # collision check along straight chord
            if not collision_free_segment(env_map, x,y, xn,yn):
                continue
            rn, cn = world_to_grid(xn, yn)
            if not in_bounds(env_map, rn, cn):
                continue
            keyn = (rn, cn, heading_bin(thn))
            gn   = g + edge_cost
            if gn < g_cost.get(keyn, 1e15):
                g_cost[keyn] = gn
                came_from[keyn] = ((x,y,th), label)
                hn = dist_to_goal_rect_cm(xn, yn)
                fn = gn + hn
                heapq.heappush(open_heap, (fn, gn, xn, yn, thn))

    # No path
    return []