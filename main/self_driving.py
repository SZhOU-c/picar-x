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
GRID_SIZE = 100
PIXEL_SIZE = 1
# Example car position at the bottom center (x=0, y=0 in world coords -> mapped to (50,0))

CELL_CM       = 2.0       # 1 grid cell = 2 cm (adjust to your grid)
V0_CM_S       = 28.0      # straight speed at your chosen command (from your calibration)
TURN_FACTOR   = 0.85      # v(±30°) ≈ 0.85 * V0
DT_S          = 0.2       # primitive duration
DELTA_DEG     = 30.0      # fixed steering for turns
L_CM          = 12.0      # wheelbase (measure once)
HEADING_BINS  = 16        # quantization for CLOSED set
TURN_PENALTY  = 0.5       # small cm penalty to prefer straights
R_TURN_CM     = L_CM / math.tan(math.radians(DELTA_DEG))  # ~20.8 cm

# robot-specific steering angles for your servo (adjust if needed)
STEER_STRAIGHT = 0
STEER_LEFT     = +30
STEER_RIGHT    = -30

SAFE_STOP_CM   = 12.0       # emergency stop if something appears very close

SAMPLE_STEP_CM= CELL_CM/2 # collision sampling resolution
# Goal region in GRID coords (row 0 at bottom)
GOAL_ROW_MIN, GOAL_ROW_MAX = 90, 99
GOAL_COL_MIN, GOAL_COL_MAX = 90, 99

px = Picarx()



def map_and_plan(env_map, car_x, car_y, th0):
    print(f"map and plan (car_x={car_x}, car_y={car_y})")
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


    print("dilated map:")
    print(env_map)
    
    actions, waypoints = hybrid_astar_plan(env_map, (car_x, car_y, th0))
    print(actions)

    return env_map, actions

def _wrap_angle(theta):
    """Wrap to [-pi, pi)."""
    return (theta + math.pi) % (2*math.pi) - math.pi

def move(x_cm, y_cm, theta, actions):
    """
    Execute one primitive from `actions` and update world pose.
    θ = 0 points along +Y (north). Returns updated (x, y, θ).
    - px: Picarx instance
    - (x_cm, y_cm, theta): current world pose
    - actions: deque of labels ['F0','FL30','FR30', ...]
    - power: the same motor command you calibrated (e.g., 50)
    """
    if not actions:
        return x_cm, y_cm, theta

    label = actions.popleft()

    # Safety: quick proximity check before committing the step
    try:
        d = float(px.ultrasonic.read())
        if d < SAFE_STOP_CM:
            px.forward(0)
            # you may want to push the label back or switch to map/replan
            return x_cm, y_cm, theta
    except Exception:
        pass  # if ultrasonic glitches, continue but your planner should replan often

    if label == 'F0':
        # steering straight
        px.set_dir_servo_angle(STEER_STRAIGHT)
        px.forward(POWER)
        time.sleep(DT_S)
        px.forward(0)

        s = V0_CM_S * DT_S
        # θ=0 is +Y ⇒ forward vector = (sinθ, cosθ)
        x_cm = x_cm + s * math.sin(theta)
        y_cm = y_cm + s * math.cos(theta)
        # theta unchanged

    elif label == 'FL30':
        # 30° left arc
        px.set_dir_servo_angle(STEER_LEFT)
        px.forward(POWER)
        time.sleep(DT_S)
        px.forward(0)

        v   = V0_CM_S * TURN_FACTOR
        s   = v * DT_S
        dth = + s / R_TURN_CM

        thx = theta + math.pi/2.0  # rotate frame so θ=0(+Y) aligns with +X in arc formulas
        x_cm = x_cm + R_TURN_CM * (math.sin(thx + dth) - math.sin(thx))
        y_cm = y_cm - R_TURN_CM * (math.cos(thx + dth) - math.cos(thx))
        theta = _wrap_angle(theta + dth)

    elif label == 'FR30':
        # 30° right arc
        px.set_dir_servo_angle(STEER_RIGHT)
        px.forward(POWER)
        time.sleep(DT_S)
        px.forward(0)

        v   = V0_CM_S * TURN_FACTOR
        s   = v * DT_S
        dth = - s / R_TURN_CM

        thx = theta + math.pi/2.0
        x_cm = x_cm + R_TURN_CM * (math.sin(thx + dth) - math.sin(thx))
        y_cm = y_cm - R_TURN_CM * (math.cos(thx + dth) - math.cos(thx))
        theta = _wrap_angle(theta + dth)

    else:
        # Unknown label: do nothing (or handle reverse later)
        pass

    return x_cm, y_cm, theta


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
    th0    = 0.0  # facing +Y

    try:
        while True:
            print("current status:", status, "steps = ", steps)
            if status == "move" and steps < 5:

                car_x, car_y, th0 = move(car_x, car_y, th0, actions)
                steps = steps + 1

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

def world_to_grid(x_cm, y_cm):
    """World: x right, y up  ->  grid: (row bottom-up, col right)"""
    col = int(math.floor(x_cm / CELL_CM))
    row = int(math.floor(y_cm / CELL_CM))    # row 0 = bottom
    return row, col

def grid_to_world_center(row, col):
    x_cm = (col + 0.5) * CELL_CM
    y_cm = (row + 0.5) * CELL_CM
    return x_cm, y_cm

def in_goal_region_rc(row, col):
    return (GOAL_ROW_MIN <= row <= GOAL_ROW_MAX) and (GOAL_COL_MIN <= col <= GOAL_COL_MAX)

def dist_to_goal_rect_cm(x_cm, y_cm):
    """Euclidean distance from point to the goal rectangle (in cm)."""
    x_min, x_max = GOAL_COL_MIN*CELL_CM, (GOAL_COL_MAX+1)*CELL_CM
    y_min, y_max = GOAL_ROW_MIN*CELL_CM, (GOAL_ROW_MAX+1)*CELL_CM
    dx = 0.0 if (x_min <= x_cm <= x_max) else (x_min - x_cm if x_cm < x_min else x_cm - x_max)
    dy = 0.0 if (y_min <= y_cm <= y_max) else (y_min - y_cm if y_cm < y_min else y_cm - y_max)
    return math.hypot(dx, dy)

# ====== Heading quantization (θ=0 is +Y) ======
def heading_bin(theta):
    """Quantize heading for CLOSED set; θ in radians with θ=0 pointing +Y."""
    # wrap to [-pi, pi)
    t = (theta + math.pi) % (2*math.pi) - math.pi
    step = 2*math.pi / HEADING_BINS
    return int(round(t / step)) % HEADING_BINS

# ====== Vehicle primitives (θ=0 is +Y) ======
DELTA_RAD  = math.radians(DELTA_DEG)
R_TURN_CM  = L_CM / math.tan(DELTA_RAD)  # ~20.8 cm for L=12, 30°
V_TURN_CM_S= V0_CM_S * TURN_FACTOR

def step_straight(x, y, theta):
    s = V0_CM_S * DT_S
    # θ=0 is +Y ⇒ forward direction vector = (sinθ, cosθ)
    return (x + s*math.sin(theta), y + s*math.cos(theta), theta, s, False)

def step_turn(x, y, theta, left=True):
    v   = V_TURN_CM_S
    s   = v * DT_S
    dth = (s / R_TURN_CM) * ( +1 if left else -1 )
    # Use standard bicycle arc equations but with θx = θ + 90° (since θ=0 is +Y)
    thx = theta + math.pi/2.0
    x2  = x + R_TURN_CM * ( math.sin(thx + dth) - math.sin(thx) )
    y2  = y - R_TURN_CM * ( math.cos(thx + dth) - math.cos(thx) )
    th2 = theta + dth
    return (x2, y2, th2, s, True)

def primitive_successors(x, y, theta):
    """Return list of (x',y',theta', edge_cost_cm, label) successors."""
    out = []
    x2,y2,th2,s,_ = step_straight(x,y,theta)
    out.append( (x2,y2,th2, s, 'F0') )
    x2,y2,th2,s,_ = step_turn(x,y,theta,left=True)
    out.append( (x2,y2,th2, s+TURN_PENALTY, 'FL30') )
    x2,y2,th2,s,_ = step_turn(x,y,theta,left=False)
    out.append( (x2,y2,th2, s+TURN_PENALTY, 'FR30') )
    return out
    # (Add reverse primitive later if needed.)

# ====== Collision checking along a primitive ======
def collision_free_segment(grid, x0,y0, x1,y1, samples_cm=SAMPLE_STEP_CM):
    """Sample along the segment/arc approximated as a straight chord for collision;
       good enough at small DT_S with inflated map. For higher fidelity, sample along the true arc."""
    total = math.hypot(x1-x0, y1-y0)
    n = max(1, int(math.ceil(total / samples_cm)))
    for i in range(n+1):
        t = i / n
        xs = x0 + t*(x1-x0)
        ys = y0 + t*(y1-y0)
        r,c = world_to_grid(xs, ys)
        if not in_bounds(grid, r, c) or not passable(grid, r, c):
            return False
    return True

# ====== Hybrid A* (world-up, θ=0 +Y, goal is 10x10 box) ======
def hybrid_astar_plan(grid, start_xytheta):
    """
    grid: 2D numpy array (0 free, 1 obstacle), row 0 is bottom.
    start_xytheta: (x_cm, y_cm, theta_rad) with theta=0 pointing +Y.
    Returns: deque of primitive labels ['F0','FL30',...] and a list of waypoints [(x,y,theta),...]
    """
    x0, y0, th0 = start_xytheta
    r0, c0 = world_to_grid(x0, y0)
    if not in_bounds(grid, r0, c0) or not passable(grid, r0, c0):
        return None, None

    # goal heuristic in cm
    h0 = dist_to_goal_rect_cm(x0, y0)
    open_heap = []
    heapq.heappush(open_heap, (h0, 0.0, (x0,y0,th0)))
    came_from = {}   # (row,col,bin) -> ((x,y,th), label)
    g_cost    = {}   # key -> g(cm)
    key0 = (r0, c0, heading_bin(th0))
    g_cost[key0] = 0.0

    closed = set()

    while open_heap:
        f, g, (x,y,th) = heapq.heappop(open_heap)
        key = (world_to_grid(x,y)[0], world_to_grid(x,y)[1], heading_bin(th))
        if key in closed:
            continue
        closed.add(key)

        # Goal test
        rr, cc = world_to_grid(x, y)
        if in_goal_region_rc(rr, cc):
            # reconstruct
            path_labels = deque()
            waypoints   = [(x,y,th)]
            curkey = key
            while curkey in came_from:
                (px,py,pth), lab = came_from[curkey]
                path_labels.appendleft(lab)
                waypoints.append( (px,py,pth) )
                curkey = (world_to_grid(px,py)[0], world_to_grid(px,py)[1], heading_bin(pth))
            waypoints.reverse()
            return path_labels, waypoints

        # Expand successors
        for (xn,yn,thn, edge_cost, label) in primitive_successors(x,y,th):
            # collision check along this primitive (approx. by straight sampling of chord)
            if not collision_free_segment(grid, x,y, xn,yn):
                continue
            rn, cn = world_to_grid(xn, yn)
            if not in_bounds(grid, rn, cn):
                continue
            keyn = (rn, cn, heading_bin(thn))
            gn   = g + edge_cost
            if gn < g_cost.get(keyn, 1e15):
                g_cost[keyn] = gn
                came_from[keyn] = ((x,y,th), label)
                hn = dist_to_goal_rect_cm(xn, yn)
                fn = gn + hn
                heapq.heappush(open_heap, (fn, gn, (xn,yn,thn)))

    return None, None