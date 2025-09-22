from picarx import Picarx
import time
import random
import numpy as np
import math
from vilib import Vilib
import readchar
import cv2 
#from scipy.ndimage import binary_dilation


POWER = 50
SAFE_DISTANCE = 40     # >40 cm = safe, go forward
BACKUP_DISTANCE = 15   # <20 cm = backup immediately

# Create an empty 2D map: 100x100 grid initialized with zeros
GRID_SIZE = 100

# Example car position at the bottom center (x=0, y=0 in world coords -> mapped to (50,0))
CAR_X, CAR_Y = GRID_SIZE // 2, 0

NORTH = 0
EAST = 30
WEST = -30
SOUTH = 0

px = Picarx()

status = "move"
steps = 0
actions = []

def map(env_map, px):
    px.set_cam_tilt_angle(-10)
    for i in range(-60, 61, 2):
        px.set_cam_pan_angle(i)
        
        distance = round(px.ultrasonic.read(), 2)
        print(distance) 

        angle_rad = i * math.pi / 180
        obstacle_x = CAR_X + int(round(distance * math.cos(angle_rad)))
        obstacle_y = CAR_Y + int(round(distance * math.sin(angle_rad)))
        if 0 <= obstacle_x < GRID_SIZE and 0 <= obstacle_y < GRID_SIZE:
            env_map[obstacle_y, obstacle_x] = 1

    print("scanned map:")
    print(env_map)
    # wrap the obstacles to fill the measuring gap.
    # After marking obstacles, you can expand them to account for obstacle width
    env_map = cv2.dilate(env_map, np.ones((3,3), np.uint8), iterations=1)
    print("dilated map:")
    print(env_map)
    return env_map

def move():
    match actions.pop():
        case "N":
            px.set_dir_servo_angle(NORTH)
            px.forward(POWER)
            time.sleep(0.1)
            px.forward(0)
        case "E":
            px.set_dir_servo_angle(EAST)
            px.forward(POWER)
            time.sleep(0.1)
            px.forward(0)
        case "W":
            px.set_dir_servo_angle(WEST)
            px.forward(POWER)
            time.sleep(0.1)
            px.forward(0)
        case "S":
            px.set_dir_servo_angle(EAST)
            px.backward(POWER)
            time.sleep(0.1)
            px.backward(0)

    return 

def search():

    return 

def take_photo():
    _time = strftime('%Y-%m-%d-%H-%M-%S',localtime(time()))
    name = 'photo_%s'%_time
    username = os.getlogin()

    path = f"/home/{username}/picar-x/"
    Vilib.take_photo(name, path)
    print('photo save as %s%s.jpg'%(path,name))



def main():
    Vilib.camera_start(vflip=False,hflip=False)
    Vilib.display(local=True,web=True)

    env_map = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)
    
    Target_address = 100,100

    try:
        while True:
            distance = round(px.ultrasonic.read(), 2)
            print(f"Distance: {distance} cm")

            if status == "move" and steps < 5:
                move()
                steps = steps + 1
            elif status == "move" and steps >= 5:
                move()
                status = "map"
            elif status == "map":
                map()
                status = "move"
                steps = 0
            elif status == "finished":
                return 
            elif status == "stop":
                time.sleep(0.5)
            

            key = readchar.readkey()
            key = key.lower()
            if key == 'm':
                env_map = map(env_map, px)
            elif key =="f":
                take_photo()
            time.sleep(0.5)
            

    finally:
        px.forward(0)

def reset():
    px.set_dir_servo_angle(0)

if __name__ == "__main__":
    main()