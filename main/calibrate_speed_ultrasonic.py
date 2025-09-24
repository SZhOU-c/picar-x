from picarx import Picarx
import time
import readchar 
import web_ui 
import numpy as np
import math


SafeDistance = 40   # > 40 safe
DangerDistance = 20 # > 20 && < 40 turn around, 
                    # < 20 backward
GRID_SIZE = 200
CELL_CM = 0.5
px = Picarx()

def main():
    try:
        px.set_cam_tilt_angle(0)
        px.set_dir_servo_angle(0)
        env_map = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)
        # 200 * 0.5 / 2 = 50.0 cm
        car_x = (GRID_SIZE * CELL_CM) / 2.0
        car_y = 0
        th0    = 0.0  # facing +Y
        web_ui.start_server()

        print("init sucessfully")
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

        
        while(True):
            update_web(env_map, car_x, car_y)
            time.sleep(1)
    finally:
        px.forward(0)

def update_web(env_map, car_x, car_y):
    web_ui.ENV_MAP = env_map.copy()
    web_ui.CAR_POSE = (car_x, car_y)
    web_ui.WAYPOINTS = None

def velocity_test():
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
       # px = Picarx(ultrasonic_pins=['D2','D3']) # tring, echo
    px.ultrasonic.read()

    start = px.ultrasonic.read()
    px.forward(power)

    time.sleep(0.5)
    start = px.ultrasonic.read()
    time.sleep(0.5)
    end = px.ultrasonic.read()
    px.forward(0)
    distance = start - end
    print("distance",distance , " velocity = ", distance/0.5)


def speed_test():
    px.set_dir_servo_angle(30)
    px.set_cam_pan_angle(0)
    while True:
        key = readchar.readkey()
            
        key = key.lower()
        power = 50
        px.ultrasonic.read()
        if key == 'd':
            px.set_dir_servo_angle(30)
            px.forward(power)
            time.sleep(0.3)
            px.forward(0)

        if key == 'a':
            px.set_dir_servo_angle(-30)
            px.forward(power)
            time.sleep(0.3)
            px.forward(0)
            
        if key == 'w':
            px.set_dir_servo_angle(0)
            px.forward(power)
            time.sleep(0.3)
            px.forward(0)

        if key == 's':
            px.set_dir_servo_angle(0)
            px.backward(power)
            time.sleep(0.3)
            px.backward(0) 

            # 3 4/16, 3 5/16, 3 5/16, 3 7/16, 3 5/16， 3 6/16， 3 6/16 inch 
            # forward distance 8.48cm
            # 7 cm back distance
            # 7, 5, 7, 7, 6 degree 6.4° right turn
            # 2 9/16, 2 13/16, 2 14/16 inch 6.99 cm right turn y
            # 2 1/2 10 degree, 2 4.5/16 7 degree, 2 7/16 5 degree, 2 8/16 7.8 degree 6.17 cm  7.45° 7 degree


if __name__ == "__main__":
    main()

