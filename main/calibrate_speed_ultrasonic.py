from picarx import Picarx
import time


SafeDistance = 40   # > 40 safe
DangerDistance = 20 # > 20 && < 40 turn around, 
                    # < 20 backward
px = Picarx()

def main():
    try:
        power = 50
        
        px.set_dir_servo_angle(30)
        px.set_cam_pan_angle(0)
        # px = Picarx(ultrasonic_pins=['D2','D3']) # tring, echo
        px.ultrasonic.read()

        px.forward(power)

        time.sleep(0.2)
        px.forward(0)
    finally:
        px.forward(0)

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
        power = 50
        distance = [0.0,1.1,2.1,3.1]
        distance[0] = round(px.ultrasonic.read(), 2)
        print("initial distance: ",distance[0])

        
        px.forward(power)
        time.sleep(0.1)
        px.forward(0)
        distance[1] = round(px.ultrasonic.read(), 2)
        moved = distance[0] - distance[1]
        print("after 0.1 second moved ",moved , " velocity = ", moved/0.1)


        px.forward(power)
        time.sleep(0.2)
        px.forward(0)
        distance[2] = round(px.ultrasonic.read(), 2)
        moved = distance[1] - distance[2]
        print("after 0.2 second moved ",moved , " velocity = ", moved/0.2)

        px.forward(power)
        time.sleep(0.5)
        px.forward(0)
        distance[3] = round(px.ultrasonic.read(), 2)
        moved = distance[2] - distance[3]
        print("after 0.5 second moved ",moved , " velocity = ", moved/0.5)


        power = 30
        distance[0] = round(px.ultrasonic.read(), 2)
        print("Now power = 30, initial distance: ",distance[0])

        px.forward(power)
        time.sleep(0.1)
        px.forward(0)
        distance[1] = round(px.ultrasonic.read(), 2)
        moved = distance[0] - distance[1]
        print("after 0.1 second moved ",moved , " velocity = ", moved/0.1)


        px.forward(power)
        time.sleep(0.2)
        px.forward(0)
        distance[2] = round(px.ultrasonic.read(), 2)
        moved = distance[1] - distance[2]
        print("after 0.2 second moved ",moved , " velocity = ", moved/0.2)

        px.set_dir_servo_angle(0)
        px.forward(power)
        time.sleep(0.5)
        px.forward(0)
        distance[3] = round(px.ultrasonic.read(), 2)
        moved = distance[2] - distance[3]
        print("after 0.5 second moved ",moved , " velocity = ", moved/0.5)


if __name__ == "__main__":
    main()

