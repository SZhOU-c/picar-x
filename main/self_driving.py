from picarx import Picarx
import time
import random

POWER = 50
SAFE_DISTANCE = 40     # >40 cm = safe, go forward
BACKUP_DISTANCE = 15   # <20 cm = backup immediately


def avoid_obstacle(px, distance):

    px.forward(0)  # stop first
    
    if distance < BACKUP_DISTANCE:
        print("Too close! Backing up...")

        px.backward(POWER)
        backup_time = 1 / max(distance , 4) # backup_time = [0.025, 0.25]
        turn_angle = random( 15, 30)
        turn_direction = random.choice((-1, 1)) * turn_angle
        px.set_dir_servo_angle(turn_direction)
        time.sleep(backup_time)  # back up a bit longer
    else:
        print("Obstacle ahead, turning...")
        turn_angle = random( 15, 30)
        turn_direction = random.choice((-1, 1)) * turn_angle
        px.set_dir_servo_angle(turn_direction)
        px.forward(POWER)
        time.sleep(0.1)

def main():
    px = Picarx()
    try:
        while True:
            distance = round(px.ultrasonic.read(), 2)
            print(f"Distance: {distance} cm")

            if distance >= SAFE_DISTANCE:
                # Safe → drive straight
                px.set_dir_servo_angle(0)
                px.forward(POWER)
            else:
                # Obstacle detected → avoid
                avoid_obstacle(px, distance)

            time.sleep(0.1)

    finally:
        px.forward(0)

def reset():
    px.set_dir_servo_angle(0)

if __name__ == "__main__":
    main()