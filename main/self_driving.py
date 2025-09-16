from picarx import Picarx
import time
import random

POWER = 50
SAFE_DISTANCE = 40     # >40 cm = safe, go forward
DANGER_DISTANCE = 20   # 20–40 cm = obstacle ahead, turn
BACKUP_DISTANCE = 20   # <20 cm = backup immediately
TURN_ANGLE = 30

def avoid_obstacle(px, distance):

    px.forward(0)  # stop first
    
    if distance < BACKUP_DISTANCE:
        print("Too close! Backing up...")

        px.backward(POWER)
        backup_time = 1 / max(distance * 2, 4) # backup_time = [0.025, 0.25]
        time.sleep(backup_time)  # back up a bit longer
    else:
        print("Obstacle ahead, turning...")
        turn_direction = random.choice((-1, 1)) * TURN_ANGLE
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