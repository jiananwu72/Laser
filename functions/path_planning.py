import math
import sys
import os
import board
import numpy as np
import time
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

sys.path.append(os.path.expanduser("~/Laser"))
from functions import laser_function as lf
from functions import motor_function as mf 
import math
import time

def drive_arc_to_waypoint(x, y, theta, x_next, y_next):
    MAX_STEER_ANGLE = 27
    CONSTANT_SPEED = 0.14
    WHEELBASE = 27.8
    dx = x_next - x
    dy = y_next - y
    d  = math.sqrt(dx*dx + dy*dy)
    if d < 1e-6:
        return x_next, y_next, theta

    theta_target = math.atan2(dy, dx)
    dtheta = theta_target - theta
    dtheta = (dtheta + math.pi) % (2*math.pi) - math.pi
    if dtheta > math.pi/2:
        dtheta -= math.pi
    elif dtheta < -math.pi/2:
        dtheta += math.pi

    delta_radians = 0.0
    if abs(dtheta) > 1e-6:
        delta_radians = math.atan2(WHEELBASE * dtheta, d)

    steer_cmd_deg = math.degrees(delta_radians)

    if steer_cmd_deg > MAX_STEER_ANGLE:
        steer_cmd_deg = MAX_STEER_ANGLE
    elif steer_cmd_deg < -MAX_STEER_ANGLE:
        steer_cmd_deg = -MAX_STEER_ANGLE

    v = 41.7
    T = d / v

    # command_vehicle(0, steer_cmd_deg)
    # time.sleep(1)
    command_vehicle(CONSTANT_SPEED, steer_cmd_deg)
    time.sleep(T)
    command_vehicle(0, 0)

    return (x_next, y_next, theta_target)


def command_vehicle(speed_cmd, steer_cmd):
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 100
    steer_servo = servo.Servo(pca.channels[14])

    MAX_STEER_ANGLE = 27
    mf.motor_speed(pca, speed_cmd)
    steer_degrees = steer_cmd * 90/MAX_STEER_ANGLE + 90  # Scale to servo range
    steer_servo.angle = steer_degrees
    # print(f"Speed Cmd: {speed_cmd:.2f} m/s, Steering Cmd: {steer_degrees:.2f}°")

def follow_waypoints(waypoints, x0=0.0, y0=0.0, theta0=0.0):
    """
    waypoints:np.array([y1, x1], ..., [yN, xN])
    """
    waypoints[:, 0] += 35 # add offsets for starting
    waypoints[:, 1] = -waypoints[:, 1]
    x_curr, y_curr, theta_curr = x0, y0, theta0
    for (x_next, y_next) in waypoints:
        x_curr, y_curr, theta_curr = drive_arc_to_waypoint(
            x_curr, y_curr, theta_curr,
            x_next, y_next
        )

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 100  # Adjust as needed

    # Define channels for servos
    x_direction = 0
    y_direction = 1
    horizontal_servo = servo.Servo(pca.channels[x_direction])
    vertical_servo = servo.Servo(pca.channels[y_direction])
    horizontal_servo.angle = 96
    vertical_servo.angle = 65

    x0 = 0.0
    y0 = 0.0
    theta0 = 0.0 
    waypoints = np.array([
        [30, -10.0],
        [60, 0.0],
        [90, 10.0],
        [120, 0.0]
    ])

    try:
        follow_waypoints(waypoints, x0, y0, theta0)
        print("Done following waypoints!")
    except KeyboardInterrupt:
        command_vehicle(0, 0)
        pca.deinit()


if __name__ == '__main__':
    main()