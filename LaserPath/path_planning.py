import math
import sys
import os
import board
import numpy as np
import time
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_mpu6050 import MPU6050

sys.path.append(os.path.expanduser("~/Laser"))
from functions import laser_function as lf
from functions import motor_function as mf 

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 100  # Adjust as needed

# Define channels for servos
x_direction = 0
y_direction = 1
steer = 14
horizontal_servo = servo.Servo(pca.channels[x_direction])
vertical_servo = servo.Servo(pca.channels[y_direction])
horizontal_servo.angle = 96
vertical_servo.angle = 65
steer_servo = servo.Servo(pca.channels[steer])

# IMU initialization
i2c_imu = board.I2C()
mpu = MPU6050(i2c_imu)

# Configuration
WAYPOINT_THRESHOLD = 0.05
MAX_STEER_ANGLE = 27
DT = 0.1
# State variables
c_x = 0.0
c_y = 0.0
c_yaw = 0.0
c_vx = 0.0
c_vy = 0.0
CONSTANT_SPEED = 0.135

# class PIDController:
#     def __init__(self, kp, ki, kd):
#         self.kp = kp
#         self.ki = ki
#         self.kd = kd
#         self.integral = 0.0
#         self.prev_error = 0.0

#     def compute(self, target, current):
#         error = target - current
#         self.integral += error * DT
#         derivative = (error - self.prev_error) / DT
#         output = self.kp * error + self.ki * self.integral + self.kd * derivative
#         self.prev_error = error
#         return output

def steering_control_to_waypoint(current_x, current_y, current_yaw, target_waypoint):
    dx = target_waypoint[0] - current_x
    dy = target_waypoint[1] - current_y

    local_x = math.cos(-current_yaw) * dx - math.sin(-current_yaw) * dy
    local_y = math.sin(-current_yaw) * dx + math.cos(-current_yaw) * dy
    desired_angle = math.atan2(local_y, local_x)
    print("lx:", local_x, "ly:", local_y, "da:", desired_angle)
    desired_angle = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, desired_angle))
    return -desired_angle

def update_vehicle_state():
    global c_x, c_y, c_yaw, c_vx, c_vy
    # Read gyro and update yaw
    gyro_z = mpu.gyro[2]
    c_yaw += gyro_z * DT

    # Read accelerometer and rotate to world frame
    accel = mpu.acceleration
    ax_body = -accel[0]
    ay_body = -accel[1]
    ax_world = ax_body * math.cos(c_yaw) - ay_body * math.sin(c_yaw)
    ay_world = ax_body * math.sin(c_yaw) + ay_body * math.cos(c_yaw)

    # Integrate acceleration to velocity
    c_vx += ax_world * DT
    c_vy += ay_world * DT

    # Integrate velocity to position
    c_x += c_vx * DT
    c_y += c_vy * DT

    current_speed = math.sqrt(c_vx**2 + c_vy**2)
    return c_x, c_y, c_yaw, current_speed

def command_vehicle(speed_cmd, steer_cmd):
    mf.motor_speed(pca, speed_cmd)
    steer_degrees = math.degrees(steer_cmd) * 90/27 + 90  # Scale to servo range
    steer_servo.angle = steer_degrees
    # print(f"Speed Cmd: {speed_cmd:.2f} m/s, Steering Cmd: {steer_degrees:.2f}°")

def main():
    waypoints = [
        (1, 0)
        # (0.3, -0.1),
        # (0.4, 0),
        # (0.5, 0.05),
        # (0.55, 0.1)
    ]
    current_waypoint_index = 0
    # lateral_pid = PIDController(kp=1.0, ki=0.0, kd=0.1)

    try:
        while current_waypoint_index < len(waypoints):
            current_x, current_y, yaw, current_speed = update_vehicle_state()
            target_wp = waypoints[current_waypoint_index]

            dist = math.hypot(target_wp[0] - current_x, target_wp[1] - current_y)
            print(f"Position: ({current_x:.2f}, {current_y:.2f}), Target: {target_wp}, Distance: {dist:.2f}")

            if dist < WAYPOINT_THRESHOLD:
                print(f"Reached waypoint {current_waypoint_index}")
                current_waypoint_index += 1
                continue

            steer_cmd = steering_control_to_waypoint(current_x, current_y, yaw, target_wp)
            # lateral_error = 0.0  # Optional PID input
            # steer_adjust = lateral_pid.compute(0.0, lateral_error)
            # command_vehicle(CONSTANT_SPEED, steer_cmd + steer_adjust)
            print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (mpu.acceleration))
            command_vehicle(CONSTANT_SPEED, steer_cmd)
            time.sleep(DT)

        mf.motor_speed(pca, 0)
        print("All waypoints reached")
    except ValueError:
        print("I stoped")
        mf.motor_speed(pca, 0)
        pca.deinit()
    except KeyboardInterrupt:
        mf.motor_speed(pca, 0)
        pca.deinit()

if __name__ == '__main__':
    main()