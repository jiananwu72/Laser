import time
import csv
import cv2
import math
import sys
import os
import numpy as np
from math import floor
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_rplidar import RPLidar

sys.path.append(os.path.expanduser("~/Laser"))
from functions import laser_function as lf
from functions import motor_function as mf
from functions import lidar_function as ldf

# -----------------------------
# Hardware Initialization
# -----------------------------
# Initialize PCA9685 and servos
PORT_NAME = "/dev/ttyUSB0"
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100

# Steering servo on channel 14
steer_channel = 14  
steer_servo = servo.Servo(pca.channels[steer_channel])
steer_servo.angle = 90  # Neutral position

# Camera servo initialization
x_direction = 0
y_direction = 1
horizontal_servo = servo.Servo(pca.channels[x_direction])
vertical_servo = servo.Servo(pca.channels[y_direction])
horizontal_angle = 96
vertical_angle = 30
horizontal_servo.angle = horizontal_angle
vertical_servo.angle = vertical_angle

# Default values for laser tracking fallback
steer_angle = 90
current_scan_angle = [0, 0]

# Initialize camera capture
cap = cv2.VideoCapture(0)

# -----------------------------
# LiDAR and Detection Parameters
# -----------------------------
# LiDAR setup
lidar = RPLidar(None, PORT_NAME, timeout=3)

# LiDAR detection parameters
region_min = 120     # degrees: minimum angle of LiDAR region to check
region_max = 240     # degrees: maximum angle of LiDAR region to check
detection_threshold = 800   # mm: if distance < threshold, it's considered an obstacle
safe_width = 400     # mm: minimum gap width for safe passage

# -----------------------------
# Other Parameters
# -----------------------------
lightness_threshold = 240  # for laser detection (camera)
kp = 0.01                  # proportional gain for laser tracking
scan_h = 5                 # horizontal step for laser searching (fallback)
scan_v = 5                 # vertical step for laser searching (fallback)
min_area = 10              # for laser detection contour area

# Initialize motor (stop initially)
pca = mf.servo_motor_initialization()
mf.motor_speed(pca, 0)
time.sleep(1)

# Get an iterator for LiDAR scans so we don't block indefinitely.
lidar_scans = lidar.iter_scans()

print("Starting combined obstacle avoidance and laser tracking...")

try:
    while True:
        # Capture a camera frame
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            break
        frame = cv2.flip(frame, 0)  # Adjust if needed

        # Attempt to get a LiDAR scan
        try:
            scan = next(lidar_scans)
        except StopIteration:
            continue
        
        # Process LiDAR data: keep points within region_min to region_max.
        scan_data = [(angle, distance) for (_, angle, distance) in scan if region_min <= angle <= region_max]
        # Check for obstacles: if any point in the region is below the threshold.
        obstacle_present = any(distance < detection_threshold for angle, distance in scan_data)

        if obstacle_present:
            # Obstacle avoidance mode using LiDAR data.
            best_gap_angle = ldf.find_open_space(scan_data, region_min, region_max, detection_threshold, safe_width)
            if best_gap_angle is not None:
                new_steer_angle = ldf.lidar_to_steer(best_gap_angle)
                steer_servo.angle = new_steer_angle
                mf.motor_speed(pca, 0.135)
                print(f"[Obstacle Avoidance] Open gap at LiDAR angle {best_gap_angle:.1f}° => Steering angle {new_steer_angle:.1f}°")
            else:
                # If no valid gap is found, stop the motor.
                mf.motor_speed(pca, 0)
                print("[Obstacle Avoidance] Obstacle detected but no valid gap found. Stopping motor.")
        else:
            # If no obstacles detected by LiDAR, use laser tracking from the camera.
            tracking_result = lf.laser_tracking(frame, kp, horizontal_angle, vertical_angle)
            if tracking_result is not None:
                mf.motor_speed(pca, 0.135)
                horizontal_angle, vertical_angle = tracking_result
                horizontal_servo.angle = horizontal_angle
                vertical_servo.angle = vertical_angle
                # Here, steer angle is derived from horizontal servo (example relationship)
                steer_angle = 180 - horizontal_angle
                steer_servo.angle = steer_angle
                print(f"[Laser Tracking] H_angle={horizontal_angle:.2f}, V_angle={vertical_angle:.2f}, Steer={steer_angle:.2f}")
            else:
                # If laser tracking fails, fall back to scanning mode.
                mf.motor_speed(pca, 0)
                current_scan_angle = lf.laser_searching(current_scan_angle, scan_h, scan_v)
                horizontal_servo.angle, vertical_servo.angle = current_scan_angle
                print(f"[Laser Searching] H_angle={current_scan_angle[0]:.2f}, V_angle={current_scan_angle[1]:.2f}")

        time.sleep(0.1)

except KeyboardInterrupt:
    mf.motor_speed(pca, 0)
    pca.deinit()
    print("Exiting combined mode...")
    cap.release()

finally:
    lidar.stop()
    lidar.disconnect()
    print("LiDAR connection closed.")
    cap.release()
