import math
import sys
import os
from board import SCL,SDA
import busio
import cv2
import numpy as np
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
sys.path.append(os.path.expanduser("~/Laser"))
# sys.path.append("Laser")
from functions import laser_function as lf
from functions import motor_function as mf 

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100  # Adjust as needed

# Define channels for horizontal and vertical servo control
x_direction = 0
y_direction = 1
steer=14

horizontal_servo = servo.Servo(pca.channels[x_direction])
vertical_servo = servo.Servo(pca.channels[y_direction])
steer_servo =servo.Servo(pca.channels[steer])
# Set initial servo positions to 90° (center)
horizontal_angle = 96
vertical_angle = 30
horizontal_servo.angle = horizontal_angle
vertical_servo.angle = vertical_angle

steer_angle=90
steer_servo.angle=steer_angle

cap = cv2.VideoCapture(0,cv2.CAP_V4L2)

# Define a high lightness threshold (0-255) for the laser
lightness_threshold = 240  # Slightly lowered for better detection

# Proportional gain for servo adjustments (tune as necessary)
kp = 0.01
scan_horizontal_angle=0
scan_vertical_angle=0
current_scan_angle=[0,0]
scan_h=5
scan_v=5
scan_step=2

# Minimum contour area to consider (laser point is typically small)
min_area = 10
pca = mf.servo_motor_initialization()
mf.motor_speed(pca, 0)
time.sleep(1)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            break

        # Flip frame if necessary (depends on camera orientation)
        frame = cv2.flip(frame, 0)

        # Try to track the laser
        tracking_result = lf.laser_tracking(frame, kp, horizontal_angle, vertical_angle)
        
        if tracking_result is not None:
            mf.motor_speed(pca, 0.135) 
            horizontal_angle, vertical_angle = tracking_result
            # Update servo positions based on tracking
            horizontal_servo.angle = horizontal_angle
            vertical_servo.angle = vertical_angle
            steer_angle = 180 - horizontal_angle  # Example: steer based on horizontal servo
            steer_servo.angle = steer_angle
            print(f"Laser tracking: H_angle={horizontal_angle:.2f}, V_angle={vertical_angle:.2f}, Steer={steer_angle:.2f}")
        else:

            # If no laser found, update scanning angles and motor stoped 
            mf.motor_speed(pca, 0) 
            current_scan_angle = lf.laser_searching(current_scan_angle, scan_h, scan_v)
            horizontal_servo.angle, vertical_servo.angle = current_scan_angle
            print(f"Laser searching: H_angle={current_scan_angle[0]:.2f}, V_angle={current_scan_angle[1]:.2f}")

        time.sleep(0.01)

except KeyboardInterrupt:
    mf.motor_speed(pca, 0)  
    pca.deinit()
    print("Exiting laser detection.")

cap.release()