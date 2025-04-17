import cv2
import numpy as np
from scipy import ndimage
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import sys
import os
sys.path.append(os.path.expanduser("~/Laser"))

from functions import path_planning as pp
from functions import video_taker as vt
from functions import extract_waypoints as ew
from functions import motor_function as mf
from functions import camera_to_spatial as cts

# Step 1: Capture video

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100
steer_servo = servo.Servo(pca.channels[14])
steer_servo.angle = 90
horizontal_servo = servo.Servo(pca.channels[0])
vertical_servo = servo.Servo(pca.channels[1])
horizontal_servo.angle = 96
vertical_servo.angle = 65
detected, output_path = vt.video_taker()
if detected:
    print(output_path)

# Step 2: Extract waypoints

output_path = '/home/chen/Laser/LaserPath/videos/laser_path.mp4'
waypoints = ew.extract_waypoints(output_path, window_num=5)
print("Processing complete. Waypoints extracted.")

# Step 3: Following path

pp.follow_waypoints(waypoints)