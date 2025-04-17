
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
from functions import laser_function as lf
from functions import motor_function as mf 
from math import floor
from adafruit_rplidar import RPLidar

# Define the LIDAR port (Change if needed)
PORT_NAME = "/dev/ttyUSB0"
pca = mf.servo_motor_initialization()
# mf.motor_start(pca)
mf.motor_speed(pca, 0)
time.sleep(1)
mf.motor_speed(pca, 0.12) 
try:
    while True:
        
        time.sleep(1)
except KeyboardInterrupt:
    mf.motor_speed(pca, 0)  
    pca.deinit()
