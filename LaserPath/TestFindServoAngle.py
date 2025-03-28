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

steer=14
steer_servo =servo.Servo(pca.channels[steer])

steer_servo.angle = 0