import time
import csv
from math import floor
from adafruit_rplidar import RPLidar

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
from math import floor
from adafruit_rplidar import RPLidar

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100  
PORT_NAME = "/dev/ttyUSB0"
pca = mf.servo_motor_initialization()
mf.motor_speed(pca, 0)
time.sleep(1)


# Define the LIDAR port (Change if needed)
PORT_NAME = "/dev/ttyUSB0"
steer=14
detection=1000
steer_servo =servo.Servo(pca.channels[steer])
# Initialize the LIDAR
lidar = RPLidar(None, PORT_NAME, timeout=3)
output_file = "test_data/lidar_data.csv"
dis_less=[]
distances=[]
steer_angle=90
steer_servo.angle=steer_angle

try:
    print("Starting LIDAR scan...")
    
    with open(output_file, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Angle", "Distance","Speed","Description"])  # CSV Header

        for scan in lidar.iter_scans():
                for (_, angle, distance) in scan:
                    if angle >= 105 and angle <= 255:
                        distances.append(distance)
                        if distance < detection:
                             dis_less.append(distance)
                ratio= len(dis_less)/len(distances)
                print(f"ratio: {ratio}, ")
                if ratio>0.5:
                    mf.motor_speed(pca, 0)     
                    writer.writerow([floor(angle), distance,0, "obstacle detected"]) 
                else:
                    mf.motor_speed(pca, 0.13) 
                    writer.writerow([floor(angle), distance, 0.1,"no obstacle detected"])
                dis_less=[]
                distances=[]

                            

                print(f"Saved {len(scan)} points...")

except KeyboardInterrupt:
    print("Exiting laser detection.")
    mf.motor_speed(pca, 0)  
    pca.deinit()
    print("Exiting laser detection.")


finally:
    lidar.stop()
    lidar.disconnect()
    print(f"Data saved to {output_file}")
