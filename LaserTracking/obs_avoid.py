import time
import csv
import math
import sys
import os
from math import floor
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_rplidar import RPLidar

sys.path.append(os.path.expanduser("~/Laser"))
from functions import motor_function as mf
from functions import lidar_function as ldf

# LiDAR configuration
PORT_NAME = "/dev/ttyUSB0"
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100
steer_channel = 14  # Steering servo channel
steer_servo = servo.Servo(pca.channels[steer_channel])
steer_servo.angle = 90  # Initialize to neutral position
pca = mf.servo_motor_initialization()
mf.motor_speed(pca, 0)
# Detection parameters
region_min = 120    # Minimum detection angle (degrees)
region_max = 240    # Maximum detection angle (degrees)
detection_threshold = 1000  # mm: open space means distance >= threshold
safe_width = 400    # mm: minimum gap width for safe passage

# Initialize LiDAR
lidar = RPLidar(None, PORT_NAME, timeout=3)



print("Starting LiDAR scan for open space detection...")

try:
    for scan in lidar.iter_scans():
        scan_data = [(angle, distance) for (_, angle, distance) in scan if region_min <= angle <= region_max]
        obstacle_present = any(distance < detection_threshold for angle, distance in scan_data)
        
        if obstacle_present:
            best_gap_angle = ldf.find_open_space(scan_data, region_min, region_max, detection_threshold, safe_width)
            if best_gap_angle is not None:
            # Map the chosen LiDAR middle angle to a steering angle
                new_steer_angle = ldf.lidar_to_steer(best_gap_angle)
                steer_servo.angle = new_steer_angle
                mf.motor_speed(pca,0.135)
                print(f"Open space detected at LiDAR angle {best_gap_angle:.1f}° => Steering angle {new_steer_angle:.1f}°")
                
            else:
                print("No open space detected! Consider stopping or taking other measures.")
                mf.motor_speed(pca,0)
        else:
            steer_servo.angle = 90
            mf.motor_speed(pca, 0.135)
        time.sleep(0.1)

except KeyboardInterrupt:
    mf.motor_speed(pca, 0)  
    pca.deinit()
    print("Exiting open space detection mode...")

finally:
    lidar.stop()
    lidar.disconnect()
    print("LiDAR connection closed.")