import time
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

CAR_WIDTH=320 #32 cm
EXTRA_SAFE = 80 #8 cm

def lidar_to_steer(lidar_angle):
    steer_angle = 630 - 3 * lidar_angle
    return max(0, min(180, steer_angle))

def find_open_space(scan_data, min_angle, max_angle, threshold, safe_width):
    free_spaces = []
    current_space = []
    for angle, distance in scan_data:
        if min_angle <= angle <= max_angle:
            if distance >= threshold:
                current_space.append((angle, distance))
            else:
                if current_space:
                    free_spaces.append(current_space)
                    current_space = []
    if current_space:
        free_spaces.append(current_space)
    valid_spaces = []
    for space in free_spaces:
        start_angle = space[0][0]
        end_angle = space[-1][0]
        angle_span = end_angle - start_angle  # in degrees
        
        # Use the average distance of the gap as an estimate
        avg_distance = sum(d for a, d in space) / len(space)
        gap_width = 2 * avg_distance * math.sin(math.radians(angle_span / 2))
        
        if gap_width >= safe_width:
            valid_spaces.append(space)
    
    if not valid_spaces:
        return None  # No valid open space found
    
    # Choose the largest valid space (by angular span)
    largest_space = max(valid_spaces, key=lambda s: s[-1][0] - s[0][0])
    middle_angle = (largest_space[0][0] + largest_space[-1][0]) / 2
    return middle_angle

#
def split_gaps(scan_data, threshold):
    gaps = []
    current = []
    for angle, distance in scan_data:
        if distance >= threshold:
            current.append((angle, distance))
        else:
            if current:
                gaps.append(current)
                current = []
    if current:
        gaps.append(current)
    return gaps

def filter_valid_spaces(gaps, safe_width):
    valid = []
    for gap in gaps:
        width = compute_gap_width(gap)
        if width >= safe_width:
            valid.append(gap)
    return valid

def compute_gap_width(gap):
    angle_span = gap[-1][0] - gap[0][0]
    avg_distance = sum(d for a, d in gap) / len(gap)
    return 2 * avg_distance * math.sin(math.radians(angle_span / 2))
