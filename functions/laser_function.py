import cv2
import numpy as np
import time
from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

def find_roundest(contours):
    # For faster processing speed
    if len(contours) == 1:
        return contours[0]
    
    max_circularity = 0
    roundest_contour = None
    for c in contours:
        area = cv2.contourArea(c)
        # Optionally skip very small contours to avoid noise:
        if area < 5:
            continue

        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue

        circularity = 4 * np.pi * area / (perimeter * perimeter)
        if circularity > max_circularity:
            max_circularity = circularity
            roundest_contour = c
            
    return roundest_contour

def find_laser(image):
    # Convert the image from BGR to HSV/YUV color space
    hlsImage = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    yuvImage = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)

    # Define Light Color Ranges and Create a Mask
    lower_hls1 = np.array([0, 230, 0])
    upper_hls1 = np.array([180, 255, 255])
    lower_yuv2 = np.array([200, 0, 0])
    upper_yuv2 = np.array([255, 255, 160])
    mask1 = cv2.inRange(hlsImage, lower_hls1, upper_hls1)
    mask2 = cv2.inRange(yuvImage, lower_yuv2, upper_yuv2)
    mask = cv2.bitwise_and(mask1, mask2)

    # Denoise the mask using morphological operations
    kernel = np.array([[0.5, 1, 0.5], [1, 2, 1], [0.5, 1, 0.5]])
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find Countours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        roundest_contour = find_roundest(contours)

        M = cv2.moments(roundest_contour)
        # M["m00"] will not be 0 practically
        if M["m00"] == 0:
            return None
        centerX = int(M["m10"] / M["m00"])
        centerY = int(M["m01"] / M["m00"])
        return centerX, centerY
    else:
        return None

def laser_tracking(img,kp,h_angle,v_angle):
    laser_center=find_laser(img)
    if laser_center is None:
        return None
    frame_center=(img.shape[1] // 2, img.shape[0] // 2)
    error=(frame_center[0]-laser_center[0], frame_center[1]-laser_center[1])
    h_angle += error[0]*kp
    v_angle += error[1]*kp
    h_angle = max(0, min(180, h_angle))
    v_angle   = max(0, min(180, v_angle))
    return h_angle,v_angle

def laser_searching(current_angle,scan_h, scan_v):
        h_angle, v_angle = current_angle
        h_angle += scan_h
        if h_angle >= 180:
            h_angle = 0
            v_angle += scan_v
            v_angle = max(0, min(180, v_angle))
        if v_angle >= 180:
            v_angle = 0
            print(f"Scanning: Horizontal Angle={h_angle:.2f}, Vertical Angle={v_angle[1]:.2f}") 
        return h_angle,v_angle