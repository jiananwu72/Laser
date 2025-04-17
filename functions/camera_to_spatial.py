import cv2
import numpy as np
import sys
import os
sys.path.append(os.path.expanduser("~/Laser"))

def camera_to_spatial(x, y):
    H = np.load('/home/chen/Laser/functions/Homography.npy')
    img_point = np.array([x, y, 1], dtype=np.float32)
    spacial_point_homogeneous = H.dot(img_point)
    spacial_point = spacial_point_homogeneous / spacial_point_homogeneous[2]
    return spacial_point
