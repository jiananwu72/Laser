import cv2
import numpy as np

def camera_to_spatial(x, y):
    H = np.load('Laser/functions//Homography.npy')
    img_point = np.array([0, x, y], dtype=np.float32)
    spacial_point_homogeneous = H.dot(img_point)
    spacial_point = spacial_point_homogeneous / spacial_point_homogeneous[2]
    return spacial_point
