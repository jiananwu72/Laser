import cv2
import numpy as np

# Points in the image
img_pts = np.array([
    [227, -41],
    [-230, -31],
    [220, 105],
    [-221, 115]
], dtype=np.float32)

# Corresponding points in the real-world in cm
spacial_pts = np.array([
    [10, 30.3],
    [-10, 30.3],
    [20, 60.6],
    [-20, 60.6]
], dtype=np.float32)

H, status = cv2.findHomography(img_pts, spacial_pts)
print("Homography Matrix:\n", H)
np.save('Laser/LaserPath/Utils/Homography.npy', H)

img_point = np.array([0, -36, 1], dtype=np.float32)
spacial_point_homogeneous = H.dot(img_point)
spacial_point = spacial_point_homogeneous / spacial_point_homogeneous[2]
print("Spacial point for green point 1:\n", spacial_point)

img_point = np.array([1, 110, 1], dtype=np.float32)
spacial_point_homogeneous = H.dot(img_point)
spacial_point = spacial_point_homogeneous / spacial_point_homogeneous[2]
print("Spacial point for green point 2:\n", spacial_point)

spacial_point = np.array([-10, 44.3, 1], dtype=np.float32)
image_point_homogeneous = np.linalg.inv(H).dot(spacial_point)
image_point = image_point_homogeneous / image_point_homogeneous[2]
print("Image point for (-10, -46.3):\n", image_point)

# Homography Matrix:
#  [[ 5.01634997e-02 -3.36875705e-04  4.88757911e-02]
#  [-3.71041247e-04 -8.32597371e-03  3.44357610e+01]
#  [-9.51348750e-05 -4.06282513e-03  1.00000000e+00]]
# Spacial point for green point 1:
#  [ 0.05321936 30.30328585  1.        ]
# Spacial point for green point 2:
#  [ 0.11208612 60.61463009  1.        ]
# Image point for (-10, -46.3):
#  [-153.49933754   60.90163513    1.        ]