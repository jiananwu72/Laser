import cv2
import numpy as np
import glob
from scipy import ndimage
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

from LaserPath.Archived.FindLaser import FindLaser
from Utils.CameraCalibration import Undistort

# Initialize I2C connection and PCA9685 board
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100

# camera height: 11.7cm at 66 degrees
# ground block width: 30.3cm

# Define the servo channel (e.g., channel 0)
x_channel = 0
y_channel = 1
horizontal_servo = servo.Servo(pca.channels[x_channel])
vertical_servo = servo.Servo(pca.channels[y_channel])

h_angle = 96
horizontal_servo.angle = h_angle
v_angle = 65

x_green1 = 0
y_green1 = -36
x_green2 = 1
y_green2 = 110
x_green3 = -153
y_green3 = 61
x_blue1 = 227
y_blue1 = -41
x_blue2 = -230
y_blue2 = -31
x_red1 = 220
y_red1 = 105
x_red2 = -221
y_red2 = 115
vertical_servo.angle = v_angle

# take frame
frame = cv2.imread('Laser/LaserPath/RawImages/Degree tests/65Raw.jpg')
# frame = Undistort(frame)
frame = cv2.rotate(frame, cv2.ROTATE_180)

# write frame to file
cv2.circle(frame, (320+x_green1, 240-y_green1), 1, (0, 255, 0), 2)
cv2.circle(frame, (320+x_green2, 240-y_green2), 1, (0, 255, 0), 2)
cv2.circle(frame, (320+x_green3, 240-y_green3), 1, (0, 255, 0), 2)
cv2.circle(frame, (320+x_blue1, 240-y_blue1), 1, (255, 0, 0), 2)
cv2.circle(frame, (320+x_blue2, 240-y_blue2), 1, (255, 0, 0), 2)
cv2.circle(frame, (320+x_red1, 240-y_red1), 1, (0, 0, 255), 2)
cv2.circle(frame, (320+x_red2, 240-y_red2), 1, (0, 0, 255), 2)
cv2.imwrite(f"Laser/LaserPath/ProcessedImages/CoordinateTransformation/{v_angle}.jpg", frame)
# release camera
