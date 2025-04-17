import cv2
import numpy as np
from scipy import ndimage
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

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
x_blue = 181
y_blue= 4
x_red = 0
y_red = 102
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
print(f"Setting servo to initial vertical angle: {v_angle}°")
vertical_servo.angle = v_angle

# take frame
ret, frame = cap.read()
frame = cv2.flip(frame, 0)
# frame = ndimage.rotate(frame, -1, reshape=False)
# write frame to file
cv2.circle(frame, (320, 240), 1, (0, 255, 0), 2)
cv2.circle(frame, (320+x_blue, 240-y_blue), 1, (255, 0, 0), 2)
cv2.circle(frame, (320+x_red, 240-y_red), 1, (0, 0, 255), 2)
cv2.imwrite(f"Laser/LaserPath/videos/Degree tests/{v_angle}_labeled.jpg", frame)
# release camera
time.sleep(2)
cap.release()
