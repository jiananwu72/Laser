import cv2
import numpy as np
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# -----------------------------
# Setup for PCA9685 and Servos
# -----------------------------
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 100  # Adjust frequency as needed

# Define channels for horizontal and vertical servo control
x_channel = 0
y_channel = 1

horizontal_servo = servo.Servo(pca.channels[x_channel])
vertical_servo = servo.Servo(pca.channels[y_channel])

# Initialize servos at 90° (center position)
horizontal_angle = 90
vertical_angle = 90
horizontal_servo.angle = horizontal_angle
vertical_servo.angle = vertical_angle

# -----------------------------
# Setup for Video Capture and Laser Detection
# -----------------------------
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Laser detection parameters
lightness_threshold = 240  # Adjust threshold as needed
min_area = 10              # Minimum contour area for laser detection

def search_laser():
    global horizontal_angle, vertical_angle
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        return False

    # Flip frame if necessary (depends on camera orientation)
    frame = cv2.flip(frame, 0)
    
    # Convert to grayscale and threshold to isolate bright spots
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, lightness_threshold, 255, cv2.THRESH_BINARY)

    # Reduce noise using morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Pick the largest contour (assuming it's the laser spot)
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > min_area:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                print("Laser found at:", cX, cY)
                return True

    # If laser not found, perform a simple scanning motion:
    # Increment horizontal angle; wrap around and increment vertical angle when needed.
    horizontal_angle += 1
    if horizontal_angle > 180:
        horizontal_angle = 0
        vertical_angle += 5  # Change vertical step size as needed
        if vertical_angle > 180:
            vertical_angle = 0

    # Update servo positions
    horizontal_servo.angle = horizontal_angle
    vertical_servo.angle = vertical_angle
    print("Scanning... Horizontal Angle =", horizontal_angle, "Vertical Angle =", vertical_angle)
    return False

try:
    while True:
        if search_laser():
            print("Laser detected! Stopping search.")
            break
        time.sleep(0.05)  # Delay to control scanning speed

except KeyboardInterrupt:
    print("Exiting search.")

cap.release()
