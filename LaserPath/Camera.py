import cv2
import numpy as np

# open camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

# take frame
ret, frame = cap.read()
cv2.imwrite('Laser/LaserPath/RawImages/Degree tests/65Raw.jpg', frame)
cap.release()