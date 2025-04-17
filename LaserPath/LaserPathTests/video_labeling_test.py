import cv2
import sys
import os

# Append Laser module path
sys.path.append(os.path.expanduser("~/Laser"))
from functions.laser_function import find_laser

# Input and output paths from command line

input_path = '/home/chen/Laser/LaserPath/videos/laser_path.mp4'
output_path = '/home/chen/Laser/LaserPath/videos/laser_path_lb.mp4'

cap = cv2.VideoCapture(input_path)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
w      = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h      = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
out    = cv2.VideoWriter(output_path, fourcc, fps, (w, h))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect laser
    pt = find_laser(frame)
    if pt is not None:
        x, y = int(pt[0]), int(pt[1])
        # Draw a small red filled circle at laser position
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

    # Write annotated frame
    out.write(frame)

cap.release()
out.release()
print("Done.")
