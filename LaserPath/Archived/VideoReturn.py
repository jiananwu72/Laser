import cv2
import numpy as np
import sys
import os
sys.path.append(os.path.expanduser("~/Laser"))
from functions import laser_function as lf 

def video_return():
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
    out = None
        
    while True:
        ret, frame = cap.read()

        if out is None:
            height, width = frame.shape[:2]
            fps = 20.0
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter('LaserPath/output.mp4', fourcc, fps, (width, height))
        
        laser_pos = lf.find_laser(frame)
        if laser_pos:
            centerX, centerY = laser_pos
            print("Laser position:", centerX, centerY)

            cv2.circle(frame, (centerX, centerY), 10, (0, 255, 0), 2)
            cv2.putText(frame, f"({centerX}, {centerY})", (centerX + 10, centerY + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        out.write(frame)

def main():
    video_return()

if __name__ == "__main__":
    main()