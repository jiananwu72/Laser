import cv2
import numpy as np
import sys
import os
from itertools import dropwhile

# ensure your Laser functions & homography loader are on the path
sys.path.append(os.path.expanduser("~/Laser"))
from functions.laser_function import find_laser
from functions.camera_to_spatial import camera_to_spatial

def extract_waypoints(video_path,
                      window_num=5):
    """
    Read `video_path`, detect laser in each frame, map to spatial coords,
    and subsample so consecutive waypoints are at least `min_dist` apart.

    Returns:
        np.ndarray of shape (N,2) with X,Y waypoints in meters.
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError(f"Extract waypoints: Cannot open video file: {video_path}")

    raw_pts = []
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        laser_pt = find_laser(frame)
        if laser_pt is not None:
            x_img, y_img = laser_pt
            x_img -= 360
            y_img = -y_img + 240
            sp = camera_to_spatial(x_img, y_img)  # returns [X, Y, 1]
            raw_pts.append((float(sp[0]), float(sp[1])))
        else:
            raw_pts.append(laser_pt)

    cap.release()
    
    valid_idxs = [i for i, pt in enumerate(raw_pts) if pt is not None]
    if valid_idxs:
        first, last = valid_idxs[0], valid_idxs[-1]
        raw_pts = raw_pts[first:last+1]
    else:
        return np.empty((0, 2))

    # subsample to enforce minimum distance
    waypoints = np.empty((0, 2), dtype=float)
    n = len(raw_pts)
    window_size = n // window_num
    for i in range(0, n, window_size):
        window = raw_pts[i:i + window_size]
        coords = [pt for pt in window if pt is not None]
        if coords:
            xs = [c[0] for c in coords]
            ys = [c[1] for c in coords]
            avg_x = np.mean(xs)
            avg_y = np.mean(ys)
            waypoints = np.vstack((waypoints, [avg_y, avg_x]))
            
    return waypoints