import cv2
import time
import sys
import os
from collections import deque

# ensure your Laser functions are on the path
sys.path.append(os.path.expanduser("~/Laser"))
from functions import laser_function as lf
from functions import camera_to_spatial as cts

def video_taker(source='/dev/video0',
                output_path='/home/chen/Laser/LaserPath/videos/laser_path.mp4',
                init_timeout=5.0,
                final_timeout=5.0,
                detection_ratio=0.05):
    """
    Capture from `source` until:
      1) Init phase (init_timeout seconds): if detected_frames/total_frames < detection_ratio → abort
      2) Final phase (sliding window of final_timeout seconds): if detection_ratio not met → stop

    Saves to output_path (MP4). Returns the path.
    """
    # Open video source
    cap = cv2.VideoCapture(source, cv2.CAP_V4L)
    if not cap.isOpened():
        raise IOError(f"Cannot open video source: {source}")

    # Ensure .mp4 extension
    if not output_path.lower().endswith('.mp4'):
        output_path += '.mp4'

    # Setup MP4 writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w      = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h      = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out    = cv2.VideoWriter(output_path, fourcc, fps, (w, h))

    # Phase tracking
    phase            = 'init'
    phase_start      = time.time()
    total_frames     = 0
    detected_frames  = 0
    buffer           = deque()  # for sliding-window in final phase
    detected         = False

    print("Starting capture...")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("End of stream reached.")
            break

        # Write every frame
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        out.write(frame)
        now      = time.time()
        detected = lf.find_laser(frame) is not None

        if phase == 'init':
            total_frames += 1
            if detected:
                detected_frames += 1
            elapsed = now - phase_start
            if elapsed >= init_timeout:
                ratio = detected_frames / total_frames if total_frames else 0
                if ratio < detection_ratio:
                    print(f"[INIT] Detected in {ratio*100:.1f}% of frames (< threshold) → abort.")
                    break
                else:
                    print(f"[INIT] Detected in {ratio*100:.1f}% of frames → entering final phase.")
                    phase       = 'final'
                    phase_start = now
                    detected = True
                    buffer.clear()

        else:
            # Append to sliding-window buffer
            buffer.append((now, detected))
            # Purge old entries
            while buffer and buffer[0][0] < now - final_timeout:
                buffer.popleft()

            # Check only after initial window elapses
            if now - phase_start >= final_timeout:
                total         = len(buffer)
                detected_count = sum(1 for ts, d in buffer if d)
                ratio         = detected_count / total if total else 0
                if ratio < detection_ratio:
                    print(f"[FINAL] Laser seen only {ratio*100:.1f}% over last {final_timeout}s → stopping.")
                    break
                # else:
                    print(f"[FINAL] Laser present ({ratio*100:.1f}% ≥ threshold). Continuing.")

    cap.release()
    out.release()
    print(f"Saved video to {output_path}")
    return detected, output_path
