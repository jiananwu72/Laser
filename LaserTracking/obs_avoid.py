import time
from adafruit_rplidar import RPLidar

# LiDAR configuration and detection parameters
PORT_NAME = "/dev/ttyUSB0"  # Change if needed
detection_threshold = 1000  # Distance threshold in mm
region_min = 105            # Minimum angle of detection region in degrees
region_max = 255            # Maximum angle of detection region in degrees

# Initialize the LiDAR
lidar = RPLidar(None, PORT_NAME, timeout=3)

print("Starting LiDAR scan for obstacle edge detection...")
try:
    for scan in lidar.iter_scans():
        obstacle_angles = []  # List to store angles of points considered as obstacles

        # Iterate over each point in the scan
        for (_, angle, distance) in scan:
            # Only consider points within the specified region
            if region_min <= angle <= region_max:
                # If the distance is below the threshold, consider it an obstacle point
                if distance < detection_threshold:
                    obstacle_angles.append(angle)

        # Determine the left and right edges of the obstacle cluster
        if obstacle_angles:
            left_edge = min(obstacle_angles)
            right_edge = max(obstacle_angles)
            print(f"Obstacle detected: Left edge at {left_edge:.1f}°, Right edge at {right_edge:.1f}°")
        else:
            print("No obstacles detected in the region.")

        # Delay between scans to avoid overloading the console
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting obstacle edge detection...")

finally:
    lidar.stop()
    lidar.disconnect()
    print("LiDAR connection closed.")
