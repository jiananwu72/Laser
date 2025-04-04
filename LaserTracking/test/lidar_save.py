import csv
import time
from math import floor
from adafruit_rplidar import RPLidar

# Define the LIDAR port (Change if needed)
PORT_NAME = "/dev/ttyUSB0"

# Initialize the LIDAR
lidar = RPLidar(None, PORT_NAME, timeout=3)

# Define output file
output_file = "test_data/lidar_data.csv"

try:
    print("Starting LIDAR scan...")
    
    with open(output_file, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Angle", "Distance"])  # CSV Header

        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                if angle>0 and angle<360:
                    writer.writerow([floor(angle), distance])  # Save angle & distance
            print(f"Saved {len(scan)} points...")
            
            time.sleep(0.1)  # Adjust for desired scan frequency

except KeyboardInterrupt:
    print("Stopping LIDAR scan...")

finally:
    lidar.stop()
    lidar.disconnect()
    print(f"Data saved to {output_file}")
