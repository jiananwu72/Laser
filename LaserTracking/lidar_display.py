
import csv
import numpy as np
import matplotlib.pyplot as plt

# Load LIDAR data
angles = []
distances = []

with open("test_data/lidar_data.csv", "r") as file:
    reader = csv.reader(file)
    next(reader)  # Skip header

    for row in reader:
        angles.append(float(row[0]))  # Angle
        distances.append(float(row[1]))  # Distance

# Convert polar to Cartesian for plotting
#angles_rad = np.radians(angles)
#x = np.array(distances) * np.cos(angles_rad)
#y = np.array(distances) * np.sin(angles_rad)

# Plot LIDAR scan
plt.figure(figsize=(6,6))
plt.scatter(angles, distances, s=2, color="blue")  # Small dots for each point
plt.xlabel("X Distance (mm)")
plt.ylabel("Y Distance (mm)")
plt.title("LIDAR Scan Visualization")
plt.grid(True)
plt.show()
plt.savefig('test_img/lidar.png')
