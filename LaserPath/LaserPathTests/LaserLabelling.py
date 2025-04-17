import cv2
import numpy as np
from LaserPath.Utils.FindRoundest import FindRoundest

# Load the image
image = cv2.imread('LaserPath/RawImages/CB1.jpg')

# Convert the image from BGR to HSV/YUV color space
hlsImage = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
yuvImage = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)

# Define Light Color Ranges and Create a Mask
lower_hls1 = np.array([0, 230, 0])
upper_hls1 = np.array([180, 255, 255])
lower_yuv2 = np.array([200, 0, 0])
upper_yuv2 = np.array([255, 255, 160])
mask1 = cv2.inRange(hlsImage, lower_hls1, upper_hls1)
mask2 = cv2.inRange(yuvImage, lower_yuv2, upper_yuv2)
mask = cv2.bitwise_and(mask1, mask2)

# Denoise the mask using morphological operations
kernel = np.array([[0.5, 1, 0.5], [1, 2, 1], [0.5, 1, 0.5]])
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

# Find Countours
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if contours:
    roundest_contour = FindRoundest(contours)

    M = cv2.moments(roundest_contour)
    # M["m00"] will not be 0 practically
    centerX = int(M["m10"] / M["m00"])
    centerY = int(M["m01"] / M["m00"])

    # cv2.circle(image, (centerX, centerY), 10, (0, 255, 0), -1)  # Green circle
    cv2.drawContours(image, [roundest_contour], -1, (0, 0, 255), 2)  # Red contour
    x, y, w, h = cv2.boundingRect(roundest_contour)
    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Blue rectangle
    label_text = f"Laser: ({centerX}, {centerY})"
    cv2.putText(image, label_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
else:
    roundest_countour = None
    print("No red laser beam detected.")

cv2.imwrite("LaserPath/ProcessedImages/CB1/ContourLabel.jpg", image)

# Optionally, display the result in a window (press any key to close)
# cv2.imshow("Detected Laser", image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
