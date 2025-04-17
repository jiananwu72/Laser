import cv2
import numpy as np

def FindRoundest(contours):
    # For faster processing speed
    if len(contours) == 1:
        return contours[0]
    
    max_circularity = 0
    roundest_contour = None
    for c in contours:
        area = cv2.contourArea(c)

        # Optionally skip very small contours to avoid noise:
        if area < 30:
            continue

        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue

        circularity = 4 * np.pi * area / (perimeter * perimeter)
        if circularity > max_circularity:
            max_circularity = circularity
            roundest_contour = c
            
    return roundest_contour