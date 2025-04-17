import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image
image = cv2.imread('LaserPath/RawImages/MP2.jpg')
hls_image = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

# np.savetxt("LaserPath/HLSmap/H_map.txt", H, delimiter=",", fmt='%d')

H, L, S = cv2.split(hls_image)
cv2.imwrite("LaserPath/ProcessedImages/MP2/H_map.png", H)
cv2.imwrite("LaserPath/ProcessedImages/MP2/L_map.png", L)
cv2.imwrite("LaserPath/ProcessedImages/MP2/S_map.png", S)

# Display results and save plots
fig, ax = plt.subplots(1, 4, figsize=(15, 5))

titles = ["Original Image", "Hue", "Lightness", "Saturation"]
images = [cv2.cvtColor(image, cv2.COLOR_BGR2RGB), H, L, S]
cmaps = [None, "hsv", "gray", "gray"]

for i in range(4):
    im = ax[i].imshow(images[i], cmap=cmaps[i])
    ax[i].set_title(titles[i])
    ax[i].axis("off")
    
    if i > 0:
        cbar = fig.colorbar(im, ax=ax[i])
        cbar.ax.tick_params(labelsize=8)

# Save the combined figure
plt.savefig("LaserPath/ProcessedImages/MP2/HLSmap.png", bbox_inches="tight", dpi=300)
