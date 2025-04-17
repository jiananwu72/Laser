import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image
image = cv2.imread('LaserPath/RawImages/Raw2.jpg')
yuv_image = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)

# np.savetxt("LaserPath/YUVmap/Y_map.txt", Y, delimiter=",", fmt='%d')

Y, U, V = cv2.split(yuv_image)
cv2.imwrite("LaserPath/ProcessedImages/Raw2/Y_map.png", Y)
cv2.imwrite("LaserPath/ProcessedImages/Raw2/U_map.png", U)
cv2.imwrite("LaserPath/ProcessedImages/Raw2/V_map.png", V)

# Display results and save plots
fig, ax = plt.subplots(1, 4, figsize=(15, 5))

titles = ["Original Image", "Y", "U", "V"]
images = [cv2.cvtColor(image, cv2.COLOR_BGR2RGB), Y, U, V]
cmaps = [None, "gray", "gray", "gray"]

for i in range(4):
    im = ax[i].imshow(images[i], cmap=cmaps[i])
    ax[i].set_title(titles[i])
    ax[i].axis("off")
    
    if i > 0:
        cbar = fig.colorbar(im, ax=ax[i])
        cbar.ax.tick_params(labelsize=8)

# Save the combined figure
plt.savefig("LaserPath/ProcessedImages/Raw2/YUVmap.png", bbox_inches="tight", dpi=300)
