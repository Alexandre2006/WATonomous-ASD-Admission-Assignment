import numpy as np
import matplotlib.pyplot as plt
global_map = []
size = int(len(global_map) ** 0.5)
grid = np.array(global_map).reshape((size, size))
img = (grid / grid.max() * 255).astype(np.uint8) if grid.max() > 0 else grid
plt.imshow(img, cmap="gray", origin="upper")
plt.axis("off")
plt.savefig("costmap.png", bbox_inches="tight", pad_inches=0)
plt.show()