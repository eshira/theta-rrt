import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from PIL import Image

img = Image.open('map1.png').convert('1')
imarray = np.array(img)

imgplot = plt.imshow(img)
print(imarray[99][1])
plt.show()