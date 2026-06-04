import cv2 as cv
import numpy as np

cv.imread('Photos/yo-tag.png')

# Create a black image window
img = np.zeros((512, 512, 3), np.uint8)
cv.imshow("Test Window", img)

# Keep the window open until a key is pressed
cv.waitKey(0)
cv.destroyAllWindows()
