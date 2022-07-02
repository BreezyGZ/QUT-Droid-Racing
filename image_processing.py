import cv2 as cv
import numpy as np

def frameRescale(frame, scale):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dim = (width, height)
    return cv.resize(frame, dim, interpolation = cv.INTER_AREA)

img = cv.imread('Photos/drc_paint_lines.png')
img_resized = frameRescale(img, 0.15)

hsv_img = cv.cvtColor(img_resized, cv.COLOR_BGR2HSV)

lower = np.array([22, 93, 0], dtype="uint8")
upper = np.array([45, 255, 255], dtype="uint8")
mask = cv.inRange(hsv_img, lower, upper)

cv.imshow("penis", hsv_img)
cv.imshow("original", img_resized)
cv.imshow ("mask", mask)
cv.waitKey(20000)
cv.destroyAllWindows()