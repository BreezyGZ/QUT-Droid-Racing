import cv2 as cv
import numpy as np
import math
import sys

def frameRescale(frame, scale):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dim = (width, height)
    return cv.resize(frame, dim, interpolation = cv.INTER_AREA)

# def significantLine(mask_blue, mask_yellow):
#     penis


img = cv.imread('Photos/aditha.jpg')
img_resized = frameRescale(img, 0.10)

hsv_img = cv.cvtColor(img_resized, cv.COLOR_BGR2HSV)

yellow_lower = np.array([15, 93, 0], dtype="uint8")
yellow_upper = np.array([45, 255, 255], dtype="uint8")
yellow_mask = cv.inRange(hsv_img, yellow_lower, yellow_upper)

blue_lower = np.array([105, 93, 0])
blue_upper = np.array([135, 255, 255])
blue_mask = cv.inRange(hsv_img, blue_lower, blue_upper)

edge_yellow = cv.Canny(yellow_mask, 50, 150)
edge_blue = cv.Canny(blue_mask, 50, 150)

lines_yellow = cv.HoughLinesP(edge_yellow, 1, np.pi / 180, 50, None, 50, 10)
lines_blue = cv.HoughLinesP(edge_blue, 1, np.pi / 180, 50, None, 50, 10)
print(lines_yellow)
print(lines_blue)
cv.imshow("hsv", hsv_img)
cv.imshow("original", img_resized)
cv.imshow ("yellow_mask", yellow_mask)
cv.imshow ("blue_mask", blue_mask)
cv.imshow ("yellow_edge", edge_yellow)
cv.imshow ("blue_edge", edge_blue)


cv.waitKey(20000)
cv.destroyAllWindows()