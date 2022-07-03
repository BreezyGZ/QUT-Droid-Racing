import cv2 as cv
import numpy as np
import math
import sys

def frameRescale(frame, scale):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dim = (width, height)
    return cv.resize(frame, dim, interpolation = cv.INTER_AREA)

# finds y-value of the non-zero pixel closest to top of image
def findMinY(mask):
    points = cv.findNonZero(mask)
    minY = mask.shape[0]
    for point in points:
        if point[0][1] < minY:
            minY = point[0][1]
    return minY

# finds y-value of the non-zero pixel closest to bottom of image
def findMaxY(mask):
    points = cv.findNonZero(mask)
    maxY = 0
    for point in points:
        if point[0][1] > maxY:
            maxY = point[0][1]
    return maxY

# finds average of x-values at a certain y value
def findAverageX(mask, y):
    points = cv.findNonZero(mask)
    sum = 0
    total = 0
    for point in points:
        if point[0][1] == y:
            sum += point[0][0]
            total += 1
    return sum/total

# finds the gradient of the line drawn between the bottom most non-zero pixel 
# and the centre of all non-zero pixels
def gradientOfMask(mask):
    ave_y = (findMinY(mask) + findMaxY(mask))/2
    max = (findAverageX(mask, findMaxY(mask)), findMaxY(mask))
    ave = (findAverageX(mask, ave_y), ave_y)
    print(max)
    print(ave)
    # divison by zero check?
    return (ave[1] - max[1])/(ave[0] - max[0])


img = cv.imread('Photos/curved_yellow.png')
img_resized = frameRescale(img, 0.45)

hsv_img = cv.cvtColor(img_resized, cv.COLOR_BGR2HSV)

# global variables?
yellow_lower = np.array([15, 93, 0], dtype="uint8")
yellow_upper = np.array([45, 255, 255], dtype="uint8")
yellow_mask = cv.inRange(hsv_img, yellow_lower, yellow_upper)

blue_lower = np.array([105, 93, 0])
blue_upper = np.array([135, 255, 255])
blue_mask = cv.inRange(hsv_img, blue_lower, blue_upper)

edge_yellow = cv.Canny(yellow_mask, 50, 150)
edge_blue = cv.Canny(blue_mask, 50, 150)

# ditching hough approach
# lines_yellow = cv.HoughLinesP(edge_yellow, 1, np.pi / 180, 50, None, 50, 10)
# lines_blue = cv.HoughLinesP(edge_blue, 1, np.pi / 180, 50, None, 50, 10)

# cv.imshow("hsv", hsv_img)
cv.imshow("original", img_resized)
cv.imshow ("yellow_mask", yellow_mask)
# cv.imshow ("blue_mask", blue_mask)
cv.imshow ("yellow_edge", edge_yellow)
# cv.imshow ("blue_edge", edge_blue)

# compare gradients of yellow and blue masks and choose whichever is greater, 
# use that gradient to determine steering amount 
print(gradientOfMask(edge_yellow))




cv.waitKey(0000)
cv.destroyAllWindows()