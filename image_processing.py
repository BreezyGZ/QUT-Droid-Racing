from cmath import pi
import cv2 as cv
import numpy as np
import math
import sys
from functions import frameRescale, perspectiveShift, findAverageX, findMaxY, findMinY, direction, gradientOfMask, goStraight, TurnLeft, TurnRight

BLUE_LOWER = np.array([105, 93, 0])
BLUE_UPPER = np.array([135, 255, 255])
YELLOW_LOWER = np.array([15, 93, 0], dtype="uint8")
YELLOW_UPPER = np.array([45, 255, 255], dtype="uint8")
PERSPECTIVE_SHIFT_COORDS = [(), (), (), ()]

    
img = cv.imread('Photos/test/test_right.jpg')
img_resized = frameRescale(img, 0.20)
perspective_shifted = perspectiveShift(img_resized)
hsv_img = cv.cvtColor(perspective_shifted, cv.COLOR_BGR2HSV)

blue_mask = cv.inRange(hsv_img, BLUE_LOWER, BLUE_UPPER)
yellow_mask = cv.inRange(hsv_img, YELLOW_LOWER, YELLOW_UPPER)

edge_yellow = cv.Canny(yellow_mask, 40, 150)
edge_blue = cv.Canny(blue_mask, 40, 150)

# ditching hough approach
# lines_yellow = cv.HoughLinesP(edge_yellow, 1, np.pi / 180, 50, None, 50, 10)
# lines_blue = cv.HoughLinesP(edge_blue, 1, np.pi / 180, 50, None, 50, 10)

# cv.imshow("hsv", hsv_img)
# cv.imshow("original", img_resized)
# cv.imshow("perspective shift", perspective_shifted)
# cv.imshow ("yellow_mask", yellow_mask)
# cv.imshow ("blue_mask", blue_mask)
# cv.imshow ("yellow_edge", edge_yellow)
# cv.imshow ("blue_edge", edge_blue)

# compare gradients of yellow and blue masks and choose whichever is greater, 
# use that gradient to determine steering amount 

working_gradient = direction(edge_blue, edge_yellow)

print(f'working gradient = {working_gradient}')

if working_gradient is None:
    goStraight()

else:
    gradient_angle = math.atan(working_gradient)

    if gradient_angle < 0:
        turn_angle = math.degrees(math.pi/2 + gradient_angle)
        TurnRight(turn_angle)

    else:
        turn_angle = math.degrees(math.pi/2 - gradient_angle)
        TurnLeft(turn_angle)

cv.waitKey(40000)
cv.destroyAllWindows()