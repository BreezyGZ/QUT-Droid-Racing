from cmath import pi
import cv2 as cv
import numpy as np
import math
import sys
from matplotlib import pyplot as plt
from pyparsing import match_previous_expr
from functions import frameRescale, perspectiveShift, findAverageX, findMaxY, findMinY, direction, gradientOfMask, findLargestContour
from global_variables import BLACK_THRESHOLD, CONTOUR_AREA_THRESHOLD_BLACK, SIMILARITY_THRESHOLD, YELLOW_LOWER, YELLOW_UPPER, BLUE_LOWER, BLUE_UPPER, PURPLE_LOWER, PURPLE_UPPER

"""
Pseudo-code:

if purple detected and ultra sonic detected:
    measure distance between purple and yellow line horozontally, and purple and blue

    if distance(purple, blue) < distance(purple, yellow):
        
"""

img = cv.imread("Photos/palette_wheel.png")
img_resized = frameRescale(img, 0.5)
hsv_img = cv.cvtColor(img_resized, cv.COLOR_BGR2HSV)
blue_mask = cv.inRange(hsv_img, BLUE_LOWER, BLUE_UPPER)
yellow_mask = cv.inRange(hsv_img, YELLOW_LOWER, YELLOW_UPPER)
purple_mask = cv.inRange(hsv_img, PURPLE_LOWER, PURPLE_UPPER)

cv.imshow("original", img_resized)
cv.imshow("purple", purple_mask)

cv.waitKey(10000)
cv.destroyAllWindows()