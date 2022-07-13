from cmath import pi
from dis import dis
import cv2 as cv
from cv2 import findNonZero
import numpy as np
import math
import sys
from matplotlib import pyplot as plt
from functions import TurnLeft, TurnRight, biasedSendTurn, frameRescale, perspectiveShift, findAverageX, findMaxY, findMinY, direction, gradientOfMask
from global_variables import BLACK_THRESHOLD, CONTOUR_AREA_THRESHOLD_BLACK, CONTOUR_AREA_THRESHOLD_PURPLE, SIMILARITY_THRESHOLD, CONTOUR_AREA_THRESHOLD_LINE, YELLOW_LOWER, YELLOW_UPPER, BLUE_LOWER, BLUE_UPPER, PURPLE_LOWER, PURPLE_UPPER

"""
Pseudo-code:

if purple detected and ultra sonic detected:
    measure distance between purple and yellow line horozontally, and purple and blue

    if distance(purple, blue) < distance(purple, yellow):
        
"""

# img = cv.imread("Photos/palette_wheel.png")
# img_resized = frameRescale(img, 0.5)
# hsv_img = cv.cvtColor(img_resized, cv.COLOR_BGR2HSV)
# blue_mask = cv.inRange(hsv_img, BLUE_LOWER, BLUE_UPPER)
# yellow_mask = cv.inRange(hsv_img, YELLOW_LOWER, YELLOW_UPPER)
# purple_mask = cv.inRange(hsv_img, PURPLE_LOWER, PURPLE_UPPER)
# edge_yellow = findLargestContour(yellow_mask, CONTOUR_AREA_THRESHOLD_LINE)[0]
# edge_blue = findLargestContour(blue_mask, CONTOUR_AREA_THRESHOLD_LINE)[0]
# block_edges = findLargestContour(purple_mask, CONTOUR_AREA_THRESHOLD_PURPLE)[0]

def obstacle_avoid_script(ser, object_distance, hsv_img, blue_mask, yellow_mask):
    purple_mask = cv.inRange(hsv_img, PURPLE_LOWER, PURPLE_UPPER)
    if cv.findNonZero(purple_mask) is None:
        return
    obs_base = (findAverageX(purple_mask, findMaxY(purple_mask)), findMaxY(purple_mask))

    if blue_mask is None:
        distance_from_blue = 0
    else:
        distance_from_blue = abs(obs_base[0] - findAverageX(blue_mask, obs_base[1]))
    if yellow_mask is None:
        distance_from_yellow = 0
    else:
        distance_from_yellow = abs(obs_base[0] - findAverageX(yellow_mask, obs_base[1]))

    distance_diff = distance_from_blue - distance_from_yellow
    if object_distance > 50:
        if distance_diff > 0:
            biasedSendTurn(ser, direction(blue_mask, yellow_mask), "left")
        if distance_diff <= 0:
            biasedSendTurn(ser, direction(blue_mask, yellow_mask), "right")

    else:
        if distance_diff > 0:
            while object_distance <= 50:
                TurnLeft(ser, 45)
        if distance_diff <= 0:
            while object_distance <= 50:
                TurnRight(ser, 45)
    return


# if block_edges is None:
#     help = "help"
# obs_base = (findAverageX(block_edges, findMaxY(block_edges)), findMaxY(block_edges))

# if edge_blue is None:
#     distance_from_blue = 0
# else:
#     distance_from_blue = abs(obs_base[0] - findAverageX(edge_blue, obs_base[1]))

# if edge_yellow is None:
#     distance_from_yellow = 0
# else:
#     distance_from_yellow = abs(obs_base[0] - findAverageX(edge_yellow, obs_base[1]))



# cv.imshow("original", img_resized)
# cv.imshow("purple", purple_mask)

# cv.waitKey(10000)
# cv.destroyAllWindows()