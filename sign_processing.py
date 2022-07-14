from cmath import pi
import cv2 as cv
import numpy as np
import math
import sys
from matplotlib import pyplot as plt
from functions import biasedSendTurn, sendTurn, TurnLeft, TurnRight, goStraight, frameRescale, perspectiveShift, findAverageX, findMaxY, findMinY, direction, gradientOfMask, crop
from global_variables import BLACK_LOWER, BLACK_THRESHOLD, CONTOUR_AREA_THRESHOLD_BLACK, DURATION_OF_TURN, SIMILARITY_THRESHOLD, CONTOUR_LEFT, CONTOUR_RIGHT
import time

# sign_left = frameRescale(cv.imread("Photos/turn_left.png"), 0.15)
# sign_right = frameRescale(cv.imread("Photos/turn_right.png"), 0.15)
# sign_left_mask = cv.bitwise_not(cv.cvtColor(sign_left, cv.COLOR_BGR2GRAY))
# sign_right_mask = cv.bitwise_not(cv.cvtColor(sign_right, cv.COLOR_BGR2GRAY))
# cnts_left, hierarchy_left = cv.findContours(sign_left_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
# cnts_right, hierarchy_right = cv.findContours(sign_right_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
# contour_left = cnts_left[0]
# contour_right = cnts_right[0]


# # sign_left = frameRescale(cv.imread("Photos/turn_left.png"), 0.15)
# # sign_right = frameRescale(cv.imread("Photos/turn_right.png"), 0.15)
# # sign_left_mask = cv.bitwise_not(cv.cvtColor(sign_left, cv.COLOR_BGR2GRAY))
# # sign_right_mask = cv.bitwise_not(cv.cvtColor(sign_right, cv.COLOR_BGR2GRAY))
# # cnts_left, hierarchy_left = cv.findContours(sign_left_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
# # cnts_right, hierarchy_right = cv.findContours(sign_right_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
# # contour_left = cnts_left[0]
# # contour_right = cnts_right[0]

def signRecognise(frame):
    """
    signRecognise searches an image for a black object over the size of a given threshold, and compares that 
    image to contours of the sign for left and right. If the object looks similar to one of the signs,
    it returns a string "left" or "right" respectively. If no black objects big enough are detected, or if the 
    object doesn't look similar enough it returns None.
    """
    blur = crop(cv.medianBlur(frame, 5))
    greyscale = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)
    black_mask = cv.inRange(greyscale, BLACK_LOWER, BLACK_THRESHOLD)
    # cv.imshow("black_mask", black_mask)
    # cv.waitKey(5000)
    # cv.destroyAllWindows()
    cnts_img = cv.findContours(black_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]
    contour_max = None
    contour_area = 0
    for c in cnts_img:
        area = cv.contourArea(c)
        if area > max(CONTOUR_AREA_THRESHOLD_BLACK, contour_area):
            contour_max = c
            contour_area = area

    if contour_max is None:
        return None
    match_left = cv.matchShapes(CONTOUR_LEFT, contour_max, 1, 0.0)
    match_right = cv.matchShapes(CONTOUR_RIGHT, contour_max, 1, 0.0)
    if match_left < min(match_right, SIMILARITY_THRESHOLD)):
        return "left"
    elif match_right < min(match_left, SIMILARITY_THRESHOLD):
        return "right"
    return None

def sign_detected_script(ser, sign_direction, edge_blue, edge_yellow):
    if sign_direction == "left":
        TurnLeft(ser, 45)
    if sign_direction == "right":
        TurnRight(ser, 45)
    
    count = DURATION_OF_TURN
    while count > 0:
        biasedSendTurn(ser, direction(edge_blue, edge_yellow), sign_direction)
        count -= 1
        time.sleep(0.1)
    return

# big_contours = []
# for c in cnts_img:
#     area = cv.contourArea(c)
#     if area > CONTOUR_AREA_THRESHOLD:
#         big_contours.append(c)

# match_left = cv.matchShapes(contour_left, big_contours[0], 1, 0.0)
# match_right = cv.matchShapes(contour_right, big_contours[0], 1, 0.0)
# print(match_left)
# print(match_right)
# # contours,hierarchy = cv.findContours(black_mask,2,1)
# # cnt1 = contours[0]
# # contours,hierarchy = cv.findContours(sign_left_mask,2,1)
# # cnt2 = contours[0]

# # ret = cv.matchShapes(cnt1,cnt2,1,0.0)
# cv.drawContours(img_resized, [big_contours[0]], -1, (0,255,0), 3)
# cv.imshow("original", img_resized)
# cv.imshow("black_mask", black_mask)

# cv.waitKey(10000)
# cv.destroyAllWindows()

# img = cv.imread("Photos/test/test_still_right.jpg")
# img_resized = frameRescale(img, 0.15)
# signRecognise(img_resized)