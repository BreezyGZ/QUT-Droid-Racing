from cmath import pi
import cv2 as cv
import numpy as np
import math
import sys
from matplotlib import pyplot as plt
from pyparsing import match_previous_expr
from functions import frameRescale, perspectiveShift, findAverageX, findMaxY, findMinY, direction, gradientOfMask

BLACK_THRESHOLD = 20
# ??? Will need to test these parameters
CONTOUR_AREA_THRESHOLD = 1000
SIMILARITY_THRESHOLD = 3.3

sign_left = frameRescale(cv.imread("Photos/turn_left.png"), 0.15)
sign_right = frameRescale(cv.imread("Photos/turn_right.png"), 0.15)
sign_left_mask = cv.bitwise_not(cv.cvtColor(sign_left, cv.COLOR_BGR2GRAY))
sign_right_mask = cv.bitwise_not(cv.cvtColor(sign_right, cv.COLOR_BGR2GRAY))
cnts_left, hierarchy_left = cv.findContours(sign_left_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
cnts_right, hierarchy_right = cv.findContours(sign_right_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
contour_left = cnts_left[0]
contour_right = cnts_right[0]

img = cv.imread("Photos/aditha.jpg")
img_resized = frameRescale(img, 0.15)
blur = cv.medianBlur(img_resized, 9)
greyscale = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)
black_mask = cv.inRange(greyscale, 0, BLACK_THRESHOLD)


def signRecognise(frame, contour_sign_left, contour_sign_right, black_threshold, similarity_threshold):
    """
    signRecognise searches an image for a black object over the size of a given threshold, and compares that 
    image to contours of the sign for left and right. If the object looks similar to one of the signs,
    it returns a string "left" or "right" respectively. If no black objects big enough are detected, or if the 
    object doesn't look similar enough it returns None.
    """
    blur = cv.medianBlur(frame, 9)
    greyscale = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)
    black_mask = cv.inRange(greyscale, 0, black_threshold)
    cnts_img = cv.findContours(black_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]
    big_contours = []

    for c in cnts_img:
        area = cv.contourArea(c)
        if area > CONTOUR_AREA_THRESHOLD:
            big_contours.append(c)
    if len(big_contours) == 0:
        return None

    match_left = cv.matchShapes(contour_sign_left, big_contours[0], 1, 0.0)
    match_right = cv.matchShapes(contour_sign_right, big_contours[0], 1, 0.0)
    if (match_left < match_right and match_left < similarity_threshold):
        return "left"
    elif (match_right < match_left and match_right < similarity_threshold):
        return "right"
    
    return None







big_contours = []
for c in cnts_img:
    area = cv.contourArea(c)
    if area > CONTOUR_AREA_THRESHOLD:
        big_contours.append(c)

match_left = cv.matchShapes(contour_left, big_contours[0], 1, 0.0)
match_right = cv.matchShapes(contour_right, big_contours[0], 1, 0.0)
print(match_left)
print(match_right)
# contours,hierarchy = cv.findContours(black_mask,2,1)
# cnt1 = contours[0]
# contours,hierarchy = cv.findContours(sign_left_mask,2,1)
# cnt2 = contours[0]

# ret = cv.matchShapes(cnt1,cnt2,1,0.0)
cv.drawContours(img_resized, [big_contours[0]], -1, (0,255,0), 3)
cv.imshow("original", img_resized)
cv.imshow("black_mask", black_mask)

cv.waitKey(10000)
cv.destroyAllWindows()