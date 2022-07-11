from cmath import pi
import cv2 as cv
import numpy as np
import math
import sys

BLACK_THRESHOLD = 20

def frameRescale(frame, scale):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dim = (width, height)
    return cv.resize(frame, dim, interpolation = cv.INTER_AREA)

sign_left = cv.imread("Photos/turn_left.png")
sign_right = cv.imread("Photos/turn_right.png")
img = cv.imread("Photos/test/test_sign_left.jpg")
img_resized = frameRescale(img, 0.20)
greyscale = cv.cvtColor(img_resized, cv.COLOR_BGR2GRAY)


sign_left_mask = cv.bitwise_not(sign_left)
sign_right_mask = cv.bitwise_not(sign_right)
black_mask = cv.inRange(greyscale, 0, BLACK_THRESHOLD)

# contours,hierarchy = cv.findContours(black_mask,2,1)
# cnt1 = contours[0]
# contours,hierarchy = cv.findContours(sign_left_mask,2,1)
# cnt2 = contours[0]

# ret = cv.matchShapes(cnt1,cnt2,1,0.0)

cv.imshow("original", img)
cv.imshow("black_mask", black_mask)

cv.waitKey(10000)
cv.destroyAllWindows()