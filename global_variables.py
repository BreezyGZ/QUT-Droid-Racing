import cv2 as cv
import numpy as np
from functions import frameRescale

PERSPECTIVE_SHIFT_COORDS = [(), (), (), ()]

BLUE_LOWER = np.array([100, 93, 15])
BLUE_UPPER = np.array([120, 255, 255])
PURPLE_LOWER = np.array([135, 93, 15])
PURPLE_UPPER = np.array([155, 255, 255])
YELLOW_LOWER = np.array([15, 93, 15])
YELLOW_UPPER = np.array([45, 255, 255])
BLACK_THRESHOLD = 20
# ??? Will need to test these parameters
CONTOUR_AREA_THRESHOLD_BLACK = 1000
CONTOUR_AREA_THRESHOLD_PURPLE = 500
CONTOUR_AREA_THRESHOLD_LINE = 400
SIMILARITY_THRESHOLD = 3.3
DURATION_OF_TURN = 10

sign_left = frameRescale(cv.imread("Photos/turn_left.png"), 0.15)
sign_right = frameRescale(cv.imread("Photos/turn_right.png"), 0.15)
sign_left_mask = cv.bitwise_not(cv.cvtColor(sign_left, cv.COLOR_BGR2GRAY))
sign_right_mask = cv.bitwise_not(cv.cvtColor(sign_right, cv.COLOR_BGR2GRAY))
CONTOUR_LEFT = cv.findContours(sign_left_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0][0]
CONTOUR_RIGHT = cv.findContours(sign_right_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0][0]