import cv2 as cv
import numpy as np
from functions import frameRescale

PERSPECTIVE_SHIFT_COORDS = [(), (), (), ()]

BLUE_LOWER = np.array([100, 93, 15])
BLUE_UPPER = np.array([120, 255, 255])
PURPLE_LOWER = np.array([135, 93, 15])
PURPLE_UPPER = np.array([155, 255, 255])
YELLOW_LOWER = np.array([15, 93, 15])
YELLOW_UPPER = np.array([30, 255, 255])
GREEN_LOWER = np.array([35, 40, 40])
GREEN_UPPER = np.array([65, 255, 255])
BLACK_THRESHOLD = 40

#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24

# ??? Will need to test these parameters
CONTOUR_AREA_THRESHOLD_BLACK = 1000
CONTOUR_AREA_THRESHOLD_PURPLE = 500
CONTOUR_AREA_THRESHOLD_LINE = 400
SIMILARITY_THRESHOLD = 3.3
DURATION_OF_TURN = 10

sign_left = frameRescale(cv.imread("Photos/sign_left_still_.jpg"), 0.15)
sign_right = frameRescale(cv.imread("Photos/sign_right_still_.jpg"), 0.15)
grey_left = cv.cvtColor(sign_left, cv.COLOR_BGR2GRAY)
grey_right = cv.cvtColor(sign_right, cv.COLOR_BGR2GRAY)
sign_left_mask = cv.inRange(grey_left, 0, BLACK_THRESHOLD)
sign_right_mask = cv.inRange(grey_right, 0, BLACK_THRESHOLD)
contours_left = cv.findContours(sign_left_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]
contours_right = cv.findContours(sign_right_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]
# print(contours_left)
# max_contour_left = contours_left[0]
# max_contour_right = contours_right[0]
# for contour in contours_left:
#     if cv.contourArea(contour) > cv.contourArea(max_contour_left):
#         max_contour_left = contour
        
# for contour in contours_right:
#     if cv.contourArea(contour) > cv.contourArea(max_contour_right):
#         max_contour_left = contour

# cv.imshow("contour", sign_left_mask)
# cv.imshow("contour1", sign_right_mask)
# cv.waitKey(20000)
# cv.destroyAllWindows()

CONTOUR_LEFT = cv.findContours(sign_left_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0][0]
CONTOUR_RIGHT = cv.findContours(sign_right_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0][0]