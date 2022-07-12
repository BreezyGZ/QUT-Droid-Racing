from cmath import pi
import cv2 as cv
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math
import sys
import serial
from functions import frameRescale, perspectiveShift, findAverageX, findMaxY, findMinY, direction, gradientOfMask, goStraight, TurnLeft, TurnRight
from sign_processing import signRecognise

BLUE_LOWER = np.array([100, 93, 15])
BLUE_UPPER = np.array([120, 255, 255])
YELLOW_LOWER = np.array([15, 93, 15])
YELLOW_UPPER = np.array([45, 255, 255])
BLACK_THRESHOLD = 20
# ??? Will need to test these parameters
CONTOUR_AREA_THRESHOLD = 1000
SIMILARITY_THRESHOLD = 3.3

sign_left = frameRescale(cv.imread("Photos/turn_left.png"), 0.15)
sign_right = frameRescale(cv.imread("Photos/turn_right.png"), 0.15)
sign_left_mask = cv.bitwise_not(cv.cvtColor(sign_left, cv.COLOR_BGR2GRAY))
sign_right_mask = cv.bitwise_not(cv.cvtColor(sign_right, cv.COLOR_BGR2GRAY))
CONTOUR_LEFT = cv.findContours(sign_left_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0][0]
CONTOUR_RIGHT = cv.findContours(sign_right_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0][0]

camera = PiCamera()
camera.resolution = (820, 616)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(820, 616))

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()

time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    if ser.inWaiting() > 0:
        img = frame.array
        img_resized = frameRescale(img, 1)
        perspective_shifted = perspectiveShift(img_resized)
        hsv_img = cv.cvtColor(perspective_shifted, cv.COLOR_BGR2HSV)

        blue_mask = cv.inRange(hsv_img, BLUE_LOWER, BLUE_UPPER)
        yellow_mask = cv.inRange(hsv_img, YELLOW_LOWER, YELLOW_UPPER)

        edge_yellow = cv.Canny(yellow_mask, 40, 150)
        edge_blue = cv.Canny(blue_mask, 40, 150)

        sign_detected = signRecognise(img_resized, CONTOUR_LEFT, CONTOUR_RIGHT, BLACK_THRESHOLD, SIMILARITY_THRESHOLD)
        # maybe write something more complex lol?
        # could run all straight turns as left/right turns for a few loops?
        if sign_detected == "left":
            TurnLeft(45)
            working_gradient = direction(edge_blue, edge_blue)
        if sign_detected == "right":
            TurnRight(45)
            working_gradient = direction(edge_yellow, edge_yellow)
        else:
            working_gradient = direction(edge_blue, edge_yellow)
        print(f'working gradient = {working_gradient}')

        # cv.imshow("hsv", hsv_img)
        # cv.imshow("original", img_resized)
        # cv.imshow("perspective shift", perspective_shifted)
        # cv.imshow("yellow_mask", yellow_mask)
        # cv.imshow("blue_mask", blue_mask)
        # cv.imshow ("yellow_edge", edge_yellow)
        # cv.imshow ("blue_edge", edge_blue)
        key = cv.waitKey(1) & 0xFF

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
        rawCapture.truncate(0)
    

capture.release()
cv.destroyAllWindows()