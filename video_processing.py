from cmath import pi
import cv2 as cv
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math
import sys
import serial
from functions import frameRescale, perspectiveShift, findAverageX, findMaxY, findMinY, direction, gradientOfMask, goStraight, TurnLeft, TurnRight, sendTurn, distance
# from sign_processing import signRecognise, sign_detected_script
from obstacle_detection import obstacle_avoid_script
from global_variables import BLUE_LOWER, BLUE_UPPER, YELLOW_LOWER, YELLOW_UPPER, GREEN_LOWER, GREEN_UPPER, BLACK_THRESHOLD, SIMILARITY_THRESHOLD, CONTOUR_AREA_THRESHOLD_BLACK, CONTOUR_AREA_THRESHOLD_LINE, PERSPECTIVE_SHIFT_COORDS, CONTOUR_LEFT, CONTOUR_RIGHT, GPIO_ECHO, GPIO_TRIGGER

# Libraries and software controlling ultrasonic sensor
import RPi.GPIO as GPIO
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

camera = PiCamera()
camera.resolution = (832, 624)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(832, 624))

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()

time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # if ser.inWaiting() > 0:
    time.sleep(0.1)
    img = frame.array
    img_resized = frameRescale(img, 1)
    perspective_shifted = perspectiveShift(img_resized)
    hsv_img = cv.cvtColor(perspective_shifted, cv.COLOR_BGR2HSV)
    
    blue_mask = cv.inRange(hsv_img, BLUE_LOWER, BLUE_UPPER)
    yellow_mask = cv.inRange(hsv_img, YELLOW_LOWER, YELLOW_UPPER)
    green_mask = cv.inRange(hsv_img, GREEN_LOWER, GREEN_UPPER)
    object_distance = distance(GPIO_TRIGGER, GPIO_ECHO)
    print("Object distance: %.1f" % object_distance)
    # edge_yellow = findLargestContour(yellow_mask, CONTOUR_AREA_THRESHOLD_LINE)[0]
    # edge_blue = findLargestContour(blue_mask, CONTOUR_AREA_THRESHOLD_LINE)[0]

    # is_sign = signRecognise(img_resized, CONTOUR_LEFT, CONTOUR_RIGHT, BLACK_THRESHOLD, SIMILARITY_THRESHOLD)
    # # maybe write something more complex lol?
    # # could run all straight turns as left/right turns for a few loops?
    # if is_sign is not None:
    #     sign_detected_script(ser, is_sign, blue_mask, yellow_mask)
    
    # if object_detected: 
    #     obstacle_avoid_script()

    working_gradient = direction(blue_mask, yellow_mask)
    print(working_gradient)
    sendTurn(ser, working_gradient)
    
    # if detect green, break after a certain time
    if ():
        goStraight(ser)
        time.sleep(10)
        break

    # cv.imshow("hsv", hsv_img)
    # cv.imshow("original", img_resized)
    # cv.imshow("perspective shift", perspective_shifted)
#     cv.imshow("yellow_mask", yellow_mask)
#     cv.imshow("blue_mask", blue_mask)
    # cv.imshow ("yellow_edge", edge_yellow)
    # cv.imshow ("blue_edge", edge_blue)
    # cv.imshow("green_mask", green_mask)
    key = cv.waitKey(1) & 0xFF
    
    rawCapture.truncate(0)
    if key == ord("q"):
        break

ser.write('kill\n'.encode('utf-8'))
ser.flushInput()
camera.close()
# cv.destroyAllWindows()

