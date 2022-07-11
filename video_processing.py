from cmath import pi
import cv2 as cv
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math
import sys
import serial

BLUE_LOWER = np.array([100, 93, 15])
BLUE_UPPER = np.array([120, 255, 255])
YELLOW_LOWER = np.array([15, 93, 15])
YELLOW_UPPER = np.array([45, 255, 255])

# -------------------------------------------------------------------------------
# IMPORTANT
def sendTurn(angle):
    
    return
def goStraight():
    if ser.inWaiting() > 0:
        print("Go straight")
        ser.write('f'.encode('utf-8'))
        ser.flushInput()
    return

def TurnLeft(angle):
    if ser.inWaiting() > 0:
        print(f"Turn left: {angle}")
        ser.write('l'.encode('utf-8'))
        ser.flushInput()
    return

def TurnRight(angle):
    if ser.inWaiting() > 0:
        print(f"Turn right: {angle}")
        ser.write('r'.encode('utf-8'))
        ser.flushInput()
    
    return

# ---------------------------------------------------------------------------------
# resizes the image to be more manageable
def frameRescale(frame, scale):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dim = (width, height)
    return cv.resize(frame, dim, interpolation = cv.INTER_AREA)

# takes the ground directly in front of the car and manipulates it into birdseye view
def perspectiveShift(frame):
    frame_x = frame.shape[1]
    frame_y = frame.shape[0]
    coords = np.float32([
        [frame_x * 0.25, frame_y * 0.5], [frame_x * 0.75, frame_y * 0.5], 
        [0, frame_y * 0.75], [frame_x, frame_y * 0.75]
        ])
    transformed = np.float32([[0 ,0], [frame_x, 0], [0, frame_y], [frame_x, frame_y]])
    
    matrix =  cv.getPerspectiveTransform(coords, transformed)
    return cv.warpPerspective(frame, matrix, (frame_x, frame_y))
    
# finds y-value of the non-zero pixel closest to top of image
def findMinY(mask):
    points = cv.findNonZero(mask)
    minY = mask.shape[0]
    for point in points:
        if point[0][1] < minY:
            minY = point[0][1]
    return minY

# finds y-value of the non-zero pixel closest to bottom of image
def findMaxY(mask):
    points = cv.findNonZero(mask)
    maxY = 0
    for point in points:
        if point[0][1] > maxY:
            maxY = point[0][1]
    return maxY

# finds average of x-values at a certain y value
def findAverageX(mask, y):
    points = cv.findNonZero(mask)
    x_values = []
    for point in points:
        if point[0][1] == y:
            x_values.append(point[0][0])
    return np.mean(x_values)

# finds the gradient of the line drawn between the bottom most non-zero pixel 
# and the centre of all non-zero pixels
def gradientOfMask(mask):
    max = (findAverageX(mask, findMaxY(mask)), findMaxY(mask))
    min = (findAverageX(mask, findMinY(mask)), findMinY(mask))
    # divison by zero check?
    if min[0] - max[0] == 0:
        return math.inf
    return (min[1] - max[1])/(min[0] - max[0])

# returns the gradient of the direction the car will steer, None if no yellow or blue can be seen
def direction(mask_1, mask_2):
    if cv.findNonZero(mask_1) is None and cv.findNonZero(mask_2) is None:
        return None
    elif cv.findNonZero(mask_2) is None:
        return gradientOfMask(mask_1)
    elif cv.findNonZero(mask_1) is None:
        return gradientOfMask(mask_2)
    else:
        return (gradientOfMask(mask_1) + gradientOfMask(mask_2))/2

# no video feed yet lol but img_processing for each frame should work
camera = PiCamera()
camera.resolution = (820, 616)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(820, 616))

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()

time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    time.sleep(0.1)
    img = frame.array
    img_resized = frameRescale(img, 1)
    perspective_shifted = perspectiveShift(img_resized)
    print(img_resized.shape)
    hsv_img = cv.cvtColor(perspective_shifted, cv.COLOR_BGR2HSV)

    blue_mask = cv.inRange(hsv_img, BLUE_LOWER, BLUE_UPPER)
    yellow_mask = cv.inRange(hsv_img, YELLOW_LOWER, YELLOW_UPPER)

    edge_yellow = cv.Canny(yellow_mask, 40, 150)
    edge_blue = cv.Canny(blue_mask, 40, 150)

    working_gradient = direction(edge_blue, edge_yellow)

    print(f'working gradient = {working_gradient}')

#     cv.imshow("hsv", hsv_img)
    cv.imshow("original", img_resized)
    # cv.imshow("perspective shift", perspective_shifted)
    cv.imshow ("yellow_mask", yellow_mask)
    cv.imshow ("blue_mask", blue_mask)
#     cv.imshow ("yellow_edge", edge_yellow)
#     cv.imshow ("blue_edge", edge_blue)
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