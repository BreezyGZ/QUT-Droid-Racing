from cmath import pi
import cv2 as cv
import numpy as np
import math
import sys
# from picamera.array import PiRGBArray
# from picamera import PiCamera
import time
import serial
# import RPi.GPIO as GPIO

def goStraight(ser):
    print("Go straight")
    ser.write('S\n'.encode('utf-8'))
    ser.flushInput()
    return

def TurnLeft(ser, angle):
    if angle < 10:
        goStraight(ser)
        return
    print(f"Turn left: {angle}")
    ser.write('L\n'.encode('utf-8'))
    ser.flushInput()
    return

def TurnRight(ser, angle):
    if angle < 10:
        goStraight(ser)
        return
    print(f"Turn right: {angle}")
    ser.write('R\n'.encode('utf-8'))
    ser.flushInput()
    return

# def goStraight(ser):
#     print("Go straight")
#     ser.write('512\n'.encode('utf-8'))
#     ser.flushInput()
#     return

# def TurnLeft(ser, angle):
#     voltage = 512 - (392*angle/90)
#     print(f"Turn left: {angle}")
#     ser.write(f'{voltage}\n'.encode('utf-8'))
#     ser.flushInput()
#     return

# def TurnRight(ser, angle):
#     voltage = 512 + (392*angle/90)
#     print(f"Turn right: {angle}")
#     ser.write(f'{voltage}\n'.encode('utf-8'))
#     ser.flushInput()
#     return

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
        [frame_x * (4/9), frame_y * (3/6.5)], [frame_x * (6/9), frame_y * (3.2/6.5)], 
        [0, frame_y], [frame_x, frame_y]
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
def direction(blue_mask, yellow_mask):
    if cv.findNonZero(blue_mask) is None and cv.findNonZero(yellow_mask) is None:
        return None
    elif cv.findNonZero(yellow_mask) is None:
        return -1
    elif cv.findNonZero(blue_mask) is None:
        return 1
    else:
        return (gradientOfMask(blue_mask) + gradientOfMask(yellow_mask))/2

# def direction_obstacle(blue_mask, yellow_mask):
#     if cv.findNonZero(blue_mask) is None or cv.findNonZero(yellow_mask) is None:
#         return None
#     else:
#         return (gradientOfMask(blue_mask) + gradientOfMask(yellow_mask))/2

# def findLargestContour(mask, area_threshold):
#     blank_image = np.zeros((mask.shape[0], mask.shape[1], 3), dtype = "uint8")
#     cnts_img = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]
#     largest_contour = None
#     largest_area = 0

#     for contour in cnts_img:
#         contour_area = cv.contourArea(contour)
#         if (contour_area > largest_area) and contour_area > area_threshold:
#             largest_contour = contour
#     if largest_contour is not None:
#         cv.drawContours(blank_image, [largest_contour], -1, (255,255,255), 2)
#     return (blank_image, largest_area)

def sendTurn(ser, working_gradient):
    # while ser.inWaiting() <= 0:
    #     time.sleep(0.05)
    if working_gradient is not None:
        gradient_angle = math.atan(working_gradient)
        if gradient_angle < 0:
            turn_angle = math.degrees(math.pi/2 + gradient_angle)
            if turn_angle > 10:
                TurnRight(ser, turn_angle)
                return
        else: 
            turn_angle = math.degrees(math.pi/2 - gradient_angle)
            if turn_angle > 10:
                TurnLeft(ser, turn_angle)
                return
    goStraight(ser)

def biasedSendTurn(ser, working_gradient, turn_direction):
    # while ser.inWaiting() <= 0:
    #     time.sleep(0.05)
    if working_gradient is not None:
        gradient_angle = math.atan(working_gradient)
        if gradient_angle < 0:
            turn_angle = math.degrees(math.pi/2 + gradient_angle)
            if turn_angle > 45:
                TurnRight(ser, turn_angle)
                return
            
        else:
            turn_angle = math.degrees(math.pi/2 - gradient_angle)
            if turn_angle > 45:
                TurnLeft(ser, turn_angle)
                return
    if turn_direction == "left": 
        TurnLeft(ser, 45)
        return
    if turn_direction == "right":
        TurnRight(ser, 45)
        return

# finds the distance b/w the ultrasonic sensor and in front
# def distance(gpio_trigger, gpio_echo):
#     # set Trigger to HIGH
#     GPIO.output(gpio_trigger, True)
 
#     # set Trigger after 0.01ms to LOW
#     time.sleep(0.00001)
#     GPIO.output(gpio_trigger, False)
 
#     StartTime = time.time()
#     StopTime = time.time()
 
#     # save StartTime
#     while GPIO.input(gpio_echo) == 0:
#         StartTime = time.time()
 
#     # save time of arrival
#     while GPIO.input(gpio_echo) == 1:
#         StopTime = time.time()
 
#     # time difference between start and arrival
#     TimeElapsed = StopTime - StartTime
#     # multiply with the sonic speed (34300 cm/s)
#     # and divide by 2, because there and back
#     distance = (TimeElapsed * 34300) / 2
 
#     return distance

def crop(frame):
    frame_x = frame.shape[1]
    frame_y = frame.shape[0]
    coords = np.float32([
        [frame_x * (0.2), frame_y * (0.5)], [frame_x * (0.8), frame_y * (0.5)], 
        [frame_x * (0.2), frame_y], [frame_x, frame_y]
        ])
    transformed = np.float32([[0 ,0], [frame_x, 0], [0, frame_y], [frame_x, frame_y]])
    
    matrix =  cv.getPerspectiveTransform(coords, transformed)
    return cv.warpPerspective(frame, matrix, (frame_x, frame_y))