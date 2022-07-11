from cmath import pi
import cv2 as cv
import numpy as np
import math
import sys

BLUE_LOWER = np.array([105, 93, 0])
BLUE_UPPER = np.array([135, 255, 255])
YELLOW_LOWER = np.array([15, 93, 0], dtype="uint8")
YELLOW_UPPER = np.array([45, 255, 255], dtype="uint8")
PERSPECTIVE_SHIFT_COORDS = [(), (), (), ()]

# IMPORTANT
def goStraight():
    print("Go straight")
    return

def TurnLeft(angle):
    print(f"Turn left: {angle}")
    return

def TurnRight(angle):
    print(f"Turn right: {angle}")
    return

def frameRescale(frame, scale):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dim = (width, height)
    return cv.resize(frame, dim, interpolation = cv.INTER_AREA)

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

def direction(mask_1, mask_2):
    if cv.findNonZero(mask_1) is None and cv.findNonZero(mask_2) is None:
        return None
    elif cv.findNonZero(mask_2) is None:
        return gradientOfMask(mask_1)
    elif cv.findNonZero(mask_1) is None:
        return gradientOfMask(mask_2)
    else:
        return (gradientOfMask(mask_1) + gradientOfMask(mask_2))/2

    
img = cv.imread('Photos/test/test_straight.jpg')
img_resized = frameRescale(img, 0.20)
perspective_shifted = perspectiveShift(img_resized)
hsv_img = cv.cvtColor(perspective_shifted, cv.COLOR_BGR2HSV)

blue_mask = cv.inRange(hsv_img, BLUE_LOWER, BLUE_UPPER)
yellow_mask = cv.inRange(hsv_img, YELLOW_LOWER, YELLOW_UPPER)

edge_yellow = cv.Canny(yellow_mask, 40, 150)
edge_blue = cv.Canny(blue_mask, 40, 150)

# ditching hough approach
# lines_yellow = cv.HoughLinesP(edge_yellow, 1, np.pi / 180, 50, None, 50, 10)
# lines_blue = cv.HoughLinesP(edge_blue, 1, np.pi / 180, 50, None, 50, 10)

# cv.imshow("hsv", hsv_img)
# cv.imshow("original", img_resized)
# cv.imshow("perspective shift", perspective_shifted)
# cv.imshow ("yellow_mask", yellow_mask)
# cv.imshow ("blue_mask", blue_mask)
# cv.imshow ("yellow_edge", edge_yellow)
# cv.imshow ("blue_edge", edge_blue)

# compare gradients of yellow and blue masks and choose whichever is greater, 
# use that gradient to determine steering amount 

print(cv.findNonZero(edge_blue))

working_gradient = direction(edge_blue, edge_yellow)

print(f'working gradient = {working_gradient}')

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




cv.waitKey(40000)
cv.destroyAllWindows()