# from cmath import pi
# import cv2 as cv
# import numpy as np
# import math
# import sys
# from matplotlib import pyplot as plt
# from functions import biasedSendTurn, frameRescale, perspectiveShift, findAverageX, findMaxY, findMinY, direction, gradientOfMask, findLargestContour
# from global_variables import BLACK_THRESHOLD, CONTOUR_AREA_THRESHOLD_BLACK, CONTOUR_AREA_THRESHOLD_PURPLE, SIMILARITY_THRESHOLD, CONTOUR_AREA_THRESHOLD_LINE, YELLOW_LOWER, YELLOW_UPPER, BLUE_LOWER, BLUE_UPPER, PURPLE_LOWER, PURPLE_UPPER

# """
# Pseudo-code:

# if purple detected and ultra sonic detected:
#     measure distance between purple and yellow line horozontally, and purple and blue

#     if distance(purple, blue) < distance(purple, yellow):
        
# """

# img = cv.imread("Photos/palette_wheel.png")
# img_resized = frameRescale(img, 0.5)
# hsv_img = cv.cvtColor(img_resized, cv.COLOR_BGR2HSV)
# blue_mask = cv.inRange(hsv_img, BLUE_LOWER, BLUE_UPPER)
# yellow_mask = cv.inRange(hsv_img, YELLOW_LOWER, YELLOW_UPPER)
# purple_mask = cv.inRange(hsv_img, PURPLE_LOWER, PURPLE_UPPER)
# edge_yellow = findLargestContour(yellow_mask, CONTOUR_AREA_THRESHOLD_LINE)[0]
# edge_blue = findLargestContour(blue_mask, CONTOUR_AREA_THRESHOLD_LINE)[0]
# block_edges = findLargestContour(purple_mask, CONTOUR_AREA_THRESHOLD_PURPLE)[0]

# def obstacle_avoid_script(object_distance, hsv_img, edge_blue, edge_yellow):
#     purple_mask = cv.inRange(hsv_img, PURPLE_LOWER, PURPLE_UPPER)
#     block_edges = findLargestContour(purple_mask, CONTOUR_AREA_THRESHOLD_PURPLE)[0]
#     if block_edges is None:
#         return
#     obs_base = (findAverageX(block_edges, findMaxY(block_edges)), findMaxY(block_edges))

#     if edge_blue is None:
#         distance_from_blue = 0
#     else:
#         distance_from_blue = abs(obs_base[0] - findAverageX(edge_blue, obs_base[1]))
#     if edge_yellow is None:
#         distance_from_yellow = 0
#     else:
#         distance_from_yellow = abs(obs_base[0] - findAverageX(edge_yellow, obs_base[1]))

#     distance_diff = distance_from_blue - distance_from_yellow
#     if object_distance > 40:
#         if distance_diff > 0:
#             biasedSendTurn(direction(edge_blue, edge_yellow), "left")
#         if distance_diff <= 0:
#             biasedSendTurn(direction(edge_blue, edge_yellow), "right")
#     return
# if block_edges is None:
#     help = "help"
# obs_base = (findAverageX(block_edges, findMaxY(block_edges)), findMaxY(block_edges))

# if edge_blue is None:
#     distance_from_blue = 0
# else:
#     distance_from_blue = abs(obs_base[0] - findAverageX(edge_blue, obs_base[1]))

# if edge_yellow is None:
#     distance_from_yellow = 0
# else:
#     distance_from_yellow = abs(obs_base[0] - findAverageX(edge_yellow, obs_base[1]))



# cv.imshow("original", img_resized)
# cv.imshow("purple", purple_mask)

# cv.waitKey(10000)
# cv.destroyAllWindows()