from cmath import pi
import cv2 as cv
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math
import sys
import serial
from functions import TurnRight

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    time.sleep(0.1)
    while True:
    # TurnRight(ser, 20)
#         ser.write('582\n'.encode('utf-8'))
#         ser.flushInput()
#         time.sleep(0.5)
        ser.write('right\n'.encode('utf-8'))
        ser.flushInput()
        time.sleep(0.5)
# camera = PiCamera()
# camera.resolution = (832, 624)
# camera.framerate = 32
# rawCapture = PiRGBArray(camera, size=(832, 624))
# time.sleep(0.1)
# 
# for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#     img = frame.array
#     
#     cv.imshow("original", img)
#     key = cv.waitKey(1) & 0xFF
#     
#     rawCapture.truncate(0)
#     if key == ord("q"):
#         break
# camera.close()
# cv.destroyAllWindows()
