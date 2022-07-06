# python program to control the motor functions of the RC car
from gpiozero import Motor
import RPi.GPIO as GPIO

motor1 = Motor(4, 14)
motor2 = Motor(17, 27)

while True:
    motor1.forward()


# servo control