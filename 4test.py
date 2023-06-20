import time

from jetbot import Robot
robot = Robot()

robot.set_motors(0.2, 0.2)
time.sleep(1)
robot.stop()
