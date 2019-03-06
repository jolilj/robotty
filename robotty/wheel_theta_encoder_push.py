import time
import math
import RPi.GPIO as GPIO

TICKS_PER_ROT = 40

class WheelEncoder(object):
    def __init__(self, theta0, theta_cb):
        # TODO: move this to some global init
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17, GPIO.IN)

        self.theta = theta0
        self.theta_cb = theta_cb
        GPIO.add_event_detect(17, GPIO.FALLING, callback=self.tick, bouncetime=100)

    def tick(self, pin):
        self.theta = self.theta + (2 * math.pi / TICKS_PER_ROT)
        self.theta_cb(self.theta)
