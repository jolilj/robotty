import time
import math
import RPi.GPIO as GPIO

TICKS_PER_ROT = 40

class WheelEncoder(object):
    def __init__(self, theta_cb):
        # TODO: move this to some global init
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17, GPIO.IN)

        # self.theta = theta0
        self.last_t = time.time()
        self.theta_cb = theta_cb
        GPIO.add_event_detect(17, GPIO.FALLING, callback=self.tick, bouncetime=100)

    def tick(self, pin):
        now = time.time()
        dt = now - self.last_t
        # self.theta = self.theta + (2 * math.pi / TICKS_PER_ROT)
        self.theta_cb(dt, (2 * math.pi / TICKS_PER_ROT))
