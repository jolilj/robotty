import time
import math
import RPi.GPIO as GPIO

TICKS_PER_ROT = 40
IS_STILL_TIME = 0.5

class WheelEncoder(object):
    def __init__(self, theta_cb, pin):
        # TODO: move this to some global init
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN)

        # self.theta = theta0
        self.last_t = time.time()
        self.theta_cb = theta_cb
        GPIO.add_event_detect(pin, GPIO.FALLING, callback=self.tick, bouncetime=10)

    def tick(self, pin):
        now = time.time()
        dt = now - self.last_t
        self.last_t = now
        # self.theta = self.theta + (2 * math.pi / TICKS_PER_ROT)
        self.theta_cb(dt, (2 * math.pi / TICKS_PER_ROT))

    def is_still():
        now = time.time()
        if (now - self.last_t > IS_STILL_TIME):
            return True
        else:
            return False


