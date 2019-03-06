import time
import math
import RPi.GPIO as GPIO

class WheelEncoder(object):
    def __init__(self, omega_cb):
        # TODO: move this to some global init
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17, GPIO.IN)

        self.last_t = time()
        self.omega_cb
        GPIO.add_event_detect(17, GPIO.FALLING, callback=self.tick, bouncetime=300)  # TODO: tweak the bouncetime!!

    def tick(self, pin):
        now = time.time()
        dt = now - self.last_t
        omega = (2 * math.pi) / (self.ticks_per_rot * dt)
        self.omega_cb(omega)
