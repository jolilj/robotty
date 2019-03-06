import time
import math
import RPi.GPIO as GPIO

TICKS_PER_ROT = 20

class WheelEncoder(object):
    def __init__(self, omega_cb):
        # TODO: move this to some global init
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17, GPIO.IN)

        self.last_t = time.time()
        self.omega_cb = omega_cb
        GPIO.add_event_detect(17, GPIO.FALLING, callback=self.tick, bouncetime=10)

    def tick(self, pin):
        now = time.time()
        dt = now - self.last_t
        omega = (2 * math.pi) / (TICKS_PER_ROT * dt)
	self.last_t = now
        self.omega_cb(omega)
