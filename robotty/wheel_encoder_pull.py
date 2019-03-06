import math
import RPi.GPIO as GPIO
from threading import Lock

from robotty.rts import RTS


class WheelEncoder(RTS):
    def __init__(self, freq):
        super(WheelEncoder, self).__init__(freq)

        # TODO: move this to some global init
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17, GPIO.IN)

        self.omega = 0
        self.ticks = 0
        self.ticks_lock = Lock()
        self.ticks_per_rot = 20  # TODO: is this correct?????????

        GPIO.add_event_detect(17, GPIO.FALLING, callback=self.increment_ticks, bouncetime=300)  # TODO: tweak the bouncetime!!

    def increment_ticks(self, pin):
        self.ticks_lock.acquire()
        self.ticks = self.ticks + 1
        self.ticks_lock.release()

    def reset_ticks(self):
        self.ticks_lock.acquire()
        self.ticks = 0
        self.ticks_lock.release()

    def update(self):
        self.omega = (2 * math.pi / self.ticks_per_rot) * self.ticks * self.freq
        # TODO: remove this!
        print(self.omega)
	    self.reset_ticks()

    def fetch(self):
        return self.omega
