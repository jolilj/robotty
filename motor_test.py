import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
LEFT = 22
RIGHT = 23

GPIO.setup(LEFT, GPIO.OUT)
GPIO.setup(RIGHT, GPIO.OUT)
pl = GPIO.PWM(LEFT, 50)  # 50 Hz
pr = GPIO.PWM(RIGHT, 50)  # 50 Hz
pl.start(0)
pr.start(0)
try:
    while True:
        for dc in range(0, 101, 5):
            pl.ChangeDutyCycle(dc)
            pr.ChangeDutyCycle(100-dc)
            time.sleep(0.1)
        for dc in range(100, -1, -5):
            pl.ChangeDutyCycle(dc)
            pr.ChangeDutyCycle(100-dc)
            time.sleep(0.1)
except KeyboardInterrupt:
    pass
pl.stop()
pr.stop()
GPIO.cleanup()
