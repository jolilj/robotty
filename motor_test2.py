import sys, termios, tty, os, time
import RPi.GPIO as GPIO

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
LEFT = 22
RIGHT = 23
GPIO.setup(LEFT, GPIO.OUT)
GPIO.setup(RIGHT, GPIO.OUT)
pl = GPIO.PWM(LEFT, 50)  # 50 Hz
pr = GPIO.PWM(RIGHT, 50)  # 50 Hz

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

button_delay = 0.2

while True:
    char = getch()

    if (char == "w"):
        print("W pressed")
        pl.ChangeDutyCycle(100)
        time.sleep(button_delay)

    elif (char == "s"):
        print("S pressed")
        pl.ChangeDutyCycle(0)
        time.sleep(button_delay)

    elif (char == "o"):
        print("O")
        pr.ChangeDutyCycle(100)

    elif (char == "l"):
        print("L")
        pr.ChangeDutyCycle(0)

pl.stop()
pr.stop()
GPIO.cleanup()
