from robotty.wheel_encoder_push import WheelEncoder
import time

def main():
    def cb(omega):
        print(omega)

    we = WheelEncoder(cb)
    time.sleep(1000)

if __name__ == "__main__":
    main()
