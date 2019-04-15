from robotty.wheel_theta_encoder_push import WheelEncoder
import time
import numpy as np
from threading import Lock
import kalman
from robot_model import get_prediction_model, L, r_L, r_R, R, get_left_wheel_model, get_right_wheel_model


h = 0.01  # based on at most 0.5 m/s with 20 ticks per rotation on the wheels :) NOTE: 20 was WRONG!!!
x = np.zeros((5,1))
P = np.eye(5)

Q = np.array([[0.0001, 0.0001], [0.0001, 1]])
H = np.array([1, 0])

def main():
    global x, P
    state_lock = Lock()

    def cb_left(dt, theta):
        global x, P, i, Pi, K, R, r_L
        H_left = get_left_wheel_model(dt)

        state_lock.acquire()
        x, P, i, Pi, K = kalman.update(x, theta, P, H_left, R)
        state_lock.release()
        # print("%.2f, %.2f, %.2f".format(i, Pi, K))
        print("-> %.2f" % theta)


    def cb_right(dt, theta):
        global x, P, i, Pi, K, R, r_R
        H_right = get_right_wheel_model(dt)

        state_lock.acquire()
        x, P, i, Pi, K = kalman.update(x, theta, P, H_right, R)
        state_lock.release()
        # print("%.2f, %.2f, %.2f".format(i, Pi, K))
        print("-> %.2f" % theta)


    we_left = WheelEncoder(cb_left, 17)
    we_right = WheelEncoder(cb_right, 18)

    i = 0
    while True:
        tic = time.time()

        state_lock.acquire()
        #if wheel_left.is_still()
        #    kalman.update()
        x,A = get_prediction_model(x,h)
        _, P = kalman.predict(x, P, A, Q)
        state_lock.release()

        toc = time.time()
        remaining = h - (toc - tic)
        if (remaining > 0):
            time.sleep(remaining)
        else:
            print("Warning! Update took %.2f s too long!".format(-remaining))

        if i == 0:
            print(x)

        i = (i + 1) % 50


if __name__ == "__main__":
    main()
