from robotty.wheel_theta_encoder_push import WheelEncoder
import time
import numpy as np
from threading import Lock
import kalman

h = 0.01  # based on at most 0.5 m/s with 20 ticks per rotation on the wheels :)
alpha = 0
x = np.matrix('0;0')
P = 1 * np.eye(x.ndim)
A = np.matrix([[1, h], [0, alpha]])
Q = np.matrix([[0.0001, 0.0001], [0.0001, 1]])
H = np.matrix([1, 0])

def main():
    global x, P
    state_lock = Lock()

    def cb(theta):
        global x, P, i, Pi, K
        state_lock.acquire()
        R = 0.01
        x, P, i, Pi, K = kalman.update(x, omega, P, H, R)
        state_lock.release()
        # print("%.2f, %.2f, %.2f".format(i, Pi, K))

    we = WheelEncoder(x[0, 0], cb)

    i = 0
    while True:
        tic = time.time()

        state_lock.acquire()
        x, P = kalman.predict(x, P, A, Q)
        state_lock.release()
        theta = x[0, 0]

        if i == 0:
            print(theta)

        toc = time.time()
        remaining = h - (toc - tic)
        if (remaining > 0):
            time.sleep(remaining)
        else:
            print("Warning! Update took %.2f s too long!".format(-remaining))
        i = (i + 1) % 50


if __name__ == "__main__":
    main()
