from robotty.wheel_theta_encoder_push import WheelEncoder
import time
import numpy as np
from threading import Lock
import kalman
from robot_model import (get_prediction_model,
                         L,
                         r_L,
                         r_R,
                         R,
                         R_still,
                         get_left_wheel_model,
                         get_right_wheel_model,
                         get_process_noise_model)


h = 0.01  # based on at most 0.5 m/s with 20 ticks per rotation on the wheels :) NOTE: 20 was WRONG!!!
x = np.zeros((5,1))
P = np.eye(5)
Q = get_process_noise_model(h)
H_left_still = np.array([[0],[0],[0],[1.0],[0.0]]).T
H_right_still = np.array([[0],[0],[0],[0],[1.0]]).T

def main():
    global x, P
    state_lock = Lock()

    def cb_left(dt, theta):
        global x, P, i, Pi, K, R, r_L
        H_left = get_left_wheel_model(dt)

        state_lock.acquire()
        x[:,0], P, i, Pi, K = kalman.update(x[:,0], theta, P, H_left, R)
        state_lock.release()
        # print("%.2f, %.2f, %.2f".format(i, Pi, K))
        print("-> %.2f" % dt)
        print("---------------")


    def cb_right(dt, theta):
        global x, P, i, Pi, K, R, r_R
        H_right = get_right_wheel_model(dt)

        state_lock.acquire()
        x[:,0], P, i, Pi, K = kalman.update(x[:,0], theta, P, H_right, R)
        state_lock.release()
        # print("%.2f, %.2f, %.2f".format(i, Pi, K))
        print("-> %.2f" % dt)
        print("---------------")


    we_left = WheelEncoder(cb_left, 17)
    we_right = WheelEncoder(cb_right, 18)

    with open("log/log.csv", "w") as log:
        log.write("t,x,y\n")

        i = 0
        while True:
            tic = time.time()
    
            state_lock.acquire()
    
            # Zero velocity updates
            if we_left.is_still():
                x[:,0], P, inno, Pi, K = kalman.update(x[:,0], 0.0, P, H_left_still, R_still)
    
            if we_right.is_still():
               x[:,0], P, inno, Pi, K = kalman.update(x[:,0], 0.0, P, H_right_still, R_still)
    
            x[:,0], A = get_prediction_model(x[:,0],h)
            _, P = kalman.predict(x, P, A, Q)
    
            state_lock.release()
    
            toc = time.time()
            remaining = h - (toc - tic)
            if (remaining > 0):
                time.sleep(remaining)
            else:
                print("Warning! Update took {} s too long!".format(-remaining))
    
            if i == 0:
                log.write("0,{},{}\n".format(x[0,0], x[1,0]))
                log.flush()
    
            i = (i + 1) % 25


if __name__ == "__main__":
    main()
