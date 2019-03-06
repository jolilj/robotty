from robotty.wheel_encoder_push import WheelEncoder
import time
from threading import Lock


def main():
    h = 0.01  # based on at most 0.5 m/s with 20 ticks per rotation on the wheels :)
    alpha = 0.9
    state_lock = Lock()

    x = np.matrix('0;0')
    P = 1 * np.eye(x.ndim)
    A = np.matrix([[1, h], [0, alpha]])
    Q = np.matrix([[0.0001, 0.0001], [0.0001, 1]])

    H = np.matrix([0 1])

    def cb(omega):
        state_lock.aquire()
        R = 0.1 * omega
        x, P, i, Pi, K = kalman.update(x,omega,P,H,R)
        state_lock.release()
        # print("%.2f, %.2f, %.2f".format(i, Pi, K))


    we = WheelEncoder(cb)

    while True:
        tic = time.time()

        state_lock.aquire()
        x, P = predict(x, P, A, Q)
        state_lock.release()
        theta = x[0, 0]
        print(theta)

        toc = time.time()
        remaining = h - (toc - tic)
        if (remaining > 0):
            time.sleep(remaining)
        else:
            print("Warning! Update took %.2f s too long!".format(-remaining))


if __name__ == "__main__":
    main()
