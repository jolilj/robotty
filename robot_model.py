import numpy as np


R = 0.001
r_R = 0.035
r_L = 0.035
L = 0.1  #Axis length
wheel_speed_noise_std_per_sec = 0.1 #m per sec

def get_prediction_model(x,h):

    theta = x[2]
    v_L   = x[3]
    v_R   = x[4]
    f    = x + h*np.array([1./2.*(v_L+v_R)*np.cos(theta),
                            1./2.*(v_L+v_R)*np.sin(theta),
                            1./L*(v_R-v_L),
                            0,
                            0])

    delA = np.zeros((5,5))
    delA[0,2] = -1/2*(x[3]+x[4])*np.sin(x[2])
    delA[0,3] = 1/2*np.cos(x[2])
    delA[0,4] = 1/2*np.cos(x[2])
    delA[1,2] = 1/2*(x[3]+x[4])*np.cos(x[2])
    delA[1,3] = 1/2*np.sin(x[2])
    delA[1,4] = 1/2*np.sin(x[2])
    delA[2,3] = -1/L
    delA[2,4] = 1/L
    A = np.eye(5) + h*delA
    return f, A


def get_left_wheel_model(dt):
    H_left = np.array([[0],[0],[0],[dt/r_L],[0]]).T
    return H_left


def get_right_wheel_model(dt):
    H_right = np.array([[0],[0],[0],[0],[dt/r_R]]).T
    return H_right

def get_process_noise_model(h):
    Q = np.zeros((5,5))
    Q[3,3] = wheel_speed_noise_std_per_sec*wheel_speed_noise_std_per_sec*h
    Q[4,4] = wheel_speed_noise_std_per_sec*wheel_speed_noise_std_per_sec*h
    return Q
