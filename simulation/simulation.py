import sys,os
sys.path.append(os.getcwd()+'/../')

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import random
import kalman
from robot_model import get_prediction_model, L, r_L, r_R, R, get_left_wheel_model, get_right_wheel_model, get_noise_process_model, wheel_speed_noise_std_per_sec

#Robot params
TICKS_PER_WHEEL = 40
RAD_PER_TICK = 2*np.pi/TICKS_PER_WHEEL

class WheelEncoderSim:
    def __init__(self,rad_per_tick,r,R):
        self.rad_per_tick = rad_per_tick
        self.r = r
        self.R = R
        self.v = 0
        self.theta = 0
        self.t = 0
        self.t_prev = 0

    def updateState(self, dT):
        self.t = self.t + dT
        Dtheta = dT*self.v/self.r
        self.theta = self.theta + Dtheta
        if abs(self.theta) > self.rad_per_tick:
            self.theta = 0
            t_diff = self.t-self.t_prev
            self.t_prev = self.t
            return True, t_diff
        return False, 0



# Simulation time and step length
T_end = 200
h = 0.001

# Time vector
T = np.arange(0,T_end,h)

# State initialization
x_true = np.zeros((5,len(T)))
x_true[:,0] = np.array([0,0,0,0.1,0.1])

# Simulated sensors
wheel_L = WheelEncoderSim(RAD_PER_TICK,r_L,wheel_speed_noise_std_per_sec*wheel_speed_noise_std_per_sec)
wheel_R = WheelEncoderSim(RAD_PER_TICK,r_R,wheel_speed_noise_std_per_sec*wheel_speed_noise_std_per_sec)

x_est = np.zeros((5,len(T)))
x_est[:,0] = np.array([0,0,0,0,0])
P_est = np.zeros((5,5,len(T)))
P_est[:,:,0] = np.eye(5)

K_print = np.zeros((5,1,len(T)))
Q = get_noise_process_model(h)

# Simulation
for i in range(1,len(T)):
    # Move based on current state (assume perfect model)
    x_true[:,i], _ = get_prediction_model(x_true[:,i-1],h)


    # Predict kalman filter
    f, A = get_prediction_model(x_est[:,i-1],h)
    _, Pp = kalman.predict(x_est[:,i-1],P_est[:,:,i-1],A,Q)

    x_est[:,i] = f
    P_est[:,:,i] = Pp

    # Update wheel rotation speed
    x_true[3,i] = x_true[3,i-1] + (random.random()-0.5)*wheel_speed_noise_std_per_sec*h
    x_true[4,i] = x_true[4,i-1] + (random.random()-0.5)*wheel_speed_noise_std_per_sec*h

    # Update wheel rotation
    wheel_L_upd, T_L = wheel_L.updateState(h)
    wheel_R_upd, T_R = wheel_R.updateState(h)

    # Update wheel speed
    wheel_L.v = x_true[3,i]
    wheel_R.v = x_true[4,i]

    # Estimate the states
    # Predict
    # If update available
    #   update
    if wheel_L_upd:
        z_L = RAD_PER_TICK
        H_L = get_left_wheel_model(T_L)
        #print("UPDATE")
        #print(P_est[:,:,i])
        xp,Pp,inno,Pi,K = kalman.update(x_est[:,i],z_L,P_est[:,:,i],H_L,R)
        K_print[:,:,i] = K
        x_est[:,i] = xp
        P_est[:,:,i] = Pp
        #print(P_est[:,:,i])
        #print("END UPDATE")

    #if False:
    if wheel_R_upd:
        z_R = RAD_PER_TICK
        H_R = get_right_wheel_model(T_R)
        xp,Pp,inno,Pi,K = kalman.update(x_est[:,i],z_R,P_est[:,:,i],H_R,R)
        #print(Pi)
        x_est[:,i] = xp
        P_est[:,:,i] = Pp

    #print("==============================")
    # Regulate based on measured state

print(x_est[0,:])
print(x_est[1,:])

#figure
fig, ax = plt.subplots()
ax.plot(x_true[0,:],x_true[1,:])
ax.plot(x_est[0,:],x_est[1,:])
plt.show()

#fig, ax = plt.subplots()
#ax.plot(T, x_true[2,:])
#ax.plot(T, x_est[2,:])
#plt.show()



# fig, ax = plt.subplots()
# ax.plot(T, x_true[3,:])
# ax.plot(T, x_est[3,:])
# ax.plot(T, x_true[4,:])
# ax.plot(T, x_est[4,:])
# plt.show()

# Data for plotting
#t = np.arange(0.0, 2.0, 0.01)
#s = 1 + np.sin(2 * np.pi * t)

#fig, ax = plt.subplots()
#ax.plot(t, s)

#ax.set(xlabel='time (s)', ylabel='voltage (mV)',
#       title='About as simple as it gets, folks')
#ax.grid()

#fig.savefig("test.png")
#plt.show()
