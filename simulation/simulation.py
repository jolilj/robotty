import sys,os
sys.path.append(os.getcwd()+'/../')

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import random
import kalman

#Robot params
r = 0.05 #Wheel radius
L = 0.1  #Axis length
wheel_rate_noise_std_per_sec = 0.1 #rad per sec
TICKS_PER_WHEEL = 40
RAD_PER_TICK = 2*np.pi/TICKS_PER_WHEEL

class WheelEncoderSim:
    def __init__(self,rad_per_tick,r,R):
        self.rad_per_tick = rad_per_tick
        self.r = r
        self.R = R
        self.v = 0
        self.theta = 0
        self.t = 0;
        self.t_prev = 0
    
    def updateState(self, dT):
        self.t = self.t + dT
        theta_new = self.theta + dT*self.v/self.r
        Dtheta = theta_new-self.theta
        if abs(Dtheta) > self.rad_per_tick:
            t_diff = self.t-self.t_prev
            self.t_prev = self.t
            return True,Dtheta,t_diff
        return False,0,0
        
    
def move_diff_rob(x,h,r,L):
    theta = x[2]
    v_L   = x[3]
    v_R   = x[4]
    xp    = x + h*np.array([1./2.*(v_L+v_R)*np.cos(theta),
                            1./2.*(v_L+v_R)*np.sin(theta),
                            1./L*(v_R-v_L),
                            0,
                            0])
    return xp

def get_prediction_model(x,h,L):
    delA = np.zeros((5,5))
    delA[0,2] = -1/2*(x[3]+x[4])*np.sin(x[2])
    delA[0,3] = 1/2*np.cos(x[2])
    delA[0,4] = 1/2*np.cos(x[2])
    delA[1,2] = 1/2*(x[3]+x[4])*np.cos(x[2])
    delA[1,3] = 1/2*np.sin(x[2])
    delA[1,4] = 1/2*np.sin(x[2])
    delA[2,3] = 1/L
    delA[2,4] = -1/L
    A = np.eye(5) + h*delA
    return A


# Simulation time and step length
T_end = 1
h = 0.1

# Time vector
T = np.arange(0,T_end,h)

# State initialization
x_true = np.zeros((5,len(T)))
x_true[:,0] = np.array([0,0,0,np.pi/4,np.pi/4])

# Simulated sensors
wheel_L = WheelEncoderSim(RAD_PER_TICK,0.35,0.1)
wheel_R = WheelEncoderSim(RAD_PER_TICK,0.35,0.1)

x_est = np.zeros((5,len(T)))
x_est[:,0] = np.array([0,0,0,0,0])
P_est = np.zeros((5,5,len(T)))
P_est[:,:,0] = np.eye(5)

# Simulation
for i in range(1,len(T)):   
    # Move based on current state
    x_true[:,i] = move_diff_rob(x_true[:,i-1],h,r,L)
    
    # Predict kalman filter
    A = get_prediction_model(x_est[:,i],h,L)
    
    Q = np.zeros((5,5))
    Q[3,3] = (wheel_rate_noise_std_per_sec*h)*(wheel_rate_noise_std_per_sec*h)
    Q[4,4] = (wheel_rate_noise_std_per_sec*h)*(wheel_rate_noise_std_per_sec*h)
    
    xp, Pp = kalman.predict(x_est[:,i-1],P_est[:,:,i-1],A,Q)
    x_est[:,i] = xp
    P_est[:,:,i] = Pp
    
    # Update wheel rotation speed
    x_true[3,i] = x_true[3,i-1] + (random.random()-0.5)*wheel_rate_noise_std_per_sec*h
    x_true[4,i] = x_true[4,i-1] + (random.random()-0.5)*wheel_rate_noise_std_per_sec*h
    
    # Update wheel rotation
    wheel_L_upd, Dtheta_L, T_L = wheel_L.updateState(h)
    wheel_R_upd, Dtheta_R, T_R = wheel_R.updateState(h)
    
    # Update wheel speed
    wheel_L.v = x_true[3,i]
    wheel_R.v = x_true[4,i]

    
    # Estimate the states
    # Predict
    # If update available
    #   update
    if wheel_L_upd:
        T_L = T_L
        z_L = Dtheta_L
        r_L = wheel_L.r
        R_L = np.array([wheel_L.R])
        H_L = np.array([[0],[0],[0],[T_L/r_L],[0]]).T
        xp,Pp,inno,Pi,K = kalman.update(x_est[:,i],z_L,P_est[:,:,i],H_L,R_L)
        x_est[:,i] = xp
        P_est[:,:,i] = Pp

    if wheel_R_upd:
        T_R = T_R
        z_R = Dtheta_R
        r_R = wheel_L.r
        R_R = wheel_L.R
        H_R = np.array([[0],[0],[0],[0],[T_R/r_R]]).T
        xp,Pp,inno,Pi,K = kalman.update(x_est[:,i],z_R,P_est[:,:,i],H_R,R_R)
        x_est[:,i] = xp
        P_est[:,:,i] = Pp
    # Regulate based on measured state

#figure
fig, ax = plt.subplots()
ax.plot(x_true[0,:],x_true[1,:])
ax.plot(x_est[0,:],x_est[1,:])
plt.show()


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
