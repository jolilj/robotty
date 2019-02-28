import numpy as np

def predict(x,P,A,Q):
    xp = A*x
    Pp = A * P * A.T + Q
    return xp, Pp

def update(x,z,P,H,R):
    I = np.matrix(np.eye(x.ndim))
    i = z - H*x
    Pi = H*P*H.T
    K = P*H.T*np.linalg.inv(Pi)
    xp = x+K*i
    Pp = (I-K*H)*P*(I-K*H).T + K*R*K.T
    return xp,Pp,i,Pi