import numpy as np

def predict(x,P,A,Q):
    xp = A.d # TODO: the real shit
    Pp = A.dot(P.dot(A.T)) + Q
    return xp, Pp

def update(x,z,P,H,R):
    I = np.matrix(np.eye(*x.shape))
    i = z - H.dot(x)
    Pi = H.dot(P.dot(H.T))+R
    if Pi.ndim == 0:
        K = P.dot(H.T)/Pi
    else:
        K = P.dot(H.T).dot(np.linalg.inv(Pi))
    xp = x+K.dot(i)
    Pp = (I-K.dot(H)).dot(P)
    return xp,Pp,i,Pi,K
