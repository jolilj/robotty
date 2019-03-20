import numpy as np

def predict(x,P,A,Q):
    xp = A.dot(x)
    Pp = A.dot(P.dot(A.T)) + Q
    return xp, Pp

def update(x,z,P,H,R):
    I = np.matrix(np.eye(x.ndim))
    i = z - H.dot(x)
    Pi = H.dot(P.dot(H.T))+R
    K = P.dot(H.T)/Pi # assume scalar
    xp = x+K.dot(i)
    Pp = (I-K.dot(H)).dot(P.dot((I-K.dot(H.T)))) + (K*R).dot(K.T)
    return xp,Pp,i,Pi,K