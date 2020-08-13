"""
Author: Antariksh Narain

Description:
Python file to implement Kalman Filter algorithm
"""

from numpy import dot, sum, identity, array
from numpy.linalg import inv

class KalmanFilter:
    def __init__(self):
        self.A = array([
            [1,dt,-dt,dt**2/2,-dt**2/2],
            [0, 1, 0, dt, -dt],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]])
        self.B = identity(n=5)
        self.C = array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0]])
        

    def predict(self, X, P, Q, U):
        X = dot(A, X) + dot(B, U)
        P = dot(A, dot(P, A.T)) + Q
        return (X, P)
    
    def update(self, X, P, Y, H, R):
        IM = dot(H, X)
        IS = R+ dot(H, dot(P, H.T))
        K = dot(P, dot(H.T, inv(IS)))
        X = X + dot(K, (Y - IM))
        P = P - dot(K, dot(IS, K.T))
        LH = gauss_pdf(Y, IM, IS)
        return (X, P, K, IM, IS, LH)
    
    def gauss_pdf(self, X, M, S):
        pass
