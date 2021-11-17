import numpy as np

class kalman_filter:
    '''
    A,B,H,Q,R,P   matrix
    x0          vector
    delta_time  float
    '''
    def __init__(self,A,B,H,Q,R,P0,x0,delta_time):
        self.A = A
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        self.P = P0
        self.x = x0
        self.delta_time = delta_time

        self.I = np.eye(A.shape[0])
        self.ERROR = ""

        if not self.check_params():
            print("params check failed ! Error: %s" % (self.ERROR))

    def check_params(self):
        self.ERROR = "no error"
        return True
    '''
    u,z   vector
    '''
    def iter(self,u,z):
        self.x = self.A.dot(self.x) + self.B.dot(u)
        self.P = self.A.dot(self.P).dot(self.A.T) + self.Q

        self.K = self.P.dot(self.H.T) / (self.H.dot(self.P).dot(self.H.T) + self.R)
        # self.K = np.array([[0.0]])
        self.x = self.x + self.K.dot(z - self.H.dot(self.x))
        self.P = (self.I - self.K.dot(self.H)).dot(self.P)

        return self.x,self.K