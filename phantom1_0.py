import numpy as np
from numpy import cos, sin

class Phantom1_0:
    def __init__(self):
        
        self.l1 = 133.35
        self.l2 = 133.35

        self.org = np.array([0, 0, 0, 1],
            dtype='float32').transpose()

    #forward kinematics
    def forward(self, t1, t2, t3):
        t4 = t2 - t3 + (np.pi/2)
        
        T_B_0 = np.array([
            [0, 1, 0, 0], 
            [0, 0, 1, 0], 
            [1, 0, 0, 0], 
            [0, 0, 0, 1]
            ], dtype='float32')

        T_0_1 = np.array([
            [cos(t1), -sin(t1), 0, -self.l1],
            [-sin(t1), -cos(t1), 0, 0],
            [0, 0, -1, self.l2],
            [0, 0, 0, 1]
            ])

        T_1_2 = np.array([
            [cos(t2), -sin(t2), 0, 0],
            [0, 0, 1, 0],
            [-sin(t2), -cos(t2), 0, 0],
            [0, 0, 0, 1]
            ])

        T_2_3 = np.array([
            [cos(t4), sin(t4), 0, self.l1],
            [-sin(t4), cos(t4), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])

        T_3_4 = np.array([
            [0, -1, 0, self.l2],
            [0, 0, 1, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]
            ])

        T_B_4 = T_B_0.dot(T_0_1).dot(T_1_2).dot(T_2_3).dot(T_3_4)
        
        end_effector = T_B_4.dot(self.org)[:3]
        
        #end_effector = np.array(
        #    [-1 * sin(t1) * (self.l1 * cos(t2) + self.l2 * sin(t3)),
        #    self.l2 - self.l2 * cos(t3) + self.l1 * sin(t2),
        #    -self.l1 + cos(t1) * (self.l1 * cos(t2) + self.l2 * sin(t3))
        #    ], dtype='float32')
        return end_effector

    def inverse(self, p1, p2, p3):
        t1 = np.arctan2(-p1, p3 + self.l1)
        
        r = np.sqrt(p1**2 + (p3 + self.l1)**2)
        s = np.sqrt(p1**2 + (p2 - self.l2)**2 + (p3 + self.l1)**2)

        c2 = (self.l1**2 + s**2 - self.l2**2)/(2 * self.l1 * s)
        s2 = -np.sqrt(1 - c2**2)
        t2 = np.arctan2(p2 - self.l2, r) - np.arctan2(s2, c2)
        
        c3 = (self.l1**2 + self.l2**2 - s**2)/(2*self.l1*self.l2)
        s3 = np.sqrt(1 - c3**2)

        t3 = t2 + np.arctan2(s3, c3) - np.pi/2

        return np.array([t1, t2, t3])
