import numpy as np
from phantom1_0 import *

def read_data():
    poss = []
    angles = []

    with open("data.txt") as f:
        for l in f:
            parts = l.split()
            if l.startswith("pos"):
                poss.append(np.array([parts[2], parts[5], parts[8]],
                    dtype='float32'))
            elif l.startswith("angle"):
                angles.append(np.array([parts[2], parts[5], parts[8]], dtype='float32'))
    
    print "poss length: ", len(poss)
    print "angles length: ", len(angles)

    return poss, angles

def check_forward(arm, poss, angles):
    eps = 1e-3
    count = 0.0
    for i in range(len(angles)):
        t1, t2, t3 = angles[i]
        ep = arm.forward(t1, t2, t3)
        diff_norm = np.linalg.norm(ep - poss[i])
        
        if diff_norm > eps:
            print "Test Failed: ", i
            print "Angle: ", angles[i]
            print "Poss: ", poss[i]
            print "Forward: ", ep
            print ""
            count += 1

    print "Inverse test has been finished, failure ratio: ", count / len(angles)

def check_inverse(arm, poss, angles):
    eps = 1e-3
    count = 0.0
    for i in range(len(angles)):
        p1, p2, p3 = poss[i]
        ang = arm.inverse(p1, p2, p3)
        diff_norm = np.linalg.norm(ang - angles[i])
        
        if diff_norm > eps:
            print "Test Failed: ", i
            print "Angle: ", angles[i]
            print "Poss: ", poss[i]
            print "Inverse: ", ang
            print ""
            count += 1

    print "Forward test has been finished, failure ratio: ", count / len(angles)

def main():
    poss, angles = read_data()
    robotic_arm = Phantom1_0()

    check_forward(robotic_arm, poss, angles)
    check_inverse(robotic_arm, poss, angles)

if __name__ == "__main__":
    main()
