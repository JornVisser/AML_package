from cv2  import Rodrigues
from math import sqrt

import numpy as np

def rvecs_to_quats(rvecs):
    # Create a list to save quaternions
    qauts = []
    # Covert the rotation vector into quaternions
    for i in range(len(rvecs)):
        R =  Rodrigues(rvecs[i][0])[0]                                          # Convert rotation vector to 3x3 rotation matrix
        qw = sqrt(1+R[0][0]+R[1][1]+R[2][2])/2
        qx = (R[2][1]-R[1][2])/(4*qw)
        qy = (R[0][2]-R[2][0])/(4*qw)
        qz = (R[1][0]-R[0][1])/(4*qw)
        q  = np.array([qx, qy, qz, qw])                                            # Create quaternion vector q(qw, qx, qy, qz)
        qauts.append(q)
    return qauts

def quats_to_rvecs(quats):
    # Create a list to save rotation vectors
    rvecs = []
    # Convert quaternions to rotation vectors
    for i in range(len(quats)):
        # First create the 3x3 rotation matrix:
        # R = [[R11 R12 R13],
        #      [R21 R22 R23],
        #      [R31 R32 R33]]
        R11 = 1 - 2*quats[i][1]**2 - 2*quats[i][2]
        R12 = 2*quats[i][0]*quats[i][1] - 2*quats[i][2]*quats[i][3]
        R13 = 2*quats[i][0]*quats[i][2] + 2*quats[i][1]*quats[i][3]
        R21 = 2*quats[i][0]*quats[i][1] + 2*quats[i][2]*quats[i][3]
        R22 = 1 - 2*quats[i][0]**2 - 2*quats[i][2]
        R23 = 2*quats[i][1]*quats[i][2] - 2*quats[i][0]*quats[i][3]
        R31 = 2*quats[i][0]*quats[i][2] - 2*quats[i][1]*quats[i][3]
        R32 = 2*quats[i][1]*quats[i][2] + 2*quats[i][0]*quats[i][3]
        R33 = 1 - 2*quats[i][0]**2 - 2*quats[i][1]
        R   = np.array([[R11, R12, R13],
                        [R21, R22, R23],
                        [R31, R32, R33]])
        rvec = Rodrigues(R)[0]
        rvecs.append(rvec)
    return rvecs
