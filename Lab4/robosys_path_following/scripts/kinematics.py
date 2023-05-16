#!/usr/bin/env python3

"""
Compute state space kinematic matrices for xArm7 robot arm (5 links, 7 joints)
"""

import numpy as np
import math
from math import cos, sin

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

class xArm7_kinematics():
    def __init__(self):

        self.l1 = 0.267
        self.l2 = 0.293
        self.l3 = 0.0525
        self.l4 = 0.3512
        self.l5 = 0.1232

        self.theta1 = 0.2225 #(rad) (=12.75deg)
        self.theta2 = 0.6646 #(rad) (=38.08deg)

        pass

    def compute_angles(self, ee_position):

        joint_1 = 0
        joint_2 = 0
        joint_3 = 0
        joint_4 = 0
        joint_5 = 0
        joint_6 = 0.75
        joint_7 = 0

        joint_angles = np.matrix([ joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7 ])

        return joint_angles

    def compute_jacobian(self, r_joints_array):

        J_11 = - self.l3*(cos(r_joints_array[0])*sin(r_joints_array[2]) + cos(r_joints_array[1])*cos(r_joints_array[2])*sin(r_joints_array[0]))\
               - self.l2*sin(r_joints_array[0])*sin(r_joints_array[1]) \
               - self.l4*cos(self.theta1)*(sin(r_joints_array[3])*(cos(r_joints_array[0])*sin(r_joints_array[2]) + cos(r_joints_array[1])*cos(r_joints_array[2])*sin(r_joints_array[0])) - cos(r_joints_array[3])*sin(r_joints_array[0])*sin(r_joints_array[1]))\
               - self.l4*sin(self.theta1)*(cos(r_joints_array[3])*(cos(r_joints_array[0])*sin(r_joints_array[2]) + cos(r_joints_array[1])*cos(r_joints_array[2])*sin(r_joints_array[0])) + sin(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[3]))\
               - self.l5*cos(self.theta2)*(cos(r_joints_array[5])*(sin(r_joints_array[3])*(cos(r_joints_array[0])*sin(r_joints_array[2]) + cos(r_joints_array[1])*cos(r_joints_array[2])*sin(r_joints_array[0])) - cos(r_joints_array[3])*sin(r_joints_array[0])*sin(r_joints_array[1])) - sin(r_joints_array[5])*(cos(r_joints_array[5])*(cos(r_joints_array[3])*(cos(r_joints_array[0])*sin(r_joints_array[2]) + cos(r_joints_array[1])*cos(r_joints_array[2])*sin(r_joints_array[0])) + sin(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[3])) - sin(r_joints_array[4])*(cos(r_joints_array[0])*cos(r_joints_array[2]) - cos(r_joints_array[1])*sin(r_joints_array[0])*sin(r_joints_array[2]))))\
               - self.l5*sin(self.theta2)*(sin(r_joints_array[5])*(sin(r_joints_array[3])*(cos(r_joints_array[0])*sin(r_joints_array[2]) + cos(r_joints_array[1])*cos(r_joints_array[2])*sin(r_joints_array[0])) - cos(r_joints_array[3])*sin(r_joints_array[0])*sin(r_joints_array[1])) + cos(r_joints_array[5])*(cos(r_joints_array[4])*(cos(r_joints_array[3])*(cos(r_joints_array[0])*sin(r_joints_array[2]) + cos(r_joints_array[1])*cos(r_joints_array[2])*sin(r_joints_array[0])) + sin(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[3])) - sin(r_joints_array[4])*(cos(r_joints_array[0])*cos(r_joints_array[2]) - cos(r_joints_array[1])*sin(r_joints_array[0])*sin(r_joints_array[2]))))
        
        J_12 = self.l2*cos(r_joints_array[0])*cos(r_joints_array[1])\
             - self.l5*sin(self.theta2)*(sin(r_joints_array[5])*(cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[3]) + cos(r_joints_array[0])*cos(r_joints_array[2])*sin(r_joints_array[1])*sin(r_joints_array[3])) - cos(r_joints_array[5])*(cos(r_joints_array[4])*(cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[3]) - cos(r_joints_array[0])*cos(r_joints_array[2])*cos(r_joints_array[3])*sin(r_joints_array[1])) - cos(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[2])*sin(r_joints_array[4])))\
             - self.l5*cos(self.theta2)*(cos(r_joints_array[5])*(cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[3]) + cos(r_joints_array[0])*cos(r_joints_array[2])*sin(r_joints_array[1])*sin(r_joints_array[3])) + sin(r_joints_array[5])*(cos(r_joints_array[4])*(cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[3]) - cos(r_joints_array[0])*cos(r_joints_array[2])*cos(r_joints_array[3])*sin(r_joints_array[1])) - cos(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[2])*sin(r_joints_array[4])))\
             - self.l4*cos(self.theta1)*(cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[3]) + cos(r_joints_array[0])*cos(r_joints_array[2])*sin(r_joints_array[1])*sin(r_joints_array[3]))\
             + self.l4*sin(self.theta1)*(cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[3]) - cos(r_joints_array[0])*cos(r_joints_array[3])*cos(r_joints_array[3])*sin(r_joints_array[1])) - self.l3*cos(r_joints_array[0])*cos(r_joints_array[2])*sin(r_joints_array[1])
        
        J_13 = self.l5*cos(self.theta2)*(sin(r_joints_array[5])*(sin(r_joints_array[4])*(sin(r_joints_array[0])*sin(r_joints_array[2]) - cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2])) + cos(r_joints_array[3])*cos(r_joints_array[4])*(cos(r_joints_array[2])*sin(r_joints_array[0]) + cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[2]))) - cos(r_joints_array[5])*sin(r_joints_array[3])*(cos(r_joints_array[2])*sin(r_joints_array[0]) + cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[2])))\
             - self.l3*(cos(r_joints_array[2])*sin(r_joints_array[0]) + cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[2]))\
             - self.l5*sin(self.theta2)*(cos(r_joints_array[5])*(sin(r_joints_array[4])*(sin(r_joints_array[0])*sin(r_joints_array[2]) - cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2])) + cos(r_joints_array[3])*cos(r_joints_array[4])*(cos(r_joints_array[2])*sin(r_joints_array[0]) + cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[2]))) + sin(r_joints_array[3])*sin(r_joints_array[5])*(cos(r_joints_array[2])*sin(r_joints_array[0]) + cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[2])))\
             - self.l4*cos(r_joints_array[3])*sin(self.theta1)*(cos(r_joints_array[2])*sin(r_joints_array[0]) + cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[2]))\
             - self.l4*cos(self.theta1)*sin(r_joints_array[3])*(cos(r_joints_array[2])*sin(r_joints_array[0]) + cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[2]))
        
        J_14 = self.l4*sin(self.theta1)*(sin(r_joints_array[3])*(sin(r_joints_array[0])*sin(r_joints_array[2]) - cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2])) + cos(r_joints_array[0])*cos(r_joints_array[3])*sin(r_joints_array[1]))\
             - self.l5*sin(self.theta2)*(sin(r_joints_array[5])*(cos(r_joints_array[3])*(sin(r_joints_array[0])*sin(r_joints_array[2]) - cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2])) - cos(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[3])) - cos(r_joints_array[4])*cos(r_joints_array[5])*(sin(r_joints_array[3])*(sin(r_joints_array[0])*sin(r_joints_array[2]) - cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2])) + cos(r_joints_array[0])*cos(r_joints_array[3])*sin(r_joints_array[1])))\
             - self.l4*cos(self.theta1)*(cos(r_joints_array[3])*(sin(r_joints_array[0])*sin(r_joints_array[2]) - cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2])) - cos(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[3]))\
             - self.l5*cos(self.theta2)*(cos(r_joints_array[5])*(cos(r_joints_array[3])*(sin(r_joints_array[0])*sin(r_joints_array[2]) - cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2])) - cos(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[3])) + cos(r_joints_array[4])*sin(r_joints_array[5])*(sin(r_joints_array[3])*(sin(r_joints_array[0])*sin(r_joints_array[2]) - cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2])) + cos(r_joints_array[0])*cos(r_joints_array[3])*sin(r_joints_array[1])))
        
        J_15 = -self.l5*sin(r_joints_array[5] - self.theta2)*(cos(r_joints_array[2])*cos(r_joints_array[4])*sin(r_joints_array[0]) + cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[4])*sin(r_joints_array[2]) - cos(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[3])*sin(r_joints_array[4]) + cos(r_joints_array[3])*sin(r_joints_array[0])*sin(r_joints_array[2])*sin(r_joints_array[4]) - cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2])*cos(r_joints_array[3])*sin(r_joints_array[4]))
        
        J_16 = self.l5*cos(self.theta2)*(sin(r_joints_array[5])*(sin(r_joints_array[3])*(sin(r_joints_array[0])*sin(r_joints_array[2]) - cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2])) + cos(r_joints_array[0])*cos(r_joints_array[3])*sin(r_joints_array[1])) + cos(r_joints_array[5])*(cos(r_joints_array[5])*(cos(r_joints_array[3])*(sin(r_joints_array[0])*sin(r_joints_array[2]) - cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2])) - cos(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[3])) - sin(r_joints_array[4])*(cos(r_joints_array[2])*sin(r_joints_array[0]) + cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[2]))))\
             - self.l5*sin(self.theta2)*(cos(r_joints_array[5])*(sin(r_joints_array[3])*(sin(r_joints_array[0])*sin(r_joints_array[2]) - cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2])) + cos(r_joints_array[0])*cos(r_joints_array[3])*sin(r_joints_array[1])) - sin(r_joints_array[5])*(cos(r_joints_array[4])*(cos(r_joints_array[3])*(sin(r_joints_array[0])*sin(r_joints_array[2]) - cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2])) - cos(r_joints_array[0])*sin(r_joints_array[2])*sin(r_joints_array[3])) - sin(r_joints_array[4])*(cos(r_joints_array[2])*sin(r_joints_array[0]) + cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[2]))))
        
        J_17 = 0

        J_21 = self.l2*cos(q1)*sin(q2) - self.l3*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - self.l4*cos(self.theta1)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2))\
              - self.l5*cos(self.theta2)*(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              - self.l4*sin(self.theta1)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4))\
              - self.l5*sin(self.theta2)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) 
        
        J_22 = self.l2*cos(q2)*sin(q1) - self.l5*sin(self.theta2)*(sin(q6)*(cos(q2)*cos(q4)*sin(q1) + cos(q3)*sin(q1)*sin(q2)*sin(q4)) - cos(q6)*(cos(q5)*(cos(q2)*sin(q1)*sin(q4) - cos(q3)*cos(q4)*sin(q1)*sin(q2)) - sin(q1)*sin(q2)*sin(q3)*sin(q5)))\
             - self.l5*cos(self.theta2)*(cos(q6)*(cos(q2)*cos(q4)*sin(q1) + cos(q3)*sin(q1)*sin(q2)*sin(q4)) + sin(q6)*(cos(q5)*(cos(q2)*sin(q1)*sin(q4) - cos(q3)*cos(q4)*sin(q1)*sin(q2)) - sin(q1)*sin(q2)*sin(q3)*sin(q5)))\
             - self.l4*cos(self.theta1)*(cos(q2)*cos(q4)*sin(q1) + cos(q3)*sin(q1)*sin(q2)*sin(q4))\
             + self.l4*sin(self.theta1)*(cos(q2)*sin(q1)*sin(q4) - cos(q3)*cos(q4)*sin(q1)*sin(q2))\
             - self.l3*cos(q3)*sin(q1)*sin(q2)
        
        J_23 = self.l3*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)) - self.l5*cos(self.theta2)*(sin(q6)*(sin(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - cos(q6)*sin(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))\
             + self.l5*sin(self.theta2)*(cos(q6)*(sin(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) + sin(q4)*sin(q6)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))\
             + self.l4*cos(q4)*sin(self.theta1)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))\
             + self.l4*cos(self.theta1)*sin(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))

        J_24 = self.l5*cos(self.theta2)*(cos(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))\
             + self.l5*sin(self.theta2)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))\
             + self.l4*cos(self.theta1)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4))\
             - self.l4*sin(self.theta1)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2))
        
        J_25 = self.l5*sin(q6 - self.theta2)*(cos(q1)*cos(q3)*cos(q5) - cos(q2)*cos(q5)*sin(q1)*sin(q3) + cos(q1)*cos(q4)*sin(q3)*sin(q5) + sin(q1)*sin(q2)*sin(q4)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5))
        
        J_26 = self.l5*sin(self.theta2)*(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
             - self.l5*cos(self.theta2)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))
        
        J_27 = 0

        J_31 = 0

        J_32 = self.l4*cos(self.theta1)*(cos(q4)*sin(q2) - cos(q2)*cos(q3)*sin(q4)) - self.l2*sin(q2)\
             - self.l4*sin(self.theta1)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) - self.l3*cos(q2)*cos(q3)\
             + self.l5*cos(self.theta2)*(sin(q6)*(cos(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) + cos(q2)*sin(q3)*sin(q5)) + cos(q6)*(cos(q4)*sin(q2) - cos(q2)*cos(q3)*sin(q4)))\
             - self.l5*sin(self.theta2)*(cos(q6)*(cos(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) + cos(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q4)*sin(q2) - cos(q2)*cos(q3)*sin(q4)))
        
        J_33 = self.l4*cos(self.theta1)*(cos(q4)*sin(q2) - cos(q2)*cos(q3)*sin(q4)) - self.l2*sin(q2)\
             - self.l4*sin(self.theta1)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) - self.l3*cos(q2)*cos(q3)\
             + self.l5*cos(self.theta2)*(sin(q6)*(cos(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) + cos(q2)*sin(q3)*sin(q5)) + cos(q6)*(cos(q4)*sin(q2) - cos(q2)*cos(q3)*sin(q4)))\
             - self.l5*sin(self.theta2)*(cos(q6)*(cos(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) + cos(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q4)*sin(q2) - cos(q2)*cos(q3)*sin(q4)))
        
        J_34 = self.l4*cos(self.theta1)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2))\
             + self.l4*sin(self.theta1)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))\
             + self.l5*cos(self.theta2)*(cos(q6)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))\
             + self.l5*sin(self.theta2)*(sin(q6)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))
        
        J_35 = self.l5*sin(q6 - self.theta2)*(cos(q5)*sin(q2)*sin(q3) + cos(q2)*sin(q4)*sin(q5) - cos(q3)*cos(q4)*sin(q2)*sin(q5))
        
        J_36 = - self.l5*cos(self.theta2)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))\
               - self.l5*sin(self.theta2)*(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))
        
        J_37 = 0

        J_41 = 0
        
        J_42 = (cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              *(sin(q7)*(sin(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) - cos(q2)*cos(q5)*sin(q3)) - cos(q7)*(cos(q6)*(cos(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) + cos(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q4)*sin(q2) - cos(q2)*cos(q3)*sin(q4))))\
              -(sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              *(cos(q7)*(sin(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) - cos(q2)*cos(q5)*sin(q3)) + sin(q7)*(cos(q6)*(cos(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) + cos(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q4)*sin(q2) - cos(q2)*cos(q3)*sin(q4))))\
              +(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              *(sin(q6)*(cos(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) + cos(q2)*sin(q3)*sin(q5)) + cos(q6)*(cos(q4)*sin(q2) - cos(q2)*cos(q3)*sin(q4)))
        
        J_43 = (cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              *(sin(q6)*(cos(q3)*sin(q2)*sin(q5) - cos(q4)*cos(q5)*sin(q2)*sin(q3)) + cos(q6)*sin(q2)*sin(q3)*sin(q4))\
              +(sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              *(cos(q7)*(cos(q3)*cos(q5)*sin(q2) + cos(q4)*sin(q2)*sin(q3)*sin(q5)) - sin(q7)*(cos(q6)*(cos(q3)*sin(q2)*sin(q5) - cos(q4)*cos(q5)*sin(q2)*sin(q3)) - sin(q2)*sin(q3)*sin(q4)*sin(q6)))\
              -(cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              *(sin(q7)*(cos(q3)*cos(q5)*sin(q2) + cos(q4)*sin(q2)*sin(q3)*sin(q5)) + cos(q7)*(cos(q6)*(cos(q3)*sin(q2)*sin(q5) - cos(q4)*cos(q5)*sin(q2)*sin(q3)) - sin(q2)*sin(q3)*sin(q4)*sin(q6)))
        
        J_44 = (sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              *(sin(q7)*(sin(q6)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))) + cos(q7)*sin(q5)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))\
              +(cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              *(cos(q7)*(sin(q6)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))) - sin(q5)*sin(q7)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))\
              +(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              *(cos(q6)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))
     
        J_45 = (sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              *(cos(q7)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - cos(q6)*sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)))\
              -(cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              *(sin(q7)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + cos(q6)*cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)))\
              +sin(q6)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3))\
              *(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))
        
        J_46 = -(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
               *(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))\
               -cos(q7)*(cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
               *(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))\
               -sin(q7)*(sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
               *(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))
        
        J_47 = -(cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) + sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
               *(cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
                -(sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
                *(sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))

        J_51 = (sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
              *(cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              -(cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) + sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
              *(sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              +(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              *(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))
        
        J_52 = (sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))\
              *(cos(q6)*(cos(q1)*cos(q2)*cos(q4) + cos(q1)*cos(q3)*sin(q2)*sin(q4)) + sin(q6)*(cos(q5)*(cos(q1)*cos(q2)*sin(q4) - cos(q1)*cos(q3)*cos(q4)*sin(q2)) - cos(q1)*sin(q2)*sin(q3)*sin(q5)))\
              -(cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) + sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
              *(sin(q7)*(sin(q6)*(cos(q1)*cos(q2)*cos(q4) + cos(q1)*cos(q3)*sin(q2)*sin(q4)) - cos(q6)*(cos(q5)*(cos(q1)*cos(q2)*sin(q4) - cos(q1)*cos(q3)*cos(q4)*sin(q2)) - cos(q1)*sin(q2)*sin(q3)*sin(q5))) - cos(q7)*(sin(q5)*(cos(q1)*cos(q2)*sin(q4) - cos(q1)*cos(q3)*cos(q4)*sin(q2)) + cos(q1)*cos(q5)*sin(q2)*sin(q3)))\
              +(sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
              *(cos(q7)*(sin(q6)*(cos(q1)*cos(q2)*cos(q4) + cos(q1)*cos(q3)*sin(q2)*sin(q4)) - cos(q6)*(cos(q5)*(cos(q1)*cos(q2)*sin(q4) - cos(q1)*cos(q3)*cos(q4)*sin(q2)) - cos(q1)*sin(q2)*sin(q3)*sin(q5))) + sin(q7)*(sin(q5)*(cos(q1)*cos(q2)*sin(q4) - cos(q1)*cos(q3)*cos(q4)*sin(q2)) + cos(q1)*cos(q5)*sin(q2)*sin(q3)))
        
        J_53 = (cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) + sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
              *(cos(q7)*(cos(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q4)*sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) - sin(q7)*(cos(q6)*(sin(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q4)*cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) + sin(q4)*sin(q6)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              -(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))\
              *(sin(q6)*(sin(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q4)*cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) - cos(q6)*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))\
              +(sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
              *(sin(q7)*(cos(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q4)*sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) + cos(q7)*(cos(q6)*(sin(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q4)*cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) + sin(q4)*sin(q6)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))
        
        J_54 = (sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))\
              *(cos(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))\
              -(cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) + sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
              *(sin(q7)*(sin(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - cos(q5)*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2))) - cos(q7)*sin(q5)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))\
              +(sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
              *(cos(q7)*(sin(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - cos(q5)*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2))) + sin(q5)*sin(q7)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))
        
        J_55 = sin(q6)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))\
              *(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))\
              -(cos(q7)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) - cos(q6)*sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              *(cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) + sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
              -(sin(q7)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) + cos(q6)*cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              *(sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))
        
        J_56 = cos(q7)*(sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
              *(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              -(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) + cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))\
              *(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              -sin(q7)*(cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) + sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
              *(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))
        
        J_57 = -(sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
               *(sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))\
               -(cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
               *(cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) + sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))

        J_61 = (cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))**2\
              +(sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))**2\
              +(cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))**2
        
        J_62 = (cos(q6)*(cos(q2)*cos(q4)*sin(q1) + cos(q3)*sin(q1)*sin(q2)*sin(q4)) + sin(q6)*(cos(q5)*(cos(q2)*sin(q1)*sin(q4) - cos(q3)*cos(q4)*sin(q1)*sin(q2)) - sin(q1)*sin(q2)*sin(q3)*sin(q5)))\
              *(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              -(sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              *(cos(q7)*(sin(q5)*(cos(q2)*sin(q1)*sin(q4) - cos(q3)*cos(q4)*sin(q1)*sin(q2)) + cos(q5)*sin(q1)*sin(q2)*sin(q3)) - sin(q7)*(sin(q6)*(cos(q2)*cos(q4)*sin(q1) + cos(q3)*sin(q1)*sin(q2)*sin(q4)) - cos(q6)*(cos(q5)*(cos(q2)*sin(q1)*sin(q4) - cos(q3)*cos(q4)*sin(q1)*sin(q2)) - sin(q1)*sin(q2)*sin(q3)*sin(q5))))\
              +(cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              *(cos(q7)*(sin(q6)*(cos(q2)*cos(q4)*sin(q1) + cos(q3)*sin(q1)*sin(q2)*sin(q4)) - cos(q6)*(cos(q5)*(cos(q2)*sin(q1)*sin(q4) - cos(q3)*cos(q4)*sin(q1)*sin(q2)) - sin(q1)*sin(q2)*sin(q3)*sin(q5))) + sin(q7)*(sin(q5)*(cos(q2)*sin(q1)*sin(q4) - cos(q3)*cos(q4)*sin(q1)*sin(q2)) + cos(q5)*sin(q1)*sin(q2)*sin(q3)))
        
        J_63 = (sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              *(cos(q7)*(cos(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - sin(q7)*(cos(q6)*(sin(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) + sin(q4)*sin(q6)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              -(cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              *(sin(q7)*(cos(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) + cos(q7)*(cos(q6)*(sin(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) + sin(q4)*sin(q6)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              +(sin(q6)*(sin(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - cos(q6)*sin(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))\
              *(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))
        
        J_64 = -(cos(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))\
               *(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
               -(sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
               *(sin(q7)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2))) - cos(q7)*sin(q5)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))\
               -(cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
               *(cos(q7)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2))) + sin(q5)*sin(q7)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))
        
        J_65 = (cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              *(sin(q7)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) + cos(q6)*cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              -(sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              *(cos(q7)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - cos(q6)*sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              -sin(q6)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))\
              *(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))
        
        J_66 = (sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              *(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              -cos(q7)*(cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              *(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))\
              -sin(q7)*(sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))\
              *(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))
        
        J_67 = sin(q2)*sin(q3)*sin(q5)*sin(q6) - cos(q3)*cos(q6)*sin(q2)*sin(q4) - cos(q2)*cos(q5)*sin(q4)*sin(q6) - cos(q2)*cos(q4)*cos(q6) + cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)

        J = np.matrix([ [ J_11 , J_12 , J_13 , J_14 , J_15 , J_16 , J_17 ],\
                        [ J_21 , J_22 , J_23 , J_24 , J_25 , J_26 , J_27 ],\
                        [ J_31 , J_32 , J_33 , J_34 , J_35 , J_36 , J_37 ],\
                        [ J_41 , J_42 , J_43 , J_44 , J_45 , J_46 , J_47 ],\
                        [ J_51 , J_52 , J_53 , J_54 , J_55 , J_56 , J_57 ],\
                        [ J_61 , J_62 , J_63 , J_64 , J_65 , J_66 , J_67 ]])
        return J

    def tf_A01(self, r_joints_array):
        tf = np.matrix([[cos(r_joints_array[0]), -sin(r_joints_array[0]), 0, 0],\
                        [sin(r_joints_array[0]),  cos(r_joints_array[0]), 0, 0],\
                        [0, 0, 1, self.l1],\
                        [0, 0, 0, 1]])
        return tf

    def tf_A02(self, r_joints_array):
        tf_A12 = np.matrix([[cos(r_joints_array[1]), -sin(r_joints_array[1]), 0, 0],\
                            [0, 0, 1, 0],\
                            [-sin(r_joints_array[1]), 0, -cos(r_joints_array[1]), 0],\
                            [0, 0 ,0, 1]])
        tf = np.dot( self.tf_A01(r_joints_array), tf_A12 )
        return tf

    def tf_A03(self, r_joints_array):
        tf_A23 = np.matrix([[cos(r_joints_array[2]), -sin(r_joints_array[2]), 0, 0],\
                            [0, 0, -1, -self.l2],\
                            [-sin(r_joints_array[2]), 0, cos(r_joints_array[2]), 0],\
                            [0, 0, 0, 1]])
        tf = np.dot( self.tf_A02(r_joints_array), tf_A23 )
        return tf

    def tf_A04(self, r_joints_array):
        tf_A34 = np.matrix([[cos(r_joints_array[3]), -sin(r_joints_array[3]), 0, self.l3],\
                            [0, 0, -1, 0],\
                            [sin(r_joints_array[3]), 0, cos(r_joints_array[3]), 0],\
                            [0, 0, 0, 1]])
        tf = np.dot( self.tf_A03(r_joints_array), tf_A34 )
        return tf

    def tf_A05(self, r_joints_array):
        tf_A45 = np.matrix([[cos(r_joints_array[4]), -sin(r_joints_array[4]), 0, self.l4*sin(self.theta1)],\
                            [0, 0, -1, -self.l4*cos(self.theta1)],\
                            [sin(r_joints_array[4]), 0, cos(r_joints_array[4]), 0],\
                            [0, 0, 0, 1]])
        tf = np.dot( self.tf_A04(r_joints_array), tf_A45 )
        return tf

    def tf_A06(self, r_joints_array):
        tf_A56 = np.matrix([[cos(r_joints_array[5]), -sin(r_joints_array[5]), 0, 0],\
                            [0, 0, -1, 0],\
                            [sin(r_joints_array[5]), 0, cos(r_joints_array[5]), 0],\
                            [0, 0, 0, 1]])
        tf = np.dot( self.tf_A05(r_joints_array), tf_A56 )
        return tf

    def tf_A07(self, r_joints_array):
        tf_A67 = np.matrix([[cos(r_joints_array[6]), -sin(r_joints_array[6]), 0, self.l5*sin(self.theta3)],\
                            [0, 0, 1, self.l5*cos(self.theta2)],\
                            [-sin(r_joints_array[6]), 0, -cos(r_joints_array[6]), 0],\
                            [0, 0, 0, 1]])
        tf = np.dot( self.tf_A06(r_joints_array), tf_A67 )
        return tf

    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(self, R) :

        assert(isRotationMatrix(R))

        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

        singular = sy < 1e-6

        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0

        return np.array([x, y, z])
