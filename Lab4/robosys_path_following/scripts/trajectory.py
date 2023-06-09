#!/usr/bin/env python3

"""
Start ROS node to publish angles for the position control of the xArm7.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

class TrajectoryGen():

    def __init__(self, initialPosition, endingPosition):
        self.initialPosition = initialPosition
        self.endingPosition = endingPosition
        self.Delta = 1
        self.t_f = 10
        self.T = 20

    def polynomialTrajectory(self):

        
        for t
        self.traj = self.initialPosition + self.s*(self.endingPosition - self.initialPosition)
        return self.traj
