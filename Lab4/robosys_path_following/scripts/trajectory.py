#!/usr/bin/env python3

"""
Trajectory generation for xArm7 path planning
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
#Math imports
from math import pow
import numpy as np


class TrajectoryGen():

    def __init__(self, initialPosition, endingPosition):
        self.initialPosition = initialPosition
        self.endingPosition = endingPosition
        self.Delta = 1.0
        self.t_f = 10.0
        self.T = 20.0
        self.s_0 = 0.0
        self.s_f = 1.0
        self.V = (self.s_f - self.s_0)/(self.t_f - self.Delta)
        self.a = self.V/self.Delta
        self.s = np.zeros(1, dtype = np.float64)
        self.s_dot = np.zeros(1, dtype = np.float64)
        self.traj_position = np.zeros((6,1), dtype = np.float64)
        self.traj_velocity = np.zeros((6,1), dtype = np.float64)

    def polynomialTrajectory(self, t):

        if (t <= self.Delta):
           self.s = (self.a/2)*pow(t,2)
           self.s_dot = self.a*t
        
        elif (t > self.Delta and t <= (self.t_f - self.Delta)):
           self.s = 0.5 - (self.T/4)*self.V + self.V*t
           self.s_dot = self.V
        
        elif (t > (self.t_f - self.Delta) and t <= self.t_f):
            self.s = 1.0 - (self.a*pow(self.T,2))/8 + (self.a*self.t_f)*t - (self.a/2)*pow(t,2)
            self.s_dot = self.a*self.t_f - self.a*t
        
        elif (t > self.t_f and t <= self.t_f + self.Delta):
            self.s = 1.0 - self.a*pow(t - self.t_f, 2)
            self.s_dot = - self.a*(t - self.t_f)
        
        elif ((t > self.t_f + self.Delta) and (t <= self.T - self.Delta)):
            self.s = (1 + self.V*self.T)/2 - self.V*(t - self.t_f)
            self.s_dot = - self.V
        
        elif ((t > self.T - self.Delta) and t <= self.T):
            self.s = (self.a*pow(self.T, 2))/2 - self.a*self.T*(t - self.t_f) + (self.a/2)*pow(t - self.t_f, 2)
            self.s_dot = self.a*(t - self.t_f)
        
        for i in range(0, 2):
            self.traj_position[i] = self.initialPosition[i] + self.s*(self.endingPosition[i] - self.initialPosition[i])
            self.traj_velocity[i] = self.s_dot*(self.endingPosition[i] - self.initialPosition[i])
        for i in range(3,5):
            self.traj_position[i] = self.initialPosition[i] + self.s*(self.endingPosition[i] - self.initialPosition[i])
            self.traj_velocity[i] = self.s_dot*(self.endingPosition[i] - self.initialPosition[i])
        
        return self.traj_position, self.traj_velocity
