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
import trajectory as tg

# Arm parameters
# xArm7 kinematics class
from kinematics import xArm7_kinematics

# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

class xArm7_controller():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # Init xArm7 kinematics handler
        self.kinematics = xArm7_kinematics()

        # joints' angular positions
        self.joint_angpos = [0, 0, 0, 0, 0, 0, 0]
        # joints' angular velocities
        self.joint_angvel = [0, 0, 0, 0, 0, 0, 0]
        # joints' states
        self.joint_states = JointState()
        # joints' transformation matrix wrt the robot's base frame
        self.A01 = self.kinematics.tf_A01(self.joint_angpos)
        self.A02 = self.kinematics.tf_A02(self.joint_angpos)
        self.A03 = self.kinematics.tf_A03(self.joint_angpos)
        self.A04 = self.kinematics.tf_A04(self.joint_angpos)
        self.A05 = self.kinematics.tf_A05(self.joint_angpos)
        self.A06 = self.kinematics.tf_A06(self.joint_angpos)
        self.A07 = self.kinematics.tf_A07(self.joint_angpos)

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.joint_states_sub = rospy.Subscriber('/xarm/joint_states', JointState, self.joint_states_callback, queue_size=1)
        self.joint1_pos_pub = rospy.Publisher('/xarm/joint1_position_controller/command', Float64, queue_size=1)
        self.joint2_pos_pub = rospy.Publisher('/xarm/joint2_position_controller/command', Float64, queue_size=1)
        self.joint3_pos_pub = rospy.Publisher('/xarm/joint3_position_controller/command', Float64, queue_size=1)
        self.joint4_pos_pub = rospy.Publisher('/xarm/joint4_position_controller/command', Float64, queue_size=1)
        self.joint5_pos_pub = rospy.Publisher('/xarm/joint5_position_controller/command', Float64, queue_size=1)
        self.joint6_pos_pub = rospy.Publisher('/xarm/joint6_position_controller/command', Float64, queue_size=1)
        self.joint7_pos_pub = rospy.Publisher('/xarm/joint7_position_controller/command', Float64, queue_size=1)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)
        self.p_real = None
        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of joint 1 is stored in :: self.joint_states.position[0])

    def publish(self):

        # set configuration
        self.joint_angpos = [0, 0.75, 0, 1.5, 0, 0.75, 0]
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        self.joint4_pos_pub.publish(self.joint_angpos[3])
        tmp_rate.sleep()
        self.joint2_pos_pub.publish(self.joint_angpos[1])
        self.joint6_pos_pub.publish(self.joint_angpos[5])
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        P1 = np.matrix([0.6043, -0.2000, 0.1508, pi, 0, 0]).reshape(6,1)
        P2 = np.matrix([0.6043, 0.200, 0.1508, pi, 0, 0]).reshape(6,1)        
        A07_real = self.kinematics.tf_A07(self.joint_states.position)
        start_pos = A07_real[0:3,3]
        Ps = start_pos
        traj = tg.TrajectoryGen(P1, P2)
        
        # Gain matrix
        K = np.matrix([[0.1, 0, 0, 0, 0, 0],
                      [0, 0.1, 0, 0, 0, 0],
                      [0, 0, 0.1, 0, 0, 0],
                      [0, 0, 0, 0.1, 0, 0],
                      [0, 0, 0, 0, 0.1, 0],
                      [0, 0, 0, 0, 0, 0.1]])
    
        rostime_now = rospy.get_rostime()
        time_now = rostime_now.nsecs
        t0 = time_now
        
        while not rospy.is_shutdown():

            # Compute each transformation matrix wrt the base frame from joints' angular positions
            self.A07 = self.kinematics.tf_A07(self.joint_angpos)
            A07_real = self.kinematics.tf_A07(self.joint_states.position)
            ee_pos = A07_real[0:3,3]
            ee_ori = (self.kinematics.rotationMatrixToEulerAngles(A07_real[0:3,0:3])).reshape(3,1)
            
            # Compute jacobian matrix
            J = self.kinematics.compute_jacobian(self.joint_angpos)
            
            # pseudoinverse jacobian
            pinvJ = pinv(J)
            
            # Starting the Trajectory All Over Again
            if (ee_pos == P1[0:2]):
                Ps = P1
                t0 = rostime_now.secs
            elif (ee_pos == P2[0:2]):
                Ps = P2
                t0 = rostime_now.secs
            
            # Based on the Starting Position set the Final position
            if (Ps == start_pos):
                Pf = P1
                T = 5.0
            elif (Ps == P1):
                Pf = P2
                T = 10.0
            elif (Ps == P2):
                Pf = P1
                T = 10.0
            
            # Setting Trajectory Parameters based on Starting and Final Position
            traj.setStartEnd(Ps, Pf)
            traj.setMovementPeriod(t0,t0 + T)
            
            
            
            # print(self.p_real)
            p_desired, p_dot_desired = traj.polynomialTrajectory(rostime_now.secs)
            
            #Used To hold the orientation (I SAW SOMETHING IN MATLAB)++++SEE IF THIS GIVES OK OUTPUT
            self.e_p = np.concatenate([p_desired[0:2]-ee_pos, np.zeros((3,1), dtype = np.float64)])
            # ELSE USE:
            # self.p_real = np.concatenate([ee_pos, ee_ori])
            # e_p = p_desired - self.p_real
            q_dot = np.dot(pinvJ,(p_dot_desired + np.dot(K, e_p)))
           
            self.joint_angvel = np.squeeze(np.asarray(q_dot))
            
            print("Angular Velocity: ")
            print(self.joint_angvel)
            
            # Convertion to angular position after integrating the angular speed in time
            # Calculate time interval
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.nsecs
            dt = (time_now - time_prev)/1e9
            # Integration
            self.joint_angpos = np.add(self.joint_angpos, self.joint_angvel*dt)
            print("Angular Position: ")
            print(self.joint_angpos)
            # Publish the new joint's angular positions
            self.joint1_pos_pub.publish(self.joint_angpos[0])
            self.joint2_pos_pub.publish(self.joint_angpos[1])
            self.joint3_pos_pub.publish(self.joint_angpos[2])
            self.joint4_pos_pub.publish(self.joint_angpos[3])
            self.joint5_pos_pub.publish(self.joint_angpos[4])
            self.joint6_pos_pub.publish(self.joint_angpos[5])
            self.joint7_pos_pub.publish(self.joint_angpos[6])

            self.pub_rate.sleep()

    def turn_off(self):
        pass

def controller_py():
    # Starts a new node
    rospy.init_node('controller_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    controller = xArm7_controller(rate)
    rospy.on_shutdown(controller.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_py()
    except rospy.ROSInterruptException:
        pass
