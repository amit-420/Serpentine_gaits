#! usr/bin python3

import sys

import numpy as np
from math import pi
import rospy
from sensor_msgs.msg import JointState
from actionlib.simple_action_client import SimpleActionClient 
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from rospy.topics import Message
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from std_msgs.msg import Header



class run_dynamixel:
    def __init__(self):
        
        self.joints_str = JointTrajectory()
        self.joints_str.header = Header()
        self.joints_str.header.stamp = rospy.Time.now()
        self.joints_str.joint_names = ['left']
        self.point = JointTrajectoryPoint()
        # l3 = pi/180 * np.array([
        #                           [0,30,45,0,-30,-50,-10,0],
        #                           [0,30,45,0,-30,-50,-10,0]
        #                            ])
        # for indx in range(len(l3[0, :])):
        #     point= JointTrajectoryPoint(positions= l3[:, indx],
        #                                 time_from_start= rospy.Duration(0 + 0.5*indx))
        #     self.joints_str.points.append(point)
        point = JointTrajectoryPoint(positions=[3.57],velocities=[10],time_from_start= rospy.Duration(1))
        self.joints_str.points.append(point)
        point1 = JointTrajectoryPoint(positions=[-3.57],velocities=[10],time_from_start= rospy.Duration(2))
        self.joints_str.points.append(point1)
        # point1 = JointTrajectoryPoint(positions=[0],time_from_start= rospy.Duration(0.8))
        # self.joints_str.points.append(point1)
        print(str(self.joints_str.points)+ '\n')
    def pub_position(self,arg):
        pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory',JointTrajectory,queue_size=1)
        pub.publish(self.joints_str)
        # rospy.loginfo("command invaito: %s", self.joints_str)
    
    def listener(self):
        rospy.Subscriber('/dynamixel_workbench/joint_states',JointState,self.pub_position)


if __name__ == '__main__':
    try:
        rospy.init_node('pub_to_motor')
        r1 = run_dynamixel()
        r1.listener()
        rospy.spin()
    except KeyboardInterrupt as ke:
        pass

        