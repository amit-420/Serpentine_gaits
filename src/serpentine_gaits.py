#! usr/bin python3

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from std_msgs.msg import Header
from math import pi,floor
PI = pi

class serpentine:
    def __init__(self):
        
        self.serpentine_data = {
                                "lp": self.executeGoals("lp"),
                                "lu": self.executeGoals("lu"),
                                "sw": self.executeGoals("sw"),
                                "rl": self.executeGoals("rl"),
                                "rt": self.executeGoals("rt")
                                }
        
    
    def create_traj_msg(self,gait):
        self.joints_str = JointTrajectory()
        self.joints_str.header = Header()
        self.joints_str.header.stamp = rospy.Time.now()
        self.joints_str.joint_names = ['pan1','pan2','pan3','pan4','pan5','pan6']
        self.point = JointTrajectoryPoint()

        for i in range(len(self.serpentine_data[gait][0, :])):
            point= JointTrajectoryPoint(positions= self.serpentine_data[gait][:, i],
                                        time_from_start= rospy.Duration(0 + 0.015*i))
            self.joints_str.points.append(point)


    class setGaitParams:
        def __init__(self, gait_type = 'lp'):
            if gait_type == 'lp':
                self.Ay = 0.5*PI*60/180 # deg
                self.Ax = 0*PI*30/180 # rad
                self.wy = 5*PI/6 # Hz
                self.wx = 5*PI/6 # Hz
                self.dy = 1*PI/3 
                self.dx = 0*PI/3 
                self.phi = 0*PI/6 
            if gait_type == 'lu':
                self.Ay = 0*PI*50/180 # deg
                self.Ax = 1*PI*60/180 # rad
                self.wy = 5*PI/6 # Hz
                self.wx = 5*PI/6 # Hz
                self.dy = 0*PI/3 
                self.dx = 2*PI/3 
                self.phi = PI/6
            if gait_type == 'sw':
                self.Ay = PI*30/180 # deg
                self.Ax = PI*30/180 # rad
                self.wy = 5*PI/6 # Hz
                self.wx = 5*PI/6 # Hz 
                self.dy = 2*PI/3
                self.dx = 2*PI/3 
                self.phi = 0*PI/6                  
            if gait_type == 'rl':
                self.Ay = PI*60/180 # deg
                self.Ax = PI*60/180 # rad
                self.wy = 3*PI/6 # Hz
                self.wx = 3*PI/6 # Hz
                self.dy = 0*PI/2
                self.dx = 0*PI/2 
                self.phi = 1*PI/6
            if gait_type == 'rt':
                self.Ay = 0.5*PI*90/180 # deg
                self.Ax = 0.5*PI*90/180 # rad
                self.wy = PI/6 # Hz
                self.wx = PI/6 # Hz
                self.dy = 1*PI/2
                self.dx = 1*PI/2 
                self.phi = 1*PI/6

    def get_angle(self,A,omega,d,phi,n,t):
        return A * np.sin(omega*t + n*d + phi)

    def executeGoals(self,type):

        self.gait_params = self.setGaitParams(type)

        l1 = np.array([
                                    [self.get_angle(self.gait_params.Ay, self.gait_params.wy,self.gait_params.dy,self.gait_params.phi,1,t/50) for t in range(0,floor(2*PI*50/self.gait_params.wy),1)],
                                    [self.get_angle(self.gait_params.Ax, self.gait_params.wx,self.gait_params.dx,0,2,t/50) for t in range(0,floor(2*PI*50/self.gait_params.wx),1)],
                                    [self.get_angle(self.gait_params.Ay, self.gait_params.wy,self.gait_params.dy,self.gait_params.phi,3,t/50) for t in range(0,floor(2*PI*50/self.gait_params.wy),1)],
                                    [self.get_angle(self.gait_params.Ax, self.gait_params.wx,self.gait_params.dx,0,4,t/50) for t in range(0,floor(2*PI*50/self.gait_params.wx),1)],
                                    [self.get_angle(self.gait_params.Ay, self.gait_params.wy,self.gait_params.dy,self.gait_params.phi,5,t/50) for t in range(0,floor(2*PI*50/self.gait_params.wy),1)],
                                    [self.get_angle(self.gait_params.Ax, self.gait_params.wx,self.gait_params.dx,0,6,t/50) for t in range(0,floor(2*PI*50/self.gait_params.wx),1)],
                                    #[self.get_angle(self.gait_params.Ay, self.gait_params.wy,self.gait_params.dy,self.gait_params.phi,5.5,t/30) for t in range(0,floor(2*PI*30/self.gait_params.wy),1)]
                                ])
        
        return l1

    

    def pub_position(self,arg,gait):
        pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory',JointTrajectory,queue_size=1)

        self.create_traj_msg(gait)
        # print(self.joints_str)
        pub.publish(self.joints_str)
        # rospy.loginfo("command given: %s", self.joints_str)
    
    def listener(self,gait):
        rospy.Subscriber('/dynamixel_workbench/joint_states',JointState,self.pub_position,gait)
        


if __name__ == '__main__':
    try:
        rospy.init_node('pub_to_motor')
        r1 = serpentine()
        r1.listener('lu')
        rospy.spin()
    except KeyboardInterrupt as ke:
        pass

        