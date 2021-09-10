#!/usr/bin/env python

from dynamixel_workbench_msgs.srv import DynamixelCommand

import sys
import rospy


def run_motor(goal_pos):
    rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
    try:
        send_goal_pos = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command',DynamixelCommand)
        # output = send_goal_pos('',1,'Goal_Position',goal_pos)
        output = send_goal_pos('',1,'Moving_Speed',goal_pos)
        return output
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    

def usage():
    return "unknown command"

if __name__ == "__main__":
    # if len(sys.argv) == 1:
    goal_pos = 0
    # else:
    #     print(usage())
    #     sys.exit(1)
    print ("Requesting %s"%goal_pos)
    print(run_motor(goal_pos))