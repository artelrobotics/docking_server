#! /usr/bin/env python3

import rospy
import actionlib
from std_msgs.msg import Empty
from docking_server.msg import DockingFeedback, DockingAction, DockingResult, DockingGoal
import sys
def docking_client():
    client = actionlib.SimpleActionClient('DockingServer', DockingAction)
    client.wait_for_server()
    goal = DockingGoal(aruco_id=3, type='charging')
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('docking_client')
        result = docking_client()
        print(result.result)
        print(result.done)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)