#! /usr/bin/env python

import rospy
import actionlib
from docking_server.msg import DockingFeedback, DockingResult, DockingAction, DockingGoal
from std_msgs.msg import Empty

class Docking:

    def __init__(self, name='DockingServer', freq=10) -> None:
        self._feedback = DockingFeedback()
        self._result = DockingResult()
        self._done = False
        self._action_name = name
        self._freq = freq
        self._as = actionlib.SimpleActionServer(self._action_name, DockingAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self._cancel = False
        rospy.Subscriber('cancel', Empty, callback=self.cancel_cb)

    def cancel_cb(self, msg:Empty):
        self._cancel = True

    def execute_cb(self, goal: DockingGoal):
        rate = rospy.Rate(self._freq)
        self._feedback.feedback = f'Docking type: {goal.type} ArucoID: {goal.aruco_id} Started'
        self._as.publish_feedback(self._feedback)
        rospy.loginfo(f'{self._action_name} Started')
        while not self._done:
            if self._as.is_preempt_requested():
                rospy.loginfo(f'{self._action_name} Preemted')
                self._as.set_preempted()
                self._done = False
                break

            if self._cancel:
                rospy.loginfo(f'{self._action_name} Aborted')
                self._as.set_aborted()
                self._done = False
                break

            """
            Placeholder for code
            """
            rospy.loginfo(f'{self._action_name} Running')
            self._feedback.feedback = f'Docking type: {goal.type} ArucoID: {goal.aruco_id} Running'
            self._as.publish_feedback(self._feedback)
            rate.sleep()

        if self._done:
            rospy.loginfo(f'{self._action_name} Succeeded')
            self._result.done = self._done
            self._result.result = f'Docking type: {goal.type} ArucoID: {goal.aruco_id} Succeeded'
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('docking')
    freq = rospy.get_param('rate', default=10)
    server = Docking(freq=freq)
    rospy.spin()