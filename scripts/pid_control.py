#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from math import atan2, pi
from nav_msgs.msg import Odometry

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower')

        self.path_sub = rospy.Subscriber('/leo_bot/docking_path/plan', Path, self.path_callback)
        self.odom_sub = rospy.Subscriber('/leo_bot/odometry', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/leo_bot/cmd_vel', Twist, queue_size=10)

        self.current_pose = Pose()
        self.path = Path()
        self.path_received = False
        self.waypoint_index = 0

        self.kp_linear = 0.1
        self.kd_linear = 0.02
        self.kp_angular = 0.1
        self.kd_angular = 0.01

        self.rate = rospy.Rate(10)

    def path_callback(self, msg):
        self.path = msg
        self.path_received = True
    def odom_callback(self, msg:Odometry):
        self.current_pose = msg.pose.pose

    def get_distance(self, x1, y1, x2, y2):
        return ((x2 - x1)**2 + (y2 - y1)**2)**0.5

    def normalize(self, angle):
        if angle > pi:
            return angle - 2.0 * pi
        if angle < -pi:
            return angle + 2.0 * pi

        return angle

    def get_heading(self, x, y):
        _, _, yaw = euler_from_quaternion([self.current_pose.orientation.x,
                                           self.current_pose.orientation.y,
                                           self.current_pose.orientation.z,
                                           self.current_pose.orientation.w])
        path_x = self.path.poses[self.waypoint_index].pose.position.x
        path_y = self.path.poses[self.waypoint_index].pose.position.y
        path_heading = atan2(path_y - y, path_x - x)

        return self.normalize(path_heading - yaw)

    def follow_path(self):
        while not rospy.is_shutdown():
            if self.path_received:
                if self.waypoint_index >= len(self.path.poses):
                    rospy.loginfo("Reached the end of the path.")
                    break

                waypoint_pose = self.path.poses[self.waypoint_index]

                x = self.current_pose.position.x
                y = self.current_pose.position.y
                distance = self.get_distance(x, y, waypoint_pose.pose.position.x, waypoint_pose.pose.position.y)

                if distance < 0.2:
                    self.waypoint_index += 1
                    continue

                linear_error = distance
                angular_error = self.get_heading(x, y)
                linear_velocity = self.kp_linear * linear_error - self.kd_linear * distance
                angular_velocity = self.kp_angular * angular_error - self.kd_angular * angular_error

                # Create and publish Twist message
                twist = Twist()
                twist.linear.x = linear_velocity
                twist.angular.z = angular_velocity
                self.cmd_vel_pub.publish(twist)

            self.rate.sleep()

        # Stop the robot at the end of the path
        # self.cmd_vel_pub.publish(Twist())

if __name__ == '__main__':
    try:
        path_follower = PathFollower()
        path_follower.follow_path()
    except rospy.ROSInterruptException:
        pass
