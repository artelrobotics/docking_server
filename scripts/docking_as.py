#! /usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
from docking_server.msg import DockingFeedback, DockingAction, DockingResult, DockingGoal
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from math import atan2, pi
import math
from geometry_msgs.msg import Twist
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float64
from nav_msgs.msg import Path, Odometry
from robotnik_msgs.msg import BatteryStatus
from std_srvs.srv import SetBool
from dynamic_reconfigure.server import Server
from docking_server.cfg import ConfigConfig
from laser_line_extraction.msg import LineSegmentList

class Docking:

    def __init__(self, name='DockingServer', freq=10):
        self.station_points = []
        self.detection_command = Bool()
        self.goal_aruco_id = 0
        self.wheelbase = 0.3
        self.aruco_found = False
        self.cmd = Twist()
        self.current_pose = Pose()
        self.path = Path()
        self.path_received = False
        self.waypoint_index = 0
        self.pre_point_reached = False
        self.kp_linear = 0.1
        self.kd_linear = 0.01
        self.kp_angular = 0.7
        self.kd_angular = 0.1
        self.forward_bool = False
        self._feedback = DockingFeedback()
        self._result = DockingResult()
        self._done = False
        self.cart_found = False
        self._action_name = name
        self._freq = freq
        self.cmd_vel_pub = rospy.Publisher("cmd_vel" , Twist, queue_size= 1)

        self._cancel = False
        self._break = False

        self.front_distance = Float64()
        self._robot_back_edges = rospy.get_param('robot_front_edges', [[-0.35, 0.32], [-0.35, -0.32]])
        self.back_obstacle_detector = False
        rospy.Subscriber('~cancel', Empty, callback=self.cancel_cb)
        rospy.Subscriber('docking_path/plan/fiducial_1', Path, callback=self.path_callback_fiducial_1)
        rospy.Subscriber('daly_bms/data', BatteryStatus, callback=self.bms_callback)
        rospy.Subscriber('line_segments', LineSegmentList, callback=self.line_segments)
        rospy.Subscriber('odometry', Odometry, callback=self.odom_callback)
        self.reconfigure = Server(ConfigConfig, self.reconfigure_callback)
        rospy.logerr("Server initialized reconfigure")
        self._as = actionlib.SimpleActionServer(self._action_name, DockingAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.aruco_detection(False)

    def line_segments(self, data):
        len_of_segments = len(data.line_segments)
        if (len_of_segments == 2):
            left_line_middle_x = (data.line_segments[0].start[0] + data.line_segments[0].end[0]) / 2
            left_line_middle_y = (data.line_segments[0].start[1] + data.line_segments[0].end[1]) / 2
            right_line_middle_x = (data.line_segments[1].start[0] + data.line_segments[1].end[0]) / 2
            right_line_middle_y = (data.line_segments[1].start[1] + data.line_segments[1].end[1]) / 2
            
            # if (left_line_middle_y)
            print("Left x and y:", left_line_middle_x, left_line_middle_y)  
            print("Right x and y:", right_line_middle_x, right_line_middle_y)
            

            self.center_x = (right_line_middle_x + left_line_middle_x)/2
            self.center_y = right_line_middle_y + left_line_middle_y

            # print("Center X and Y :", center_x, center_y)
            self.cart_found = True
        # else:
        #     # self.cart_found = False

    def go_inside_cart(self):
        if (self.center_x > 0.1 and self.cart_found == True):
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.5 * self.center_y
            self.cmd_vel_pub.publish(self.cmd)
        elif(self.center_x <= 0.1):
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd)



    def reconfigure_callback(self, config, level):
        self.kp_linear = config.kp_linear
        self.kd_linear = config.kd_linear
        self.kp_angular = config.kp_angular
        self.kd_angular = config.kd_angular
        rospy.loginfo(f'{self.kp_angular, self.kd_linear} reconfigured')
        return config

    def aruco_detection(self, bool):
        rospy.wait_for_service('enable_detections')
        try:
            switch_aruco_detection = rospy.ServiceProxy('enable_detections', SetBool)
            resp1 = switch_aruco_detection(bool)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def cancel_cb(self, msg: Empty):
        self._cancel = True

    def bms_callback(self, msg :BatteryStatus):
        self.is_charging = msg.is_charging

    def stop_motion(self):
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd)

    def path_callback_fiducial_1(self, msg):
        self.path_fiducial_1 = msg
        self.path_received_fiducial_1 = True

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

    def get_heading_fiducial_1(self, x, y):
        _, _, yaw = euler_from_quaternion([self.current_pose.orientation.x,
                                           self.current_pose.orientation.y,
                                           self.current_pose.orientation.z,
                                           self.current_pose.orientation.w])
        path_x = self.path_fiducial_1.poses[self.waypoint_index].pose.position.x
        path_y = self.path_fiducial_1.poses[self.waypoint_index].pose.position.y
        path_heading = atan2(path_y - y, path_x - x)

        return self.normalize(path_heading - yaw)

    def follow_path_fiducial_1(self):
        if self.path_received_fiducial_1:
            if self.waypoint_index >= len(self.path_fiducial_1.poses):
                rospy.loginfo("Reached the end of the fiducial 1")
                self.pre_point_reached = True

            else:
                waypoint_pose = self.path_fiducial_1.poses[self.waypoint_index]

                x = self.current_pose.position.x
                y = self.current_pose.position.y
                distance = self.get_distance(x, y, waypoint_pose.pose.position.x, waypoint_pose.pose.position.y)

                if distance < 0.2:
                    self.waypoint_index += 1

                linear_error = distance
                angular_error = self.get_heading_fiducial_1(x, y)
                linear_velocity = self.kp_linear * linear_error - self.kd_linear * distance
                angular_velocity = self.kp_angular * angular_error - self.kd_angular * angular_error

                # Create and publish Twist message
                twist = Twist()
                twist.linear.x = linear_velocity
                twist.angular.z = angular_velocity
                self.cmd_vel_pub.publish(twist)


    def execute_cb(self, goal: DockingGoal):
        self.aruco_follow_done = False
        self.aruco_detection(True)
        if(goal.type == "docking"):
            self.goal_aruco_id = goal.aruco_id
            rate = rospy.Rate(self._freq)
            self._feedback.feedback = f'Docking type: {goal.type} ArucoID: {goal.aruco_id} Started'
            self._as.publish_feedback(self._feedback)
            rospy.loginfo(f'{self._action_name} Started')
            self._break = False
            self.pre_point_reached = False
            self.docking_point_reached = False
            while not self._done and not self._break and not rospy.is_shutdown():
                if self._as.is_preempt_requested():
                    rospy.loginfo(f'{self._action_name} Preemted')
                    self._as.set_preempted()
                    self._done = False
                    self.stop_motion()
                    break

                if self._cancel:
                    rospy.loginfo(f'{self._action_name} Aborted')
                    self._as.set_preempted()
                    self._done = False
                    self._cancel = False
                    self.stop_motion()
                    break

                if (self.pre_point_reached == False or self.is_charging == False):
                    self.waypoint_index = 0
                    self.follow_path_fiducial_1()

                if (self.goal_aruco_id == 1 and self.pre_point_reached == True and self.cart_found == True):
                    self.go_inside_cart()

                if (self.is_charging):
                    self._done = True
                    self.stop_motion()
                    break

                rospy.sleep(0.1)


        if self._done:
            rospy.loginfo(f'{self._action_name} Succeeded')
            self._result.done = self._done
            self._result.result = f'Docking type: {goal.type} ArucoID: {goal.aruco_id} Succeeded'
            self._as.set_succeeded(self._result)
            self.stop_motion()
            self.aruco_detection(False)
            self._done = False



if __name__ == '__main__':
    rospy.init_node('docking')
    freq = rospy.get_param('rate', default=10)
    server = Docking(freq=freq)
    rospy.spin()
