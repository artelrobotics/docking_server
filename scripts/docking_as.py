#! /usr/bin/env python3

import rospy
import actionlib
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
from docking_server.msg import DockingFeedback, DockingAction, DockingResult, DockingGoal
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from math import degrees, atan2, pi, sqrt, atan
import math
from geometry_msgs.msg import Twist , TransformStamped
import time
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float64
from nav_msgs.msg import Path, Odometry
from robotnik_msgs.msg import BatteryStatus
import tf2_ros
from std_srvs.srv import SetBool

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
        self.kp_linear = 0.3
        self.kd_linear = 0.2
        self.kp_angular = 1.5
        self.kd_angular = 0.1
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self.forward_bool = False
        self._feedback = DockingFeedback()
        self._result = DockingResult()
        self._done = False
        self._action_name = name
        self._freq = freq
        self.detection_pub = rospy.Publisher("/leo_bot/ar_tag_detector/enable_detection" , Bool, queue_size= 1)
        self.cmd_vel_pub = rospy.Publisher("/leo_bot/cmd_vel" , Twist, queue_size= 1)

        self._cancel = False
        self._break = False

        self.front_distance = Float64()
        self._robot_back_edges = rospy.get_param('robot_front_edges', [[-0.35, 0.32], [-0.35, -0.32]])
        self.back_obstacle_detector = False
        rospy.Subscriber('cancel', Empty, callback=self.cancel_cb)
        rospy.Subscriber('/leo_bot/docking_path/plan/fiducial_1', Path, callback=self.path_callback_fiducial_1)
        rospy.Subscriber('/leo_bot/docking_path/plan/fiducial_2', Path, callback=self.path_callback_fiducial_2)
        rospy.Subscriber('/leo_bot/odometry', Odometry, callback=self.odom_callback)
        rospy.Subscriber('/leo_bot/daly_bms/data', BatteryStatus, callback=self.bms_callback)
        rospy.Subscriber('/leo_bot/scan', LaserScan, self.scan_callback)
        # rospy.Subscriber('/leo_bot/line_markers', Marker, callback=self.laser_line_callback)
        self._as = actionlib.SimpleActionServer(self._action_name, DockingAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.aruco_detection(False)

    def aruco_detection(self, bool):
        rospy.wait_for_service('/leo_bot/enable_detections')
        try:
            switch_aruco_detection = rospy.ServiceProxy('/leo_bot/enable_detections', SetBool)
            resp1 = switch_aruco_detection(bool)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def scan_callback(self, scan_msg: LaserScan):
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        x1, y1 = self._robot_back_edges[0]
        x2, y2 = self._robot_back_edges[1]

        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)

        mask = np.logical_and(ly > y2, np.logical_and(ly < y1, np.logical_and(lx < x1, lx < x2)))
        ranges_in_back = lx[mask] - x1



        if len(ranges_in_back) > 0:
            self._min_dist_to_back = np.max(ranges_in_back)
            # rospy.loginfo(f'Min distance in Backward : {self._min_dist_to_back}')
            if(self._min_dist_to_back > -0.3):
                self.back_obstacle_detector = True
            else:
                self.back_obstacle_detector = False



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

    def path_callback_fiducial_2(self, msg):
        self.path_fiducial_2 = msg
        self.path_received_fiducial_2 = True

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

    def get_heading_fiducial_2(self, x, y):
        _, _, yaw = euler_from_quaternion([self.current_pose.orientation.x,
                                           self.current_pose.orientation.y,
                                           self.current_pose.orientation.z,
                                           self.current_pose.orientation.w])
        path_x = self.path_fiducial_2.poses[self.waypoint_index].pose.position.x
        path_y = self.path_fiducial_2.poses[self.waypoint_index].pose.position.y
        path_heading = atan2(path_y - y, path_x - x)

        return self.normalize(path_heading - yaw)

    def follow_path_fiducial_2(self):
        # while not rospy.is_shutdown():
        if self.path_received_fiducial_2:
            if self.waypoint_index >= len(self.path_fiducial_2.poses):
                rospy.loginfo("Reached the end of the fiducial 2")
                self.docking_point_reached = True

            else:
                waypoint_pose = self.path_fiducial_2.poses[self.waypoint_index]

                x = self.current_pose.position.x
                y = self.current_pose.position.y
                distance = self.get_distance(x, y, waypoint_pose.pose.position.x, waypoint_pose.pose.position.y)

                if distance < 0.2:
                    self.waypoint_index += 1

                linear_error = distance
                angular_error = self.get_heading_fiducial_2(x, y)
                linear_velocity = self.kp_linear * linear_error - self.kd_linear * distance
                angular_velocity = self.kp_angular * angular_error - self.kd_angular * angular_error

                # Create and publish Twist message
                twist = Twist()
                twist.linear.x = linear_velocity
                twist.angular.z = angular_velocity
                self.cmd_vel_pub.publish(twist)

    def move_backward(self, target_distance):
        last_point_x = self.current_pose.position.x
        last_point_y = self.current_pose.position.y
        moved_distance = 0
        while abs(moved_distance)< abs(target_distance) and not rospy.is_shutdown():
            if (self.back_obstacle_detector == True):
                self.cmd.linear.x = 0.0  # Adjust the linear velocity as needed
                self.cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd)
                rospy.logerr("Obstcale detector in backward side !")


            elif self.current_pose is not None:
                current_x = self.current_pose.position.x
                current_y = self.current_pose.position.y

                delta_distance = math.sqrt((current_x - last_point_x)**2 + (current_y - last_point_y)**2)
                moved_distance += delta_distance
                self.cmd.linear.x = -0.1  # Adjust the linear velocity as needed
                self.cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd)
                last_point_x = current_x
                last_point_y = current_y


            rospy.sleep(0.067)


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


                # if (self.pre_point_reached == True):
                #     self.waypoint_index = 0
                #     self.follow_path_fiducial_2()
                #     time.sleep(1)
                #     self.aruco_follow_done = True


                if (self.is_charging):
                    self._done = True
                    self.stop_motion()
                    break

                rospy.sleep(0.067)

        if(goal.type == "undocking"):
            self.move_backward(0.4)
            self._done = True

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
