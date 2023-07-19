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
from geometry_msgs.msg import Twist , TransformStamped
import time
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float64
from nav_msgs.msg import Path, Odometry
from robotnik_msgs.msg import BatteryStatus
import tf2_ros
class Docking:

    def __init__(self, name='DockingServer', freq=10):
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
        self.kd_linear = 0.15
        self.kp_angular = 1.0
        self.kd_angular = 0.3
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
        self._min_dist_to_front = 0
        self._robot_front_edges = rospy.get_param('robot_front_edges', [[0.27, 0.27], [0.27, -0.27]])
        self._marker_frame = rospy.get_param('marker_frame', default='ar_marker_1')
        self._base_frame = rospy.get_param('robot_frame', default='leo_bot/base_link')
        self._transform_tolerance = rospy.get_param('transform_tolerance', default=0.5)
        rospy.Subscriber('cancel', Empty, callback=self.cancel_cb)
        rospy.Subscriber('/leo_bot/ar_pose_marker', AlvarMarkers, callback=self.aruco_pose_cb)
        rospy.Subscriber('/leo_bot/scan', LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber('/leo_bot/docking_path/plan', Path, callback=self.path_callback)
        rospy.Subscriber('/leo_bot/odometry', Odometry, callback=self.odom_callback)
        rospy.Subscriber('/leo_bot/daly_bms/data', BatteryStatus, callback=self.bms_callback)
        self._as = actionlib.SimpleActionServer(self._action_name, DockingAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def scan_callback(self, scan_msg: LaserScan):
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        x1, y1 = self._robot_front_edges[0]
        x2, y2 = self._robot_front_edges[1]

        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)

        mask = np.logical_and(ly > y2, np.logical_and(ly < y1, np.logical_and(lx > x1, lx > x2)))
        ranges_in_front = lx[mask] - x1

        if len(ranges_in_front) > 0:
            self._min_dist_to_front = np.min(ranges_in_front)
            


    def cancel_cb(self, msg: Empty):
        self._cancel = True

    def bms_callback(self, msg :BatteryStatus):
        self.is_charging = msg.is_charging
    
    def get_transform(self, target_frame, source_frame) -> TransformStamped:
        try:
            can_transform = self._tf_buffer.can_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(1.0))
            if can_transform:
                transform: TransformStamped = self._tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
                if not (rospy.Time.now() - rospy.Duration(self._transform_tolerance)) > transform.header.stamp:
                    return transform
                else: 
                    return None
            else:
                None
        except tf2_ros.TransformException as ex:
            rospy.logwarn(str(ex))
            rospy.sleep(1.0)

    def aruco_pose_cb(self, msg: AlvarMarkers):
        if len(msg.markers) > 0: 
            for marker in msg.markers:
                if(marker.id == self.goal_aruco_id):
                    self.aruco_found = True
                    self.pose_x = marker.pose.pose.position.x
                    self.pose_z = marker.pose.pose.position.z
                    quat = (marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w)
                    euler = euler_from_quaternion(quat)
                    self.yaw = degrees(euler[1])
                    # rospy.loginfo(f'{self.pose_z}')
                
                else:
                    # rospy.loginfo('No Goal ID Aruco')
                    self.aruco_found = False
        else:
            # rospy.loginfo('Aruco could not found')
            self.aruco_found = False
    

    def stop_motion(self):
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd)
        # self.detection_command.data = False
        # self.detection_pub.publish(self.detection_command)
    
    def path_callback(self, msg):
        self.path = msg
        self.path_received = True
    
    def odom_callback(self, msg:Odometry):
        self.current_pose = msg.pose.pose

    def get_distance(self, x1, y1, x2, y2):
        return ((x2 - x1)**2 + (y2 - y1)**2)**0.5

    # def yaw_correction(self, tolerance):
    #     marker_point = self.get_transform(self._base_frame, self._marker_frame)

    #     if(self.aruco_found and self.yaw_correction_bool == False):
    #         angle = atan(marker_point.transform.translation.y / marker_point.transform.translation.x)
    #         rospy.loginfo(f'Angle: {angle}')
    #         rospy.loginfo(f'pose_y: {marker_point.transform.translation.y}')
    #     #     if(abs( > tolerance):
    #     #         self.cmd.linear.x = 0.0
    #     #         self.cmd.angular.z = - 0.1 * (self.yaw + 500 * self.pose_x) / abs(self.yaw + 500 * self.pose_x)
    #     #         self.cmd_vel_pub.publish(self.cmd)
            
    #     #     else:
    #     #         # self.yaw_correction_bool = True
    #     #         self.cmd.linear.x = 0.0
    #     #         self.cmd.angular.z = 0.0
    #     #         self.cmd_vel_pub.publish(self.cmd)

    #     # if(self.aruco_found == False and self.yaw_correction_bool == False):
    #     #     self.cmd.linear.x = 0.0
    #     #     self.cmd.angular.z = 0.2
    #     #     self.cmd_vel_pub.publish(self.cmd)
    
    def forward(self, limit_distance):
        if(self._min_dist_to_front > limit_distance):
            self.cmd.linear.x = 0.08
            self.cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd)
        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd)

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
        # while not rospy.is_shutdown():
        if self.path_received:
            if self.waypoint_index >= len(self.path.poses):
                rospy.loginfo("Reached the end of the path.")
                self.pre_point_reached = True
            
            else:    
                waypoint_pose = self.path.poses[self.waypoint_index]

                x = self.current_pose.position.x
                y = self.current_pose.position.y
                distance = self.get_distance(x, y, waypoint_pose.pose.position.x, waypoint_pose.pose.position.y)

                if distance < 0.2:
                    self.waypoint_index += 1
                    
                linear_error = distance
                angular_error = self.get_heading(x, y)
                linear_velocity = self.kp_linear * linear_error - self.kd_linear * distance
                angular_velocity = self.kp_angular * angular_error - self.kd_angular * angular_error

                # Create and publish Twist message
                twist = Twist()
                twist.linear.x = linear_velocity
                twist.angular.z = angular_velocity
                self.cmd_vel_pub.publish(twist)

            

    def execute_cb(self, goal: DockingGoal):
        self.detection_command.data = True
        self.yaw_correction_bool = False
        # self.detection_pub.publish(True)
        # time.sleep(3)
        self.goal_aruco_id = goal.aruco_id
        rate = rospy.Rate(self._freq)
        self._feedback.feedback = f'Docking type: {goal.type} ArucoID: {goal.aruco_id} Started'
        self._as.publish_feedback(self._feedback)
        rospy.loginfo(f'{self._action_name} Started')
        self._break = False
        self.pre_point_reached = False
        self.waypoint_index = 0
        while not self._done and not self._break:
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

            # if (self.yaw_correction_bool == False):
            #     self.yaw_correction(3)
            
            if (self.pre_point_reached == False):
                self.follow_path()

            
            if (self.pre_point_reached == True):
                rospy.loginfo(f'{self._min_dist_to_front}')
                limit_distance = 0.03
                self.forward(limit_distance)
                time.sleep(0.5)
                if (self.is_charging):
                    self._done = True
                    self.stop_motion()
                    break
                else:
                    self.forward(0.020)
               
            rospy.sleep(0.067)
            

        if self._done:
            rospy.loginfo(f'{self._action_name} Succeeded')
            self._result.done = self._done
            self._result.result = f'Docking type: {goal.type} ArucoID: {goal.aruco_id} Succeeded'
            self._as.set_succeeded(self._result)
            self.stop_motion()
            self._done = False



if __name__ == '__main__':
    rospy.init_node('docking')
    freq = rospy.get_param('rate', default=10)
    server = Docking(freq=freq)
    # time.sleep(3)
    # server.stanley_follower()
    rospy.spin()
    
   