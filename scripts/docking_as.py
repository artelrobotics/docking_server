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
import time
from fiducial_msgs.msg import FiducialTransformArray 
import dynamic_reconfigure.client

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
        rospy.Subscriber('docking_path/plan/fiducial_2', Path, callback=self.path_callback_fiducial_2)
        rospy.Subscriber('docking_path/plan/fiducial_3', Path, callback=self.path_callback_fiducial_3)
        rospy.Subscriber('daly_bms/data', BatteryStatus, callback=self.bms_callback)
        rospy.Subscriber('fiducial_transforms', FiducialTransformArray, callback=self.fiducial_distance_callback)
        self.hook_ctrl = rospy.ServiceProxy('hooks_ctrl', SetBool)
        # rospy.Subscriber('line_segments', LineSegmentList, callback=self.line_segments)
        rospy.Subscriber('odometry', Odometry, callback=self.odom_callback)
        self.reconfigure = Server(ConfigConfig, self.reconfigure_callback)
        self.client_local = dynamic_reconfigure.client.Client("/camel_amr_500_001/move_base/local_costmap", timeout=30, config_callback=self.callback)
        self.client_global = dynamic_reconfigure.client.Client("/camel_amr_500_001/move_base/global_costmap", timeout=30, config_callback=self.callback)
        rospy.logerr("Server initialized reconfigure")
        self._as = actionlib.SimpleActionServer(self._action_name, DockingAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.aruco_detection(False)
        self.fiducial_2_distance = 100
        self.hook = False
        
    def callback(self ,config):
        rospy.loginfo("Local Footprint Config set to ".format(**config))
    
    def reconfigure_callback(self, config, level):
        self.kp_linear = config.kp_linear
        self.kd_linear = config.kd_linear
        self.kp_angular = config.kp_angular
        self.kd_angular = config.kd_angular
        rospy.loginfo(f'{self.kp_angular, self.kd_linear} reconfigured')
        return config

    def hook_service(self, bool):
        time.sleep(3)
        self.hook_ctrl(bool)
        if bool:
            self.client_local.update_configuration({"footprint":[[-0.75,-0.75],[-0.75,0.75],[0.75,0.75],[0.75,-0.75]]})
            self.client_global.update_configuration({"footprint":[[-0.75,-0.75],[-0.75,0.75],[0.75,0.75],[0.75,-0.75]]})
        if not bool:
            self.client_local.update_configuration({"footprint":[[-0.647,-0.42],[-0.647,0.42],[0.647,0.42],[0.647,-0.42]]})
            self.client_global.update_configuration({"footprint":[[-0.647,-0.42],[-0.647,0.42],[0.647,0.42],[0.647,-0.42]]})
        time.sleep(3)
        self.hook = bool

    
    def fiducial_distance_callback(self, msg):
        for tf in msg.transforms:
            if (tf.fiducial_id == 2):
                self.fiducial_2_distance = tf.transform.translation.z
            
            if (tf.fiducial_id == 3):
                self.fiducial_3_distance = tf.transform.translation.z
            
           

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

    def backward_motion(self, duration):
        initial_time = time.time()
        now = time.time()
        while(now - initial_time < duration):
            self.cmd.linear.x = -0.2
            self.cmd.angular.z = 0
            self.cmd_vel_pub.publish(self.cmd)
            now = time.time()

    def path_callback_fiducial_1(self, msg):
        self.path_fiducial_1 = msg
        self.path_received_fiducial_1 = True

    def path_callback_fiducial_2(self, msg):
        self.path_fiducial_2 = msg
        self.path_received_fiducial_2 = True

    def path_callback_fiducial_3(self, msg):
        self.path_fiducial_3 = msg
        self.path_received_fiducial_3 = True

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
        path_x = self.path_fiducial_2.poses[self.waypoint_index_2].pose.position.x
        path_y = self.path_fiducial_2.poses[self.waypoint_index_2].pose.position.y
        path_heading = atan2(path_y - y, path_x - x)
        

        return self.normalize(path_heading - yaw)

    def follow_path_fiducial_2(self):
        if self.path_received_fiducial_2:
            if self.waypoint_index_2 >= len(self.path_fiducial_2.poses):
                rospy.loginfo("Reached the end of the fiducial 2")
                self.pre_point_reached = True

            else:
                waypoint_pose = self.path_fiducial_2.poses[self.waypoint_index_2]

                x = self.current_pose.position.x
                y = self.current_pose.position.y
                
                distance = self.get_distance(x, y, waypoint_pose.pose.position.x, waypoint_pose.pose.position.y)

                if distance < 0.2:
                    self.waypoint_index_2 += 1

                linear_error = distance
                angular_error = self.get_heading_fiducial_2(x, y)
                linear_velocity = self.kp_linear * linear_error - self.kd_linear * distance
                angular_velocity = self.kp_angular * angular_error - self.kd_angular * angular_error

                # Create and publish Twist message
                twist = Twist()
                twist.linear.x = linear_velocity
                twist.angular.z = angular_velocity
                self.cmd_vel_pub.publish(twist)

    def get_heading_fiducial_3(self, x, y):
        _, _, yaw = euler_from_quaternion([self.current_pose.orientation.x,
                                           self.current_pose.orientation.y,
                                           self.current_pose.orientation.z,
                                           self.current_pose.orientation.w])
        path_x = self.path_fiducial_3.poses[self.waypoint_index_3].pose.position.x
        path_y = self.path_fiducial_3.poses[self.waypoint_index_3].pose.position.y
        path_heading = atan2(path_y - y, path_x - x)
        

        return self.normalize(path_heading - yaw)

    def follow_path_fiducial_3(self):
        if self.path_received_fiducial_3:
            if self.waypoint_index_3 >= len(self.path_fiducial_3.poses):
                rospy.loginfo("Reached the end of the fiducial 3")
                self.pre_point_reached = True

            else:
                waypoint_pose = self.path_fiducial_3.poses[self.waypoint_index_3]

                x = self.current_pose.position.x
                y = self.current_pose.position.y
                
                distance = self.get_distance(x, y, waypoint_pose.pose.position.x, waypoint_pose.pose.position.y)

                if distance < 0.2:
                    self.waypoint_index_2 += 1

                linear_error = distance
                angular_error = self.get_heading_fiducial_3(x, y)
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
        self.fiducial_2_distance = 100
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

                if (self.pre_point_reached == False):
                    self.waypoint_index = 0
                    self.follow_path_fiducial_1()

                if (self.pre_point_reached == True ):
                    print(self.fiducial_2_distance)
                    if(self.fiducial_2_distance < 1.0 and self.hook == False):
                        self.hook_service(True)
                    
                    if(self.fiducial_2_distance < 0.4 and self.hook):
                        self._done = True
                        self.stop_motion()
                        break

                    self.waypoint_index_2 = 0
                    self.follow_path_fiducial_2()


                rospy.sleep(0.1)

        if (goal.type == "undocking"):
            self.aruco_detection(True)
            self._feedback.feedback = f'UnDocking type: {goal.type} ArucoID: {goal.aruco_id} Started'
            self._as.publish_feedback(self._feedback)
            rospy.loginfo(f'Undocking Started')
            self._break = False
            self.fiducial_2_distance = 10
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
                
                if(self.fiducial_2_distance < 0.7):
                    print(self.fiducial_2_distance)
                    print("backward_motion")
                    self.hook_service(False)
                    self.backward_motion(10)
                    self._done = True
                    break
                
                else:
                    self.waypoint_index_2 = 0
                    self.follow_path_fiducial_2()

                rospy.sleep(0.1)              

        if (goal.type == "charging" and goal.aruco_id==3):
            self.aruco_detection(True)
            self._feedback.feedback = f'UnDocking type: {goal.type} ArucoID: {goal.aruco_id} Started'
            self._as.publish_feedback(self._feedback)
            rospy.loginfo(f'Dock to charging station started')
            self._break = False
            self.fiducial_3_distance = 10
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

                print(self.fiducial_3_distance)
                if(self.fiducial_3_distance > 0.7):
                    self.waypoint_index_3 = 0
                    self.follow_path_fiducial_3()
                
                if(self.is_charging):
                        self._done = True
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
    rospy.init_node('docking_as')
    freq = rospy.get_param('rate', default=10)
    server = Docking(freq=freq)
    rospy.spin()
