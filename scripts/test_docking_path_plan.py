#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion, Point, Transform, Vector3
from std_msgs.msg import Header
import tf2_ros
import numpy as np
import tf2_geometry_msgs
import sys

class SimplePathGen:
    def __init__(self) -> None:
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._marker_frame = rospy.get_param('marker_frame', default='ar_marker_1')
        self._base_frame = rospy.get_param('robot_frame', default='leo_bot/base_link')
        self._world_frame = rospy.get_param('world_frame', default='leo_bot/odom')
        self._interval = rospy.get_param('interval', default=0.05)
        self._transform_tolerance = rospy.get_param('transform_tolerance', default=0.5)
        self._path_pub = rospy.Publisher('docking_path/test_plan', Path, queue_size=1)

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

    def gen_path(self):
        base_point = TransformStamped(Header(frame_id=self._base_frame, stamp=rospy.Time.now()), self._base_frame, Transform(Vector3(), Quaternion(w=1.0)))
        marker_point = self.get_transform(self._base_frame, self._marker_frame)
        interval = self._interval
        if marker_point is not None:
            x_start, y_start = base_point.transform.translation.x, base_point.transform.translation.y
            x_end, y_end = marker_point.transform.translation.x - 0.5, marker_point.transform.translation.y 
            A = np.array([[x_start**2, x_start, 1],
                        [x_end**2, x_end, 1],
                        [2 * x_start, 1, 0]])
            B = np.array([y_start, y_end, 0])
            a, b, c = np.linalg.solve(A, B)
            x = np.arange(x_start, x_end, interval)
            y = a * x**2 + b * x + c
            return x, y
        else:
            rospy.logwarn(f"Failed to get transform {self._marker_frame} to {self._base_frame}.")
            return None

    def pub_path_in_base(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self._world_frame
        pose_stamped_list = []
        result  = self.gen_path()
        if result is not None:
            x, y = result
            for x_val, y_val in zip(x, y):
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.header.frame_id = self._world_frame
                pose_stamped.pose.position = Point(x_val, y_val, 0.0)
                pose_stamped.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
                pose_stamped_list.append(pose_stamped)

            path_msg.poses = pose_stamped_list
            self._path_pub.publish(path_msg)
        else:
            rospy.logwarn("Failed to generate path. Cannot publish path message.")
            self._path_pub.publish(path_msg)

    def pub_path_in_world(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self._world_frame
        poses: list[PoseStamped] = []
        result  = self.gen_path()
        if result is not None:
            x, y = result
            for x_val, y_val in zip(x, y):
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.header.frame_id = self._base_frame
                pose_stamped.pose.position = Point(x_val, y_val, 0.0)
                pose_stamped.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
                try:
                    transform = self._tf_buffer.lookup_transform(self._world_frame, self._base_frame, rospy.Time(0), rospy.Duration(1.0))
                    pose_in_odom = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
                    poses.append(pose_in_odom)
                except Exception as e:
                    rospy.logwarn(str(e))
            path_msg.poses = poses
            self._path_pub.publish(path_msg)
        else:
            rospy.logwarn("Failed to generate path. Cannot publish path message.")
            self._path_pub.publish(path_msg)

    @property
    def world_frame(self):
        return self._world_frame

    @property
    def robot_frame(self):
        return self._base_frame

if __name__ == '__main__':
    rospy.init_node('test_simple_path_generetor')
    freq = rospy.get_param('freq', default=10.0)
    rate = rospy.Rate(freq)
    simple_path = SimplePathGen()
    while not rospy.is_shutdown():
        try:
            if simple_path.world_frame == simple_path.robot_frame:
                simple_path.pub_path_in_base()
            else:
                simple_path.pub_path_in_world()
            rate.sleep()
        except KeyboardInterrupt:
            break
    else:
        sys.exit(0)
    rospy.spin()
