#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from math import cos, sin

def scan_callback(scan_data):
    # Extract laser scan data
    ranges = scan_data.ranges
    angle_increment = scan_data.angle_increment
    angle_min = scan_data.angle_min

    # Define variables to represent a line
    line_points = []
    threshold_distance = 0.1  # Adjust this threshold as needed
    current_line = None

    for i, range_value in enumerate(ranges):
        # Calculate the angle for the current range reading
        angle = angle_min + i * angle_increment

        # Check if the range is within a threshold distance
        point = Point()
        next_point = Point()
        point.x = range_value * cos(angle)
        point.y = range_value * sin(angle)
        next_point.x = ranges[i+1] * cos(angle)
        next_point.y = ranges[i+1] * sin(angle)
        if (abs(point.x - next_point.x < threshold_distance) or abs(point.x - next_point.x) < threshold_distance ):
            line_points.append(point)
            if current_line is None:
                current_line = [point]
            else:
                current_line.append(point)
        else:
            if current_line is not None:
                # Publish the detected line as a visualization marker or store it as needed
                publish_line_marker(current_line)
                current_line = None
                line_points = []

    if current_line is not None:
        # Publish the last detected line
        publish_line_marker(current_line)

def publish_line_marker(line_points):
    # Create a Marker message to represent the detected line
    line_marker = Marker()
    line_marker.header = Header(frame_id='camel_amr_500_001/base_link')  # Modify frame_id as needed
    line_marker.type = Marker.LINE_STRIP
    line_marker.action = Marker.ADD
    line_marker.scale.x = 0.01  # Line width
    line_marker.color.r = 1.0  # Line color (red)
    line_marker.color.a = 1.0  # Line transparency (fully visible)

    # Set the points for the line
    line_marker.points = line_points

    # Publish the line marker to a ROS topic
    line_marker_publisher.publish(line_marker)

def line_detection_node():
    rospy.init_node('line_detection_node', anonymous=True)
    rospy.Subscriber('scan', LaserScan, scan_callback)  # Modify topic name
    global line_marker_publisher
    line_marker_publisher=rospy.Publisher("scan_lines", Marker, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        line_detection_node()
    except rospy.ROSInterruptException:
        pass
