#!/usr/bin/env python3
import rospy

import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

class Docking_front_scan:
    def __init__(self):
        self.count = 0
        self.front_distance = Float64()
        self._robot_front_edges = rospy.get_param('robot_front_edges', [[0.35, 0.33], [0.35, -0.33]])
        self.pub = rospy.Publisher("/leo_bot/docking_front_distance",Float64, queue_size=10)
        rospy.Subscriber('/leo_bot/scan', LaserScan, self.scan_callback, queue_size=1)
        
            
    def scan_callback(self, scan_msg: LaserScan):
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        x1, y1 = self._robot_front_edges[0]
        x2, y2 = self._robot_front_edges[1]

        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)

        mask = np.logical_and(ly > y2, np.logical_and(ly < y1, np.logical_and(lx > x1, lx > x2)))

        rospy.loginfo(ly[mask] * 100)


        # if len(ranges_in_front) > 0:
        #     # self._min_dist_to_front = np.min(ranges_in_front)
        #     # rospy.loginfo(f'{self._min_dist_to_front}')
        #     self.front_distance.data = np.min(ranges_in_front)
        #     self.pub.publish(self.front_distance)
        
        

        

if __name__ == "__main__":
    rospy.init_node("Docking_Front_scan")
    r = rospy.Rate(20)
    docking_front_distance = Docking_front_scan()
    rospy.spin()            



