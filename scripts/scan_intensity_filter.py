#!/usr/bin/env python3

import numpy as np
import sys
import rospy
from sensor_msgs.msg import LaserScan
import copy

#For urx, check documentation: http://www.me.umn.edu/courses/me5286/robotlab/Resources/scriptManual-3.5.4.pdf


class intensity_filter:
    def __init__(self):
        self.input_topic ="/cob_scan_unifier/scan_unified"
        self.output_topic ="/scan_unified_filtered"

        self.scan = LaserScan
        self.filtered_scan = LaserScan

        self.range_threshold = 1000
    

        self.ros_node = rospy.init_node('scan_intensity_filter', anonymous=True)
        self.scan_subs = rospy.Subscriber(self.input_topic, LaserScan, self.input_scan_callback)
        self.scan_pubs = rospy.Publisher(self.output_topic, LaserScan, queue_size=2)
        
        self.rate = rospy.Rate(100)

        rospy.spin()

    def input_scan_callback(self, input_scan):
        self.scan = input_scan
        self.filtered_scan = input_scan
        self.filtered_scan.ranges = list(input_scan.ranges)

        for i in range(len(input_scan.ranges)):
            if self.filtered_scan.ranges[i] > self.range_threshold:
                self.filtered_scan.ranges[i] = 0

        self.scan_pubs.publish(self.filtered_scan)
                
if __name__ == '__main__':
    filter = intensity_filter()
    exit()
