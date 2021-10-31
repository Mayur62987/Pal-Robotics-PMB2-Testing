#!/usr/bin/env python

import rospy
import numpy as np
import math
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Time, Int32, String, Int8, Float32


class Scanner():
    def __init__(self):
        print("scanner data init")
        self.Scan_data = rospy.Subscriber(
            'scan_raw', LaserScan, self.scan_callback)
        self.Scanner = LaserScan()
        self.scan_dist = 0.0
        self._scan_pub = rospy.Publisher('/ave_lidar_dist',Float32,
         queue_size=10)
        self.average_dist = 0.0
        self.rate = rospy.Rate(20)


    def scan_callback(self,msg):
        #print("scancallback")
        self.Scanner.ranges = msg.ranges
        self.Scanner.angle_increment = msg.angle_increment
        self.average_dist = (self.Scanner.ranges[330]*math.cos(self.Scanner.angle_increment*3) +
         self.Scanner.ranges[331]*math.cos(self.Scanner.angle_increment*2)+
         self.Scanner.ranges[332]*math.cos(self.Scanner.angle_increment)+
         self.Scanner.ranges[333]+ self.Scanner.ranges[334]*math.cos(self.Scanner.angle_increment)+
         self.Scanner.ranges[335]*math.cos(self.Scanner.angle_increment*2)+
         self.Scanner.ranges[336]*math.cos(self.Scanner.angle_increment*3))/7
        self._scan_pub.publish(self.average_dist)

        # print(self.average_dist)
        # print(self.Scanner.ranges[330]*math.cos(self.Scanner.angle_increment*3))
        # print(self.Scanner.ranges[331]*math.cos(self.Scanner.angle_increment*2))
        # print(self.Scanner.ranges[332]*math.cos(self.Scanner.angle_increment))
        # print(self.Scanner.ranges[333])
        # print(self.Scanner.ranges[334]*math.cos(self.Scanner.angle_increment))
        # print(self.Scanner.ranges[335]*math.cos(self.Scanner.angle_increment*2))
        # print(self.Scanner.ranges[336]*math.cos(self.Scanner.angle_increment*3))


    # def Ave_dist_publisher(self):
    #     self._scan_pub.publish(self.average_dist)
    #     self.rate.sleep()



    # def scan_calc(self):
    #     time.sleep(0.07)

        # self.scan_dist = (scan_ptl2+scan_1+scan_c+scan_ptr1+scan_ptr2)/7
        # print(self.scan_dist)
        # self.scan_dist = (self.Scanner.ranges[331]* math.cos(
        #  self.Scanner.angle_increment*2) + self.Scanner.ranges[332] * math.cos(
        #  self.Scanner.angle_increment) + self.Scanner.ranges[333] +
        #  self.Scanner.ranges[334]*math.cos(self.Scanner.angle_increment) +
        #  self.Scanner.ranges[335]*math.cos(self.Scanner.angle_increment*2))/5
        # print(self.scan_dist)
        # self._scan_pub


if __name__ == '__main__':
    try:
        rospy.init_node('scan_dist')
        this_scan = Scanner()
        # this_scan._scan_pub.publish(this_scan.average_dist)
        this_scan.rate.sleep()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
