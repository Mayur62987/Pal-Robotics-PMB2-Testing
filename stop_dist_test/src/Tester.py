#!/usr/bin/env python

#Author : Mayur Ramjee

#Description : Testing node for pmb2 robot stopping distance (calc_distance)
#a range of linear velocities (0.1-1) m/s are applied with stopping distance determined
#from both Sick Lidar 571 and odemetery, resutls published on /stop_test_results topic

import rospy
import numpy as np
import time
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from stop_dist_test.msg import Results

from sensor_msgs.msg import LaserScan

from gazebo_msgs.msg import ModelState
from std_msgs.msg import String, Float32,Float64,Int32, Time

class Tester():
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/mobile_base_controller/odom',Odometry,
         self.odom_clbk)
        # self.laser_dist_sub = rospy.Subscriber('/ave_lidar_dist',Float32,self.lidar_clback)

        self.laser_sub = rospy.Subscriber('/scan_raw',LaserScan,self.laser_clbk)

        self.model_pub = rospy.Publisher('/gazebo/set_model_state',ModelState,
         queue_size = 10)
        self.vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel',Twist,
         queue_size = 10)
        self.data_publisher = rospy.Publisher('/stop_test_results',Results,queue_size=10)
        self.results = Results()
        self.model_state = ModelState()
        self.velocity = Twist()
        self.odometry_real = Odometry()

        self.Scanner = LaserScan()
        self.breaking_vel  = 0

        self.laser_dist = 0
        self.move_dist = 0
        self.stop_dist = 0
        self.lidar_difference = 0
        self.odom_move_dist = Pose()
        self.odom_stop_dist = Pose()
        self.odom_difference = Pose()
        self.uptospeed = False
        self.stopped = False
        self.average_dist  = 0

        self.measure = False

    def odom_clbk(self, msg):
        self.odometry_real.twist.twist.linear.x = msg.twist.twist.linear.x
        self.odometry_real.twist.twist.angular.z = msg.twist.twist.angular.z #for future use
        self.odometry_real.pose.pose.position.x = msg.pose.pose.position.x
        self.odometry_real.pose.pose.orientation.z = msg.pose.pose.orientation.z #for future use
        # print(self.odometry_real.pose.pose.position.x)
        # print(self.odometry_real.twist.twist.linear.x)
    #
    # def lidar_clback(self, msg):case
    #     self.laser_dist = msg.data
    #     # print(self.laser_dist)


    def move_item_in_world(self,name,pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w):
        self.model_state.model_name = name*
        self.model_state.pose.position.x = pos_x
        self.model_state.pose.position.y = pos_y
        self.model_state.pose.position.z = pos_z
        self.model_state.pose.orientation.x = ori_x
        self.model_state.pose.orientation.y = ori_y
        self.model_state.pose.orientation.z = ori_z
        self.model_state.pose.orientation.w = ori_w
        self.model_state.reference_frame = "world"
        time.sleep(1)
        self.model_pub.publish(self.model_state)
        time.sleep(1)


    def add_slip(self):
        self.move_item_in_world("slippery_patch",0,0,0,0,0,0,0)
        time.sleep(2)
        # self.move_item_in_world("slippery_patch_0",1,0,0,0,0,0,0)
        # time.sleep(2)

    def reset(self):
        self.move_item_in_world("pmb2",0,0,0,0,0,0,0)
        self.velocity = Twist()
        self.capture = Results()
        self.odometry_real = Odometry()

        self.breaking_vel = 0

        self.laser_dist = 0
        self.move_dist = 0

        self.stop_dist = 0
        self.differnce = 0
        self.odom_move_dist = Pose()
        self.odom_stop_dist = Pose()
        self.odom_difference = Pose()
        self.time1 = 0
        self.time2 = 0
        self.measure = False
        self.stoptime = 0
        self.uptospeed = False
        self.average_dist  = 0
        self.stopped = False

    def add_slip(self):
        self.move_item_in_world("slippery_patch",0.25,0,0,0,0,0,0)
        time.sleep(1)
        self.move_item_in_world("slippery_patch_0",0.75,0,0,0,0,0,0)
        time.sleep(1)
        self.move_item_in_world("slippery_patch_1",1.25,0,0,0,0,0,0)


    # def check_scan(self):    # def check_scan(self):
    #     self.move_dist = self.laser_dist
    #     print(self.move_dist)
    #     self.move_dist = self.laser_dist
    #     print(self.move_dist)





    def laser_clbk(self,msg):
        self.Scanner.ranges = msg.ranges        # self.move_item_in_world("slippery_patch_0",0.75,0,0,0,0,0,0)
        # self.move_item_in_world("slippery_patch_1",1.25,0,0,0,0,0,0)
        self.Scanner.angle_increment = msg.angle_increment
        self.average_dist = (self.Scanner.ranges[330]*math.cos(self.Scanner.angle_increment*3) +
         self.Scanner.ranges[331]*math.cos(self.Scanner.angle_increment*2)+
         self.Scanner.ranges[332]*math.cos(self.Scanner.angle_increment)+
         self.Scanner.ranges[333]+ self.Scanner.ranges[334]*math.cos(self.Scanner.angle_increment)+
         self.Scanner.ranges[335]*math.cos(self.Scanner.angle_increment*2)+
         self.Scanner.ranges[336]*math.cos(self.Scanner.angle_increment*3))/7




    def set_velocity(self,linear):
        self.target_vel = linear
        self.velocity.linear.x = self.target_vel
        self.velocity.angular.z = 0
        self.vel_pub.publish(self.velocity)
*
    def run_velocities(self,surface="standard"):
        for vel in np.arange(0.1, 1.1, 0.1):
            self.breaking_vel = vel
            self.surface = surface
            print("test with linear velocity=",round(vel,1))
            self.calc_distance(vel,surface)

    def calculate_difference(self,mode):
        if(mode == 1):
            # self.move_dist = self.laser_dist
            self.move = self.average_dist
            self.odom_move_dist.position.x = self.odometry_real.pose.pose.position.x
            # print("odom break:")
            # print(self.odom_move_dist.position.x)
            # print("lidar break:")
            # print(self.move)
            self.time1 = time.time()
        elif(mode == 2):
            # self.stop_dist = self.laser_dist
            self.stop_dist = self.average_dist
            self.odom_stop_dist.position.x = self.odometry_real.pose.pose.position.x
            # print("odom stop:")
            # print(self.odom_stop_dist.position.x)
            # print("lidar stop")
            # print(self.stop_dist)
            self.time2 = time.time()
        elif(mode == 3):
            self.lidar_difference = abs(self.stop_dist-self.move)
            self.odom_difference.position.x = abs(self.odom_stop_dist.position.x - self.odom_move_dist.position.x)
            # print("odom dist:")
            # print(self.odom_difference.position.x)
            # print("lidar dist:")
            # print(self.lidar_difference)
            self.stoptime = self.time2 - self.time1


    def data_capture(self,vel):
        self.capture.surface = self.surface
        self.capture.velocity_before_break = round(vel,1)
        print(self.capture.velocity_before_break)
        self.capture.stop_dist_lidar = self.lidar_difference
        self.capture.stop_dist_odom = self.odom_difference.position.x
        self.capture.stop_time = self.stoptime
        self.capture.dist_error = (self.capture.stop_dist_lidar - self.capture.stop_dist_odom)

        print("publishing results to /stop_results")
        print("---------------------")

        print("surface=", self.capture.surface)
        print("velocity before break applied =", self.capture.velocity_before_break)
        print("stop_dist_odoom=",self.capture.stop_dist_odom)
        print("stop_dist_lidar=", self.capture.stop_dist_lidar)
        print("stop_dist_error=", self.capture.dist_error)
        print("stop_time=", self.capture.stop_time)
        print("----------------------")

        time.sleep(2)
        self.data_publisher.publish(self.capture)
        time.sleep(2)

    def calc_distance(self,x,surface="standard"):
        self.surface = surface
        self.reset()
        # time.sleep(0.2)
        self.set_velocity(x)
        while True:
            if (round(self.odometry_real.twist.twist.linear.x,2) < round(x,1) and self.uptospeed == False):
                self.set_velocity(x)
                # print(x)
                # print(self.odometry_real.twist.twist.linear.x)
            else:
                self.uptospeed = True
                self.set_velocity(0)
                if self.measure == False:
                    self.calculate_difference(1)
                    self.measure = True
            if (round(self.odometry_real.twist.twist.linear.x,2) <= 0 and self.uptospeed == True):
                self.stopped = True
                self.calculate_difference(2)
                break
        if self.stopped == True:
            self.calculate_difference(3)
        self.data_capture(x)



if __name__ == '__main__':
    try:
        rospy.init_node('stopping_test_dist_tester')
        pub_rate = rospy.Rate(1)
        aut = Tester()
        aut.run_velocities("standard")
        # aut.add_slip()
        aut.reset()
        # aut.run_velocities("slippery surface")
        # aut.reset()
        rospy.loginfo("stopping distance test completed")

        #
        # aut = Tester()
        # time.sleep(1)
        # aut.add_slip()
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass
