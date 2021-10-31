#!/usr/bin/env python

import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt

class Analysis():
    def __init__(self):
        #
        # self.velocities = self.data_frame([])
        plt.close('all')
        print(os.getcwd())
        os.chdir("/home/mayur/pmb2_public_ws/src/stop_dist_test/log")
        print(os.getcwd())
        self.data_frame1 = pd.read_csv("stop_dist_test-30_10_2021_16_43_59.csv", index_col = False)
        print(self.data_frame1)
        self.velocities = self.data_frame1['velocity_before_break'].to_numpy()
        # print(self.velocities)
        # print(self.velocities.dtype)
        self.times = self.data_frame1['stop_time'].to_numpy()
        # print(self.times)
        self.stop_odom = self.data_frame1['stop_distance_odom'].to_numpy()
        # print(self.stop_odom)
        self.stop_lidar = self.data_frame1['stop_distance_lidar'].to_numpy()
        # print(self.stop_lidar)
        plt.figure(1)
        plt.title("Velocity vs Stopping Time")
        plt.xlabel("velocity before stop (m/s)")
        plt.ylabel("stopping time (s)")
        plt.plot(self.velocities,self.times,'r-')
        plt.savefig('velocity_vs_stop_time.png')
        plt.figure(2)
        plt.title("Velocity vs Odometry stopping distance")
        plt.xlabel("stopping distance (m/s)")
        plt.ylabel("stopping time (m)")
        plt.plot(self.velocities,self.stop_odom,'g-')
        plt.savefig('velocity_vs_stop_distance_odom.png')
        plt.figure(3)
        plt.title("Velocity vs Lidar stopping distance")
        plt.xlabel("stopping distance (m)")
        plt.ylabel("stopping time (s)")
        plt.plot(self.velocities,self.stop_lidar)
        plt.savefig('velocity_vs_stop_distance_lidar.png')
        plt.show()
analyze = Analysis()
