#!/usr/bin/env python

#Author : Mayur Ramjee
#Description : Data logger from stopping distance tests which reads from stop_test_results Topic
#and publishes to a csv file
#adapted from https://github.com/rbrobots/pmb2_stopping_distance/tree/main/src/pmb2_automation
#todo impliment live plot function

import rospy
import math
import csv
import matplotlib.pyplot as plt
from stop_dist_test.msg import Results
from datetime import datetime


class Data():
  def __init__(self):
    self.log = Results()
    dt = datetime.now()
    # self.rate = rospy.Rate(1)
    self.dt_format = dt.strftime("%d_%m_%Y_%H_%M_%S")
    self.s_dist_sub = rospy.Subscriber('stop_test_results', Results, self.s_dist_clbck)
    self.fig, self.ax = plt.subplots(2)
    self._ln1 = plt.plot([],[],'ro')
    self._ln2 = plt.plot([],[],'b-')
    self.velocity_in = []
    self.time_stop  = []
    self.lidar_stop = []
    self.odom_stop = []



    csv_name = 'stop_dist_test-'+str(self.dt_format)+'.csv'

    record=(str("rostime")+ ',' + str("surface") + ',' + str("velocity_before_break") + ',' + str("stop_distance_lidar")
     + ',' + str('stop_distance_odom') + ',' + str("stop_time"))

    with open("./log/"+ csv_name, 'a') as log:
        log.write(record + "\n")
        log.flush()

    while True:
      self.rate.sleep()
      self.rostime = rospy.get_time()
      record = (str(str.format('{0:.1f}',self.rostime)) + ',' + str(self.log.surface) + ','  +
       str(str.format('{0:.2f}',self.log.velocity_before_break))
       + ',' + str(str.format('{0:.6f}',self.log.stop_dist_lidar)) + ',' +
       str(str.format('{0:.6f}',self.log.stop_dist_odom)) +
       ',' + str(str.format('{0:.8f}',self.log.stop_time)))

      with open("./log/"+ csv_name, 'a') as log:
        log.write(record + "\n")
        log.flush()


  def s_dist_clbck(self,msg):
      self.log.surface = msg.surface
      self.log.velocity_before_break = msg.velocity_before_break
      self.log.stop_dist_lidar = msg.stop_dist_lidar
      self.log.stop_dist_odom = msg.stop_dist_odom
      self.log.stop_time = msg.stop_time
      self.log.dist_error = msg.dist_error

      self.velocity_in.append(msg.velocity_before_break)
      self.lidar_stop.append(msg.stop_dist_lidar)
      self.odom_stop.append(msg.stop_dist_odom)
      self.time_stop.append(msg.stop_time)

  def plot_init(self):
      sef.ax[0].set_xlim(0.1,1.1)
      self.ax[0].set_ylim(0,1)
      self.ax[1].set_xlim(0.1,1.1)
      self.ax[1].set_ylim(0,1)
      return self._ln1, self._ln2

  def plotdata(self):
      self._ln1.set_data(self.velocity_in, self.lidar_stop)
      self._ln2.set_data(self.velocity_in, self.time_stop)
      return self._ln1 , self._ln2


if __name__ == '__main__':

    rospy.init_node('logger', disable_signals = True)
    rate = rospy.Rate(1)
    logger_ = Data()
    # ani = FuncAnimation(vis.fig, vis.update_plot, init_func=logger.plot_init)
    # plt.show(block=True)
    rospy.spin()
