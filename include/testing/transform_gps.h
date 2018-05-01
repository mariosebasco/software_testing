/*
 *
 *
 *
 *
 *
 */

#ifndef TRANSFORM_GPS_H
#define TRANSFORM_GPS_H


#include <math.h>
#include <stdio.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "AperiodicTask.h"

class TransformGPS: public AperiodicTask {
 public:
  double init_northing;
  double init_easting;
  bool received_data;

  TransformGPS();
  int Init();

 private:
  ros::NodeHandle nh;
  ros::Publisher odom_pub;
  ros::Subscriber odom_sub;
  nav_msgs::Odometry odom_data;

  double current_northing;
  double current_easting;

  void Task();
  void odomCallback(nav_msgs::Odometry msg);

};

#endif
