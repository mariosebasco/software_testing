/*
 *
 *
 *
 *
 *
 */

#ifndef PATH_H
#define PATH_H

#include <math.h>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "testing/Path_msg.h"
#include "AperiodicTask.h"


class Path: public AperiodicTask {
 public:
  Path();
  int Init();

 private:
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  ros::Publisher path_pub;

  bool received_odom;
  nav_msgs::Odometry odom_msg;
  testing::Path_msg path_msg;

  void Task();
  float FindLookAheadDistance();
  float FindCurrMaxVel(float _vel2, float _vel1, float _dist2, float _dist1);
  double FindDistToSegment(double _x1, double _y1, double _x2, double _y2);
  void OdomCB(nav_msgs::Odometry msg);
};

#endif
