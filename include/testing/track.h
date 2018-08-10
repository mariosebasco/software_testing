/*
 *
 *  Path follower - 
 *  INPUT: desired poses
 *  OUTPUT: command velocity
 */


#ifndef TRACKER_H
#define TRACKER_H

#include <iostream>
#include <math.h>
#include <list>
#include <fstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include "collision.h"
#include "AperiodicTask.h"
#include "testing/Path_msg.h"

class TrackPoint: public AperiodicTask {
 public:
  bool INTERRUPT;

  TrackPoint();
  int Init();

 private:
  ros::NodeHandle nh;
  ros::Publisher vel_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber path_sub;
  
  geometry_msgs::Twist cmd_vel;
  nav_msgs::Odometry odom_msg;
  tf::Quaternion odom_quat;
  testing::Path_msg path_msg;
  bool received_odom, received_path;

  void Task();
  void PublishSpeed(float lin_vel, float ang_vel);
  void OdomCB(const nav_msgs::Odometry &msg);
  void PathCB(const testing::Path_msg &msg);
  float FindAngleError(float x_des, float y_des);
  float FindPositionError(float x_des, float y_des);
  double WrapAngle(double angle);
};
  
#endif
