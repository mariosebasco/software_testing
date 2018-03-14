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

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

class TrackPoint {
 public:
  bool should_start, goal_interrupt;

  TrackPoint();
  void tracker(geometry_msgs::Pose *desired_poses);
  void odom_callback(nav_msgs::Odometry msg);
  void publishSpeed(float lin_vel, float ang_vel);
  void updateOdom();
  float findAngleError(float x_des, float y_des);
  float findPositionError(float x_des, float y_des);
  

 private:
  ros::NodeHandle nh;
  ros::Publisher vel_pub;
  ros::Subscriber odom_sub;
  tf::Quaternion odom_quat;
  geometry_msgs::Twist cmd_vel;
  float odom_x, odom_y, angle_KP, position_KP;

};
  
		

#endif
