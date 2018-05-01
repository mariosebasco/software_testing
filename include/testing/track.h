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

#include "collision.h"
#include "AperiodicTask.h"
#include "state_controller.h"

class TrackPoint: public AperiodicTask {
 public:
  bool should_start, goal_interrupt;
  
  TrackPoint();
  int Init();
  void Task();
  void publishSpeed(float lin_vel, float ang_vel);
  void updateOdom();

 private:
  ros::NodeHandle nh;
  ros::Publisher vel_pub;
  ros::Subscriber odom_sub;
  tf::Quaternion odom_quat;
  
  geometry_msgs::Twist cmd_vel;
  float odom_x, odom_y, angle_KP, position_KP, turn_in_place_KP;
  //double imu_drift;

  void odom_callback(nav_msgs::Odometry msg);
  float findAngleError(float x_des, float y_des);
  float findPositionError(float x_des, float y_des);
  float findAngleCost(double des_northing, double des_easting, double curr_x, double curr_y, double curr_theta);
  float findPositionCost(double des_northing, double des_easting, double curr_x, double curr_y);
  void turnInPlace(double theta_des);

  void DWA(double des_northing, double des_easting);
  float propagateState(double des_northing, double des_easting, float lin_vel, float ang_vel, float move_time);
  double wrapAngle(double angle);
};
  
		

#endif
