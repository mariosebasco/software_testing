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
//#include "state_controller.h"

class TrackPoint: public AperiodicTask {
 public:
  
  bool should_start, goal_interrupt;
  bool collision_detected;

  //variables used by the state transition file
  bool COLLISION_DETECTED;
  float GOAL_X, GOAL_Y;
  
  TrackPoint(Collision* _collisionObject);
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

  Collision *collisionObject;

  void odom_callback(nav_msgs::Odometry msg);
  float findAngleError(float x_des, float y_des);
  float findPositionError(float x_des, float y_des);
  //void turnInPlace(double theta_des);

  double wrapAngle(double angle);
};
  
		

#endif
