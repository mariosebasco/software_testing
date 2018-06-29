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
  
  //variables used by the state transition file
  bool COLLISION_DETECTED;
  float GOAL_X, GOAL_Y, ORIENTATION;
  
  TrackPoint(Collision* _collisionObject);
  int Init();
  void Task();
  void PublishSpeed(float lin_vel, float ang_vel);
  void UpdateOdom();

 private:
  ros::NodeHandle nh;
  ros::Publisher vel_pub;
  ros::Subscriber odom_sub;
  tf::Quaternion odom_quat;
  
  geometry_msgs::Twist cmd_vel;
  float odom_x, odom_y, odom_vel, angle_KP, position_KP, turn_in_place_KP;
  bool should_start;
  bool collision_detected;

  Collision *collisionObject;

  void OdomCallback(nav_msgs::Odometry msg);
  float FindAngleError(float x_des, float y_des);
  float FindPositionError(float x_des, float y_des);
  //void turnInPlace(double theta_des);
  float FindLookAheadDistance();
  float FindCurrMaxVel(float _vel2, float _vel1, float _dist2, float _dist1);
  double WrapAngle(double angle);
};
  
		

#endif
