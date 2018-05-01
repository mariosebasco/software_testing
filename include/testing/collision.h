/*
 *
 * given state of the vehicle:
 * - propagate the dynamics forward and check for collision
 * - if collision is going to occur return TRUE, else FALSE
 *
 *
 */


#ifndef COLLISION_H
#define COLLISION_H

#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <vector>

#include "state_controller.h"
#include "PeriodicTask.h"

class Collision: public PeriodicTask {
 public:
  bool received_map;
  bool received_odom;
    
  struct position{
    double x_pos;
    double y_pos;
    double theta;
  } positionStruct;
  
  Collision();
  int Init(char *name, double rate, int priority, double move_time_input, double resolution_input);
  void Task();

 private:
  double move_time, resolution;
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  ros::Subscriber costmap_sub;
  nav_msgs::Odometry odom_msg;
  tf::Quaternion odom_quat;
  nav_msgs::OccupancyGrid costmap;
  
  bool propagateState(double move_time_input, double resolution_input);
  bool costmapCheck();
  void odomCallback(nav_msgs::Odometry msg);
  void costmapCallback(nav_msgs::OccupancyGrid msg);

  inline double wrapAngle(double angle);
};

#endif